/**
 * @file DFRobot_Voreo.cpp
 * @brief  DFRobot Voreo library
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @version 1.0
 * @date 2025-12-12
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */

#include "DFRobot_Voreo.h"

static uint16_t calculateCRC16(const uint8_t *data, size_t len)
{
    uint16_t crc = CRC16_INITIAL_VALUE;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// 构建请求帧
static void buildRequestFrame(uint8_t *buf, uint8_t cmd)
{
    buf[0] = VOREO_CMD_HEAD;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = cmd;
    uint16_t crc = calculateCRC16(buf, 4);
    buf[4] = crc >> 8;
    buf[5] = crc & 0xFF;
}

// 构建命令帧
static void buildCommandFrame(uint8_t *buf, uint8_t cmd, uint8_t *pData, uint16_t dataLength)
{
    buf[0] = VOREO_CMD_HEAD;
    buf[1] = (dataLength >> 8) & 0xff;
    buf[2] = dataLength & 0xff;
    buf[3] = cmd;
    if (dataLength > 0 && pData != NULL) {
        memcpy(buf + 4, pData, dataLength);
    }
    uint16_t crc = calculateCRC16(buf, dataLength + 4);
    buf[dataLength + 4] = crc >> 8;
    buf[dataLength + 5] = crc & 0xFF;
}

// 验证CRC并返回数据
static uint8_t *validateAndReturnData(uint8_t *pBuf, uint16_t length)
{
    uint16_t calculated_crc = calculateCRC16(pBuf, length + 4);
    uint16_t received_crc = (uint16_t)pBuf[length + 4] << 8 | pBuf[length + 5];
    if (calculated_crc != received_crc) {
        DBG("crc error");
        free(pBuf);
        return NULL;
    }
    return pBuf;
}

// 处理数据解析状态机（通用解析逻辑）
static uint8_t processDataState(uint8_t data, uint8_t cmd, uint8_t *requestStatic, 
                                 uint16_t *length, uint16_t *dataLength, uint8_t **pBuf)
{
    switch (*requestStatic) {
    case DATA_STATIC_HEAD:
        if (data == VOREO_CMD_HEAD) {
            *requestStatic = DATA_STATIC_LEN_H;
            *dataLength = 0;
        }
        return 0;
        
    case DATA_STATIC_LEN_H:
        *length = (uint16_t)data << 8;
        *requestStatic = DATA_STATIC_LEN_L;
        return 0;
        
    case DATA_STATIC_LEN_L:
        *length |= data;
        *requestStatic = DATA_STATIC_CMD;
        return 0;
        
    case DATA_STATIC_CMD:
        if (data != cmd) {
            *requestStatic = DATA_STATIC_HEAD;
            *length = 0;
            *dataLength = 0;
            if (*pBuf) {
                free(*pBuf);
                *pBuf = NULL;
            }
            return 0;
        }
        
        *pBuf = (uint8_t *)malloc(*length + 6);
        if (*pBuf == NULL) {
            DBG("malloc failed");
            return 1; // 错误
        }
        (*pBuf)[0] = VOREO_CMD_HEAD;  
        (*pBuf)[1] = *length >> 8;
        (*pBuf)[2] = *length & 0xff;
        (*pBuf)[3] = cmd;
        *requestStatic = (*length == 0) ? DATA_STATIC_CRC_H : DATA_STATIC_DATA;
        return 0;
        
    case DATA_STATIC_DATA:
        if (*dataLength < *length) {
            (*pBuf)[*dataLength + 4] = data;
            (*dataLength)++;
        }
        if (*dataLength >= *length) {
            *requestStatic = DATA_STATIC_CRC_H;
        }
        return 0;
        
    case DATA_STATIC_CRC_H:
        (*pBuf)[*length + 4] = data;
        *requestStatic = DATA_STATIC_CRC_L;
        return 0;
        
    case DATA_STATIC_CRC_L:
        (*pBuf)[*length + 5] = data;
        return 2; // 完成
        
    default:
        return 0;
    }
} 

uint8_t DFRobot_Voreo::begin(void)
{
    uint8_t *pBuf = readData(VOREO_CMD_BEGIN);
    if (pBuf == NULL) {
        return RETURN_ERROR;
    }
    uint8_t result = (pBuf[4] != 0) ? RETURN_OK : RETURN_ERROR;
    free(pBuf);
    return result;
}

uint8_t DFRobot_Voreo::queryText(void)
{
    uint8_t *pBuf = readData(VOREO_CMD_QUERY_TEXT);
    if (pBuf == NULL) {
        return RETURN_ERROR;
    }
    uint8_t result = (pBuf[4] != 0) ? RETURN_OK : RETURN_ERROR;
    free(pBuf);
    return result;
}

String DFRobot_Voreo::requestText(void)
{
    uint8_t *pBuf = readData(VOREO_CMD_GET_TEXT);
    if(pBuf == NULL)
    {
        DBG("readData failed");
        return "";
    }
    
    // 从 pBuf 中提取长度和数据
    // pBuf[0] = 协议头, pBuf[1-2] = 长度, pBuf[3] = 命令
    // pBuf[4] 开始是实际文本数据
    uint16_t length = (pBuf[1] << 8) | pBuf[2];
    
    if(length == 0)
    {
        free(pBuf);
        return "";
    }
    
    // 从 pBuf[4] 开始提取文本数据，长度为 length
    // 创建临时缓冲区并添加 null 终止符
    char *tempBuf = (char *)malloc(length + 1);
    if(tempBuf == NULL)
    {
        free(pBuf);
        return "";
    }
    memcpy(tempBuf, pBuf + 4, length);
    tempBuf[length] = '\0';  // 添加 null 终止符
    
    String text = String(tempBuf);
    free(tempBuf);
    free(pBuf);
    return text;
}

/** UTF-8 lead-byte → character length (ASCII=1, Latin ext=2, CJK≈3, emoji=4) */
static uint8_t utf8CharByteLen(uint8_t lead)
{
    if ((lead & 0x80) == 0x00) {
        return 1;
    }
    if ((lead & 0xE0) == 0xC0) {
        return 2;
    }
    if ((lead & 0xF0) == 0xE0) {
        return 3;
    }
    if ((lead & 0xF8) == 0xF0) {
        return 4;
    }
    return 1;
}

/** Largest prefix of [s, s+total) with length <= maxLen that ends on a UTF-8 char boundary */
static uint16_t utf8SafeChunkLen(const uint8_t *s, uint16_t total, uint16_t maxLen)
{
    if (total <= maxLen) {
        return total;
    }
    uint16_t i = 0;
    while (i < maxLen) {
        uint8_t cl = utf8CharByteLen(s[i]);
        if ((uint16_t)(i + cl) > maxLen) {
            break;
        }
        if ((uint16_t)(i + cl) > total) {
            break;
        }
        i = (uint16_t)(i + cl);
    }
    return (i == 0) ? 1 : i;
}

uint8_t DFRobot_Voreo::sendText(uint8_t *pText)
{
    if (pText == NULL) {
        return RETURN_ERROR;
    }
    uint16_t total = (uint16_t)strlen((const char *)pText);
    uint16_t offset = 0;
    while (offset < total) {
        uint16_t remain = (uint16_t)(total - offset);
        uint16_t chunk = utf8SafeChunkLen(pText + offset, remain, VOREO_TTS_PAYLOAD_MAX);
        if (chunk == 0) {
            return RETURN_ERROR;
        }
        if (!sendCommand(VOREO_CMD_SEND_TEXT, pText + offset, chunk)) {
            return RETURN_ERROR;
        }
        offset = (uint16_t)(offset + chunk);
        if (offset < total) {
            delay(VOREO_TTS_SEGMENT_DELAY_MS);
        }
    }
    return RETURN_OK;
}

uint8_t DFRobot_Voreo::sendText(String text)
{
    uint8_t *pText = (uint8_t *)text.c_str();
    return sendText(pText);
}

uint16_t DFRobot_Voreo::getAngle(void)
{
    uint8_t *pBuf = readData(VOREO_CMD_ANGLE);
    if (pBuf == NULL) {
        return RETURN_ERROR;
    }
    uint16_t angle = (uint16_t)pBuf[4] << 8 | pBuf[5];
    free(pBuf);
    return angle;
}

uint8_t DFRobot_Voreo::setWakeUp(uint8_t* pData)
{
    if (pData == NULL) {
        return RETURN_ERROR;
    }
    uint16_t length = (uint16_t)strlen((const char *)pData);
    if (length > VOREO_I2C_FRAME_MAX) {
        length = VOREO_I2C_FRAME_MAX;
    }
    return sendCommand(VOREO_CMD_WAKE_UP, pData, length);
}

uint8_t DFRobot_Voreo::setWakeUp(String data)
{
    uint8_t *pData = (uint8_t *)data.c_str();
    return setWakeUp(pData);
}

uint8_t DFRobot_Voreo::setSpeed(uint8_t speed)
{
    if (speed > VOREO_TTS_LEVEL_MAX) {
        speed = VOREO_TTS_LEVEL_MAX;
    }
    return sendCommand(VOREO_CMD_SET_SPEED, &speed, 1);
}

uint8_t DFRobot_Voreo::setVolume(uint8_t volume)
{
    if (volume > VOREO_TTS_LEVEL_MAX) {
        volume = VOREO_TTS_LEVEL_MAX;
    }
    return sendCommand(VOREO_CMD_SET_VOLUME, &volume, 1);
}

uint8_t DFRobot_Voreo::getASRState(void)
{
    uint8_t *pBuf = readData(VOREO_CMD_GET_ASR_STATE);
    if (pBuf == NULL) {
        return RETURN_ERROR;
    }
    uint8_t state = pBuf[4];
    free(pBuf);
    return state;
}


DFRobot_Voreo_I2C::DFRobot_Voreo_I2C(TwoWire *pWire)
{
    _pWire = pWire;
}

DFRobot_Voreo_I2C::~DFRobot_Voreo_I2C()
{
    _pWire = NULL;
}

uint8_t DFRobot_Voreo_I2C::begin(void)
{
    _pWire->begin();
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    /* Default Wire buffer is 128; TTS app frame is 128+6 — enlarge to one-shot write */
    _pWire->setBufferSize(VOREO_I2C_WIRE_BUF_SIZE);
#endif
#if defined(WIRE_HAS_TIMEOUT) || defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_ESP32)
    _pWire->setTimeOut(2000);
#endif
    return DFRobot_Voreo::begin();
}

uint8_t DFRobot_Voreo_I2C::writeRaw(const uint8_t *data, uint16_t len)
{
    /* 每笔 Write: [REG_FIFO][chunk…]，从机追加到应用帧缓冲 */
    const uint16_t maxPayload = (VOREO_I2C_CHUNK_SIZE > 1) ? (uint16_t)(VOREO_I2C_CHUNK_SIZE - 1) : 1;
    uint16_t offset = 0;
    while (offset < len) {
        uint16_t remain = len - offset;
        uint16_t chunk = (remain > maxPayload) ? maxPayload : remain;
        _pWire->beginTransmission(VOREO_I2C_ADDR);
        size_t written = _pWire->write((uint8_t)VOREO_REG_FIFO);
        written += _pWire->write(data + offset, chunk);
        uint8_t err = _pWire->endTransmission(true);
        if (err != 0 || written != (size_t)(chunk + 1)) {
            DBG("I2C FIFO write failed");
            return 0;
        }
        offset = (uint16_t)(offset + chunk);
        if (offset < len) {
            delay(VOREO_I2C_CHUNK_DELAY_MS);
        }
    }
    return 1;
}

void DFRobot_Voreo_I2C::sendAbort(void)
{
    _pWire->beginTransmission(VOREO_I2C_ADDR);
    _pWire->write((uint8_t)VOREO_REG_CTRL);
    _pWire->write((uint8_t)VOREO_REG_CTRL_ABORT);
    _pWire->endTransmission();
    delay(VOREO_I2C_STATUS_POLL_MS);
}

uint8_t DFRobot_Voreo_I2C::pollStatus(uint16_t *outLen)
{
    *outLen = 0;
    _pWire->beginTransmission(VOREO_I2C_ADDR);
    _pWire->write((uint8_t)VOREO_REG_STATUS);
    if (_pWire->endTransmission() != 0) {
        return VOREO_I2C_STATUS_ERROR;
    }
    delay(30); /* 等从机任务处理选寄存器后再 Read */
    if (_pWire->requestFrom((uint8_t)VOREO_I2C_ADDR, (uint8_t)3) != 3) {
        DBG("STATUS read failed");
        return VOREO_I2C_STATUS_ERROR;
    }
    uint8_t status = _pWire->read();
    uint16_t len = ((uint16_t)_pWire->read() << 8) | _pWire->read();
    *outLen = len;
    return status;
}

uint8_t *DFRobot_Voreo_I2C::readFrameByLen(uint16_t frameLen, uint8_t expectCmd)
{
    if (frameLen < 6 || frameLen > (VOREO_I2C_FRAME_MAX + 6)) {
        return NULL;
    }

    /* 选中 DATA 寄存器后再流式 Read */
    _pWire->beginTransmission(VOREO_I2C_ADDR);
    _pWire->write((uint8_t)VOREO_REG_DATA);
    if (_pWire->endTransmission() != 0) {
        return NULL;
    }
    delay(30);

    uint8_t *pBuf = NULL;
    uint8_t requestState = DATA_STATIC_HEAD;
    uint16_t length = 0;
    uint16_t dataLength = 0;
    uint16_t totalBytesRead = 0;
    uint8_t result = 0;

    while (totalBytesRead < frameLen) {
        uint16_t remain = frameLen - totalBytesRead;
        uint8_t chunk = (remain > VOREO_I2C_CHUNK_SIZE) ? VOREO_I2C_CHUNK_SIZE : (uint8_t)remain;
        if (_pWire->requestFrom((uint8_t)VOREO_I2C_ADDR, chunk) == 0) {
            DBG("DATA requestFrom failed");
            if (pBuf) {
                free(pBuf);
            }
            return NULL;
        }
        for (uint8_t i = 0; i < chunk && totalBytesRead < frameLen; i++) {
            if (!_pWire->available()) {
                if (pBuf) {
                    free(pBuf);
                }
                return NULL;
            }
            uint8_t data = _pWire->read();
            result = processDataState(data, expectCmd, &requestState, &length, &dataLength, &pBuf);
            totalBytesRead++;
            if (result == 1) {
                if (pBuf) {
                    free(pBuf);
                }
                return NULL;
            }
        }
        if (totalBytesRead < frameLen) {
            delay(VOREO_I2C_CHUNK_DELAY_MS);
        }
    }

    if (result != 2) {
        if (pBuf) {
            free(pBuf);
        }
        return NULL;
    }
    return validateAndReturnData(pBuf, length);
}

uint8_t DFRobot_Voreo_I2C::sendCommand(uint8_t cmd, uint8_t *pData, uint16_t dataLength)
{
    if (dataLength > VOREO_I2C_FRAME_MAX) {
        dataLength = VOREO_I2C_FRAME_MAX;
    }
    uint16_t totalLength = dataLength + 6;
    uint8_t *buf = (uint8_t *)malloc(totalLength);
    if (buf == NULL) {
        return 0;
    }
    buildCommandFrame(buf, cmd, pData, dataLength);
    uint8_t ok = writeRaw(buf, totalLength);
    free(buf);
    return ok;
}

uint8_t *DFRobot_Voreo_I2C::readData(uint8_t cmd)
{
    uint8_t requestBuf[6];
    buildRequestFrame(requestBuf, cmd);

    /* 最多整事务重发 1 次；轮询 STATUS 期间不重发业务帧 */
    for (uint8_t txn = 0; txn < 2; txn++) {
        if (!writeRaw(requestBuf, 6)) {
            sendAbort();
            continue;
        }
        delay(20); /* 等从机解析并置 READY */

        uint16_t frameLen = 0;
        uint8_t status = VOREO_I2C_STATUS_BUSY;
        uint32_t start = millis();
        while ((millis() - start) < VOREO_REQUEST_TIMEOUT) {
            status = pollStatus(&frameLen);
            if (status == VOREO_I2C_STATUS_READY && frameLen >= 6) {
                delay(VOREO_I2C_STATUS_POLL_MS);
                uint8_t *frame = readFrameByLen(frameLen, cmd);
                if (frame != NULL) {
                    return frame;
                }
                DBG("DATA read/CRC failed, abort and retry txn");
                sendAbort();
                break;
            }
            if (status == VOREO_I2C_STATUS_ERROR) {
                DBG("STATUS ERROR");
                sendAbort();
                break;
            }
            /* IDLE/BUSY：继续轮询，不重发请求 */
            delay(VOREO_I2C_STATUS_POLL_MS);
        }
    }

    DBG("I2C readData timeout");
    return NULL;
}

DFRobot_Voreo_UART::DFRobot_Voreo_UART(Stream *pSerial)
{
    _pSerial = pSerial;
}

DFRobot_Voreo_UART::~DFRobot_Voreo_UART()
{
    _pSerial = NULL;
}

uint8_t DFRobot_Voreo_UART::begin(void)
{
    return DFRobot_Voreo::begin();
}

uint8_t DFRobot_Voreo_UART::sendCommand(uint8_t cmd, uint8_t *pData, uint16_t dataLength)
{
    if (dataLength > VOREO_I2C_FRAME_MAX) {
        dataLength = VOREO_I2C_FRAME_MAX;
    }
    uint16_t totalLength = dataLength + 6;
    uint8_t *buf = (uint8_t *)malloc(totalLength);
    if (buf == NULL) {
        return 0;
    }
    buildCommandFrame(buf, cmd, pData, dataLength);
    _pSerial->write(buf, totalLength);
    free(buf);
    return 1;
}



uint8_t *DFRobot_Voreo_UART::readData(uint8_t cmd)
{
    uint8_t requestBuf[6];
    buildRequestFrame(requestBuf, cmd);
    _pSerial->write(requestBuf, 6);

    uint8_t *pBuf = NULL;
    uint8_t requestStatic = DATA_STATIC_HEAD;
    uint16_t length = 0;
    uint16_t dataLength = 0;

    uint32_t start = millis();
    while (millis() - start < VOREO_REQUEST_TIMEOUT) {
        if (!_pSerial->available()) {
            continue;
        }

        uint8_t data = _pSerial->read();
        uint8_t result = processDataState(data, cmd, &requestStatic, &length, &dataLength, &pBuf);
        
        if (result == 1) { // 错误
            return NULL;
        } else if (result == 2) { // 完成
            return validateAndReturnData(pBuf, length);
        }
    }

    if (pBuf) {
        free(pBuf);
    }
    DBG("request timeout");
    return NULL;
}

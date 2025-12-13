/**
 * @file DFRobot_Voero.cpp
 * @brief  DFRobot Voreo library
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @version 1.0
 * @date 2025-12-12
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */

#include "DFRobot_Voero.h"

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
    buf[0] = VOERO_CMD_HEAD;
    buf[1] = 0;
    buf[2] = 1;
    buf[3] = cmd;
    uint16_t crc = calculateCRC16(buf, 4);
    buf[4] = crc >> 8;
    buf[5] = crc & 0xFF;
}

// 构建命令帧
static void buildCommandFrame(uint8_t *buf, uint8_t cmd, uint8_t *pData, uint8_t dataLength)
{
    buf[0] = VOERO_CMD_HEAD;
    buf[1] = (dataLength >> 8) & 0xff;
    buf[2] = dataLength & 0xff;
    buf[3] = cmd;
    if (dataLength > 0) {
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
        if (data == VOERO_CMD_HEAD) {
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
        (*pBuf)[0] = VOERO_CMD_HEAD;
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

uint8_t DFRobot_Voero::begin(void)
{
    uint8_t *pBuf = readData(VOERO_CMD_BEGIN);
    if (pBuf == NULL) {
        return RETURN_ERROR;
    }
    uint8_t result = (pBuf[4] != 0) ? RETURN_OK : RETURN_ERROR;
    free(pBuf);
    return result;
}

uint8_t DFRobot_Voero::queryText(void)
{
    uint8_t *pBuf = readData(VOERO_CMD_QUERY_TEXT);
    if (pBuf == NULL) {
        return RETURN_ERROR;
    }
    uint8_t result = (pBuf[4] != 0) ? RETURN_OK : RETURN_ERROR;
    free(pBuf);
    return result;
}

String DFRobot_Voero::requestText(void)
{
    uint8_t *pBuf = readData(VOERO_CMD_GET_TEXT);
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

uint8_t DFRobot_Voero::sendText(uint8_t *pText)
{
    uint16_t length = strlen(pText);
    sendCommand(VOERO_CMD_SEND_TEXT, pText, length);
    return 1;
}

uint8_t DFRobot_Voero::sendText(String text)
{
    uint8_t *pText = (uint8_t *)text.c_str();
    return sendText(pText);
}

uint16_t DFRobot_Voero::getAngle(void)
{
    uint8_t *pBuf = readData(VOERO_CMD_ANGLE);
    if (pBuf == NULL) {
        return RETURN_ERROR;
    }
    uint16_t angle = (uint16_t)pBuf[4] << 8 | pBuf[5];
    free(pBuf);
    return angle;
}

uint8_t DFRobot_Voero::setSpeed(uint8_t speed)
{
    
    return 1;
}


DFRobot_Voero_I2C::DFRobot_Voero_I2C(TwoWire *pWire)
{
    _pWire = pWire;
}

DFRobot_Voero_I2C::~DFRobot_Voero_I2C()
{
    _pWire = NULL;
}

uint8_t DFRobot_Voero_I2C::begin(void)
{
    _pWire->begin();
    return DFRobot_Voero::begin();
}

uint8_t DFRobot_Voero_I2C::sendCommand(uint8_t cmd, uint8_t *pData, uint8_t dataLength)
{
    uint8_t buf[dataLength + 6];
    buildCommandFrame(buf, cmd, pData, dataLength);
    _pWire->beginTransmission(VOERO_I2C_ADDR);
    _pWire->write(buf, dataLength + 6);
    _pWire->endTransmission();
    return 1;
}

uint8_t *DFRobot_Voero_I2C::readData(uint8_t cmd)
{
    uint8_t requestBuf[6];
    buildRequestFrame(requestBuf, cmd);
    
    _pWire->beginTransmission(VOERO_I2C_ADDR);
    _pWire->write(requestBuf, 6);
    _pWire->endTransmission();
    
    delay(10);
    
    uint8_t *pBuf = NULL;
    uint8_t requestStatic = DATA_STATIC_HEAD;
    uint16_t length = 0;
    uint16_t dataLength = 0;
    
    uint32_t start = millis();
    while (millis() - start < VOERO_REQUEST_TIMEOUT) {
        if (_pWire->requestFrom(VOERO_I2C_ADDR, 1) == 0) {
            continue;
        }
        
        uint8_t data = _pWire->read();
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

DFRobot_Voero_UART::DFRobot_Voero_UART(Stream *pSerial)
{
    _pSerial = pSerial;
}

DFRobot_Voero_UART::~DFRobot_Voero_UART()
{
    _pSerial = NULL;
}

uint8_t DFRobot_Voero_UART::begin(void)
{
    return DFRobot_Voero::begin();
}

uint8_t DFRobot_Voero_UART::sendCommand(uint8_t cmd, uint8_t *pData, uint8_t dataLength)
{
    uint8_t buf[dataLength + 6];
    buildCommandFrame(buf, cmd, pData, dataLength);
    _pSerial->write(buf, dataLength + 6);
    return 1;
}

uint8_t *DFRobot_Voero_UART::readData(uint8_t cmd)
{
    uint8_t requestBuf[6];
    buildRequestFrame(requestBuf, cmd);
    _pSerial->write(requestBuf, 6);

    uint8_t *pBuf = NULL;
    uint8_t requestStatic = DATA_STATIC_HEAD;
    uint16_t length = 0;
    uint16_t dataLength = 0;

    uint32_t start = millis();
    while (millis() - start < VOERO_REQUEST_TIMEOUT) {
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

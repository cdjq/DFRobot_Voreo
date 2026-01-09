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
static void buildCommandFrame(uint8_t *buf, uint8_t cmd, uint8_t *pData, uint8_t dataLength)
{
    buf[0] = VOREO_CMD_HEAD;
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

uint8_t DFRobot_Voreo::sendText(uint8_t *pText)
{
    uint16_t length = strlen(pText);
    sendCommand(VOREO_CMD_SEND_TEXT, pText, length);
    return 1;
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
    sendCommand(VOREO_CMD_WAKE_UP, pData, strlen(pData));
    return 1;
}

uint8_t DFRobot_Voreo::setWakeUp(String data)
{
    uint8_t *pData = (uint8_t *)data.c_str();
    return setWakeUp(pData);
}

uint8_t DFRobot_Voreo::setSpeed(uint8_t speed)
{
    
    return 1;
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
    return DFRobot_Voreo::begin();
}

uint8_t DFRobot_Voreo_I2C::sendCommand(uint8_t cmd, uint8_t *pData, uint8_t dataLength)
{
    // 常量定义
    const uint16_t MAX_I2C_PACKET_SIZE = 28;      // I2C最大数据包长度
    const uint16_t PACKET_DELAY_MS = 10;          // 分包发送间隔(ms)
    
    // 构建命令帧
    uint16_t totalLength = dataLength + 6;
    uint8_t buf[totalLength];
    buildCommandFrame(buf, cmd, pData, dataLength);
    
    // 如果数据长度超过I2C包大小限制，进行分包发送
    if (totalLength > MAX_I2C_PACKET_SIZE) {
        DBG("data length exceeds I2C packet size, splitting into multiple packets");
        uint16_t offset = 0;
        
        while (offset < totalLength) {
            // 计算本次发送的字节数（最多32字节）
            uint8_t chunkSize = (totalLength - offset > MAX_I2C_PACKET_SIZE) 
                                ? MAX_I2C_PACKET_SIZE 
                                : (totalLength - offset);
            
            // 发送数据块
            _pWire->beginTransmission(VOREO_I2C_ADDR);
            uint8_t bytesWritten = _pWire->write(buf + offset, chunkSize);
            uint8_t error = _pWire->endTransmission();
            
            // 检查发送是否成功
            if (error != 0 || bytesWritten != chunkSize) {
                DBG("I2C transmission error during packet splitting");
                return 0;  // 发送失败
            }
            
            offset += chunkSize;
            
            // 如果不是最后一包，添加延迟
            if (offset < totalLength) {
                delay(PACKET_DELAY_MS);
            }
        }
        
        return 1;  // 分包发送成功
    } else {
        // 数据长度在限制内，直接发送
        _pWire->beginTransmission(VOREO_I2C_ADDR);
        uint8_t bytesWritten = _pWire->write(buf, totalLength);
        uint8_t error = _pWire->endTransmission();
        
        // 检查发送是否成功
        if (error != 0 || bytesWritten != totalLength) {
            DBG("I2C transmission error");
            return 0;  // 发送失败
        }
        
        return 1;  // 发送成功
    }
}

uint8_t *DFRobot_Voreo_I2C::readData(uint8_t cmd)
{
    // 常量定义
    const uint8_t CMD_QUERY_LENGTH = 0x21;        // 查询数据长度命令
    const uint8_t MAX_RETRY_COUNT = 10;           // 最大重试次数
    const uint16_t MAX_I2C_PACKET_SIZE = 32;      // I2C最大数据包长度
    const uint16_t INVALID_LENGTH = 0xFFFF;       // 无效长度标识
    const uint16_t RESPONSE_DELAY_MS = 100;       // 响应延迟(ms)
    
    // 构建并发送初始请求帧
    uint8_t requestBuf[6];
    buildRequestFrame(requestBuf, cmd);
    _pWire->beginTransmission(VOREO_I2C_ADDR);
    _pWire->write(requestBuf, 6);
    _pWire->endTransmission();
    delay(RESPONSE_DELAY_MS);
    
    // 初始化状态变量
    uint8_t *pBuf = NULL;
    uint8_t requestState = DATA_STATIC_HEAD;
    uint16_t length = 0;
    uint16_t dataLength = 0;
    uint8_t retryCount = 0;
    
    // 主循环：尝试读取数据
    while (retryCount < MAX_RETRY_COUNT) {
        // 查询数据长度
        _pWire->beginTransmission(VOREO_I2C_ADDR);
        _pWire->write(CMD_QUERY_LENGTH);
        _pWire->endTransmission();
        
        // 读取长度字节
        if (_pWire->requestFrom(VOREO_I2C_ADDR, 2) == 0) {
            DBG("request length failed 1");
            retryCount++;
            delay(RESPONSE_DELAY_MS);
            continue;
        }
        
        // 解析长度（大端格式）
        uint16_t len = (_pWire->read() << 8) | _pWire->read();
        // DBG("len:");
        // Serial.println(len, HEX);
        
        // 检查长度有效性
        if (len == INVALID_LENGTH) {
            DBG("request length failed 2 - invalid length");
            // 重新发送请求帧
            _pWire->beginTransmission(VOREO_I2C_ADDR);
            _pWire->write(requestBuf, 6);
            _pWire->endTransmission();
            retryCount++;
            delay(RESPONSE_DELAY_MS);
            continue;
        }
        
        // 分包读取数据（支持超过MAX_I2C_PACKET_SIZE的长包）
        uint16_t totalBytesRead = 0;
        uint8_t result = 0;
        bool needRetry = false;
        
        // 循环读取直到读取完len字节
        while (totalBytesRead < len && !needRetry) {
            // 计算本次要读取的字节数（最多MAX_I2C_PACKET_SIZE字节）
            uint16_t bytesToRead = len - totalBytesRead;
            if (bytesToRead > MAX_I2C_PACKET_SIZE) {
                bytesToRead = MAX_I2C_PACKET_SIZE;
            }
            
            // 请求数据块
            if (_pWire->requestFrom(VOREO_I2C_ADDR, bytesToRead) == 0) {
                DBG("request data failed");
                needRetry = true;
                break;
            }
            
            // 读取并处理数据
            for (uint16_t i = 0; i < bytesToRead && totalBytesRead < len; i++) {
                if (!_pWire->available()) {
                    DBG("I2C data not available");
                    needRetry = true;
                    break;
                }
                
                uint8_t data = _pWire->read();
                result = processDataState(data, cmd, &requestState, &length, &dataLength, &pBuf);
                totalBytesRead++;
                
                // 检查处理结果
                if (result == 1) {  // 错误
                    if (pBuf) {
                        free(pBuf);
                        pBuf = NULL;
                    }
                    DBG("data processing error");
                    // 重置状态并标记需要重试
                    requestState = DATA_STATIC_HEAD;
                    length = 0;
                    dataLength = 0;
                    needRetry = true;
                    break;
                } else if (result == 2) {  // 状态机完成
                    // 状态机完成时，应该已经读取完所有数据
                    // 如果还没读完，说明协议不匹配，需要重试
                    if (totalBytesRead < len) {
                        DBG("state machine complete but data not fully read");
                        if (pBuf) {
                            free(pBuf);
                            pBuf = NULL;
                        }
                        requestState = DATA_STATIC_HEAD;
                        length = 0;
                        dataLength = 0;
                        needRetry = true;
                    }
                    // 如果已读完，跳出循环，稍后进行验证
                    break;
                }
            }
            
            // 如果还有剩余数据需要读取且未出错，添加短暂延迟
            if (totalBytesRead < len && !needRetry && result != 2) {
                delay(10);  // 短暂延迟，等待设备准备下一包数据
            }
        }
        
        // 检查是否成功读取了所有数据
        if (!needRetry && totalBytesRead >= len) {
            // 如果状态机已完成，进行CRC验证
            if (result == 2) {
                uint8_t *validatedBuf = validateAndReturnData(pBuf, length);
                if (validatedBuf != NULL) {
                    return validatedBuf;  // 验证成功，返回数据
                }
                // CRC验证失败，清理并标记需要重试
                if (pBuf) {
                    free(pBuf);
                    pBuf = NULL;
                }
                requestState = DATA_STATIC_HEAD;
                length = 0;
                dataLength = 0;
                DBG("CRC validation failed, retrying");
                needRetry = true;
            } else {
                // 数据读取完成但状态机未完成，需要重试
                DBG("data read complete but state machine not finished");
                needRetry = true;
            }
        } else if (!needRetry && totalBytesRead < len) {
            DBG("incomplete data read");
            needRetry = true;
        }
        
        // 如果需要重试或数据未完成，重新发送请求帧
        if (needRetry || result == 0) {
            if (result == 0) {
                DBG("request length failed 4 - incomplete data");
            }
            // 重新发送请求帧
            _pWire->beginTransmission(VOREO_I2C_ADDR);
            _pWire->write(requestBuf, 6);
            _pWire->endTransmission();
            retryCount++;
            delay(RESPONSE_DELAY_MS);
        }
    }
    
    // 超时清理
    if (pBuf) {
        free(pBuf);
        pBuf = NULL;
    }
    DBG("request timeout");
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

uint8_t DFRobot_Voreo_UART::sendCommand(uint8_t cmd, uint8_t *pData, uint8_t dataLength)
{
    uint8_t buf[dataLength + 6];
    buildCommandFrame(buf, cmd, pData, dataLength);
    _pSerial->write(buf, dataLength + 6);
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

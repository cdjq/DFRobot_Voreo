/*!
 * @file DFRobot_Voreo.h
 * @brief DFRobot Voreo library
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @version 1.0
 * @date 2025-12-12
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */
#ifndef __DFROBOT_VOREO_H__
#define __DFROBOT_VOREO_H__
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Wire.h>


#define VOREO_I2C_ADDR              (0x1f) ///< I2C address

#define VOREO_CMD_HEAD              (0x55) ///< Command head
#define VOREO_CMD_END               (0xFF) ///< Command end
#define VOREO_CMD_BEGIN             (0x00) ///< Command begin
#define VOREO_CMD_QUERY_TEXT        (0x01) ///< Query text command
#define VOREO_CMD_GET_TEXT          (0x02) ///< Get text command
#define VOREO_CMD_SEND_TEXT         (0x03) ///< Send text command
#define VOREO_CMD_ANGLE             (0x04) ///< Angle command
#define VOREO_CMD_DISTANCE          (0x05) ///< Distance command
#define VOREO_CMD_WAKE_UP           (0x06) ///< Wake up command
#define VOREO_CMD_SET_SPEED         (0x07) ///< Set TTS speed command (0~15)
#define VOREO_CMD_GET_ASR_STATE     (0x08) ///< Get ASR state command
#define VOREO_CMD_SET_VOLUME        (0x09) ///< Set TTS volume command (0~15)

#define VOREO_TTS_LEVEL_MIN         (0)    ///< TTS volume/speed minimum (decimal)
#define VOREO_TTS_LEVEL_MAX         (15)   ///< TTS volume/speed maximum (decimal)

/**
 * I2C register mailbox (Write [reg][data…]; Write [reg] then Read)
 *  0x00 STATUS R — read 3: [status][len_h][len_l]
 *  0x03 DATA   R — response stream
 *  0x04 CTRL   W — 0x2F ABORT
 *  0x10 FIFO   W — app frame 0x55|LEN|CMD|DATA|CRC
 */
#define VOREO_REG_STATUS            (0x00)
#define VOREO_REG_LEN_H             (0x01)
#define VOREO_REG_LEN_L             (0x02)
#define VOREO_REG_DATA              (0x03)
#define VOREO_REG_CTRL              (0x04)
#define VOREO_REG_FIFO              (0x10)
#define VOREO_REG_CTRL_ABORT        (0x2F)

#define VOREO_I2C_STATUS_IDLE       (0x00)
#define VOREO_I2C_STATUS_BUSY       (0x01)
#define VOREO_I2C_STATUS_READY      (0x02)
#define VOREO_I2C_STATUS_ERROR      (0x03)
#define VOREO_I2C_WIRE_BUF_SIZE     (256)  ///< ESP32 Wire buffer (TTS frame 128+6)
#define VOREO_I2C_CHUNK_SIZE_AVR    (16)   ///< Uno Wire~32；偏小降低 stretch 窗口（含 1B REG）
#define VOREO_I2C_CHUNK_SIZE_ESP    (200)  ///< One-shot TTS frame after setBufferSize(256)
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32) || defined(ESP8266)
#define VOREO_I2C_CHUNK_SIZE        VOREO_I2C_CHUNK_SIZE_ESP
#define VOREO_I2C_CHUNK_DELAY_MS    (40)
#else
#define VOREO_I2C_CHUNK_SIZE        VOREO_I2C_CHUNK_SIZE_AVR
#define VOREO_I2C_CHUNK_DELAY_MS    (200)  /* UNO 分包间隔：等从机 RX 放 SCL */
#endif
#define VOREO_TTS_PAYLOAD_MAX       (128)  ///< TTS single packet max (UTF-8 bytes)
#define VOREO_ASR_PAYLOAD_MAX       (4096) ///< ASR single frame max (UTF-8 bytes)
#define VOREO_I2C_FRAME_MAX         (VOREO_ASR_PAYLOAD_MAX) ///< Max app payload (ASR read)
/* UNO SRAM 小：单帧应答上限（含头尾）；ESP 主机可用满量 ASR */
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32) || defined(ESP8266)
#define VOREO_I2C_HOST_FRAME_MAX    (VOREO_I2C_FRAME_MAX + 6)
#else
#define VOREO_I2C_HOST_FRAME_MAX    (512)
#endif
#define VOREO_I2C_STATUS_POLL_MS    (20)   ///< Delay between STATUS polls
#define VOREO_TTS_SEGMENT_DELAY_MS  (80)   ///< Delay between TTS segments

#define VOREO_REQUEST_TIMEOUT       (10000) ///< Request timeout

#define RETURN_OK                   (0x01) ///< Return OK
#define RETURN_ERROR                (0x00) ///< Return ERROR

#define DATA_STATIC_HEAD            (1) ///< Data static head
#define DATA_STATIC_LEN_H           (2) ///< Data static length high
#define DATA_STATIC_LEN_L           (3) ///< Data static length low
#define DATA_STATIC_CMD             (4) ///< Data static command
#define DATA_STATIC_DATA            (5) ///< Data static data
#define DATA_STATIC_CRC_H           (6) ///< Data static CRC high
#define DATA_STATIC_CRC_L           (7) ///< Data static CRC low

#define CRC16_POLYNOMIAL 0x1021
#define CRC16_INITIAL_VALUE 0xFFFF


//#define ENABLE_DBG ///< Enable this macro to see the detailed running process of the program
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

class DFRobot_Voreo
{
public:

    /**
     * #fn begin
     * @brief Initialize the Voreo
     * @return The result of the initialize
     * @retval 1 Success
     * @retval 0 Failed
     */
    uint8_t begin(void);

    /**
     * @fn queryText
     * @brief Query the text from the Voreo
     * @return The result of the query
     * @retval 1 Success
     * @retval 0 Failed
     */
    uint8_t queryText(void); 

    /**
     * @fn requestText
     * @brief Request the text from the Voreo
     * @return The result of the request
     * @retval String The text from the Voreo
     */
    String requestText(void);

   /**
    * @fn sendText
    * @brief Send TTS text (UTF-8). Longer text is split into <=128B segments on
    *        UTF-8 character boundaries (1/2/3/4-byte; Chinese typically 3B).
    * @param pText The text to send
    * @return The result of the send
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t sendText(uint8_t *pText);
   uint8_t sendText(String text);

   /**
    * @fn getAngle
    * @brief Get the angle of the sound source
    * @return The result of the get angle
    * @retval The angle of the sound source
    */
   uint16_t getAngle(void);

   /**
    * @fn setSpeed
    * @brief Set the TTS speech speed of the Voreo
    * @param speed Speech speed, range 0~15 (decimal; values above max are clamped)
    * @return The result of the set speed
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setSpeed(uint8_t speed);

   /**
    * @fn setVolume
    * @brief Set the TTS volume of the Voreo
    * @param volume Volume level, range 0~15 (decimal; values above max are clamped)
    * @return The result of the set volume
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setVolume(uint8_t volume);

   /**
    * @fn setWakeUp
    * @brief Set the wake up of the Voreo
    * @param pData The data to set
    * @return The result of the set wake up
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setWakeUp(uint8_t* pData);
   uint8_t setWakeUp(String data);
   
   /**
    * @fn getASRState
    * @brief Get the ASR state of the Voreo
    * @return The result of the get ASR state
    * @retval 0 ASR is not working
    * @retval 1 ASR is working
    * @retval 2 ASR is working end
    */
   uint8_t getASRState(void);

   /**
    * @fn sendCommand
    * @brief Send the command to the Voreo
    * @param cmd The command to send
    * @param pData The data to send
    * @param dataLength The length of the data (max VOREO_I2C_FRAME_MAX for I2C)
    * @return The result of the send command
    * @retval 1 Success
    * @retval 0 Failed
    */
   virtual uint8_t sendCommand(uint8_t cmd, uint8_t *pData, uint16_t dataLength);

    virtual uint8_t *readData(uint8_t cmd);

    
};

class DFRobot_Voreo_I2C : public DFRobot_Voreo
{
public:
    DFRobot_Voreo_I2C(TwoWire *pWire = &Wire);
    ~DFRobot_Voreo_I2C();

    uint8_t begin(void);
    virtual uint8_t sendCommand(uint8_t cmd, uint8_t *pData, uint16_t dataLength);

    virtual uint8_t *readData(uint8_t cmd);
private:
    TwoWire *_pWire = NULL;
    uint8_t writeRaw(const uint8_t *data, uint16_t len);
    uint8_t pollStatus(uint16_t *outLen);
    uint8_t *readFrameByLen(uint16_t frameLen, uint8_t expectCmd);
    void sendAbort(void);
};

class DFRobot_Voreo_UART : public DFRobot_Voreo
{

public:
    DFRobot_Voreo_UART(Stream *pSerial);
    ~DFRobot_Voreo_UART();

    uint8_t begin(void);
    virtual uint8_t sendCommand(uint8_t cmd, uint8_t *pData, uint16_t dataLength);

    virtual uint8_t *readData(uint8_t cmd);
private:
    Stream *_pSerial = NULL;
};

#endif

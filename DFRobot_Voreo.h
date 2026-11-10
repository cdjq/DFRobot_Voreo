/*!
 * @file DFRobot_Voreo.h
 * @brief DFRobot Voreo library
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @author    [RongLi](rong.li@dfrobot.com)
 * @version 1.1
 * @date 2026-11-10
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
#define VOREO_CMD_SET_SPEED         (0x07) ///< TTS speed: query LEN=0; set LEN=1 (1~5)
#define VOREO_CMD_GET_ASR_STATE     (0x08) ///< Query RGB/device state (rgb_sm_state_t, non-consuming)
#define VOREO_CMD_SET_VOLUME        (0x09) ///< TTS volume: query LEN=0; set LEN=1 (0~15)
#define VOREO_CMD_DOA_MODE          (0x0A) ///< DOA mode: query LEN=0; set LEN=1 (0 off / 1 on / 2 LISTENING)
#define VOREO_CMD_DOA_INTENSITY     (0x0B) ///< DOA intensity query (app frame); reply 1 byte 0~255

#define VOREO_TTS_SPEED_MIN         (1)    ///< TTS speed minimum (decimal)
#define VOREO_TTS_SPEED_MAX         (5)    ///< TTS speed maximum (decimal)
#define VOREO_TTS_VOLUME_MIN        (0)    ///< TTS volume minimum (decimal)
#define VOREO_TTS_VOLUME_MAX        (15)   ///< TTS volume maximum (decimal)

#define VOREO_DOA_MODE_OFF                 (0) ///< DOA off
#define VOREO_DOA_MODE_ON                  (1) ///< DOA on
#define VOREO_DOA_MODE_LISTENING           (2) ///< DOA in LISTENING

/**
 * I2C register mailbox (Write [reg][data…]; Write [reg] then Read)
 *  0x00 STATUS R — read 3: [status][len_h][len_l]
 *  0x03 DATA   R — response stream
 *  0x04 CTRL   W — 0x2F ABORT
 *  0x06 DOA intensity R — write select then read 1 byte (<VAD→0, ≥VAD log→1~255)
 *  0x10 FIFO   W — app frame 0x55|LEN|CMD|DATA|CRC
 */
#define VOREO_REG_STATUS            (0x00)
#define VOREO_REG_LEN_H             (0x01)
#define VOREO_REG_LEN_L             (0x02)
#define VOREO_REG_DATA              (0x03)
#define VOREO_REG_CTRL              (0x04)
#define VOREO_REG_DOA_INTENSITY     (0x06) ///< DOA intensity (I2C direct); write select then read 1B
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
    *        UTF-8 boundaries, preferring punctuation kept with the preceding segment.
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
    * @fn setDoaMode
    * @brief Set DOA run mode (CMD 0x0A, LEN=1)
    * @param mode 0 off / 1 on / 2 LISTENING
    * @return The result of the set
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setDoaMode(uint8_t mode);

   /**
    * @fn getDoaMode
    * @brief Query DOA run mode (CMD 0x0A, LEN=0)
    * @return Current DOA mode
    * @retval 0 off
    * @retval 1 on
    * @retval 2 LISTENING
    * @retval 0xFF Failed
    */
   uint8_t getDoaMode(void);

   /**
    * @fn getDoaIntensity
    * @brief Query quantized DOA intensity
    * @note UART / I2C app frame CMD 0x0B; I2C preferred path is REG 0x06
    * @return Intensity: <VAD -> 0; >=VAD -> log-quantized 1~255; comm fail -> 0
    */
   virtual uint8_t getDoaIntensity(void);

   /**
    * @fn setSpeed
    * @brief Set the TTS speech speed of the Voreo (CMD 0x07, LEN=1)
    * @param speed Speech speed, range 1~5 (decimal; out-of-range values are clamped)
    * @return The result of the set speed
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setSpeed(uint8_t speed);

   /**
    * @fn getSpeed
    * @brief Query the TTS speech speed (CMD 0x07, LEN=0)
    * @return Current speed 1~5, or 0xFF on failure
    */
   uint8_t getSpeed(void);

   /**
    * @fn setVolume
    * @brief Set the TTS volume of the Voreo (CMD 0x09, LEN=1)
    * @param volume Volume level, range 0~15 (decimal; values above max are clamped)
    * @return The result of the set volume
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setVolume(uint8_t volume);

   /**
    * @fn getVolume
    * @brief Query the TTS volume (CMD 0x09, LEN=0)
    * @return Current volume 0~15, or 0xFF on failure
    */
   uint8_t getVolume(void);

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
    * @brief Query RGB/device state (same source as AT+STATE?, non-consuming)
    * @return rgb_sm_state_t value
    * @retval 0 BOOTING
    * @retval 1 STANDBY
    * @retval 2 LISTENING
    * @retval 3 TTS
    * @retval 4 WAKEWORD_LEARNING
    * @retval 5 WAKEWORD_LEARN_OK
    * @retval 6 WAKEWORD_LEARN_FAIL
    * @retval 7 BOOT_FAIL
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
    /**
     * @brief I2C: write-select REG 0x06 then read 1 byte
     */
    virtual uint8_t getDoaIntensity(void);
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

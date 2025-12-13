/*!
 * @file DFRobot_Voero.h
 * @brief DFRobot Voreo library
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @version 1.0
 * @date 2025-12-12
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */
#ifndef __DFROBOT_VOERO_H__
#define __DFROBOT_VOERO_H__
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Wire.h>


#define VOERO_I2C_ADDR              (0x1f) ///< I2C address

#define VOERO_CMD_HEAD              (0x55) ///< Command head
#define VOERO_CMD_END               (0xFF) ///< Command end
#define VOERO_CMD_BEGIN             (0x00) ///< Command begin
#define VOERO_CMD_QUERY_TEXT        (0x01) ///< Query text command
#define VOERO_CMD_GET_TEXT          (0x02) ///< Get text command
#define VOERO_CMD_SEND_TEXT         (0x03) ///< Send text command
#define VOERO_CMD_ANGLE             (0x04) ///< Angle command
#define VOERO_CMD_DISTANCE          (0x05) ///< Distance command
#define VOERO_CMD_SET_SPEED         (0x06) ///< Set speed command

#define VOERO_REQUEST_TIMEOUT       (10000) ///< Request timeout

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


#define ENABLE_DBG ///< Enable this macro to see the detailed running process of the program
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

class DFRobot_Voero
{
public:

    /**
     * #fn begin
     * @brief Initialize the Voero
     * @return The result of the initialize
     * @retval 1 Success
     * @retval 0 Failed
     */
    uint8_t begin(void);

    /**
     * @fn queryText
     * @brief Query the text from the Voero
     * @return The result of the query
     * @retval 1 Success
     * @retval 0 Failed
     */
    uint8_t queryText(void); 

    /**
     * @fn requestText
     * @brief Request the text from the Voero
     * @return The result of the request
     * @retval String The text from the Voero
     */
    String requestText(void);

   /**
    * @fn sendText
    * @brief Send the text to the Voero
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
    * @brief Set the speed of the Voero
    * @param speed The speed to set
    * @return The result of the set speed
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setSpeed(uint8_t speed);

   /**
    * @fn sendCommand
    * @brief Send the command to the Voero
    * @param cmd The command to send
    * @param pData The data to send
    * @param dataLength The length of the data
    * @return The result of the send command
    * @retval 1 Success
    * @retval 0 Failed
    */
   virtual uint8_t sendCommand(uint8_t cmd, uint8_t *pData, uint8_t dataLength);

    virtual uint8_t *readData(uint8_t cmd);

    
};

class DFRobot_Voero_I2C : public DFRobot_Voero
{
public:
    DFRobot_Voero_I2C(TwoWire *pWire = &Wire);
    ~DFRobot_Voero_I2C();

    uint8_t begin(void);
    virtual uint8_t sendCommand(uint8_t cmd, uint8_t *pData, uint8_t dataLength);

    virtual uint8_t *readData(uint8_t cmd);
private:
    TwoWire *_pWire = NULL;
};

class DFRobot_Voero_UART : public DFRobot_Voero
{

public:
    DFRobot_Voero_UART(Stream *pSerial);
    ~DFRobot_Voero_UART();

    uint8_t begin(void);
    virtual uint8_t sendCommand(uint8_t cmd, uint8_t *pData, uint8_t dataLength);

    virtual uint8_t *readData(uint8_t cmd);
private:
    Stream *_pSerial = NULL;
};

#endif

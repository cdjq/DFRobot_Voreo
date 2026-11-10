/*!
 * @file getASRToText.ino
 * @brief ASR to text, TTS playback, and volume / speed demo
 * @note Define I2C_COMMUNICATION for I2C; comment it out for UART
 *       Speed 1~5, volume 0~15 (decimal)
 * ---------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |
 *  UART RX   |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |
 *  UART TX   |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |
 *   I2C SDA  |             SDA                |        SDA           |    A4     |   SDA   |  SDA  |     X      |
 *   I2C SCL  |             SCL                |        SCL           |    A5     |   SCL   |  SCL  |     X      |
 * ---------------------------------------------------------------------------------------------------------------
 *
 * @copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @author    [RongLi](rong.li@dfrobot.com)
 * @version 1.1
 * @date 2026-11-10
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */

#include <DFRobot_Voreo.h>

/* use I2C for communication, but use the serial port for communication if the line of codes were masked */
//#define I2C_COMMUNICATION  

#if defined(I2C_COMMUNICATION)
DFRobot_Voreo_I2C voreo;
#else
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial mySerial(/*rx=*/5, /*tx=*/4);
DFRobot_Voreo_UART voreo(&mySerial);
#else
DFRobot_Voreo_UART voreo(&Serial1);
#endif
#endif
/*Cannot use spaces in the text*/
char *text = "欢迎使用DFRobot Voreo语音识别模块,该模块可以实现语音转文字和文字转语音的功能";

void setup()
{
#if !defined(I2C_COMMUNICATION)
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    mySerial.begin(9600);
#else
    Serial1.begin(9600);
#endif
#endif
    Serial.begin(115200);
#if defined(I2C_COMMUNICATION)
    Serial.println("Communication mode: I2C");
#else
    Serial.println("Communication mode: UART");
#endif

    while (!voreo.begin()) {
        Serial.println("Voreo begin failed, please check the connection");
        delay(1000);
    }
    Serial.println("Voreo begin ok");

    voreo.setSpeed(5);  /* speed 1~5 */
    voreo.setVolume(5); /* volume 0~15 */
    delay(200);
    Serial.print("speed=");
    Serial.print(voreo.getSpeed());
    Serial.print(" volume=");
    Serial.println(voreo.getVolume());
    Serial.println(voreo.sendText(text) ? "sendText ok, still waiting for the playback to complete..." : "sendText fail");
    delay(8000);
}

void loop()
{
    if (voreo.queryText()) {
        Serial.println(voreo.requestText());
    }
    delay(1000);
    Serial.println("After being awakened, speak....");
}

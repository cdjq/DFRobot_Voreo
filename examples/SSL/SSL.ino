/*!
 * @file SSL.ino
 * @brief Sound Source Localization example
 * @note Define I2C_COMMUNICATION for I2C; comment it out for UART
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

// #define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

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
    while (!voreo.begin()) {
        Serial.println("Voreo begin failed");
        delay(1000);
    }
    Serial.println("Voreo begin ok");

    /* DOA mode: 0 off / 1 on / 2 LISTENING */
    Serial.println(voreo.setDoaMode(VOREO_DOA_MODE_ON) ? "setDoaMode ok" : "setDoaMode fail");
    Serial.print("DOA mode: ");
    Serial.println(voreo.getDoaMode());
}

void loop()
{
    Serial.print("Angle: ");
    Serial.print(voreo.getAngle());
    Serial.print(" intensity: ");
    Serial.println(voreo.getDoaIntensity()); /* <VAD->0, >=VAD log->1~255 */
    delay(200);
}

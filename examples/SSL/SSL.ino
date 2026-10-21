/*!
 * @file SSL.ino
 * @brief Sound Source Localization example
 * @note Switch UART / I2C with VOREO_COMM_MODE below
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
 * @version 1.0
 * @date 2025-12-12
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */

#include <DFRobot_Voreo.h>

/* 通信模式：0=UART，1=I2C */
#define VOREO_COMM_MODE_UART  0
#define VOREO_COMM_MODE_I2C   1
#define VOREO_COMM_MODE       VOREO_COMM_MODE_I2C

#if VOREO_COMM_MODE == VOREO_COMM_MODE_UART
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial mySerial(/*rx=*/5, /*tx=*/4);
DFRobot_Voreo_UART voreo(&mySerial);
#else
DFRobot_Voreo_UART voreo(&Serial1);
#endif
#else
DFRobot_Voreo_I2C voreo;
#endif

void setup()
{
#if VOREO_COMM_MODE == VOREO_COMM_MODE_UART
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    mySerial.begin(9600);
#else
    Serial1.begin(9600);
#endif
#endif
    Serial.begin(115200);
    delay(500);
#if VOREO_COMM_MODE == VOREO_COMM_MODE_UART
    Serial.println("DFRobot_Voreo UART example");
#else
    Serial.println("DFRobot_Voreo I2C example");
    Wire.begin();
    Wire.beginTransmission(0x1F);
    uint8_t probe = Wire.endTransmission();
    Serial.print("I2C probe 0x1F: ");
    Serial.println(probe); /* 0=ACK, 非0=无应答/接线或模组非I2C模式 */
#endif
    Serial.println("calling begin()...");
    Serial.flush();
    while (!voreo.begin()) {
        Serial.println("Voreo begin failed");
        Serial.flush();
        delay(1000);
    }
    Serial.println("Voreo begin ok");
}

void loop()
{
    Serial.println("getAngle...");
    Serial.flush();
    uint16_t angle = voreo.getAngle();
    Serial.print("Angle: ");
    Serial.println(angle);
    Serial.flush();
    delay(200);
}

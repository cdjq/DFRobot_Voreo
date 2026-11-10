/*!
 * @file getASRState.ino
 * @brief Get ASR state example
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
}

void loop()
{
    uint8_t state = voreo.getASRState();
    Serial.print("Device state: ");
    Serial.print(state);
    Serial.print(" ");
    switch (state) {
        case 0: Serial.println("BOOTING"); break;
        case 1: Serial.println("STANDBY"); break;
        case 2: Serial.println("LISTENING"); break;
        case 3: Serial.println("TTS"); break;
        case 4: Serial.println("WAKEWORD_LEARNING"); break;
        case 5: Serial.println("WAKEWORD_LEARN_OK"); break;
        case 6: Serial.println("WAKEWORD_LEARN_FAIL"); break;
        case 7: Serial.println("BOOT_FAIL"); break;
        default: Serial.println("UNKNOWN"); break;
    }
    delay(500);
}

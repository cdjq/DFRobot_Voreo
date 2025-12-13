/*!
 * @file SSL.ino
 * @brief Sound Source Localization example
 * ---------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |
 * ---------------------------------------------------------------------------------------------------------------
 * 
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @version 1.0
 * @date 2025-12-12
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */

#include <DFRobot_Voero.h>
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
#include <SoftwareSerial.h>
#endif

#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
SoftwareSerial mySerial(5, 4);
DFRobot_Voero_UART voero(&mySerial);
#else
DFRobot_Voero_UART voero(&Serial1);
#endif

void setup()
{
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
    mySerial.begin(9600);
#else
    Serial1.begin(9600);
#endif
    Serial.begin(115200);
    Serial.println("DFRobot_Voero_UART example");
    while(!voero.begin())
    {
        Serial.println("Voero begin failed");
        delay(1000);
    }
}

void loop()
{
    uint16_t angle = voero.getAngle();
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(100);
    
}

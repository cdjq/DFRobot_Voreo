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
#include <Adafruit_NeoPixel.h>
#include <DFRobot_Voero.h>
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
#include <SoftwareSerial.h>
#endif

#define LED_PIN    8
#define LED_COUNT  24

#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
SoftwareSerial mySerial(5, 4);
DFRobot_Voero_UART voero(&mySerial);
#else
DFRobot_Voero_UART voero(&Serial1);
#endif

Adafruit_NeoPixel strip(
  LED_COUNT,
  LED_PIN,
  NEO_GRB + NEO_KHZ800
);

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
   strip.begin();
    strip.show();        // 全灭
    strip.setBrightness(50);
}

void loop()
{
    uint16_t angle = voero.getAngle();
    Serial.print("Angle: ");
    Serial.println(angle);
    showAngle(angle);
    delay(100);
    
}

void showAngle(int angle) {
  angle = angle % 360;

  int index = (LED_COUNT - 1) - (angle / 15);
  // 0~11

  strip.clear();

  // 当前方向灯（红色）
  strip.setPixelColor(index, strip.Color(255, 0, 0));

  // 可选：前后渐变效果
  int prev = (index - 1 + LED_COUNT) % LED_COUNT;
  int next = (index + 1) % LED_COUNT;

  strip.setPixelColor(prev, strip.Color(50, 0, 0));
  strip.setPixelColor(next, strip.Color(50, 0, 0));

  strip.show();
}


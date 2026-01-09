/**
 * @file SSL.ino
 * @brief Sound Source Localization example
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @version 1.0
 * @date 2026-01-09
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */

 #include <DFRobot_Voreo.h>
 DFRobot_Voreo_I2C voreo;
 void setup()
 {
     Serial.begin(115200);
     Serial.println("DFRobot_Voreo_I2C example");
     while(!voreo.begin())
     {
         Serial.println("Voreo begin failed");
         delay(1000);
     }
     
 }
 
 void loop()
 {
     uint16_t angle = voreo.getAngle();
     Serial.print("Angle: ");
     Serial.println(angle);
     delay(100);
 }
 
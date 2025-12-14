/**
 * @file SSL.ino
 * @brief Sound Source Localization example
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @version 1.0
 * @date 2025-12-12
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */

 #include <DFRobot_Voero.h>

 DFRobot_Voero_I2C voero;
 
 void setup()
 {
     Serial.begin(115200);
     Serial.println("DFRobot_Voero_I2C example");
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
 
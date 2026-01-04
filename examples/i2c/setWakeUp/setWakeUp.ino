/**
 * @file setWakeUp.ino
 * @brief Set Wake Up example
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @version 1.0
 * @date 2025-12-12
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
     voreo.setWakeUp("kai deng");
 }
 
 void loop()
 {
     delay(100);
 }
 
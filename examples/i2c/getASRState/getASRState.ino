/**
 * @file getASRState.ino
 * @brief Get ASR state example
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
     uint8_t state = voreo.getASRState();
     Serial.print("ASR state: ");
     Serial.println(state);
     printASRState(state);
     delay(100);
 }
 
 void printASRState(uint8_t state)
 {
    switch(state)
    {
        case 0:
            Serial.println("ASR is not working");
            break;
        case 1:
            Serial.println("ASR is working");
            break;
        case 2:
            Serial.println("ASR is working end");
            break;
        default:
            Serial.println("Unknown state");
            break;
    }
}   
 
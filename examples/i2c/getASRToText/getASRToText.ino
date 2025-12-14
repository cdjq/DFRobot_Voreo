/**
 * @file getASRToText.ino
 * @brief Get ASR to Text example
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    [TangJie](jie.tang@dfrobot.com)
 * @version 1.0
 * @date 2025-12-12
 * @url       https://github.com/DFRobot/DFRobot_Voreo
 */

 #include <DFRobot_Voero.h>
 char *text = "欢迎使用DFRobotVoreo语音识别模块,该模块可以实现语音转文字和文字转语音的功能";
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
     voero.sendText(text);
     delay(100);
     if(voero.queryText())
     {
         String text = voero.requestText();
         Serial.println(text);
     }else{
         Serial.println("queryText failed");
     }
     delay(100);
 }
 
# DFRobot_Voreo
* [English Version](./README.md)

这是一个语音转文字，文字转语音的模块。


![产品效果图片](./resources/images/DFR0998.png)


## 产品链接（https://www.dfrobot.com.cn）

    SKU：

## 目录

  * [概述](#概述)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

  * 这是一个语音转文字，文字转语音的模块。

## 库安装

使用此库前，请首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。

## 方法

```C++
  /**
     * @fn begin
     * @brief 初始化 Voreo 模块
     * @return 初始化结果
     * @retval 1 成功
     * @retval 0 失败
     */
    uint8_t begin(void);

    /**
     * @fn queryText
     * @brief 查询 Voreo 模块中的文本
     * @return 查询结果
     * @retval 1 成功
     * @retval 0 失败
     */
    uint8_t queryText(void); 

    /**
     * @fn requestText
     * @brief 从 Voreo 模块请求文本
     * @return 请求结果
     * @retval String 从 Voreo 模块获取的文本
     */
    String requestText(void);

   /**
    * @fn sendText
    * @brief 向 Voreo 模块发送文本
    * @param pText 要发送的文本
    * @return 发送结果
    * @retval 1 成功
    * @retval 0 失败
    */
   uint8_t sendText(uint8_t *pText);
   uint8_t sendText(String text);

   /**
    * @fn getAngle
    * @brief 获取声源角度
    * @return 获取角度的结果
    * @retval 声源的角度值
    */
   uint16_t getAngle(void);

   /**
    * @fn setSpeed
    * @brief 设置 Voreo 模块的语速
    * @param speed 要设置的语速
    * @return 设置语速的结果
    * @retval 1 成功
    * @retval 0 失败
    */
   uint8_t setSpeed(uint8_t speed);

   /**
    * @fn setWakeUp
    * @brief 设置 Voreo 模块的唤醒词
    * @param pData 要设置的数据
    * @return 设置唤醒词的结果
    * @retval 1 成功
    * @retval 0 失败
    */
   uint8_t setWakeUp(uint8_t* pData);
   uint8_t setWakeUp(String data);
```

## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Uno        |      √       |              |             |

## 历史

- 2025/12/12 - 1.0.0 版本

## 创作者

Written by TangJie(jie.tang@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))






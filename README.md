# DFRobot_Voreo
* [Chinese Version](./README_CN.md)

This is a speech-to-text and text-to-speech module.


![Product Image](./resources/images/DFR0998.png)


## Product Link (https://www.dfrobot.com.cn)

    SKU：

## Table of Contents

  * [Overview](#overview)
  * [Library Installation](#library-installation)
  * [Methods](#methods)
  * [Compatibility](#compatibility)
  * [History](#history)
  * [Credits](#credits)

## Overview

  * This is a speech-to-text and text-to-speech module.

## Library Installation

Before using this library, please first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in that folder.

## Methods

```C++
  /**
     * @fn begin
     * @brief Initialize the Voreo module
     * @return The result of the initialization
     * @retval 1 Success
     * @retval 0 Failed
     */
    uint8_t begin(void);

    /**
     * @fn queryText
     * @brief Query the text from the Voreo module
     * @return The result of the query
     * @retval 1 Success
     * @retval 0 Failed
     */
    uint8_t queryText(void); 

    /**
     * @fn requestText
     * @brief Request the text from the Voreo module
     * @return The result of the request
     * @retval String The text from the Voreo module
     */
    String requestText(void);

   /**
    * @fn sendText
    * @brief Send the text to the Voreo module
    * @param pText The text to send
    * @return The result of the send
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t sendText(uint8_t *pText);
   uint8_t sendText(String text);

   /**
    * @fn getAngle
    * @brief Get the angle of the sound source
    * @return The result of getting the angle
    * @retval The angle value of the sound source
    */
   uint16_t getAngle(void);

   /**
    * @fn setSpeed
    * @brief Set the speech speed of the Voreo module
    * @param speed The speech speed to set
    * @return The result of setting the speed
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setSpeed(uint8_t speed);

   /**
    * @fn setWakeUp
    * @brief Set the wake-up word of the Voreo module 
    * @param pData The data to set
    * @return The result of setting the wake-up word
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setWakeUp(uint8_t* pData);
   uint8_t setWakeUp(String data);
```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Uno        |      √       |              |             |

## History

- 2025/12/12 - Version 1.0.0

## Credits

Written by TangJie(jie.tang@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))






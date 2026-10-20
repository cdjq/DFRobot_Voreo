# DFRobot_Voreo
* [Chinese Version](./README_CN.md)

This is a speech-to-text and text-to-speech module.


![Product Image](./resources/images/DFR0998.png)


## Product Link (https://www.dfrobot.com.cn)

    SKU：

## Table of Contents

  * [Overview](#overview)
  * [Library Installation](#library-installation)
  * [Communication Mode](#communication-mode)
  * [Methods](#methods)
  * [Compatibility](#compatibility)
  * [History](#history)
  * [Credits](#credits)

## Overview

  * This is a speech-to-text and text-to-speech module.
  * Supports UART and I2C. Examples switch mode with the `VOREO_COMM_MODE` macro.
  * Supports TTS volume and speech-speed control (`0x00`~`0x0F`).

## Library Installation

Before using this library, please first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in that folder.

## Communication Mode

All Arduino examples under `examples/` share the same UART / I2C switch:

```C++
/* Communication mode: 0 = UART, 1 = I2C */
#define VOREO_COMM_MODE_UART  0
#define VOREO_COMM_MODE_I2C   1
#define VOREO_COMM_MODE       VOREO_COMM_MODE_UART
```

- Set `VOREO_COMM_MODE` to `VOREO_COMM_MODE_UART` for UART (default).
- Set `VOREO_COMM_MODE` to `VOREO_COMM_MODE_I2C` for I2C (address `0x1F`).

Example list:

| Example | Description |
| ------- | ----------- |
| `getASRToText` | TTS playback and ASR text query |
| `getASRState` | Read ASR state |
| `setWakeUp` | Set wake-up word |
| `SSL` | Sound source localization (angle) |
| `setVolumeAndSpeed` | Set TTS volume / speed, then play text |

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
    * @brief Set the TTS speech speed of the Voreo module
    * @param speed Speech speed, range 0x00~0x0F
    * @return The result of setting the speed
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setSpeed(uint8_t speed);

   /**
    * @fn setVolume
    * @brief Set the TTS volume of the Voreo module
    * @param volume Volume level, range 0x00~0x0F
    * @return The result of setting the volume
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setVolume(uint8_t volume);

   /**
    * @fn getASRState
    * @brief Get the ASR state of the Voreo
    * @return The result of the get ASR state
    * @retval 0 ASR is not working
    * @retval 1 ASR is working
    * @retval 2 ASR is working end
    */
   uint8_t getASRState(void);

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

- 2026/10/20 - Version 1.0.1
  - Merge UART / I2C demos; switch with `VOREO_COMM_MODE`
  - Implement `setSpeed()`, add `setVolume()`
- 2025/12/12 - Version 1.0.0

## Credits

Written by TangJie(jie.tang@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))

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
  * Supports UART and I2C. Examples switch mode with the `I2C_COMMUNICATION` macro.
  * Supports TTS volume (`0`~`15`) and speech-speed (`1`~`5`) control.
  * TTS: max **128 bytes** UTF-8 per packet; `sendText()` auto-splits on UTF-8 char boundaries.
  * ASR: max **4096 bytes** UTF-8 per frame; each GTA `A2` frame replaces previous storage.
  * I2C register mailbox: write app frame to `FIFO(0x10)`; poll `STATUS(0x00)` read 3 bytes; read payload via `DATA(0x03)`.

## Library Installation

Before using this library, please first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in that folder.

## Communication Mode

All Arduino examples under `examples/` share the same UART / I2C switch:

```C++
#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked
```

- Define `I2C_COMMUNICATION` to use I2C (address `0x1F`).
- Comment out / mask that line to use UART.

### I2C transport notes

- App frame (same as UART): `0x55 | LEN | CMD | DATA | CRC16`.
- Write: Wire chunks of **28** bytes; TTS app payload max **128** bytes (UTF-8 safe split).
- ASR read payload max **4096** bytes per frame.
- Read: write request to `FIFO(0x10)` once → poll `Write STATUS(0x00)` + `Read 3` until `READY(0x02)` → `Write DATA(0x03)` then read `len` in chunks. Do **not** resend the request while polling.
- `ABORT (0x2F)` clears a stuck assemble/TX state.
- STATUS: `0=IDLE`, `1=BUSY`, `2=READY`, `3=ERROR`.

Example list:

| Example | Description |
| ------- | ----------- |
| `getASRToText` | TTS playback, volume / speed, and ASR text query |
| `getASRState` | Read ASR state |
| `setWakeUp` | Set wake-up word |
| `getDOA` | DOA mode / angle / intensity |

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
    * @fn setDoaMode
    * @brief Set DOA run mode (0 off / 1 on / 2 LISTENING)
    */
   uint8_t setDoaMode(uint8_t mode);

   /**
    * @fn getDoaMode
    * @brief Query DOA run mode
    */
   uint8_t getDoaMode(void);

   /**
    * @fn getDoaIntensity
    * @brief Query DOA intensity (<VAD -> 0; >=VAD log-quantized 1~255)
    * @note UART/I2C app frame 0x0B; I2C preferred REG 0x06
    */
   uint8_t getDoaIntensity(void);

   /**
    * @fn setSpeed
    * @brief Set the TTS speech speed of the Voreo module
    * @param speed Speech speed, range 1~5 (decimal)
    * @return The result of setting the speed
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setSpeed(uint8_t speed);
   uint8_t getSpeed(void);

   /**
    * @fn setVolume
    * @brief Set the TTS volume of the Voreo module
    * @param volume Volume level, range 0~15 (decimal)
    * @return The result of setting the volume
    * @retval 1 Success
    * @retval 0 Failed
    */
   uint8_t setVolume(uint8_t volume);
   uint8_t getVolume(void);

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

- 2026/11/10 - Version 1.1.0
  - TTS speed range `1`~`5`; merge volume/speed demo into `getASRToText`
  - Author: RongLi(rong.li@dfrobot.com)
- 2026/10/20 - Version 1.0.3
  - TTS max 128B with UTF-8-safe `sendText()` split; ASR frame max 4KB
- 2026/10/20 - Version 1.0.2
  - Redesign I2C transport: STATUS 3-byte poll, 28-byte chunks, `uint16_t` payload length
- 2026/10/20 - Version 1.0.1
  - Merge UART / I2C demos; switch with `I2C_COMMUNICATION`
  - Implement `setSpeed()`, add `setVolume()`
- 2025/12/12 - Version 1.0.0

## Credits

Written by TangJie(jie.tang@dfrobot.com), RongLi(rong.li@dfrobot.com), 2025-2026. (Welcome to our [website](https://www.dfrobot.com/))

# DFRobot_Voreo
* [English Version](./README.md)

这是一个语音转文字，文字转语音的模块。


![产品效果图片](./resources/images/DFR0998.png)


## 产品链接（https://www.dfrobot.com.cn）

    SKU：

## 目录

  * [概述](#概述)
  * [库安装](#库安装)
  * [通信模式](#通信模式)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

  * 这是一个语音转文字，文字转语音的模块。
  * 支持 UART 与 I2C，示例通过宏 `VOREO_COMM_MODE` 切换。
  * 支持调节 TTS 音量与语速（十进制范围 `0`~`15`）。
  * TTS：单包最大 **128** 字节 UTF-8；`sendText()` 按 UTF-8 字符边界自动切段（1/2/3/4 字节，汉字多为 3 字节）。
  * ASR：单帧最大 **4096** 字节；GTA 每来一帧 `A2` 即从头覆盖存储。
  * I2C 寄存器邮箱：应用帧写 `FIFO(0x10)`；轮询 `STATUS(0x00)` 读 3 字节；载荷经 `DATA(0x03)` 读取。

## 库安装

使用此库前，请首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。

## 通信模式

`examples/` 下所有 Arduino 示例使用同一套 UART / I2C 切换方式：

```C++
/* 通信模式：0=UART，1=I2C */
#define VOREO_COMM_MODE_UART  0
#define VOREO_COMM_MODE_I2C   1
#define VOREO_COMM_MODE       VOREO_COMM_MODE_UART
```

- 将 `VOREO_COMM_MODE` 设为 `VOREO_COMM_MODE_UART` 使用串口（默认）
- 将 `VOREO_COMM_MODE` 设为 `VOREO_COMM_MODE_I2C` 使用 I2C（地址 `0x1F`）

### I2C 传输说明

- 应用帧（与 UART 相同）：`0x55 | LEN | CMD | DATA | CRC16`
- 写：Wire 按 **28** 字节分包；TTS 应用载荷最大 **128** 字节（UTF-8 安全切分）
- ASR 读载荷单帧最大 **4096** 字节
- 读：请求写入 `FIFO(0x10)` 一次 → 轮询 `Write STATUS(0x00)` + `Read 3` 至 `READY(0x02)` → `Write DATA(0x03)` 后按 `len` 分块读；轮询期间**不要重发**业务请求
- `ABORT (0x2F)` 可清空卡死的组帧/应答状态
- STATUS：`0=IDLE`，`1=BUSY`，`2=READY`，`3=ERROR`

示例列表：

| 示例 | 说明 |
| ---- | ---- |
| `getASRToText` | TTS 播报与 ASR 文本查询 |
| `getASRState` | 读取 ASR 状态 |
| `setWakeUp` | 设置唤醒词 |
| `SSL` | 声源定位（角度） |
| `setVolumeAndSpeed` | 设置 TTS 音量 / 语速并播报文本 |

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
    * @brief 设置 Voreo 模块的 TTS 语速
    * @param speed 语速，范围 0~15（十进制）
    * @return 设置语速的结果
    * @retval 1 成功
    * @retval 0 失败
    */
   uint8_t setSpeed(uint8_t speed);

   /**
    * @fn setVolume
    * @brief 设置 Voreo 模块的 TTS 音量
    * @param volume 音量，范围 0~15（十进制）
    * @return 设置音量的结果
    * @retval 1 成功
    * @retval 0 失败
    */
   uint8_t setVolume(uint8_t volume);

   /**
    * @fn getASRState
    * @brief 获取 Voreo ASR 状态
    * @return 返回获取的ASR状态
    * @retval 0 ASR 未识别
    * @retval 1 ASR 开始识别
    * @retval 2 ASR 识别完成
    */
   uint8_t getASRState(void);

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

- 2026/10/20 - 1.0.3 版本
  - TTS 单包 ≤128B，`sendText()` UTF-8 安全切段；ASR 单帧 ≤4KB
- 2026/10/20 - 1.0.2 版本
  - 重设计 I2C 传输：STATUS 三字节轮询、28 字节分包、`uint16_t` 载荷长度
- 2026/10/20 - 1.0.1 版本
  - 合并 UART / I2C 示例，通过 `VOREO_COMM_MODE` 宏切换
  - 实现 `setSpeed()`，新增 `setVolume()`
- 2025/12/12 - 1.0.0 版本

## 创作者

Written by TangJie(jie.tang@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))

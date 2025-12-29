#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file example_i2c_get_asr_to_text.py
@brief Get ASR to Text example using I2C
@copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
@license The MIT License (MIT)
@author [TangJie](jie.tang@dfrobot.com)
@version 1.0
@date 2025-12-12
@url https://github.com/DFRobot/DFRobot_Voreo

This example demonstrates how to use DFRobot_Voero with I2C interface on Raspberry Pi.
It sends text to the module (TTS) and receives recognized text back (ASR).

Hardware connection:
    VCC -> 3.3V or 5V
    GND -> GND
    SDA -> SDA (GPIO 2 on Raspberry Pi)
    SCL -> SCL (GPIO 3 on Raspberry Pi)

Usage:
    python3 example_i2c_get_asr_to_text.py
"""

import time
import sys
import os

# Add the parent directory to path to import DFRobot_Voero
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from DFRobot_Voero import DFRobot_Voero_I2C, RETURN_OK, RETURN_ERROR


def main():
    """Main function"""
    print("DFRobot_Voero_I2C example - Get ASR to Text")
    print("=" * 50)
    
    # Initialize I2C interface
    # i2c_bus=1 for Raspberry Pi (corresponds to /dev/i2c-1)
    # i2c_addr=0x1f is the default address
    voero = DFRobot_Voero_I2C(i2c_bus=1, i2c_addr=0x1f)
    
    # Initialize the module
    print("Initializing Voero module...")
    while voero.begin() != RETURN_OK:
        print("Voero begin failed, retrying...")
        time.sleep(1)
    print("Voero initialized successfully!")
    print()
    
    # Text to send for TTS (Text to Speech)
    text_to_send = "欢迎使用DFRobotVoreo语音识别模块,该模块可以实现语音转文字和文字转语音的功能"
    
    # Main loop
    try:
        while True:
            # Send text to the module (TTS)
            print(f"Sending text: {text_to_send}")
            voero.send_text(text_to_send)
            time.sleep(8)  # Wait 8s
            # Query if there's recognized text available (ASR)
            if voero.query_text() == RETURN_OK:
                # Get the recognized text
                received_text = voero.request_text()
                if received_text:
                    print(f"Received text (ASR): {received_text}")
                else:
                    print("No text received")
            else:
                print("queryText failed")
            
            print("-" * 50)
            time.sleep(0.1)  # Wait 100ms before next iteration
            
    except KeyboardInterrupt:
        print("\n\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()


#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file example_uart_get_asr_to_text.py
@brief Get ASR to Text example using UART
@copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
@license The MIT License (MIT)
@author [TangJie](jie.tang@dfrobot.com)
@version 1.0
@date 2025-12-12
@url https://github.com/DFRobot/DFRobot_Voreo

This example demonstrates how to use DFRobot_Voero with UART interface on Raspberry Pi.
It sends text to the module (TTS) and receives recognized text back (ASR).

Hardware connection:
    VCC -> 3.3V or 5V
    GND -> GND
    RX  -> TX (GPIO 14 on Raspberry Pi)
    TX  -> RX (GPIO 15 on Raspberry Pi)

Note: 
    - On Raspberry Pi, you may need to enable UART:
      sudo raspi-config -> Interface Options -> Serial Port -> Enable
    - The serial port path depends on your setup:
      /dev/ttyUSB0 - USB to Serial adapter
      /dev/ttyAMA0 - Hardware UART (older Raspberry Pi)
      /dev/serial0 - Hardware UART (newer Raspberry Pi, symlink to /dev/ttyAMA0 or /dev/ttyS0)

Usage:
    python3 example_uart_get_asr_to_text.py
    
    Or specify a different serial port:
    python3 example_uart_get_asr_to_text.py /dev/ttyUSB0
"""

import time
import sys
import os

# Add the parent directory to path to import DFRobot_Voero
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from DFRobot_Voero import DFRobot_Voero_UART, RETURN_OK, RETURN_ERROR


def main():
    """Main function"""
    print("DFRobot_Voero_UART example - Get ASR to Text")
    print("=" * 50)
    
    # Get serial port from command line argument or use default
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    else:
        # Try common serial ports on Raspberry Pi
        possible_ports = ['/dev/ttyUSB0', '/dev/serial0', '/dev/ttyAMA0']
        serial_port = None
        
        # Try to find an available port
        import serial
        for port in possible_ports:
            try:
                test_serial = serial.Serial(port, 9600, timeout=0.1)
                test_serial.close()
                serial_port = port
                break
            except:
                continue
        
        if serial_port is None:
            print("Error: Could not find an available serial port.")
            print(f"Please specify the port manually: {sys.argv[0]} /dev/ttyUSB0")
            sys.exit(1)
    
    print(f"Using serial port: {serial_port}")
    
    # Initialize UART interface
    # baudrate=9600 is the default
    try:
        voero = DFRobot_Voero_UART(serial_port, baudrate=9600)
    except Exception as e:
        print(f"Failed to open serial port {serial_port}: {e}")
        print("Make sure:")
        print("  1. The device is connected")
        print("  2. You have permission to access the serial port (you may need to add user to dialout group)")
        print("  3. UART is enabled (sudo raspi-config -> Interface Options -> Serial Port)")
        sys.exit(1)
    
    # Initialize the module
    print("Initializing Voero module...")
    try:
        while voero.begin() != RETURN_OK:
            print("Voero begin failed, retrying...")
            time.sleep(1)
        print("Voero initialized successfully!")
        print()
    except KeyboardInterrupt:
        print("\n\nExiting...")
        voero.close()
        sys.exit(0)
    
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
    finally:
        # Always close the serial port when done
        voero.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()


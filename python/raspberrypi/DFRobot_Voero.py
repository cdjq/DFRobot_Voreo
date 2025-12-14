#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file DFRobot_Voero.py
@brief DFRobot Voreo library for Python (Raspberry Pi compatible)
@copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
@license The MIT License (MIT)
@author [TangJie](jie.tang@dfrobot.com)
@version 1.0
@date 2025-12-12
@url https://github.com/DFRobot/DFRobot_Voreo

Usage examples for Raspberry Pi:
    # I2C example
    from DFRobot_Voero import DFRobot_Voero_I2C
    voero = DFRobot_Voero_I2C(i2c_bus=1, i2c_addr=0x1f)  # i2c_bus=1 for /dev/i2c-1
    if voero.begin() == RETURN_OK:
        text = voero.request_text()
        print(f"Received text: {text}")
    
    # UART example
    from DFRobot_Voero import DFRobot_Voero_UART
    voero = DFRobot_Voero_UART('/dev/ttyUSB0', baudrate=9600)  # or '/dev/ttyAMA0'
    if voero.begin() == RETURN_OK:
        text = voero.request_text()
        print(f"Received text: {text}")
        voero.close()  # Don't forget to close when done

Dependencies:
    For I2C: sudo apt-get install python3-smbus (or pip install smbus2)
    For UART: pip install pyserial
"""

import time
import struct
from typing import Optional, Union

# Constants
VOERO_I2C_ADDR = 0x1f  # I2C address

VOERO_CMD_HEAD = 0x55  # Command head
VOERO_CMD_END = 0xFF  # Command end
VOERO_CMD_BEGIN = 0x00  # Command begin
VOERO_CMD_QUERY_TEXT = 0x01  # Query text command
VOERO_CMD_GET_TEXT = 0x02  # Get text command
VOERO_CMD_SEND_TEXT = 0x03  # Send text command
VOERO_CMD_ANGLE = 0x04  # Angle command
VOERO_CMD_DISTANCE = 0x05  # Distance command
VOERO_CMD_SET_SPEED = 0x06  # Set speed command

VOERO_REQUEST_TIMEOUT = 10000  # Request timeout (ms)

RETURN_OK = 0x01  # Return OK
RETURN_ERROR = 0x00  # Return ERROR

DATA_STATIC_HEAD = 1  # Data static head
DATA_STATIC_LEN_H = 2  # Data static length high
DATA_STATIC_LEN_L = 3  # Data static length low
DATA_STATIC_CMD = 4  # Data static command
DATA_STATIC_DATA = 5  # Data static data
DATA_STATIC_CRC_H = 6  # Data static CRC high
DATA_STATIC_CRC_L = 7  # Data static CRC low

CRC16_POLYNOMIAL = 0x1021
CRC16_INITIAL_VALUE = 0xFFFF

# Enable debug output
ENABLE_DBG = False


def dbg(msg):
    """Debug output function"""
    if ENABLE_DBG:
        print(f"[DEBUG] {msg}")


def calculate_crc16(data: bytes) -> int:
    """Calculate CRC16 checksum"""
    crc = CRC16_INITIAL_VALUE
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ CRC16_POLYNOMIAL
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_request_frame(cmd: int) -> bytes:
    """Build request frame"""
    buf = bytearray(6)
    buf[0] = VOERO_CMD_HEAD
    buf[1] = 0
    buf[2] = 0
    buf[3] = cmd
    crc = calculate_crc16(buf[:4])
    buf[4] = (crc >> 8) & 0xFF
    buf[5] = crc & 0xFF
    return bytes(buf)


def build_command_frame(cmd: int, data: bytes) -> bytes:
    """Build command frame"""
    data_length = len(data)
    buf = bytearray(data_length + 6)
    buf[0] = VOERO_CMD_HEAD
    buf[1] = (data_length >> 8) & 0xFF
    buf[2] = data_length & 0xFF
    buf[3] = cmd
    if data_length > 0:
        buf[4:4 + data_length] = data
    crc = calculate_crc16(buf[:data_length + 4])
    buf[data_length + 4] = (crc >> 8) & 0xFF
    buf[data_length + 5] = crc & 0xFF
    return bytes(buf)


def validate_and_return_data(p_buf: bytearray, length: int) -> Optional[bytearray]:
    """Validate CRC and return data"""
    calculated_crc = calculate_crc16(p_buf[:length + 4])
    received_crc = (p_buf[length + 4] << 8) | p_buf[length + 5]
    if calculated_crc != received_crc:
        dbg("crc error")
        return None
    return p_buf


def process_data_state(data: int, cmd: int, request_state: list, length: list,
                       data_length: list, p_buf_list: list) -> int:
    """
    Process data parsing state machine
    Returns: 0 - continue, 1 - error, 2 - complete
    p_buf_list: list containing the buffer reference (to allow modification)
    """
    state = request_state[0]
    
    if state == DATA_STATIC_HEAD:
        if data == VOERO_CMD_HEAD:
            request_state[0] = DATA_STATIC_LEN_H
            data_length[0] = 0
        return 0
    
    elif state == DATA_STATIC_LEN_H:
        length[0] = data << 8
        request_state[0] = DATA_STATIC_LEN_L
        return 0
    
    elif state == DATA_STATIC_LEN_L:
        length[0] |= data
        request_state[0] = DATA_STATIC_CMD
        return 0
    
    elif state == DATA_STATIC_CMD:
        if data != cmd:
            request_state[0] = DATA_STATIC_HEAD
            length[0] = 0
            data_length[0] = 0
            p_buf_list[0] = None
            return 0
        
        # Allocate buffer
        p_buf_list[0] = bytearray(length[0] + 6)
        p_buf_list[0][0] = VOERO_CMD_HEAD
        p_buf_list[0][1] = (length[0] >> 8) & 0xFF
        p_buf_list[0][2] = length[0] & 0xFF
        p_buf_list[0][3] = cmd
        request_state[0] = DATA_STATIC_CRC_H if length[0] == 0 else DATA_STATIC_DATA
        return 0
    
    elif state == DATA_STATIC_DATA:
        if p_buf_list[0] is None:
            return 1  # Error: buffer not allocated
        if data_length[0] < length[0]:
            p_buf_list[0][data_length[0] + 4] = data
            data_length[0] += 1
        if data_length[0] >= length[0]:
            request_state[0] = DATA_STATIC_CRC_H
        return 0
    
    elif state == DATA_STATIC_CRC_H:
        if p_buf_list[0] is None:
            return 1  # Error: buffer not allocated
        p_buf_list[0][length[0] + 4] = data
        request_state[0] = DATA_STATIC_CRC_L
        return 0
    
    elif state == DATA_STATIC_CRC_L:
        if p_buf_list[0] is None:
            return 1  # Error: buffer not allocated
        p_buf_list[0][length[0] + 5] = data
        return 2  # Complete
    
    return 0


class DFRobot_Voero:
    """Base class for DFRobot Voero"""
    
    def begin(self) -> int:
        """Initialize the Voero"""
        p_buf = self.read_data(VOERO_CMD_BEGIN)
        if p_buf is None:
            return RETURN_ERROR
        result = RETURN_OK if p_buf[4] != 0 else RETURN_ERROR
        return result
    
    def query_text(self) -> int:
        """Query the text from the Voero"""
        p_buf = self.read_data(VOERO_CMD_QUERY_TEXT)
        if p_buf is None:
            return RETURN_ERROR
        result = RETURN_OK if p_buf[4] != 0 else RETURN_ERROR
        return result
    
    def request_text(self) -> str:
        """Request the text from the Voero"""
        p_buf = self.read_data(VOERO_CMD_GET_TEXT)
        if p_buf is None:
            dbg("readData failed")
            return ""
        
        # Extract length and data
        length = (p_buf[1] << 8) | p_buf[2]
        if length == 0:
            return ""
        
        # Extract text data from p_buf[4]
        text_bytes = p_buf[4:4 + length]
        try:
            text = text_bytes.decode('utf-8')
        except UnicodeDecodeError:
            text = text_bytes.decode('utf-8', errors='ignore')
        return text
    
    def send_text(self, text: Union[str, bytes]) -> int:
        """Send the text to the Voero"""
        if isinstance(text, str):
            text_bytes = text.encode('utf-8')
        else:
            text_bytes = text
        self.send_command(VOERO_CMD_SEND_TEXT, text_bytes)
        return 1
    
    def get_angle(self) -> int:
        """Get the angle of the sound source"""
        p_buf = self.read_data(VOERO_CMD_ANGLE)
        if p_buf is None:
            return RETURN_ERROR
        angle = (p_buf[4] << 8) | p_buf[5]
        return angle
    
    def set_speed(self, speed: int) -> int:
        """Set the speed of the Voero"""
        return 1
    
    def send_command(self, cmd: int, data: bytes) -> int:
        """Send command to the Voero (to be implemented by subclasses)"""
        raise NotImplementedError("Subclass must implement send_command")
    
    def read_data(self, cmd: int) -> Optional[bytearray]:
        """Read data from the Voero (to be implemented by subclasses)"""
        raise NotImplementedError("Subclass must implement read_data")


class DFRobot_Voero_I2C(DFRobot_Voero):
    """I2C implementation of DFRobot Voero for Raspberry Pi"""
    
    def __init__(self, i2c_bus=1, i2c_addr=VOERO_I2C_ADDR):
        """
        Initialize I2C interface
        @param i2c_bus: I2C bus number (default: 1 for Raspberry Pi)
                       On Raspberry Pi: 1 for /dev/i2c-1, 0 for /dev/i2c-0
        @param i2c_addr: I2C address (default: 0x1f)
        
        Note: For Raspberry Pi, you need to install smbus:
              sudo apt-get install python3-smbus
              or
              pip install smbus
        """
        self._addr = i2c_addr
        # Try smbus2 first (more modern), then fallback to smbus
        try:
            import smbus2
            self._bus = smbus2.SMBus(i2c_bus)
            self._use_smbus2 = True
        except ImportError:
            try:
                import smbus
                self._bus = smbus.SMBus(i2c_bus)
                self._use_smbus2 = False
            except ImportError:
                raise ImportError(
                    "smbus or smbus2 module not found. "
                    "For Raspberry Pi, install with: sudo apt-get install python3-smbus "
                    "or pip install smbus2"
                )
        except Exception as e:
            raise RuntimeError(f"Failed to initialize I2C bus {i2c_bus}: {e}")
    
    def begin(self) -> int:
        """Initialize the Voero"""
        return super().begin()
    
    def send_command(self, cmd: int, data: bytes) -> int:
        """Send command to the Voero via I2C"""
        MAX_I2C_PACKET_SIZE = 28  # I2C最大数据包长度
        PACKET_DELAY_MS = 0.01  # 分包发送间隔(s)
        
        # Build command frame
        frame = build_command_frame(cmd, data)
        total_length = len(frame)
        
        # If data length exceeds I2C packet size limit, split into multiple packets
        if total_length > MAX_I2C_PACKET_SIZE:
            dbg("data length exceeds I2C packet size, splitting into multiple packets")
            offset = 0
            
            while offset < total_length:
                # Calculate bytes to send this time (max MAX_I2C_PACKET_SIZE bytes)
                chunk_size = min(MAX_I2C_PACKET_SIZE, total_length - offset)
                
                # Send data chunk
                try:
                    chunk = frame[offset:offset + chunk_size]
                    # Use write_i2c_block_data: first byte as register, rest as data
                    if len(chunk) == 1:
                        self._bus.write_byte(self._addr, chunk[0])
                    else:
                        self._bus.write_i2c_block_data(self._addr, chunk[0], list(chunk[1:]))
                except Exception as e:
                    dbg(f"I2C transmission error during packet splitting: {e}")
                    return 0
                
                offset += chunk_size
                
                # If not the last packet, add delay
                if offset < total_length:
                    time.sleep(PACKET_DELAY_MS)
            
            return 1
        else:
            # Data length within limit, send directly
            try:
                if total_length > 0:
                    if total_length == 1:
                        self._bus.write_byte(self._addr, frame[0])
                    else:
                        self._bus.write_i2c_block_data(self._addr, frame[0], list(frame[1:]))
                return 1
            except Exception as e:
                dbg(f"I2C transmission error: {e}")
                return 0
    
    def read_data(self, cmd: int) -> Optional[bytearray]:
        """Read data from the Voero via I2C"""
        CMD_QUERY_LENGTH = 0x21  # 查询数据长度命令
        MAX_RETRY_COUNT = 10  # 最大重试次数
        MAX_I2C_PACKET_SIZE = 32  # I2C最大数据包长度
        INVALID_LENGTH = 0xFFFF  # 无效长度标识
        RESPONSE_DELAY_MS = 0.1  # 响应延迟(s)
        
        # Build and send initial request frame
        request_buf = build_request_frame(cmd)
        try:
            if len(request_buf) > 0:
                if len(request_buf) == 1:
                    self._bus.write_byte(self._addr, request_buf[0])
                else:
                    self._bus.write_i2c_block_data(self._addr, request_buf[0], list(request_buf[1:]))
        except Exception as e:
            dbg(f"Failed to send request frame: {e}")
            return None
        
        time.sleep(RESPONSE_DELAY_MS)
        
        # Initialize state variables
        p_buf = None
        request_state = [DATA_STATIC_HEAD]
        length = [0]
        data_length = [0]
        retry_count = 0
        
        # Main loop: try to read data
        while retry_count < MAX_RETRY_COUNT:
            # Query data length
            try:
                self._bus.write_byte(self._addr, CMD_QUERY_LENGTH)
            except Exception as e:
                dbg(f"Failed to query length: {e}")
                retry_count += 1
                time.sleep(RESPONSE_DELAY_MS)
                continue
            
            # Read length bytes (read 2 bytes from device)
            try:
                length_bytes = self._bus.read_i2c_block_data(self._addr, 0, 2)
                if len(length_bytes) < 2:
                    dbg("request length failed 1 - insufficient data")
                    retry_count += 1
                    time.sleep(RESPONSE_DELAY_MS)
                    continue
            except Exception as e:
                dbg(f"Failed to read length: {e}")
                retry_count += 1
                time.sleep(RESPONSE_DELAY_MS)
                continue
            
            # Parse length (big-endian)
            len_val = (length_bytes[0] << 8) | length_bytes[1]
            
            # Check length validity
            if len_val == INVALID_LENGTH:
                dbg("request length failed 2 - invalid length")
                # Resend request frame
                try:
                    if len(request_buf) > 0:
                        self._bus.write_i2c_block_data(self._addr, request_buf[0], list(request_buf[1:]))
                except Exception:
                    pass
                retry_count += 1
                time.sleep(RESPONSE_DELAY_MS)
                continue
            
            # Read data in chunks (support long packets exceeding MAX_I2C_PACKET_SIZE)
            total_bytes_read = 0
            result = 0
            need_retry = False
            
            # Loop to read until len bytes are read
            while total_bytes_read < len_val and not need_retry:
                # Calculate bytes to read this time (max MAX_I2C_PACKET_SIZE bytes)
                bytes_to_read = min(MAX_I2C_PACKET_SIZE, len_val - total_bytes_read)
                
                # Request data chunk
                try:
                    data_chunk = self._bus.read_i2c_block_data(self._addr, 0, bytes_to_read)
                    if len(data_chunk) == 0:
                        dbg("request data failed")
                        need_retry = True
                        break
                except Exception as e:
                    dbg(f"Failed to read data chunk: {e}")
                    need_retry = True
                    break
                
                # Read and process data
                p_buf_ref = [p_buf]  # Use list to allow modification
                for i in range(min(bytes_to_read, len(data_chunk))):
                    if total_bytes_read >= len_val:
                        break
                    data_byte = data_chunk[i]
                    result = process_data_state(data_byte, cmd, request_state, length,
                                                 data_length, p_buf_ref)
                    p_buf = p_buf_ref[0]  # Update reference
                    total_bytes_read += 1
                    
                    # Check processing result
                    if result == 1:  # Error
                        dbg("data processing error")
                        request_state[0] = DATA_STATIC_HEAD
                        length[0] = 0
                        data_length[0] = 0
                        p_buf = None
                        p_buf_ref[0] = None
                        need_retry = True
                        break
                    elif result == 2:  # State machine complete
                        # If state machine completes but data not fully read, protocol mismatch
                        if total_bytes_read < len_val:
                            dbg("state machine complete but data not fully read")
                            request_state[0] = DATA_STATIC_HEAD
                            length[0] = 0
                            data_length[0] = 0
                            p_buf = None
                            p_buf_ref[0] = None
                            need_retry = True
                        break
                
                # If there's remaining data to read and no error, add short delay
                if total_bytes_read < len_val and not need_retry and result != 2:
                    time.sleep(0.01)  # Short delay, wait for device to prepare next packet
            
            # Check if all data was successfully read
            if not need_retry and total_bytes_read >= len_val:
                # If state machine completed, perform CRC validation
                if result == 2:
                    validated_buf = validate_and_return_data(p_buf, length[0])
                    if validated_buf is not None:
                        return validated_buf
                    # CRC validation failed, cleanup and mark for retry
                    request_state[0] = DATA_STATIC_HEAD
                    length[0] = 0
                    data_length[0] = 0
                    p_buf = None
                    dbg("CRC validation failed, retrying")
                    need_retry = True
                else:
                    # Data read complete but state machine not finished, need retry
                    dbg("data read complete but state machine not finished")
                    need_retry = True
            elif not need_retry and total_bytes_read < len_val:
                dbg("incomplete data read")
                need_retry = True
            
            # If need retry or data incomplete, resend request frame
            if need_retry or result == 0:
                if result == 0:
                    dbg("request length failed 4 - incomplete data")
                # Resend request frame
                try:
                    if len(request_buf) > 0:
                        self._bus.write_i2c_block_data(self._addr, request_buf[0], list(request_buf[1:]))
                except Exception:
                    pass
                retry_count += 1
                time.sleep(RESPONSE_DELAY_MS)
        
        # Timeout cleanup
        dbg("request timeout")
        return None


class DFRobot_Voero_UART(DFRobot_Voero):
    """UART implementation of DFRobot Voero for Raspberry Pi"""
    
    def __init__(self, serial_port: str, baudrate: int = 9600, timeout: float = 1.0):
        """
        Initialize UART interface
        @param serial_port: Serial port path 
                           On Raspberry Pi: '/dev/ttyUSB0', '/dev/ttyAMA0', or '/dev/serial0'
                           On Windows: 'COM1', 'COM2', etc.
        @param baudrate: Baud rate (default: 9600)
        @param timeout: Read timeout in seconds (default: 1.0)
        
        Note: For Raspberry Pi, you may need to enable UART:
              sudo raspi-config -> Interface Options -> Serial Port -> Enable
              Install pyserial: pip install pyserial
        """
        try:
            import serial
            self._serial = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self._timeout = timeout
            # Clear any pending data
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except ImportError:
            raise ImportError(
                "pyserial module not found. Install it with: pip install pyserial"
            )
        except Exception as e:
            raise RuntimeError(f"Failed to initialize serial port {serial_port}: {e}")
    
    def begin(self) -> int:
        """Initialize the Voero"""
        return super().begin()
    
    def send_command(self, cmd: int, data: bytes) -> int:
        """Send command to the Voero via UART"""
        frame = build_command_frame(cmd, data)
        try:
            self._serial.write(frame)
            return 1
        except Exception as e:
            dbg(f"UART transmission error: {e}")
            return 0
    
    def read_data(self, cmd: int) -> Optional[bytearray]:
        """Read data from the Voero via UART"""
        request_buf = build_request_frame(cmd)
        try:
            self._serial.write(request_buf)
        except Exception as e:
            dbg(f"Failed to send request frame: {e}")
            return None
        
        # Initialize state variables
        p_buf = None
        request_state = [DATA_STATIC_HEAD]
        length = [0]
        data_length = [0]
        p_buf_ref = [p_buf]
        
        start_time = time.time()
        timeout_seconds = VOERO_REQUEST_TIMEOUT / 1000.0  # Convert ms to seconds
        
        while (time.time() - start_time) < timeout_seconds:
            if self._serial.in_waiting == 0:
                time.sleep(0.001)  # Short sleep to avoid busy waiting
                continue
            
            try:
                data_byte = ord(self._serial.read(1))
            except Exception as e:
                dbg(f"Failed to read from serial: {e}")
                break
            
            result = process_data_state(data_byte, cmd, request_state, length,
                                        data_length, p_buf_ref)
            p_buf = p_buf_ref[0]  # Update reference
            
            if result == 1:  # Error
                dbg("data processing error")
                return None
            elif result == 2:  # Complete
                validated_buf = validate_and_return_data(p_buf, length[0])
                return validated_buf
        
        # Timeout
        dbg("request timeout")
        return None
    
    def close(self):
        """Close the serial port"""
        if hasattr(self, '_serial') and self._serial.is_open:
            self._serial.close()
         
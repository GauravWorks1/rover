"""
Serial Communication between Raspberry Pi and Arduino.

Protocol:
  RPi -> Arduino (8 bytes):
    [0xAA] [CMD] [SPEED_H] [SPEED_L] [STEER_H] [STEER_L] [CHECKSUM] [0x55]

  Arduino -> RPi (12 bytes):
    [0xBB] [MODE] [CH1_H] [CH1_L] [CH2_H] [CH2_L] [CH3_H] [CH3_L]
    [CH5_H] [CH5_L] [CHECKSUM] [0x55]
"""

import serial
import struct
import threading
import time
import logging

from config import (
    SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT,
    START_BYTE, END_BYTE, STATUS_START,
    CMD_DRIVE, CMD_STOP, CMD_QUERY, CMD_WEB_DRIVE,
    CMD_PACKET_SIZE, STATUS_PACKET_SIZE,
    MODE_RC, MODE_FOLLOW, MODE_FAILSAFE
)

logger = logging.getLogger(__name__)


class SerialComm:
    """Handles serial communication with the Arduino motor controller."""

    def __init__(self, port=SERIAL_PORT, baud=SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.connected = False
        self.lock = threading.Lock()

        # Last received status
        self.current_mode = MODE_RC
        self.rc_channels = [1500, 1500, 1500, 1500]  # CH1, CH2, CH3, CH5
        self.status_lock = threading.Lock()

        # Reader thread
        self._reader_thread = None
        self._running = False

    def connect(self):
        """Open serial connection to Arduino."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT
            )
            time.sleep(2.0)  # Wait for Arduino reset after serial connection
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.connected = True
            logger.info(f"Connected to Arduino on {self.port} at {self.baud} baud")

            # Start background reader thread
            self._running = True
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()

            return True
        except serial.SerialException as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Close serial connection."""
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=2.0)
        if self.ser and self.ser.is_open:
            self.send_stop()  # Safety: stop motors before disconnecting
            time.sleep(0.1)
            self.ser.close()
        self.connected = False
        logger.info("Disconnected from Arduino")

    # =========================================================================
    # Sending Commands
    # =========================================================================

    def _build_packet(self, cmd_type, speed=0, steer=0):
        """
        Build a command packet to send to Arduino.

        Args:
            cmd_type: CMD_DRIVE, CMD_STOP, CMD_QUERY, or CMD_WEB_DRIVE
            speed:    int16 (-255 to 255), positive = forward (or left speed for web)
            steer:    int16 (-100 to 100 for CMD_DRIVE, -255 to 255 for CMD_WEB_DRIVE)

        Returns:
            bytearray of CMD_PACKET_SIZE bytes
        """
        # Clamp values
        speed = max(-255, min(255, int(speed)))
        max_steer = 255 if cmd_type == CMD_WEB_DRIVE else 100
        steer = max(-max_steer, min(max_steer, int(steer)))

        # Pack speed and steer as signed 16-bit big-endian
        speed_h = (speed >> 8) & 0xFF
        speed_l = speed & 0xFF
        steer_h = (steer >> 8) & 0xFF
        steer_l = steer & 0xFF

        # Handle negative numbers (two's complement)
        if speed < 0:
            speed_unsigned = speed + 65536
            speed_h = (speed_unsigned >> 8) & 0xFF
            speed_l = speed_unsigned & 0xFF
        if steer < 0:
            steer_unsigned = steer + 65536
            steer_h = (steer_unsigned >> 8) & 0xFF
            steer_l = steer_unsigned & 0xFF

        # Calculate checksum (XOR of payload bytes)
        checksum = cmd_type ^ speed_h ^ speed_l ^ steer_h ^ steer_l

        packet = bytearray([
            START_BYTE,
            cmd_type,
            speed_h, speed_l,
            steer_h, steer_l,
            checksum,
            END_BYTE
        ])
        return packet

    def send_drive(self, speed, steer):
        """
        Send a drive command.

        Args:
            speed: -255 to 255 (negative = reverse)
            steer: -100 to 100 (negative = left, positive = right)
        """
        packet = self._build_packet(CMD_DRIVE, speed, steer)
        self._send(packet)

    def send_web_drive(self, left_speed, right_speed):
        """
        Send a direct Left/Right tank motor override command from Web UI.

        Args:
            left_speed:  -200 to 200 (FL & RL motors)
            right_speed: -200 to 200 (FR & RR motors)
        """
        packet = self._build_packet(CMD_WEB_DRIVE, left_speed, right_speed)
        self._send(packet)

    def send_stop(self):
        """Send emergency stop command."""
        packet = self._build_packet(CMD_STOP, 0, 0)
        self._send(packet)

    def send_query(self):
        """Request status from Arduino."""
        packet = self._build_packet(CMD_QUERY, 0, 0)
        self._send(packet)

    def _send(self, packet):
        """Send a packet over serial with thread safety."""
        if not self.connected or not self.ser:
            return
        with self.lock:
            try:
                self.ser.write(packet)
            except serial.SerialException as e:
                logger.error(f"Serial write error: {e}")
                self.connected = False

    # =========================================================================
    # Receiving Status
    # =========================================================================

    def _read_loop(self):
        """Background thread: continuously read status packets from Arduino."""
        buffer = bytearray()

        while self._running and self.connected:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)

                    # Try to parse complete packets from buffer
                    while len(buffer) >= STATUS_PACKET_SIZE:
                        # Look for start byte
                        start_idx = -1
                        for i in range(len(buffer)):
                            if buffer[i] == STATUS_START:
                                start_idx = i
                                break

                        if start_idx == -1:
                            buffer.clear()
                            break

                        # Discard bytes before start
                        if start_idx > 0:
                            buffer = buffer[start_idx:]

                        # Check if we have a complete packet
                        if len(buffer) < STATUS_PACKET_SIZE:
                            break

                        # Verify end byte
                        if buffer[STATUS_PACKET_SIZE - 1] != END_BYTE:
                            buffer = buffer[1:]  # skip this start byte, try next
                            continue

                        # Verify checksum (XOR of bytes 1 through 9)
                        checksum = 0
                        for i in range(1, STATUS_PACKET_SIZE - 2):
                            checksum ^= buffer[i]

                        if checksum != buffer[STATUS_PACKET_SIZE - 2]:
                            logger.debug("Status packet checksum mismatch")
                            buffer = buffer[1:]
                            continue

                        # Parse the valid packet
                        self._parse_status(buffer[:STATUS_PACKET_SIZE])
                        buffer = buffer[STATUS_PACKET_SIZE:]
                else:
                    time.sleep(0.005)  # 5ms sleep to prevent CPU spin

            except serial.SerialException as e:
                logger.error(f"Serial read error: {e}")
                self.connected = False
                break
            except Exception as e:
                logger.error(f"Read loop error: {e}")
                time.sleep(0.01)

    def _parse_status(self, packet):
        """
        Parse a status packet from Arduino.

        Packet format (12 bytes):
            [0xBB] [MODE] [CH1_H] [CH1_L] [CH2_H] [CH2_L]
            [CH3_H] [CH3_L] [CH5_H] [CH5_L] [CHECKSUM] [0x55]
        """
        mode = packet[1]

        # Parse channel values as signed int16 big-endian
        ch1 = struct.unpack('>h', bytes(packet[2:4]))[0]
        ch2 = struct.unpack('>h', bytes(packet[4:6]))[0]
        ch3 = struct.unpack('>h', bytes(packet[6:8]))[0]
        ch5 = struct.unpack('>h', bytes(packet[8:10]))[0]

        with self.status_lock:
            self.current_mode = mode
            self.rc_channels = [ch1, ch2, ch3, ch5]

    def get_mode(self):
        """Get current operating mode."""
        with self.status_lock:
            return self.current_mode

    def get_rc_channels(self):
        """Get latest RC channel values."""
        with self.status_lock:
            return list(self.rc_channels)

    def is_connected(self):
        """Check if serial connection is active."""
        return self.connected and self.ser is not None and self.ser.is_open

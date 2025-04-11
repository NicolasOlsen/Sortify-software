import serial
import struct
import time
from dataclasses import dataclass
from typing import Any, Optional
from enum import Enum
from crc16 import CRC16

import logging

# Setup logging
logger = logging.getLogger(__name__)

logger.propagate = False

if not logger.hasHandlers():
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
    logger.addHandler(handler)



from enum import Enum

class CommandCode(Enum):
	# High-Level Communication
	HEARTBEAT                = 0x01  # Master checks if Arduino is alive
	ACKNOWLEDGE              = 0x02  # Arduino acknowledges command

	# Servo Positioning
	REQUEST_SERVO_POSITIONS  = 0x03
	RESPOND_SERVO_POSITIONS  = 0x04

	SET_SERVO_POSITION       = 0x05  # Master sets a single servo position
	SET_ALL_POSITIONS        = 0x06  # Master sets all joint positions (excluding gripper)
	SET_SERVO_GOAL_VELOCITY  = 0x07  # Master sets goal velocity for single servo
	SET_ALL_GOAL_VELOCITY    = 0x08  # Master sets max velocity for all servos
	STOP_MOVEMENT            = 0x09  # Master stops servo movement

	# Error Handling
	REQUEST_ERROR_STATUS     = 0x0A
	RESPOND_ERROR_STATUS     = 0x0B
	COMMUNICATION_ERROR      = 0x0C  # Indicates a protocol-level issue

	# Internal / Meta Types
	COMMAND_RESPONSE         = 0xF0  # Generic command with no payload return
	BAD_CRC                  = 0xF1  # CRC mismatch
	TIMEOUT                  = 0xF2  # Response timeout (host side)
	UNKNOWN                  = 0xFF  # Invalid / unrecognized packet

class SystemStatus(Enum):
	NO_CONTACT = 0x00
	INITIALIZING = 0x01
	IDLE = 0x02
	MOVING = 0x03
	FAULT = 0x04

	def __str__(self):
		return self.name

class ComErrorCode(Enum):
	COMM_TIMEOUT = 0x01
	CHECKSUM_ERROR = 0x02
	UNKNOWN_COMMAND = 0x03
	BUFFER_OVERFLOW = 0x04
	QUEUE_FULL = 0x05
	INVALID_PAYLOAD_SIZE = 0x06
	ID_OUT_OF_RANGE = 0x07

	def __str__(self):
		return self.name


@dataclass
class ParsedPacketResult:
	packet_type: CommandCode
	command: int
	payload: bytes
	system_status: SystemStatus
	crc_ok: bool

	def __str__(self):
		return (f"[{self.packet_type.name}] Command=0x{self.command:02X}, "
				f"Payload={self.payload.hex(' ')}, Status=0x{self.system_status:02X}, "
				f"CRC={'OK' if self.crc_ok else 'FAIL'}")


@dataclass
class CommResponse:
	success: bool
	value: Any  # list[float], list[int], None, or ComErrorCode if failed
	system_status: SystemStatus


class PacketState(Enum):
	WAIT_FOR_START = 0
	WAIT_FOR_LENGTH = 1
	READ_REMAINING = 2


# ========== UART HANDLER ==========
class MasterUART:
	"""
    A UART communication handler for sending and receiving structured packets
    between a host (e.g., Raspberry Pi) and an Arduino-based servo controller.

    This class handles:
    - Constructing and sending command packets
    - Waiting for and parsing structured response packets
    - Error checking (CRC and timeouts)
    - Providing high-level APIs for specific robot actions

    Attributes:
        ser (serial.Serial): The serial interface.
        crc16 (CRC16): Instance of CRC16 checksum calculator.
        START_BYTES (bytes): Expected start bytes for every packet.
        read_timeout (float): Maximum time to wait for a full packet.
        last_packet (bytes): The last packet sent (for debug/resend if needed).
        MAX_RETRIES (int): Max number of attempts for sending a command.
    """

	def __init__(self, port, baudrate=1000000, timeout=0.1, read_timeout=1.0, start_bytes=b'\xAA\x55', max_retries=3):
		"""
        Initialize the UART handler and set up the serial port.

        Args:
            port (str): Serial port (e.g., 'COM3' or '/dev/ttyUSB0').
            baudrate (int): Baud rate for UART communication.
            timeout (float): PySerial internal byte read timeout (in seconds).
            read_timeout (float): Max time to wait for full response packet.
            start_bytes (bytes): Byte sequence that marks the start of a packet.
        """
		self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
		self.ser.setDTR(False)  # Prevent Arduino reset
		time.sleep(2)  # Let Arduino boot
		self.ser.reset_input_buffer()
		self.crc16 = CRC16()
		self.START_BYTES = start_bytes
		self.read_timeout = read_timeout
		self.last_packet = None
		self.MAX_RETRIES = max_retries


	def _build_packet(self, command, payload=b''):
		"""
        Build a complete packet with start bytes, length, command, payload, and CRC.

        Args:
            command (int): Command byte (from CommandCode).
            payload (bytes): Optional payload bytes.

        Returns:
            bytes: Fully constructed packet ready to send.
        """
		body = bytearray()
		body.append(len(payload) + 4)  # len = length + command + payload + CRC(2)
		body.append(command)
		body.extend(payload)

		crc = self.crc16.compute(body)
		body.append(crc & 0xFF)
		body.append((crc >> 8) & 0xFF)

		return self.START_BYTES + body


	def _send_command(self, command, payload=b'') -> ParsedPacketResult:
		"""
        Send a command and wait for a response, with retries on failure.

        Args:
            command (int): Command code.
            payload (bytes): Optional payload.

        Returns:
            ParsedPacketResult: Parsed response or communication error result.
        """
		packet = self._build_packet(command, payload)
		self.last_packet = packet

		for attempt in range(self.MAX_RETRIES):
			self.ser.write(packet)
			logger.info(f"[SEND] {packet.hex(' ')} (Attempt {attempt + 1})")

			raw = self._read_aligned_packet()

			if raw:
				result = self._parse_packet(raw)
				if result.crc_ok:
					return result
				else:
					logger.warning(f"CRC failed on attempt {attempt + 1}")
			else:
				logger.warning(f"No response on attempt {attempt + 1}")

		timeout_payload = struct.pack('<I', ComErrorCode.COMM_TIMEOUT.value)
		return ParsedPacketResult(
			packet_type=CommandCode.COMMUNICATION_ERROR,
			command=command,
			payload=timeout_payload,
			system_status=SystemStatus.NO_CONTACT,
			crc_ok=False,
		)



	def _read_aligned_packet(self):
		"""
        Wait for and read a full aligned packet using a state machine.

        Returns:
            bytes | None: Complete packet if received, else None on timeout.
        """
		packet = bytearray()
		length = 0
		start_time = time.time()
		start_len = len(self.START_BYTES)
		match_idx = 0

		state = PacketState.WAIT_FOR_START

		while time.time() - start_time < self.read_timeout:
			byte = self.ser.read(1)
			if not byte:
				continue

			b = byte[0]

			if state == PacketState.WAIT_FOR_START:
				if b == self.START_BYTES[match_idx]:
					packet.append(b)
					match_idx += 1
					if match_idx == start_len:
						state = PacketState.WAIT_FOR_LENGTH
				else:
					packet.clear()
					match_idx = 0

			elif state == PacketState.WAIT_FOR_LENGTH:
				length = b
				packet.append(b)
				state = PacketState.READ_REMAINING

			elif state == PacketState.READ_REMAINING:
				packet.append(b)
				if len(packet) == start_len + length:
					return bytes(packet)

		return None


	def _parse_packet(self, packet: bytes) -> ParsedPacketResult:
		"""
        Parses a received packet into its components and checks CRC.

        Args:
            packet (bytes): Raw received packet.

        Returns:
            ParsedPacketResult: Structured result containing payload, status, and type info.
        """
		start_len = len(self.START_BYTES)
		if not packet.startswith(self.START_BYTES):
			return ParsedPacketResult(CommandCode.UNKNOWN, 0, b'', 0, False)

		length = packet[start_len]
		command = packet[start_len + 1]

		status_code = packet[-3]
		try:
			system_status = SystemStatus(status_code)
		except ValueError:
			system_status = SystemStatus.FAULT

		payload = packet[start_len + 2:-3]

		received_crc = struct.unpack('<H', packet[-2:])[0]
		computed_crc = self.crc16.compute(packet[start_len:-2])
		crc_ok = received_crc == computed_crc

		if not crc_ok:
			return ParsedPacketResult(CommandCode.BAD_CRC, command, payload, system_status, False)

		if command == CommandCode.COMMUNICATION_ERROR.value:
			if len(payload) >= 4:
				code = struct.unpack('<I', payload[:4])[0]

			return ParsedPacketResult(
				packet_type=CommandCode.COMMUNICATION_ERROR,
				command=command,
				payload=payload,
				system_status=system_status,
				crc_ok=True
			)

		command_to_type = {
			CommandCode.ACKNOWLEDGE.value: CommandCode.ACKNOWLEDGE,
			CommandCode.RESPOND_SERVO_POSITIONS.value: CommandCode.RESPOND_SERVO_POSITIONS,
			CommandCode.RESPOND_ERROR_STATUS.value: CommandCode.RESPOND_ERROR_STATUS
		}

		packet_type = command_to_type.get(command, CommandCode.COMMAND_RESPONSE)

		return ParsedPacketResult(
			packet_type=packet_type,
			command=command,
			payload=payload,
			system_status=system_status,
			crc_ok=True
		)
	
	def _to_comm_response(self, result: ParsedPacketResult,	expected_type: CommandCode,
		unpack_fmt: Optional[str] = None) -> CommResponse:
		"""
        Converts a parsed packet into a CommResponse for high-level API.

        Args:
            result (ParsedPacketResult): Parsed response from device.
            expected_type (CommandCode): Expected type of response.
            unpack_fmt (str, optional): Struct format for payload unpacking (e.g., 'f' for floats).

        Returns:
            CommResponse: Result including success flag, data or error, and system status.
        """
		if result.packet_type == CommandCode.COMMUNICATION_ERROR:
			if len(result.payload) >= 4:
				code = struct.unpack('<I', result.payload[:4])[0]
				try:
					error_enum = ComErrorCode(code)
				except ValueError:
					error_enum = f"UNKNOWN_COM_ERROR_{code}"
			else:
				error_enum = "MALFORMED_COMM_ERROR"

			return CommResponse(False, error_enum, result.system_status)

		if not result.crc_ok or result.packet_type != expected_type:
			return CommResponse(False, result.packet_type, result.system_status)

		# Successful, optionally unpack
		value = None
		if unpack_fmt:
			size = struct.calcsize(unpack_fmt)
			count = len(result.payload) // size
			value = list(struct.unpack('<' + unpack_fmt * count, result.payload))

		return CommResponse(True, value, result.system_status)


	# High level API
	
	def heartbeat(self) -> CommResponse:
		"""
        Sends a heartbeat command to verify Arduino is alive.

        Returns:
            CommResponse: True if ACK received, else error type.
        """
		result = self._send_command(CommandCode.HEARTBEAT.value)
		return self._to_comm_response(result, CommandCode.ACKNOWLEDGE)

	def set_servo_position(self, servo_id: int, position: float) -> CommResponse:
		"""
        Sets the target position for a specific servo.

        Args:
            servo_id (int): Servo identifier (1–5).
            position (float): Desired angle/position in degrees or radians.

        Returns:
            CommResponse: Result of the command.
        """
		payload = bytes([servo_id]) + struct.pack('<f', position)
		result = self._send_command(CommandCode.SET_SERVO_POSITION.value, payload)
		return self._to_comm_response(result, CommandCode.ACKNOWLEDGE)

	def set_all_positions(self, positions: list[float]) -> CommResponse:
		"""
        Sets target positions for all servos (except gripper).

        Args:
            positions (list[float]): List of 4 floats representing each joint.

        Raises:
            ValueError: If the number of positions is not 4.

        Returns:
            CommResponse: Result of the command.
        """
		if len(positions) != 4:
			raise ValueError("Expected 4 joint positions")
		payload = struct.pack('<4f', *positions)
		result = self._send_command(CommandCode.SET_ALL_POSITIONS.value, payload)
		return self._to_comm_response(result, CommandCode.ACKNOWLEDGE)

	def set_servo_velocity(self, servo_id: int, velocity: float) -> CommResponse:
		"""
        Sets the goal velocity for a specific servo.

        Args:
            servo_id (int): Servo identifier.
            velocity (float): Velocity in degrees/s or unit/s.

        Returns:
            CommResponse: Result of the command.
        """
		payload = bytes([servo_id]) + struct.pack('<f', velocity)
		result = self._send_command(CommandCode.SET_SERVO_GOAL_VELOCITY.value, payload)
		return self._to_comm_response(result, CommandCode.ACKNOWLEDGE)

	def set_all_velocities(self, velocities: list[float]) -> CommResponse:
		"""
        Sets velocities for all servos.

        Args:
            velocities (list[float]): 4 values for each servo (excluding gripper).

        Raises:
            ValueError: If the list does not contain 4 values.

        Returns:
            CommResponse: Result of the command.
        """
		if len(velocities) != 4:
			raise ValueError("Expected 4 velocity values")
		payload = struct.pack('<4f', *velocities)
		result = self._send_command(CommandCode.SET_ALL_GOAL_VELOCITY.value, payload)
		return self._to_comm_response(result, CommandCode.ACKNOWLEDGE)

	def stop_movement(self) -> CommResponse:
		"""
        Halts all servo movements immediately.

        Returns:
            CommResponse: Result of the command.
        """
		result = self._send_command(CommandCode.STOP_MOVEMENT.value)
		return self._to_comm_response(result, CommandCode.ACKNOWLEDGE)

	def get_servo_positions(self) -> CommResponse:
		"""
        Requests the current positions of all joints (excluding gripper).

        Returns:
            CommResponse: List of floats if successful, else error.
        """
		result = self._send_command(CommandCode.REQUEST_SERVO_POSITIONS.value)
		return self._to_comm_response(result, CommandCode.RESPOND_SERVO_POSITIONS, unpack_fmt='f')

	def get_error_report(self) -> CommResponse:
		"""
        Requests the current communication or hardware error report.

        Returns:
            CommResponse: List of uint32 error codes if successful.
        """
		result = self._send_command(CommandCode.REQUEST_ERROR_STATUS.value)
		return self._to_comm_response(result, CommandCode.RESPOND_ERROR_STATUS, unpack_fmt='I')


	def close(self):
		self.ser.close()

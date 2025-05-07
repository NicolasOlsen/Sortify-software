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
	# Status and Control
	PING       		= 0x01
	NACK            = 0x02

	# Position
	READ_POSITION_RANGE     = 0x03
	WRITE_POSITION_RANGE    = 0x04
	STOP_MOVEMENT			= 0x05

	# Velocity
	WRITE_VELOCITY_RANGE    = 0x06

	# Error
	READ_CURRENT_ERROR_RANGE  	= 0x07
	READ_LAST_ERROR_RANGE  		= 0x08

	# Internal / Meta
	COMMAND_RESPONSE        = 0xF0
	BAD_CRC                 = 0xF1
	TIMEOUT                 = 0xF2
	UNKNOWN                 = 0xFF


class SystemStatus(Enum):
	NO_CONTACT = 0x00
	INITIALIZING = 0x01
	IDLE = 0x02
	MOVING = 0x03
	FAULT_INIT = 0x04
	FAULT_RUNTIME = 0x05

	def __str__(self):
		return self.name

class ComErrorCode(Enum):
	SYSTEM_FAULT = 0x01
	COMM_TIMEOUT = 0x02
	CHECKSUM_ERROR = 0x03
	UNKNOWN_COMMAND = 0x04
	INVALID_PAYLOAD_SIZE = 0x05
	BUFFER_OVERFLOW = 0x06
	QUEUE_FULL = 0x07
	ID_OUT_OF_RANGE = 0x08
	POSITION_OUT_OF_RANGE = 0x09

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
		time.sleep(1)  # Let Arduino boot
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
		body.append(len(payload) + 4)  # len = command(1) + payload + system_status(1) + CRC(2)
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
			self.ser.reset_input_buffer()
			self.ser.write(packet)
			logger.info(f"[SEND] {packet.hex(' ')} (Attempt {attempt + 1})")

			raw = self._read_aligned_packet()

			if raw:
				result = self._parse_packet(raw)
				if result.crc_ok:
					logger.info(f"[RECEIVED] {raw.hex(' ')}")
					return result
				else:
					logger.warning(f"CRC failed on attempt {attempt + 1}")
			else:
				logger.warning(f"No response on attempt {attempt + 1}")

		timeout_payload = struct.pack('<I', ComErrorCode.COMM_TIMEOUT.value)
		return ParsedPacketResult(
			packet_type=CommandCode.NACK,
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

		if command == CommandCode.NACK.value:
			return ParsedPacketResult(
				packet_type=CommandCode.NACK,
				command=command,
				payload=payload,
				system_status=system_status,
				crc_ok=True
			)

		try:
			packet_type = CommandCode(command)
		except ValueError:
			packet_type = CommandCode.COMMAND_RESPONSE
			logger.warning(f"Unknown command code: 0x{command:02X}")

		return ParsedPacketResult(
			packet_type=packet_type,
			command=command,
			payload=payload,
			system_status=system_status,
			crc_ok=True
		)
	
	def _to_comm_response(self, result: ParsedPacketResult, expected_type: CommandCode, unpack_fmt: Optional[str] = None) -> CommResponse:
		# FIRST handle NACK
		if result.packet_type == CommandCode.NACK:
			if result.payload:
				code = result.payload[0]
				try:
					error_enum = ComErrorCode(code)
				except ValueError:
					error_enum = f"UNKNOWN_COM_ERROR_{code}"
			else:
				error_enum = "MALFORMED_COMM_ERROR"

			return CommResponse(False, error_enum, result.system_status)

		# THEN check for CRC or unexpected command
		if not result.crc_ok or result.packet_type != expected_type:
			return CommResponse(False, result.packet_type, result.system_status)

		# Otherwise, parse the payload
		value = None
		if unpack_fmt:
			size = struct.calcsize(unpack_fmt)
			count = len(result.payload) // size
			value = list(struct.unpack('<' + unpack_fmt * count, result.payload))

		return CommResponse(True, value, result.system_status)


	# High level API
	
	def ping(self) -> CommResponse:
		"""
		Sends a ping command to verify Arduino is alive.

		Returns:
			CommResponse: True if ACK received, else error type.
		"""
		result = self._send_command(CommandCode.PING.value)
		return self._to_comm_response(result, CommandCode.PING)

	def write_position_range(self, start_id: int, values: list[float]) -> CommResponse:
		"""
		Sends target positions for a range of servos starting at start_id.

		Args:
			start_id (int): The starting servo ID.
			values (list[float]): List of float positions to set.

		Returns:
			CommResponse: True if ACK received, else error.
		"""
		payload = bytes([start_id, len(values)]) + struct.pack(f'<{len(values)}f', *values)
		result = self._send_command(CommandCode.WRITE_POSITION_RANGE.value, payload)
		return self._to_comm_response(result, CommandCode.WRITE_POSITION_RANGE)

	def read_position_range(self, start_id: int, count: int) -> CommResponse:
		"""
		Requests current positions of a range of servos.

		Args:
			start_id (int): Starting servo ID.
			count (int): Number of servos to read.

		Returns:
			CommResponse: List of float positions or error.
		"""
		payload = bytes([start_id, count])
		result = self._send_command(CommandCode.READ_POSITION_RANGE.value, payload)
		return self._to_comm_response(result, CommandCode.READ_POSITION_RANGE, unpack_fmt='f')
	
	def stop_movement(self) -> CommResponse:
		"""
		Sends a stop command to stop every smart servo.

		Returns:
			CommResponse: True if ACK received, else error type.
		"""
		result = self._send_command(CommandCode.STOP_MOVEMENT.value)
		return self._to_comm_response(result, CommandCode.STOP_MOVEMENT)

	def write_velocity_range(self, start_id: int, values: list[float]) -> CommResponse:
		"""
		Sets velocities for a range of servos starting at start_id.

		Args:
			start_id (int): The starting servo ID.
			values (list[float]): List of float velocities to set.

		Returns:
			CommResponse: True if ACK received, else error.
		"""
		payload = bytes([start_id, len(values)]) + struct.pack(f'<{len(values)}f', *values)
		result = self._send_command(CommandCode.WRITE_VELOCITY_RANGE.value, payload)
		return self._to_comm_response(result, CommandCode.WRITE_VELOCITY_RANGE)

	def read_current_error_range(self, start_id: int, count: int) -> CommResponse:
		"""
		Requests current error codes from a range of servos.

		Args:
			start_id (int): Starting servo ID.
			count (int): Number of servos to query.

		Returns:
			CommResponse: List of uint32 error flags or error.
		"""
		payload = bytes([start_id, count])
		result = self._send_command(CommandCode.READ_CURRENT_ERROR_RANGE.value, payload)
		return self._to_comm_response(result, CommandCode.READ_CURRENT_ERROR_RANGE, unpack_fmt='I')
	
	def read_last_error_range(self, start_id: int, count: int) -> CommResponse:
		"""
		Requests last error codes from a range of servos.

		Args:
			start_id (int): Starting servo ID.
			count (int): Number of servos to query.

		Returns:
			CommResponse: List of uint32 error flags or error.
		"""
		payload = bytes([start_id, count])
		result = self._send_command(CommandCode.READ_LAST_ERROR_RANGE.value, payload)
		return self._to_comm_response(result, CommandCode.READ_LAST_ERROR_RANGE, unpack_fmt='I')

	def close(self):
		self.ser.close()

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
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
formatter = logging.Formatter('[%(levelname)s] %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)


class PacketType(Enum):
	ACKNOWLEDGE = 0x02      	# Generic ACK response
	SERVO_POSITIONS = 0x04     	# Response with servo positions
	ERROR_REPORT = 0x0B        	# Response with error codes
	COMMUNICATION_ERROR = 0x0C  # Communication issue detected

	# These are meta-types, not directly from the command
	COMMAND_RESPONSE = 0xF0   	# Generic data response (e.g. to SET commands)
	BAD_CRC = 0xF1              # CRC failed
	TIMEOUT = 0xF2				# Response timeout	
	UNKNOWN = 0xFF              # Unknown packet / invalid start

class ComErrorCode(Enum):
    COMM_TIMEOUT = 0x01
    CHECKSUM_ERROR = 0x02
    UNKNOWN_COMMAND = 0x03
    BUFFER_OVERFLOW = 0x04
    QUEUE_FULL = 0x05
    INVALID_PAYLOAD_SIZE = 0x06
    ID_OUT_OF_RANGE = 0x07


@dataclass
class ParsedPacketResult:
	packet_type: PacketType
	command: int
	payload: bytes
	system_status: int
	crc_ok: bool
	error_type: Optional[ComErrorCode] = None  # None unless COMM_ERROR

	def __str__(self):
		error_str = f", Error={self.error_type.name}" if self.error_type else ""
		return (f"[{self.packet_type.name}] Command=0x{self.command:02X}, "
				f"Payload={self.payload.hex(' ')}, Status=0x{self.system_status:02X}, "
				f"CRC={'OK' if self.crc_ok else 'FAIL'}{error_str}")


@dataclass
class CommResponse:
    success: bool
    value: Any  # could be list[float], list[int], None, etc.
    system_status: int
    error_type: Optional[ComErrorCode] = None  # None if no error


class PacketState(Enum):
	WAIT_FOR_START = 0
	WAIT_FOR_LENGTH = 1
	READ_REMAINING = 2


# ========== UART HANDLER ==========
class MasterUART:
	def __init__(self, port, baudrate=1000000, timeout=0.1, start_bytes=b'\xAA\x55'):
		self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
		self.ser.setDTR(False)  # Prevent Arduino reset
		time.sleep(2)  # Let Arduino boot
		self.ser.reset_input_buffer()
		self.crc16 = CRC16()
		self.START_BYTES = start_bytes
		self.last_packet = None
		self.MAX_RETRIES = 3


	def _build_packet(self, command, payload=b''):
		body = bytearray()
		body.append(len(payload) + 4)  # length = length byte + command + payload + CRC(2)
		body.append(command)
		body.extend(payload)

		crc = self.crc16.compute(body)
		body.append(crc & 0xFF)
		body.append((crc >> 8) & 0xFF)

		return self.START_BYTES + body


	def send_command(self, command, payload=b'') -> ParsedPacketResult:
		packet = self.build_packet(command, payload)
		self.last_packet = packet

		for attempt in range(self.MAX_RETRIES):
			self.ser.write(packet)
			logger.info(f"[SEND] {packet.hex(' ')} (Attempt {attempt + 1})")

			time.sleep(0.5)
			raw = self.read_aligned_packet()

			if raw:
				result = self._parse_packet(raw)
				if result.crc_ok:
					return result
				else:
					logger.warning(f"CRC failed on attempt {attempt + 1}")
			else:
				logger.warning(f"No response on attempt {attempt + 1}")

		# After retries exhausted: simulate COMMUNICATION_ERROR with COMM_TIMEOUT
		timeout_payload = struct.pack('<I', ComErrorCode.COMM_TIMEOUT.value)
		return ParsedPacketResult(
			packet_type=PacketType.COMMUNICATION_ERROR,
			command=command,
			payload=timeout_payload,
			system_status=0,
			crc_ok=False,
			error_type=ComErrorCode.COMM_TIMEOUT
		)


	def _read_aligned_packet(self):
		packet = bytearray()
		length = 0
		timeout_total = 2.0
		start_time = time.time()
		start_len = len(self.START_BYTES)
		match_idx = 0

		state = PacketState.WAIT_FOR_START

		while time.time() - start_time < timeout_total:
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
		start_len = len(self.START_BYTES)
		if not packet.startswith(self.START_BYTES):
			return ParsedPacketResult(PacketType.UNKNOWN, 0, b'', 0, False, packet)

		length = packet[start_len]
		command = packet[start_len + 1]
		system_status = packet[-3]  # second last byte before CRC
		payload = packet[start_len + 2:-3]

		received_crc = struct.unpack('<H', packet[-2:])[0]
		computed_crc = self.crc16.compute(packet[start_len:-2])
		crc_ok = received_crc == computed_crc

		if not crc_ok:
			return ParsedPacketResult(PacketType.BAD_CRC, command, payload, system_status, False, packet)

		# Check for COMMUNICATION_ERROR first
		if command == 0x0C:
			if len(payload) >= 4:
				code = struct.unpack('<I', payload[:4])[0]
				try:
					error_type = ComErrorCode(code)
				except ValueError:
					error_type = f"UNKNOWN_COM_ERROR_{code}"
			else:
				error_type = "MALFORMED_COMM_ERROR"

			return ParsedPacketResult(
				packet_type=PacketType.COMMUNICATION_ERROR,
				command=command,
				payload=payload,
				system_status=system_status,
				crc_ok=True,
				error_type=error_type
			)

		# Default: use packet type map
		command_to_type = {
			0x02: PacketType.ACKNOWLEDGE,
			0x04: PacketType.SERVO_POSITIONS,
			0x0B: PacketType.ERROR_REPORT
		}

		packet_type = command_to_type.get(command, PacketType.COMMAND_RESPONSE)

		return ParsedPacketResult(
			packet_type=packet_type,
			command=command,
			payload=payload,
			system_status=system_status,
			crc_ok=True,
			error_type=packet_type  # Use packet type if not communication error
		)

	
	def heartbeat(self) -> CommResponse:
		result = self._send_command(0x01)
		return CommResponse(
			success=result.crc_ok and result.packet_type == PacketType.ACKNOWLEDGE,
			value=None,
			system_status=result.system_status,
			error_type=result.packet_type,
		)

	def set_servo_position(self, servo_id: int, position: float) -> CommResponse:
		payload = bytes([servo_id]) + struct.pack('<f', position)
		result = self._send_command(0x05, payload)
		return CommResponse(
			success=result.crc_ok and result.packet_type == PacketType.ACKNOWLEDGE,
			value=None,
			system_status=result.system_status,
			error_type=result.packet_type,
		)

	def set_all_positions(self, positions: list[float]) -> CommResponse:
		if len(positions) != 4:
			raise ValueError("Expected 4 joint positions")
		payload = struct.pack('<4f', *positions)
		result = self._send_command(0x06, payload)
		return CommResponse(
			success=result.crc_ok and result.packet_type == PacketType.ACKNOWLEDGE,
			value=None,
			system_status=result.system_status,
			error_type=result.packet_type,
		)

	def set_servo_velocity(self, servo_id: int, velocity: float) -> CommResponse:
		payload = bytes([servo_id]) + struct.pack('<f', velocity)
		result = self._send_command(0x07, payload)
		return CommResponse(
			success=result.crc_ok and result.packet_type == PacketType.ACKNOWLEDGE,
			value=None,
			system_status=result.system_status,
			error_type=result.packet_type,
		)

	def set_all_velocities(self, velocities: list[float]) -> CommResponse:
		if len(velocities) != 4:
			raise ValueError("Expected 4 velocity values")
		payload = struct.pack('<4f', *velocities)
		result = self._send_command(0x08, payload)
		return CommResponse(
			success=result.crc_ok and result.packet_type == PacketType.ACKNOWLEDGE,
			value=None,
			system_status=result.system_status,
			error_type=result.packet_type,
		)

	def stop_movement(self) -> CommResponse:
		result = self._send_command(0x09)
		return CommResponse(
			success=result.crc_ok and result.packet_type == PacketType.ACKNOWLEDGE,
			value=None,
			system_status=result.system_status,
			error_type=result.packet_type,
		)

	def get_servo_positions(self) -> CommResponse:
		result = self._send_command(0x03)
		positions = []
		if result.crc_ok and result.packet_type == PacketType.SERVO_POSITIONS:
			count = len(result.payload) // 4
			positions = list(struct.unpack('<' + 'f' * count, result.payload))
		return CommResponse(
			success=result.crc_ok and result.packet_type == PacketType.SERVO_POSITIONS,
			value=positions,
			system_status=result.system_status,
			error_type=result.packet_type,
		)

	def get_error_report(self) -> CommResponse:
		result = self._send_command(0x0A)
		errors = []
		if result.crc_ok and result.packet_type == PacketType.ERROR_REPORT:
			count = len(result.payload) // 4
			errors = list(struct.unpack('<' + 'I' * count, result.payload))
		return CommResponse(
			success=result.crc_ok and result.packet_type == PacketType.ERROR_REPORT,
			value=errors,
			system_status=result.system_status,
			error_type=result.packet_type,
		)


	def close(self):
		self.ser.close()


def test_all_commands(uart: MasterUART):
    print("\n--- HEARTBEAT ---")
    resp = uart.heartbeat()
    print(f"Heartbeat OK: {resp.success}, Status: 0x{resp.system_status:02X}, Type: {resp.error_type.name}")

    print("\n--- SET SERVO POSITION (Servo 1, Pos 123.456) ---")
    resp = uart.set_servo_position(1, 123.456)
    print(f"Set Servo OK: {resp.success}, Status: 0x{resp.system_status:02X}, Type: {resp.error_type.name}")

    print("\n--- SET ALL POSITIONS ---")
    resp = uart.set_all_positions([1.0, 2.0, 3.0, 4.0])
    print(f"Set All Pos OK: {resp.success}, Status: 0x{resp.system_status:02X}, Type: {resp.error_type.name}")

    print("\n--- SET SERVO VELOCITY (Servo 1, Vel 5.5) ---")
    resp = uart.set_servo_velocity(1, 5.5)
    print(f"Set Velocity OK: {resp.success}, Status: 0x{resp.system_status:02X}, Type: {resp.error_type.name}")

    print("\n--- SET ALL VELOCITIES ---")
    resp = uart.set_all_velocities([10.2, 12.4, 54.7, 1.1])
    print(f"Set All Vel OK: {resp.success}, Status: 0x{resp.system_status:02X}, Type: {resp.error_type.name}")

    print("\n--- STOP MOVEMENT ---")
    resp = uart.stop_movement()
    print(f"Stop Movement OK: {resp.success}, Status: 0x{resp.system_status:02X}, Type: {resp.error_type.name}")

    print("\n--- GET SERVO POSITIONS ---")
    resp = uart.get_servo_positions()
    if resp.success:
        print(f"Positions: {resp.value}, Status: 0x{resp.system_status:02X}")
    else:
        print(f"Failed to get positions, Status: 0x{resp.system_status:02X}, Type: {resp.error_type.name}")

    print("\n--- GET ERROR REPORT ---")
    resp = uart.get_error_report()
    if resp.success:
        print(f"Errors: {resp.value}, Status: 0x{resp.system_status:02X}")
    else:
        print(f"Failed to get errors, Status: 0x{resp.system_status:02X}, Type: {resp.error_type.name}")



# ========== MAIN ==========
if __name__ == "__main__":
	uart = MasterUART("COM3", 1000000, start_bytes=b'\xAA\x55')

	try:
		test_all_commands(uart)
	except KeyboardInterrupt:
		print("Interrupted")
	finally:
		uart.close()

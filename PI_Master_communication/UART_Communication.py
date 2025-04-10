import serial
import struct
import time
from crc16 import CRC16


# ========== CONFIGURATION ==========
SERIAL_PORT = "COM3"  # Adjust this to your port
BAUD_RATE = 1000000
TIMEOUT = 0.01
START_BYTES = b'\xAA\x55'


# ========== UART HANDLER ==========
class MasterUART:
	def __init__(self, port, baudrate):
		self.ser = serial.Serial(port, baudrate=baudrate, timeout=TIMEOUT)
		self.ser.setDTR(False)  # Prevent Arduino reset
		time.sleep(2)  # Let Arduino boot
		self.ser.reset_input_buffer()
		self.crc16 = CRC16()

	def build_packet(self, command, payload=b''):
		body = bytearray()
		body.append(len(payload) + 4)  # length = length byte + command + payload + CRC(2)
		body.append(command)
		body.extend(payload)

		crc = self.crc16.compute(body)
		body.append(crc & 0xFF)
		body.append((crc >> 8) & 0xFF)

		return START_BYTES + body

	def send_command(self, command, payload=b''):
		packet = self.build_packet(command, payload)
		print(f"[SEND] {packet.hex(' ')}")
		self.ser.write(packet)

		time.sleep(0.5)
		response = self.read_aligned_packet()
		if response:
			self.parse_packet(response)
		else:
			print("[WARN] No valid response")

	def read_aligned_packet(self):
		WAIT_FOR_START = 0
		WAIT_FOR_SECOND_START = 1
		WAIT_FOR_LENGTH = 2
		READ_REMAINING = 3

		state = WAIT_FOR_START
		packet = bytearray()
		length = 0
		timeout_total = 2.0
		start_time = time.time()

		while time.time() - start_time < timeout_total:
			byte = self.ser.read(1)
			if not byte:
				continue

			b = byte[0]

			if state == WAIT_FOR_START:
				if b == 0xAA:
					packet = bytearray([b])
					state = WAIT_FOR_SECOND_START
				continue

			elif state == WAIT_FOR_SECOND_START:
				if b == 0x55:
					packet.append(b)
					state = WAIT_FOR_LENGTH
				else:
					state = WAIT_FOR_START
				continue

			elif state == WAIT_FOR_LENGTH:
				length = b
				packet.append(b)
				state = READ_REMAINING
				continue

			elif state == READ_REMAINING:
				packet.append(b)
				if len(packet) == 2 + length:
					return bytes(packet)

		return None

	def parse_packet(self, packet: bytes):
		if not packet.startswith(START_BYTES):
			return

		length = packet[2]
		command = packet[3]
		payload = packet[4:-2]
		received_crc = struct.unpack('<H', packet[-2:])[0]
		computed_crc = self.crc16.compute(packet[2:-2])

		if received_crc != computed_crc:
			print("[CRC ERROR] CRC mismatch")
			return

		self.parse_payload(command, payload)

	def parse_payload(self, command, payload: bytes):
		if not payload:
			return

		# Commands returning floats
		if command in (0x05, 0x06, 0x07, 0x08):
			num_floats = len(payload) // 4
			floats = struct.unpack('<' + 'f' * num_floats, payload[:num_floats * 4])
			print(f"[RECV FLOATS] {floats}")

		# Commands returning uint32 (e.g., errors)
		elif command == 0x0A:
			num_ints = len(payload) // 4
			ints = struct.unpack('<' + 'I' * num_ints, payload[:num_ints * 4])
			print(f"[RECV UINT32] {ints}")

		else:
			print(f"[RECV RAW] cmd=0x{command:02X} payload={payload.hex(' ')}")

	def close(self):
		self.ser.close()


# ========== COMMAND TEST ==========
def test_all_commands(uart: MasterUART):
	float_val = 123.456
	float_array = [1.0, 2.0, 3.0, 4.0]

	commands_to_test = [
		(0x01, b''),  # HEARTBEAT
		(0x03, b''),  # REQUEST_SERVO_POSITIONS
		(0x05, b'\x01' + struct.pack('<f', float_val)),  # SET_SERVO_POSITION
		(0x06, struct.pack('<4f', *float_array)),  # SET_ALL_POSITIONS
		(0x07, struct.pack('<f', 5.5)),  # SET_SERVO_GOAL_VELOCITY
		(0x08, struct.pack('<4f', *float_array)),  # SET_ALL_GOAL_VELOCITY
		(0x09, b''),  # STOP_MOVEMENT
		(0x0A, b''),  # REQUEST_ERROR_STATUS
	]

	for command, payload in commands_to_test:
		print(f"\n--- Sending Command 0x{command:02X} ---")
		uart.send_command(command, payload)
		time.sleep(1)


# ========== MAIN ==========
if __name__ == "__main__":
	uart = MasterUART(SERIAL_PORT, BAUD_RATE)

	try:
		test_all_commands(uart)
	except KeyboardInterrupt:
		print("Interrupted")
	finally:
		uart.close()

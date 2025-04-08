import serial
import struct
import time

from crc16 import CRC16


# ========== CONFIGURATION ==========
SERIAL_PORT = "/dev/ttyACM0"  # Adjust if needed
BAUD_RATE = 1000000
TIMEOUT = 0.2  # Read timeout

START_BYTES = b'\xAA\x55'


# ========== MAIN COMMUNICATION ==========

class MasterUART:
	def __init__(self, port, baudrate):
		self.ser = serial.Serial(port, baudrate=baudrate, timeout=TIMEOUT)
		self.crc16 = CRC16()

	def build_packet(self, command, payload=b''):
		body = bytearray()
		body.append(len(payload) + 4)  # length = length + command + payload + CRC(2)
		body.append(command)
		body.extend(payload)

		# CRC now computed correctly
		crc = self.crc16.compute(body)
		body.append(crc & 0xFF)
		body.append((crc >> 8) & 0xFF)

		return START_BYTES + body

	def send_command(self, command, payload=b''):
		packet = self.build_packet(command, payload)
		print(f"[SEND] {packet.hex(' ')}")
		self.ser.write(packet)

		# Attempt to read response
		time.sleep(0.05)
		response = self.ser.read(64)
		if response:
			print(f"[RECV] {response.decode(errors='replace')}")
		else:
			print("[WARN] No response")

	def close(self):
		self.ser.close()

# ========== COMMAND TEST SEQUENCE ==========

def test_all_commands(uart: MasterUART):
	commands_to_test = [
		(0x01, b''),                          # REQUEST_STATUS
		(0x04, b''),                          # HEARTBEAT
		(0x05, b''),                          # REQUEST_SERVO_POSITIONS
		(0x07, b'\x01'),                      # REQUEST_CURRENT_SPEED (servo ID 1)
		(0x09, b'\x01' + struct.pack('<I', 1000000)),  # SET_SERVO_POSITION (servo ID 1, position 1,000,000)
		(0x0A, struct.pack('<4I', 1000000, 1000000, 1000000, 1000000)),  # SET_ALL_POSITIONS
		(0x0B, struct.pack('<B', 3)),         # SET_MAX_SPEED = 3
		(0x0C, b'\x01\x01'),                  # SYSTEM_CONTROL STOP servo 1
		(0x0D, b''),                          # REQUEST_ERROR_STATUS
		(0x0F, b'\x0C')                       # COMMUNICATION_ERROR with dummy error
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

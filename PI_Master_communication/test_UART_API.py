from UART_Communication import MasterUART, SystemStatus
import logging
import time

# Optional: Enable debug-level logging
logging.basicConfig(level=logging.INFO)


def print_result(name, response):
	print(f"\n--- {name} ---")
	print(f"Success: {response.success}")
	print(f"System Status: {response.system_status}")
	print(f"Data: {response.value}")


def main():
	uart = MasterUART(
		port="COM3",
		baudrate=1000000,
		timeout=0.01,
		read_timeout=0.03,
		start_bytes=b'\xAA\x55'
	)

	try:
		# PING
		print_result("PING", uart.ping())

		# result = uart._send_command(9)
		# print_result("PING", uart._to_comm_response(result, 9))

		# # Write velocities to servos 0-4
		# print_result("WRITE_VELOCITY_RANGE", uart.write_velocity_range(0, [30, 30, 30, 30]))

		# # Write positions to servos 0-4
		print_result("WRITE_POSITION_RANGE", uart.write_position_range(0, [180, 180]))

		# Read positions back from servos 0-4
		# print_result("READ_POSITION_RANGE", uart.read_position_range(0, 2))

		# Read error codes from servos 0-4
		# print_result("READ_CURRENT_ERROR_RANGE", uart.read_current_error_range(0, 4))
		# print_result("READ_LAST_ERROR_RANGE", uart.read_last_error_range(0, 4))

		# time.sleep(1.5)

		# print_result("STOP_MOVEMENT", uart.stop_movement())

		# status = SystemStatus.MOVING
		# while status == SystemStatus.MOVING:
		# 	res = uart.read_position_range(0, 4)
		# 	status = res.system_status
		# 	print_result("MOVING", res)

		# for i in range(5):
		#     print_result("WRITE_POSITION_RANGE", uart.write_position_range(0, [270, 270, 270, 270, 130]))

		#     time.sleep(3)

		#     print_result("WRITE_POSITION_RANGE", uart.write_position_range(0, [180, 180, 180, 180, 0]))

		#     time.sleep(3)

	except Exception as e:
		print(f"[EXCEPTION] {e}")

	finally:
		uart.close()


if __name__ == "__main__":
	main()

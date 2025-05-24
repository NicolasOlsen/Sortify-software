from UART_Communication import MasterUART, SystemStatus
import logging
import time

# Optional: Enable debug-level logging
# logging.basicConfig(level=logging.INFO)

def calculate_synchronized_velocities(current_pos, target_pos, max_velocity):
	# Find the max distance a servo has to travel
	distances = [abs(t - c) for t, c in zip(target_pos, current_pos)]
	max_distance = max(distances)
	
	if max_distance == 0:
		return [0] * len(current_pos)
	
	# Calculate the time for the max distance
	time_needed = max_distance / max_velocity
	
	# Calculate the velocity for each servo, to arrive at the same time
	velocities = [round(d / time_needed) for d in distances]
	return velocities

def set_sync_position(uart, target_pos, start_pos = None, max_velocity = 30):
	if not start_pos:
		res = uart.read_position_range(0, 4)
		if res.success:
			start_pos = res.value

	sync_vel = calculate_synchronized_velocities(start_pos, target_pos, max_velocity)

	uart.write_velocity_range(0, sync_vel)
	uart.write_position_range(0, target_pos)

def print_result(name, response):
	print(f"\n--- {name} ---")
	print(f"Success: {response.success}")
	print(f"System Status: {response.system_status}")
	print(f"Data: {response.value}")

def busyWait(uart, timeout=5.0):
	start_time = time.time()
	time.sleep(0.7)  # Initial wait for movement to start

	status = SystemStatus.MOVING
	while status == SystemStatus.MOVING:	# Will loop until the motorcontroller is not in MOVING state
		if time.time() - start_time > timeout:
			print("[WARNING] busyWait() timed out after {:.1f} seconds.".format(timeout))
			break
		res = uart.ping()
		status = res.system_status


def main():
	uart = MasterUART(
		port="COM3",
		baudrate=1000000,
		timeout=0.01,
		read_timeout=0.03,
		start_bytes=b'\xAA\x55'
	)

	try:
		start_pos = [90, 180, 130, 90, 90]

		set_sync_position(uart, start_pos, max_velocity=40)

		busyWait(uart)

		target_pos = [31, 119, 168, 88, 90]

		set_sync_position(uart, target_pos, max_velocity=30)

		busyWait(uart, 3)

		target_pos = [31, 119, 152, 105]

		set_sync_position(uart, target_pos, max_velocity=15)

		busyWait(uart)

		uart.write_position_range(4, [0])

		time.sleep(1)

		target_pos = [31, 120, 170, 83]

		set_sync_position(uart, target_pos, max_velocity=15)

		time.sleep(1.5)

		target_pos = [268, 155, 153, 104]

		set_sync_position(uart, target_pos, max_velocity=40)

		busyWait(uart, timeout=10)

		print_result("WRITE_POSITION_RANGE", uart.write_position_range(4, [90]))

		set_sync_position(uart, start_pos, max_velocity=40)

		## Next ball

		# print_result("WRITE_VELOCITY_RANGE", uart.write_velocity_range(0, [15, 15, 15, 15]))
		# print_result("WRITE_POSITION_RANGE", uart.write_position_range(0, [154, 120, 170, 83]))
		# print_result("WRITE_POSITION_RANGE", uart.write_position_range(0, [154, 118, 151, 105]))
		# print_result("WRITE_POSITION_RANGE", uart.write_position_range(4, [0]))

		# print_result("WRITE_POSITION_RANGE", uart.write_position_range(0, [154, 120, 170, 83]))

		# print_result("WRITE_POSITION_RANGE", uart.write_position_range(0, [154, 120, 170, 83]))

	except Exception as e:
		print(f"[EXCEPTION] {e}")

	finally:
		uart.close()


if __name__ == "__main__":
	main()

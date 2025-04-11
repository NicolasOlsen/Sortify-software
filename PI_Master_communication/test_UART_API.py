from UART_Communication import MasterUART, CommandCode
import logging
import time

# Optional: Enable debug-level logging from the API
logging.basicConfig(level=logging.INFO)

def print_result(name, response):
    print(f"\n--- {name} ---")
    print(f"Success: {response.success}")
    print(f"Status: {response.system_status}")
    print(f"Data: {response.value}")

def main():
    uart = MasterUART(port="COM3", baudrate=1000000, timeout=0.01, read_timeout=0.5, start_bytes=b'\xAA\x55')

    try:
        # Heartbeat test
        print_result("HEARTBEAT", uart.heartbeat())

        # Set single servo position (Servo 1 to 123.456)
        print_result("SET SERVO POSITION", uart.set_servo_position(1, 123.456))

        # Set all servo positions
        positions = [1.0, 2.0, 3.0, 4.0]
        print_result("SET ALL POSITIONS", uart.set_all_positions(positions))

        # Set velocity for a single servo (Servo 1 to 5.5)
        print_result("SET SERVO VELOCITY", uart.set_servo_velocity(1, 5.5))

        # Set velocity for all servos
        velocities = [10.2, 12.4, 54.7, 1.1]
        print_result("SET ALL VELOCITIES", uart.set_all_velocities(velocities))

        # Stop movement
        print_result("STOP MOVEMENT", uart.stop_movement())

        # Request current servo positions
        print_result("GET SERVO POSITIONS", uart.get_servo_positions())

        # Request error report
        print_result("GET ERROR REPORT", uart.get_error_report())

        res = uart.heartbeat()

        if (res.success):
            print("\nHeartbeat success")
            print(f"System state: {res.system_status}")
        else:
            print("\nHeartbeat failed")
            print(f"Com error: {res.value}")
            print(f"System state: {res.system_status}")

    except Exception as e:
        print(f"[EXCEPTION] {e}")

    finally:
        uart.close()

if __name__ == "__main__":
    main()

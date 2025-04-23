from UART_Communication import MasterUART
import logging

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
        read_timeout=0.6,
        start_bytes=b'\xAA\x55'
    )

    try:
        # Heartbeat
        print_result("HEARTBEAT", uart.heartbeat())

        # Write positions to servos 0-4
        print_result("WRITE_POSITION_RANGE", uart.write_position_range(4, [130.0]))

        # # Read positions back from servos 0-4
        # print_result("READ_POSITION_RANGE", uart.read_position_range(0, 5))

        # # Write velocities to servos 0-4
        # print_result("WRITE_VELOCITY_RANGE", uart.write_velocity_range(0, [5.0, 5.5, 6.0, 6.5]))

        # # Read error codes from servos 0-4
        # print_result("READ_ERROR_RANGE", uart.read_error_range(0, 4))

        # # Reconfirm heartbeat
        # res = uart.heartbeat()
        # print_result("HEARTBEAT CONFIRM", res)

    except Exception as e:
        print(f"[EXCEPTION] {e}")

    finally:
        uart.close()


if __name__ == "__main__":
    main()

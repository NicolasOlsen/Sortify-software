#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// === Debug Settings ===
// Global toggles for debugging output
constexpr bool DEBUG_MODE     = true;   // Enable or disable all debug output
constexpr bool USE_COLOR      = true;   // Use ANSI color codes in logs
constexpr bool USE_TIMESTAMPS = false;  // Prepend logs with milliseconds since boot

// === Servo Configuration ===
// Combined default positions for all servos (smart + analog)
// Order: Base, Shoulder, Elbow, Wrist, Gripper
constexpr float DEFAULT_SERVO_POSITIONS[] = {
    90.0f,  // Base
    45.0f,  // Shoulder
    90.0f,  // Elbow
    0.0f,   // Wrist
    90.0f   // Gripper (analog)
};

// Servo counts derived from array
constexpr uint8_t SMART_SERVO_COUNT  = 4;  // Smart servos use Dynamixel protocol
constexpr uint8_t TOTAL_SERVO_COUNT  = sizeof(DEFAULT_SERVO_POSITIONS) / sizeof(DEFAULT_SERVO_POSITIONS[0]);
constexpr uint8_t ANALOG_SERVO_COUNT = TOTAL_SERVO_COUNT - SMART_SERVO_COUNT;

// === Servo PWM Settings ===
// Constants used for converting degrees to PWM pulse length for analog servos
constexpr uint16_t SERVO_MIN_PWM          = 130;       // Minimum PWM pulse width (out of 4096)
constexpr uint16_t SERVO_MAX_PWM          = 550;       // Maximum PWM pulse width (out of 4096)
constexpr float    MAX_ANALOG_SERVO_DEGREE = 180.0f;   // Max angle allowed for analog servos
constexpr uint16_t ANALOG_SERVO_FREQ      = 50;        // PWM frequency in Hz for analog servos

// === Communication Settings ===
// UART and Dynamixel protocol configuration
constexpr uint32_t BAUDRATE_COMM         = 1000000;  // UART baudrate for comms (e.g. with Pi)
constexpr uint32_t BAUDRATE_DXL          = 1000000;  // Baudrate for Dynamixel bus
constexpr float    DXL_PROTOCOL_VERSION  = 2.0f;     // Dynamixel protocol version (usually 2.0)

#endif // CONFIG_H

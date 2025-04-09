#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Arduino.h>

// === Servo Configuration ===
// Combined default positions for all servos (smart + analog)
// Order: Base, Shoulder, Elbow, Wrist, Gripper
inline constexpr float DEFAULT_SERVO_POSITIONS[] = {
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
constexpr uint16_t  SERVO_MIN_PWM          = 130;       // Minimum PWM pulse width (out of 4096)
constexpr uint16_t  SERVO_MAX_PWM          = 550;       // Maximum PWM pulse width (out of 4096)
constexpr float     MAX_ANALOG_SERVO_DEGREE = 180.0f;   // Max angle allowed for analog servo
constexpr uint16_t  ANALOG_SERVO_FREQ      = 50;        // PWM frequency in Hz for analog servos

// === Communication Settings ===
// Dynamixel configuration
constexpr uint32_t BAUDRATE_DXL          = 1000000;  // Baudrate for Dynamixel bus
constexpr float    DXL_PROTOCOL_VERSION  = 2.0f;     // Dynamixel protocol version (usually 2.0)

#endif // SERVO_CONFIG_H

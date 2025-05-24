#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <stdint.h>

#include "control/analog_servo_class.h"
#include "control/dxl_servo_class.h"

// === Servo Configuration ===
// Combined default positions for all servos (smart + analog)
// Order: Base, Shoulder, Elbow, Wrist, Gripper
inline constexpr float DEFAULT_SERVO_POSITIONS[] = {
    90.0f,
    180.0f,
    130.0f,
    90.0f,
    90.0f
};

// Default velocities, ONLY Dxl
inline constexpr float DEFAULT_SERVO_VELOCITIES[] = {
    30.0f,
    30.0f,
    30.0f,
    30.0f,
};

inline const DxlServo DXL_SERVOS_CONFIG[] = {
    DxlServo(0, 0, 360, 0.229),     // Base
    DxlServo(1, 70, 260, 0.229),    // Shoulder
    DxlServo(2, 70, 290, 0.229),    // Elbow
    DxlServo(3, 70, 290, 0.229)     // Wrist
};

inline const AnalogServo ANALOG_SERVOS_CONFIG[] = {
    AnalogServo(0, 110, 500, 180, 0, 95)   // Gripper
};

// Servo counts derived from array
constexpr uint8_t DXL_SERVO_COUNT  = sizeof(DXL_SERVOS_CONFIG) / sizeof(DXL_SERVOS_CONFIG[0]);
constexpr uint8_t ANALOG_SERVO_COUNT = sizeof(ANALOG_SERVOS_CONFIG) / sizeof(ANALOG_SERVOS_CONFIG[0]);
constexpr uint8_t TOTAL_SERVO_COUNT = DXL_SERVO_COUNT + ANALOG_SERVO_COUNT;
static_assert(TOTAL_SERVO_COUNT <= 255, "Total size can't be greater than 255");
static_assert(sizeof(DEFAULT_SERVO_POSITIONS) / sizeof(DEFAULT_SERVO_POSITIONS[0])
                 == TOTAL_SERVO_COUNT, "The default positions, has to match the servo configs");


// === Communication Settings ===
// Dynamixel configuration
constexpr uint32_t BAUDRATE_DXL          = 1000000;
constexpr float    DXL_PROTOCOL_VERSION  = 2.0f;

#endif // SERVO_CONFIG_H

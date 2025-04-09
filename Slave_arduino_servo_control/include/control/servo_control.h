#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <DynamixelShield.h>
#include <Adafruit_PWMServoDriver.h>

#include "config/servo_config.h"     // Constants like SMART_SERVO_COUNT, servoMin, etc.
#include "shared/shared_servo_state.h" // SharedServoData structure
#include "utils/debug.h"             // Logging utilities

namespace ServoControl {

    // === External Hardware Interfaces ===
    extern DynamixelShield dxl;
    extern Adafruit_PWMServoDriver pwm;

    // === Local Configuration ===
    constexpr bool SERVO_CONTROL_DEBUG = true;
    constexpr float pwmPerDegree = (SERVO_MAX_PWM - SERVO_MIN_PWM) / MAX_ANALOG_SERVO_DEGREE;

    /**
     * @brief Pings the servo and returns true if successful. Also stores the library error if it fails.
     * @param id ID of the servo to ping
     * @return True if the ping was successful
     */
    bool PingServo(uint8_t id);

    // === Inline Helper Functions ===

    inline float clamp(float value, float min, float max) {
        return (value < min) ? min : (value > max) ? max : value;
    }

    inline uint16_t degreesToPwm(float degrees) {
        degrees = clamp(degrees, 0.0f, MAX_ANALOG_SERVO_DEGREE);
        return static_cast<uint16_t>(degrees * pwmPerDegree + SERVO_MIN_PWM);
    }

    // === Template Implementations ===

    /**
     * @brief Sets a smart servo to the position stored in the goal data array. Clears the change flag.
     * @param id ID of the servo to set
     * @param goalData Shared data structure containing the goal positions
     */
    template <uint8_t size>
    void SetSmartServoToPosition(uint8_t id, SharedServoData<float, size>& goalData) {
        if (!goalData.GetFlag(id)) return;

        if (dxl.setGoalPosition(id, goalData.Get(id, true), UNIT_DEGREE)) {
            Debug::infoln("Servo " + String(id) + " successfully set", SERVO_CONTROL_DEBUG);
        } else {
            Debug::errorln("Servo " + String(id) + " failed to set", SERVO_CONTROL_DEBUG);
        }
    }

    /**
     * @brief Sets an analog servo (currently hardcoded to servo 0 on the PWM driver) to the stored goal position. Clears the change flag.
     * @param id ID of the servo to set
     * @param goalData Shared data structure containing the goal positions
     */
    template <uint8_t size>
    void SetAnalogServoToPosition(uint8_t id, SharedServoData<float, size>& goalData) {
        if (!goalData.GetFlag(id)) return;

        pwm.setPWM(0, 0, degreesToPwm(goalData.Get(id, true)));
        Debug::infoln("Analog servo " + String(id) + " successfully set", SERVO_CONTROL_DEBUG);
    }

    /**
     * @brief Reads the current position of a smart servo and stores it in the shared goal data array.
     * @param id ID of the servo to read
     * @param goalData Shared data structure to store the position
     */
    template <uint8_t size>
    void StoreCurrentServoPosition(uint8_t id, SharedServoData<float, size>& goalData) {
        float currentPosition = dxl.getPresentPosition(id, UNIT_DEGREE);

        if (dxl.getLastLibErrCode() != DXL_LIB_OK) {
            DXLLibErrorCode_t lastError = dxl.getLastLibErrCode();
            servoErrors.Set(id, lastError);
            Debug::errorln("Failed to read position of servo " + String(id) + ", error " + String(lastError), SERVO_CONTROL_DEBUG);
        } else {
            goalData.Set(id, currentPosition);
        }
    }

    /**
     * @brief Sets all servos to the positions stored in the goal data array.
     * @param goalData Shared data structure containing goal positions
     */
    template <uint8_t size>
    void SetServosToPosition(SharedServoData<float, size>& goalData) {
        for (uint8_t id = 1; id <= SMART_SERVO_COUNT; id++) {
            SetSmartServoToPosition(id, goalData);
        }
        for (uint8_t id = SMART_SERVO_COUNT + 1; id <= TOTAL_SERVO_COUNT; id++) {
            SetAnalogServoToPosition(id, goalData);
        }
    }

    /**
     * @brief Stores the current positions of all smart servos into the shared goal data array.
     * @param goalData Shared data structure to store current positions
     */
    template <uint8_t size>
    void StoreCurrentServoPositions(SharedServoData<float, size>& goalData) {
        for (uint8_t id = 1; id <= SMART_SERVO_COUNT; id++) {
            StoreCurrentServoPosition(id, goalData);
        }
    }

} // namespace ServoControl

#endif // SERVOCONTROL_H

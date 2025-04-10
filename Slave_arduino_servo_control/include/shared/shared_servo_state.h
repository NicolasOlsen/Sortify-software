#ifndef SHARED_SERVO_STATE_H
#define SHARED_SERVO_STATE_H

#include "shared/shared_servo_data.h"
#include "config/servo_config.h"

// External declarations
extern SharedServoData<float, TOTAL_SERVO_COUNT> goalPositions;
extern SharedServoData<float, SMART_SERVO_COUNT> goalVelocities;
extern SharedServoData<float, SMART_SERVO_COUNT> currentPositions;
extern SharedServoData<DXLLibErrorCode_t, SMART_SERVO_COUNT> servoErrors;

#endif

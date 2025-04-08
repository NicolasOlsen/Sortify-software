#ifndef SHAREDSERVOSTATE_H
#define SHAREDSERVOSTATE_H

#include <DynamixelShield.h>

#include "shared/SharedServoData.h"
#include "config.h"

// External declarations
extern SharedServoData<float, TOTAL_SERVO_COUNT> goalPositions;
extern SharedServoData<float, SMART_SERVO_COUNT> currentPositions;
extern SharedServoData<DXLLibErrorCode_t, SMART_SERVO_COUNT> servoErrors;

#endif

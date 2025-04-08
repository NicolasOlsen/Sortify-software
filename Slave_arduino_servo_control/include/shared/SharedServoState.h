#ifndef SHAREDSERVOSTATE_H
#define SHAREDSERVOSTATE_H

#include "shared/SharedServoData.h"
#include <DynamixelShield.h>

// Constants
constexpr uint8_t servos = 5;
constexpr uint8_t readableServos = 4;
constexpr uint8_t dynamixelServos = 4;

// External declarations
extern SharedServoData<float, servos> goalPositions;
extern SharedServoData<float, readableServos> currentPositions;
extern SharedServoData<DXLLibErrorCode_t, dynamixelServos> servoErrors;

#endif

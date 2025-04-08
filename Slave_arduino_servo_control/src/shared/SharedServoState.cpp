#include "shared/SharedServoState.h"

SharedServoData<float, servos> goalPositions;
SharedServoData<float, readableServos> currentPositions;
SharedServoData<DXLLibErrorCode_t, dynamixelServos> servoErrors;

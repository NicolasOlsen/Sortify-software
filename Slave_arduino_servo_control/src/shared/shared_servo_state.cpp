#include "shared/shared_servo_state.h"

SharedServoData<float, TOTAL_SERVO_COUNT> goalPositions;
SharedServoData<float, SMART_SERVO_COUNT> currentPositions;
SharedServoData<DXLLibErrorCode_t, SMART_SERVO_COUNT> servoErrors;

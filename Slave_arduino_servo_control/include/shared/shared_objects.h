#ifndef SHARED_OBJECTS_H
#define SHARED_OBJECTS_H

#include "config/servo_config.h"
#include "shared/shared_servo_data.h"
#include "control/servo_manager.h"
#include "comms/communication_code.h"
#include "shared_variable.h"

using namespace Com_code;

namespace Shared
{

    // External declarations
    extern SharedServoData<float, TOTAL_SERVO_COUNT> goalPositions;
    extern SharedServoData<float, TOTAL_SERVO_COUNT> currentPositions;
    extern SharedServoData<float, DXL_SERVO_COUNT> goalVelocities;
    extern SharedServoData<DXLLibErrorCode_t, DXL_SERVO_COUNT> servoErrors;

    extern ServoManager<DXL_SERVO_COUNT, ANALOG_SERVO_COUNT> servoManager;

    extern SharedVariable<Com_code::StatusCode> systemState;
    
} // namespace Shared




#endif

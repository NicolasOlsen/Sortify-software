#ifndef SHARED_OBJECTS_H
#define SHARED_OBJECTS_H

#include "config/servo_config.h"
#include "shared/shared_array.h"
#include "shared/shared_array_with_flags.h"
#include "control/servo_manager.h"
#include "comms/communication_code.h"
#include "shared_variable.h"

namespace Shared
{

    // External declarations
    extern SharedArray<float, TOTAL_SERVO_COUNT> goalPositions;
    extern SharedArray<float, TOTAL_SERVO_COUNT> currentPositions;
    extern SharedArray<float, DXL_SERVO_COUNT> goalVelocities;
    extern SharedArrayWithFlags<DXLLibErrorCode_t, DXL_SERVO_COUNT> servoErrors;

    extern ServoManager<DXL_SERVO_COUNT, ANALOG_SERVO_COUNT> servoManager;

    extern SharedVariable<COMM_CODE::StatusCode> systemState;
    
} // namespace Shared




#endif

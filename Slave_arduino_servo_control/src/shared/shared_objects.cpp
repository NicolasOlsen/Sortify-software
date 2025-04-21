#include "shared/shared_objects.h"
#include "shared/shared_variable.h"

using namespace COMM_CODE;

namespace Shared
{
    SharedServoData<float, TOTAL_SERVO_COUNT> goalPositions;
    SharedServoData<float, TOTAL_SERVO_COUNT> currentPositions;
    SharedServoData<float, DXL_SERVO_COUNT> goalVelocities;
    SharedServoData<DXLLibErrorCode_t, DXL_SERVO_COUNT> servoErrors;

    ServoManager<DXL_SERVO_COUNT, ANALOG_SERVO_COUNT> servoManager(DXL_SERVOS_CONFIG, ANALOG_SERVOS_CONFIG);

    SharedVariable<StatusCode> systemState(StatusCode::INITIALIZING);
} // namespace Shared



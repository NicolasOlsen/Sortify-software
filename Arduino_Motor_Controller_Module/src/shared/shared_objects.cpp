#include "shared/shared_objects.h"
#include "shared/shared_variable.h"

using namespace COMM_CODE;

namespace Shared
{
    SharedArray<float, TOTAL_SERVO_COUNT> goalPositions;
    SharedArray<float, TOTAL_SERVO_COUNT> currentPositions;
    SharedArray<float, DXL_SERVO_COUNT> goalVelocities;
    SharedArrayWithFlags<DXLLibErrorCode_t, DXL_SERVO_COUNT> currentErrors;
    SharedArrayWithFlags<DXLLibErrorCode_t, DXL_SERVO_COUNT> lastErrors;

    ServoManager<DXL_SERVO_COUNT, ANALOG_SERVO_COUNT> servoManager(DXL_SERVOS_CONFIG, ANALOG_SERVOS_CONFIG);

    SharedVariable<StatusCode> systemState(StatusCode::INITIALIZING);
} // namespace Shared



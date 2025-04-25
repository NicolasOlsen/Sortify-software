#include <stdint.h>
#include <DynamixelShield.h>

#include "control/system_init.h"

#include "config/servo_config.h"
#include "config/communication_config.h"
#include "comms/uart_receive.h"
#include "shared/shared_objects.h"

#include "utils/debug_utils.h"

using namespace COMM_CODE;

void InitSystem() {
    Shared::systemState.Set(StatusCode::INITIALIZING);
    Debug::init(BAUDRATE_COMM);
    Debug::infoln("Initializing");

    UART_COMM::UART_init(BAUDRATE_COMM);

    if(!ServoManager<DXL_SERVO_COUNT, ANALOG_SERVO_COUNT>::initServoLibraries()) {
        Debug::errorln("Couldnt initiate");
        Shared::systemState.Set(StatusCode::FAULT);
    }

    if(!Shared::servoManager.initAll()) {
        Debug::errorln("Some or all of the dxl servos couldnt initiate");
        Shared::systemState.Set(StatusCode::FAULT);
    }

    // Retrieve and store initialization error codes after initAll()
    // This allows the system to inspect which servos failed to init
    DXLLibErrorCode_t tempErrors[Shared::servoManager.getDXLAmount()];
    Shared::servoManager.getErrors(
        tempErrors, 
        Shared::servoManager.getDXLAmount());
    Shared::servoErrors.Set(
        tempErrors, 
        Shared::servoManager.getDXLAmount());

    Shared::goalVelocities.Set(DEFAULT_SERVO_VELOCITIES, DXL_SERVO_COUNT);
    Shared::goalPositions.Set(DEFAULT_SERVO_POSITIONS, TOTAL_SERVO_COUNT);
    Shared::currentPositions.Set(DEFAULT_SERVO_POSITIONS, TOTAL_SERVO_COUNT);

    if (Shared::systemState.Get() == StatusCode::FAULT) return;  // Return and the system is in fault mode

    Shared::servoManager.setGoalVelocities(DEFAULT_SERVO_VELOCITIES);
    Shared::servoManager.setGoalPositions(DEFAULT_SERVO_POSITIONS);

    Shared::systemState.Set(StatusCode::IDLE);    // System is initialized and idle
    Debug::infoln("System initialized successfully");
}
#include <stdint.h>
#include <DynamixelShield.h>

#include "control/system_init.h"

#include "config/servo_config.h"
#include "config/communication_config.h"
#include "control/servo_control.h"
#include "comms/uart_receive.h"
#include "shared/shared_objects.h"

#include "utils/debug_utils.h"

using namespace COMM_CODE;

void InitSystem() {
    Shared::systemState.Set(StatusCode::INITIALIZING);
    Debug::init(BAUDRATE_COMM);
    Debug::infoln("Initializing");

    UART_COMM::UART_init(BAUDRATE_COMM);

    if(!initServoLibraries()) {
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


    Shared::goalPositions.Set(DEFAULT_SERVO_POSITIONS, TOTAL_SERVO_COUNT, 0, false);

    if (Shared::systemState.Get() == StatusCode::FAULT) return;  // Return and the system is in fault mode

    Shared::servoManager.setGoalPositions(DEFAULT_SERVO_POSITIONS, TOTAL_SERVO_COUNT);

    Shared::systemState.Set(StatusCode::IDLE);    // System is initialized and idle
    Debug::infoln("System initialized successfully");
}
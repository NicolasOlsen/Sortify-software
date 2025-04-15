#include <stdint.h>
#include <DynamixelShield.h>

#include "control/system_init.h"

#include "config/servo_config.h"
#include "config/communication_config.h"
#include "control/servo_control.h"
#include "comms/UART_communication.h"
#include "shared/shared_objects.h"

#include "utils/debug.h"

constexpr bool LOCAL_DEBUG = true;

void InitSystem() {
    Shared::systemState.Set(StatusCode::INITIALIZING);
    Debug::init(BAUDRATE_COMM);
    Debug::infoln("Initializing", LOCAL_DEBUG);

    UART_init(BAUDRATE_COMM);

    if(!initServoLibraries()) {
        Debug::errorln("Couldnt initiate", LOCAL_DEBUG);
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


    Shared::goalPositions.Set(DEFAULT_SERVO_POSITIONS, TOTAL_SERVO_COUNT, false);
    
    // Clear all update flags after initializing to prevent unintended movement
    Shared::goalPositions.SetAllFlags(false);

    if (Shared::systemState.Get() == StatusCode::FAULT) return;  // Return and the system is in fault mode

    Shared::servoManager.setGoalPositions(DEFAULT_SERVO_POSITIONS, TOTAL_SERVO_COUNT);

    Shared::systemState.Set(StatusCode::IDLE);    // System is initialized and idle
    Debug::infoln("System initialized successfully", LOCAL_DEBUG);
}
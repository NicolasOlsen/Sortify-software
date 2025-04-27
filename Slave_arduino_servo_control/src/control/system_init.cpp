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
        Debug::errorln("Initialization failed");
        Shared::systemState.Set(StatusCode::FAULT_INIT);
    }

    if(!Shared::servoManager.initAll()) {
        Debug::errorln("Some or all of the dxl servos couldnt initiate");
        Shared::systemState.Set(StatusCode::FAULT_INIT);
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

    if (Shared::systemState.Get() == StatusCode::FAULT_INIT) return;  // Return and the system is in fault mode

    // Save the real positions of the servos if there was succesfull initalising
    float tempCurrentPosition[TOTAL_SERVO_COUNT];
    for (uint8_t id = 0; id < TOTAL_SERVO_COUNT; id++) {
        if (id < Shared::servoManager.getDXLAmount()) {
            if (tempErrors[id] == DXL_LIB_OK) {
                tempCurrentPosition[id] = Shared::servoManager.getCurrentPosition(id);
            }
            else {
                tempCurrentPosition[id] = DEFAULT_SERVO_POSITIONS[id];
            }
        }
        else {
            tempCurrentPosition[id] = DEFAULT_SERVO_POSITIONS[id];
        }            
    }

    Shared::currentPositions.Set(tempCurrentPosition, TOTAL_SERVO_COUNT);


    Shared::goalVelocities.Set(DEFAULT_SERVO_VELOCITIES, DXL_SERVO_COUNT);
    Shared::goalPositions.Set(DEFAULT_SERVO_POSITIONS, TOTAL_SERVO_COUNT);

    Shared::systemState.Set(StatusCode::IDLE);    // System is initialized and idle
    Debug::infoln("System initialized successfully");
}
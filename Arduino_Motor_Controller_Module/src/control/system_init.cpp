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
    StatusCode tempStatus = StatusCode::INITIALIZING;

    Shared::systemState.Set(tempStatus);

    #ifdef DEBUG
        Debug::init(BAUDRATE_COMM);
        Debug::infoln("Initializing");
    #endif

    #ifdef TIMING_MODE
        TIMING_SERIAL.begin(TIMING_BAUD);
    #endif

    UART_COMM::UART_init(BAUDRATE_COMM);

    if(!ServoManager<DXL_SERVO_COUNT, ANALOG_SERVO_COUNT>::initServoLibraries()) {
        Debug::errorln("Initialization failed");
        tempStatus = StatusCode::FAULT_INIT;
        Shared::systemState.Set(tempStatus);
    }

    if(!Shared::servoManager.initAll()) {
        Debug::errorln("Some or all of the dxl servos couldnt initiate");
        tempStatus = StatusCode::FAULT_INIT;
        Shared::systemState.Set(tempStatus);
    }

    // Retrieve and store initialization error codes after initAll()
    // This allows the system to inspect which servos failed to init
    DXLLibErrorCode_t tempErrors[Shared::servoManager.getDXLAmount()];

     // Gets the errors
    Shared::servoManager.getErrors(
        tempErrors, 
        Shared::servoManager.getDXLAmount());

    // Save the errors
    Shared::currentErrors.Set(
        tempErrors, 
        Shared::servoManager.getDXLAmount());
    Shared::lastErrors.Set(
        tempErrors, 
        Shared::servoManager.getDXLAmount());

    // If the initaliziation failed, save standarised data, to have defined values
    if (tempStatus == StatusCode::FAULT_INIT) {
        Shared::currentPositions.Set(DEFAULT_SERVO_POSITIONS, TOTAL_SERVO_COUNT);
    }
    else { // Save the real positions of the servos if the initaliziation was succesfull 
        float tempCurrentPosition[TOTAL_SERVO_COUNT];
        for (uint8_t id = 0; id < TOTAL_SERVO_COUNT; id++) {
            if (id < Shared::servoManager.getDXLAmount()) {
                if (tempErrors[id] == DXL_LIB_OK) {
                    tempCurrentPosition[id] = Shared::servoManager.getCurrentPosition(id);
                }
            }          
        }

        Shared::currentPositions.Set(tempCurrentPosition, TOTAL_SERVO_COUNT);
    }

    Shared::goalVelocities.Set(DEFAULT_SERVO_VELOCITIES, DXL_SERVO_COUNT);
    Shared::goalPositions.Set(DEFAULT_SERVO_POSITIONS, TOTAL_SERVO_COUNT);

    // Putting set goal velocities here to prevent it from going normal speed, which is too fast
    Shared::servoManager.setGoalVelocities(DEFAULT_SERVO_VELOCITIES);

    if (tempStatus == StatusCode::FAULT_INIT) {
        Debug::warnln("System is in FAULT_INIT mode");
    } 
    else {
        Shared::systemState.Set(StatusCode::IDLE);    // System is initialized and idle
        Debug::infoln("System initialized successfully");
    }
}
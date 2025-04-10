#include <DynamixelShield.h>

#include "control/system_init.h"

#include "config/servo_config.h"
#include "config/communication_config.h"
#include "control/servo_control.h"
#include "comms/UART_communication.h"
#include "shared/shared_servo_state.h"
#include "shared/system_status.h"

#include "utils/debug.h"

using namespace ServoControl;

constexpr bool LOCAL_DEBUG = true;

void InitSystem() {
    System_status::systemState.Set(StatusCode::INITIALIZING);
    Debug::init(BAUDRATE_COMM);
    Debug::infoln("Initializing", LOCAL_DEBUG);

    UART_init(BAUDRATE_COMM);


    // Initialize shared data, mutexes, etc.
    ServoControl::dxl.begin(BAUDRATE_DXL);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    ServoControl::dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    ServoControl::pwm.begin();

    ServoControl::pwm.setOscillatorFrequency(27000000);
    ServoControl::pwm.setPWMFreq(ANALOG_SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    for (uint8_t id = 1; id <= SMART_SERVO_COUNT; id++) {
        // Get DYNAMIXEL information
        if(ServoControl::PingServo(id) == false) {
            System_status::systemState.Set(StatusCode::FAULT);   // Turn the system to fault mode, but will still ping to update the other servo statuses
            Debug::errorln("System initiated in fault mode");
            continue;
        }

        // Turn off torque when configuring items in EEPROM area
        ServoControl::dxl.torqueOff(id);
        ServoControl::dxl.setOperatingMode(id, OP_POSITION);
        ServoControl::dxl.torqueOn(id);
    }

    goalPositions.Set(DEFAULT_SERVO_POSITIONS);

    if (System_status::systemState.Get() == StatusCode::FAULT) return;  // Return and the system is in fault mode

    ServoControl::SetServosToPosition(goalPositions);

    System_status::systemState.Set(StatusCode::IDLE);    // System is initialized and idle
}
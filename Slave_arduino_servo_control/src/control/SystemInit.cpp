#include <DynamixelShield.h>

#include "control/SystemInit.h"
#include "control/ServoControl.h"
#include "comms/UART_communication.h"
#include "shared/SharedServoState.h"
#include "shared/System_status.h"
#include "utils/Debug.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr uint32_t baudrateCom = 1000000;

constexpr uint32_t baudrateDXL = 1000000;
constexpr float DXL_PROTOCOL_VERSION = 2.0;

constexpr uint16_t AnalogServoFreq = 50;    // Analog servos run at ~50 Hz updates

constexpr float startPositions[] = {0, 0, 0, 0, 0}; // Fill in for the wanted start positions

using namespace ServoControl;

void InitSystem() {
    System_status::InitSystemStatusMutex();
    System_status::SetSystemState(StatusCode::INITIALIZING);
    Debug::init(baudrateCom);
    Debug::infoln("Initializing", DEBUG_MODE);

    UART_init(baudrateCom);


    // Initialize shared data, mutexes, etc.
    ServoControl::dxl.begin(baudrateDXL);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    ServoControl::dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    ServoControl::pwm.begin();

    ServoControl::pwm.setOscillatorFrequency(27000000);
    ServoControl::pwm.setPWMFreq(AnalogServoFreq);  // Analog servos run at ~50 Hz updates

    for (uint8_t id = 1; id <= 4; id++) {
        // Get DYNAMIXEL information
        if(ServoControl::PingServo(id) == false) {
            System_status::SetSystemState(StatusCode::FAULT);   // Turn the system to fault mode, but will still ping to update the other servo statuses
            continue;
        }

        // Turn off torque when configuring items in EEPROM area
        ServoControl::dxl.torqueOff(id);
        ServoControl::dxl.setOperatingMode(id, OP_POSITION);
        ServoControl::dxl.torqueOn(id);
    }

    goalPositions.Set(startPositions);

    if (System_status::GetSystemState() == StatusCode::FAULT) return;  // Return and the system is in fault mode

    ServoControl::SetServosToGoalPosition();

    System_status::SetSystemState(StatusCode::IDLE);    // System is initialized and idle
}
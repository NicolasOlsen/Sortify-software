#include <DynamixelShield.h>

#include "control/SystemInit.h"
#include "control/ServoControl.h"
#include "shared/SharedServoData.h"
#include "utils/Debug.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr uint32_t baudrateCom = 1000000;

constexpr uint32_t baudrateDXL = 1000000;
constexpr float DXL_PROTOCOL_VERSION = 2.0;

constexpr uint16_t AnalogServoFreq = 50;    // Analog servos run at ~50 Hz updates

void InitSystem() {
    Debug::init(baudrateCom);
    while (!Serial1);
    Debug::infoln("Initializing", DEBUG_MODE);

    // Initialize shared data, mutexes, etc.
    dxl.begin(baudrateDXL);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    pwm.begin();

    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(AnalogServoFreq);  // Analog servos run at ~50 Hz updates

    InitServoDataMutexes();

    for (size_t id = 1; id <= 4; id++) {
        // Get DYNAMIXEL information
        if(PingServo(id) == false) continue;

        // Turn off torque when configuring items in EEPROM area
        dxl.torqueOff(id);
        dxl.setOperatingMode(id, OP_POSITION);
        dxl.torqueOn(id);
    }
}
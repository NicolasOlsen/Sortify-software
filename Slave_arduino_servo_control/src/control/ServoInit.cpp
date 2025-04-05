#include <DynamixelShield.h>

#include "control/ServoInit.h"
#include "shared/SharedServoData.h"
#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr uint32_t baudRate = 1000000;
constexpr float DXL_PROTOCOL_VERSION = 2.0;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void InitServoSystem() {
    // Initialize shared data, mutexes, etc.
    dxl.begin(baudRate);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    for (size_t id = 1; id <= 4; id++)
    {
        // Get DYNAMIXEL information
        if(dxl.ping(id) == false) {
            Debug::print(String(id), DEBUG_MODE);
            Debug::error(" Failed ping", DEBUG_MODE);
            Debug::errorln("error " + String(dxl.getLastLibErrCode()), DEBUG_MODE);
            continue;
        }

        // Turn off torque when configuring items in EEPROM area
        dxl.torqueOff(id);
        dxl.setOperatingMode(id, OP_POSITION);
        dxl.torqueOn(id);
    }
}
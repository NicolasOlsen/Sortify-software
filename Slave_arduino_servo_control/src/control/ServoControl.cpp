#include "control/ServoControl.h"
#include "shared/SharedServoState.h"
#include "config.h"
#include "utils/Debug.h"

namespace ServoControl {

// === External Definitions ===
DynamixelShield dxl;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


bool PingServo(uint8_t id) {
    if (dxl.ping(id)) return true;

    DXLLibErrorCode_t lastError = dxl.getLastLibErrCode();
    Debug::errorln(String(id) + " failed ping, error " + String(lastError), DEBUG_MODE);

    servoErrors.Set(id, lastError);
    return false;
}

} // namespace ServoControl

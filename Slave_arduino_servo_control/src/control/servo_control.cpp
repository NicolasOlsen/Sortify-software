#include "control/servo_control.h"
#include "shared/shared_servo_state.h"

#include "utils/Debug.h"

namespace ServoControl {

// === External Definitions ===
DynamixelShield dxl;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


bool PingServo(uint8_t id) {
    bool success = dxl.ping(id);

    DXLLibErrorCode_t lastError = dxl.getLastLibErrCode();

    if (!success) {
        Debug::errorln(String(id) + " failed ping, error " + String(lastError));
    }

    servoErrors.Set(id, lastError);
    return success;
}

bool PingServos() {
    bool success = true;
    for (uint8_t id = 1; id <= SMART_SERVO_COUNT; id++) {
        if (!PingServo(id)) {
            success = false; 
        }
    }
    return success;
}

} // namespace ServoControl

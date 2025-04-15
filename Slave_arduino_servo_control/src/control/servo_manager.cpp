#include "control/servo_manager.h"

bool initServoLibraries() {
    bool dxlOk = initDxlServoDriver();
    bool analogOk = initAnalogServoDriver();  // optional if needed

    return dxlOk && analogOk;
}
#include "control/ServoControl.h"
#include "shared/SharedServoState.h"
#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

DynamixelShield dxl;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


constexpr uint16_t servoMin = 130;              // Minimum pulse length (out of 4096)
constexpr uint16_t servoMax = 550;              // Maximum pulse length (out of 4096)
constexpr float maxAnalogServoDegree = 180.0f;  // Max safe angle for analog servo
constexpr float pwmPerDegree = (servoMax - servoMin) / maxAnalogServoDegree;  // PWM units per degree

float clamp(float value, float min, float max) {    // Helping claming the values to prevent damage to servo
    return (value < min) ? min : (value > max) ? max : value;
}

uint16_t degreesToPwm(float degrees) {              // Easily get degrees to pwm
    degrees = clamp(degrees, 0.0f, maxAnalogServoDegree);
    return static_cast<uint16_t>(degrees * pwmPerDegree + servoMin);
}


bool PingServo(uint8_t id) {
    if(dxl.ping(id) == true) return true;

    DXLLibErrorCode_t lastError = dxl.getLastLibErrCode();

    Debug::errorln(String(id) + " Failed ping, error " + String(lastError), DEBUG_MODE);

    servoErrors.Set(id, lastError);
    return false;
}

void SetSmartServoPosition(uint8_t id) {
    
    if(!goalPositions.GetFlag(id)) return;  // Check if the position has not been changed

    if (dxl.setGoalPosition(id, goalPositions.Get(id, true), UNIT_DEGREE)) {  // Is true if succesfull // Set position flag to false since it has been changed
        Debug::infoln("Servo " + String(id) + " successfully set", DEBUG_MODE);
    }
    else {
        Debug::infoln("Servo " + String(id) + " failed set", DEBUG_MODE);
    }
}

void SetAnalogServoPosition(uint8_t id) {
    if(!goalPositions.GetFlag(id)) return;  // Check if the position has not been changed
    
    pwm.setPWM(0, 0, degreesToPwm(goalPositions.Get(id, true)));   // Set position flag to false since it has been used

    Debug::infoln("Servo " + String(id) + " sucesfully set", DEBUG_MODE);
}


void GetServoPosition(uint8_t id) {
    DXLLibErrorCode_t lastError;

    float currentPosition = dxl.getPresentPosition(id, UNIT_DEGREE);

    if (dxl.getLastLibErrCode() != DXL_LIB_OK) {
        lastError = dxl.getLastLibErrCode();
        servoErrors.Set(id, lastError);
    } else {
        currentPositions.Set(id, currentPosition);
    }
}
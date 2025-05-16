#include <Adafruit_PWMServoDriver.h>

#include "control/analog_servo_class.h"
#include "utils/debug_utils.h"
#include "utils/math_utils.h"


// File-local instance of the shared driver
static Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver();

bool AnalogServo::initAnalogServoDriver() {
    if (!pwmDriver.begin()) {
        Debug::errorln("Failed to initialize analog PWM driver");
        return false;
    }

    pwmDriver.setOscillatorFrequency(27000000);
    pwmDriver.setPWMFreq(50); // Default analog servo freq (50 Hz)
    Debug::infoln("Analog PWM driver initialized successfully");
    return true;
}

AnalogServo::AnalogServo(uint8_t channel, uint16_t minPWM, uint16_t maxPWM,
                         float maxDegree, float minAllowedDegree, float maxAllowedDegree)
    : _channel(channel), _minPWM(minPWM), _maxPWM(maxPWM),
      _maxDegree(maxDegree), _minAllowedDegree(minAllowedDegree),
      _maxAllowedDegree((maxAllowedDegree < 0) ? maxDegree : maxAllowedDegree) {}

AnalogServo::AnalogServo() : AnalogServo(0, 130, 550) {}

bool AnalogServo::setToPosition(float position) {
    float clamped = Utils::clamp(position, _minAllowedDegree, _maxAllowedDegree);
    uint16_t pwmVal = degreesToPwm(clamped);

    if (pwmDriver.setPWM(_channel, 0, pwmVal) == 0) {
        #ifdef DEBUG
            Debug::infoln("AnalogServo[" + String(_channel) + 
                        "]: Set channel to " + String(clamped) + "° (PWM " + String(pwmVal) + ")");
        #endif

        return true;
    }
    #ifdef DEBUG
        Debug::errorln("AnalogServo[" + String(_channel) + 
                    "]: Failed to set position to " + String(position));
    #endif
    
    return false;
}

bool AnalogServo::trySetToPosition(float position) {
    if (!checkPositionInAllowedRange(position)) {
        #ifdef DEBUG
        Debug::warnln("AnalogServo: Refused to set channel " + String(_channel) +
                    " to out-of-range position " + String(position));
        #endif

        return false;
    }

    return setToPosition(position);
}

bool AnalogServo::checkPositionInAllowedRange(float position) const {
    return (position >= _minAllowedDegree) && (position <= _maxAllowedDegree);
}

uint16_t AnalogServo::degreesToPwm(float degrees) const {
    float pwmPerDegree = (_maxPWM - _minPWM) / _maxDegree;
    return static_cast<uint16_t>(degrees * pwmPerDegree + _minPWM);
}

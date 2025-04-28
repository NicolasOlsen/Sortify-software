#include "config/servo_config.h"

#include "control/dxl_servo_class.h"
#include "utils/debug_utils.h"
#include "utils/math_utils.h"

// File-local DynamixelShield instance
DynamixelShield DxlServo::dxl;

bool DxlServo::initDxlServoDriver() {
    dxl.begin(BAUDRATE_DXL);  // BAUDRATE_DXL if you have it defined elsewhere
    if (dxl.getLastLibErrCode() == D2A_LIB_ERROR_NULLPTR_PORT_HANDLER) {
        Debug::errorln("Dynamixel driver failed to begin()");
        return false;
    }

    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    Debug::infoln("Dynamixel driver initialized successfully");
    return true;
}

DxlServo::DxlServo(uint8_t id, float minAngle, float maxAngle, float velocityUnitScale)
    : _id(id), _minAngle(minAngle), _maxAngle(maxAngle),
      _velocityUnitScale(velocityUnitScale),
      _lastErrorCode(DXLLibErrorCode::DXL_LIB_OK) {}

DxlServo::DxlServo() : DxlServo(0) {}


bool DxlServo::init() {
    bool ok = dxl.ping(_id);
    _lastErrorCode = dxl.getLastLibErrCode();

    if (!ok) {
        Debug::errorln("DxlServo[" + String(_id) + "]: Ping failed");
        return false;
    }

    dxl.torqueOff(_id);
    dxl.setOperatingMode(_id, OP_POSITION);
    dxl.torqueOn(_id);

    Debug::infoln("DxlServo[" + String(_id) + "]: Ping successful");
    return true;
}

bool DxlServo::initWithRetry(uint8_t maxAttempts) {
    for (uint8_t attempt = 1; attempt <= maxAttempts; ++attempt) {
        Debug::infoln("DxlServo[" + String(_id) + "]: Init attempt " + String(attempt));

        if (init()) {
            Debug::infoln("DxlServo[" + String(_id) + "]: Init succeeded");
            return true;
        }

        Debug::errorln("DxlServo[" + String(_id) + "]: Init failed on attempt " + String(attempt));
    }

    Debug::errorln("DxlServo[" + String(_id) + "]: Failed to initialize after " + String(maxAttempts) + " attempts");
    return false;
}


bool DxlServo::ping() {
    bool success = dxl.ping(_id);
    _lastErrorCode = dxl.getLastLibErrCode();
    dxl.getLastStatusPacketError();

    if (success) {
        Debug::infoln("DxlServo[" + String(_id) + "]: Ping successful");
    } else {
        Debug::errorln("DxlServo[" + String(_id) + "]: Ping failed");
    }

    return success;
}


bool DxlServo::setPosition(float position) {
    float clamped = Utils::clamp(position, _minAngle, _maxAngle);
    bool ok = dxl.setGoalPosition(_id, clamped, UNIT_DEGREE);
    _lastErrorCode = dxl.getLastLibErrCode();

    if (!ok) {
        Debug::errorln("DxlServo[" + String(_id) + "]: Failed to set position to " + String(position));
        return false;
    }

    Debug::infoln("DxlServo[" + String(_id) + "]: Position set to " + String(clamped));
    return true;
}

bool DxlServo::setVelocity(float velocityDegPerSec) {
    uint32_t rawVelocity = convertDegPerSecToRaw(velocityDegPerSec);
    bool ok = dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, _id, rawVelocity);
    _lastErrorCode = dxl.getLastLibErrCode();

    if (!ok) {
        Debug::errorln("DxlServo[" + String(_id) + "]: Failed to set velocity to " + String(velocityDegPerSec) + " deg/s");
        return false;
    }

    Debug::infoln("DxlServo[" + String(_id) + "]: Velocity set to " + String(velocityDegPerSec) + " deg/s (raw: " + String(rawVelocity) + ")");
    return true;
}

float DxlServo::getPosition() {
    float position = dxl.getPresentPosition(_id, UNIT_DEGREE);
    _lastErrorCode = dxl.getLastLibErrCode();

    if (_lastErrorCode != DXL_LIB_OK) {
        Debug::errorln("DxlServo[" + String(_id) + "]: Failed to read position");
    }
    else {
        Debug::infoln("DxlServo[" + String(_id) + "]: Current position is " + String(position));
    }
    return position;
}

uint8_t DxlServo::getID() const {
    return _id;
}

bool DxlServo::checkPositionInAllowedRange(float position) const {
    return (position >= _minAngle) && (position <= _maxAngle);
}

uint32_t DxlServo::convertDegPerSecToRaw(float velocityDegPerSec) const {
    // Adjust the scale factor to better fit smaller velocities
    if (velocityDegPerSec < 2.0f) {
        velocityDegPerSec = 1.0f;  // Ensures you don't get 0, for example.
    }

    float rpm = velocityDegPerSec * (60.0f / 360.0f);  // deg/s → rpm
    float raw = rpm / _velocityUnitScale;
    return static_cast<uint32_t>(raw);
}

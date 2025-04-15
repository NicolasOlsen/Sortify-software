#include "config/servo_config.h"

#include "control/dxl_servo_class.h"
#include "utils/Debug.h"
#include "utils/math_utils.h"

// File-local DynamixelShield instance
static DynamixelShield dxlDriver;

constexpr bool DXL_DEBUG = true;

bool initDxlServoDriver() {
    dxlDriver.begin(BAUDRATE_DXL);  // BAUDRATE_DXL if you have it defined elsewhere
    if (dxlDriver.getLastLibErrCode() == D2A_LIB_ERROR_NULLPTR_PORT_HANDLER) {
        Debug::errorln("Dynamixel driver failed to begin()", DXL_DEBUG);
        return false;
    }

    dxlDriver.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    Debug::infoln("Dynamixel driver initialized successfully", DXL_DEBUG);
    return true;
}

DxlServo::DxlServo(uint8_t id, float minAngle, float maxAngle, float velocityUnitScale)
    : _id(id), _minAngle(minAngle), _maxAngle(maxAngle),
      _velocityUnitScale(velocityUnitScale),
      _lastErrorCode(DXLLibErrorCode::DXL_LIB_OK) {}

DxlServo::DxlServo() : DxlServo(0) {}


bool DxlServo::init() {
    bool ok = dxlDriver.ping(_id);
    _lastErrorCode = dxlDriver.getLastLibErrCode();

    if (!ok) {
        Debug::errorln("DxlServo[" + String(_id) + "]: Ping failed", DXL_DEBUG);
        return false;
    }

    dxlDriver.torqueOff(_id);
    dxlDriver.setOperatingMode(_id, OP_POSITION);
    dxlDriver.torqueOn(_id);

    Debug::infoln("DxlServo[" + String(_id) + "]: Ping successful", DXL_DEBUG);
    return true;
}

bool DxlServo::initWithRetry(uint8_t maxAttempts) {
    for (uint8_t attempt = 1; attempt <= maxAttempts; ++attempt) {
        Debug::infoln("DxlServo[" + String(_id) + "]: Init attempt " + String(attempt), DXL_DEBUG);

        if (init()) {
            Debug::infoln("DxlServo[" + String(_id) + "]: Init succeeded", DXL_DEBUG);
            return true;
        }

        Debug::errorln("DxlServo[" + String(_id) + "]: Init failed on attempt " + String(attempt), DXL_DEBUG);
    }

    Debug::errorln("DxlServo[" + String(_id) + "]: Failed to initialize after " + String(maxAttempts) + " attempts", DXL_DEBUG);
    return false;
}


bool DxlServo::ping() {
    bool success = dxlDriver.ping(_id);
    _lastErrorCode = dxlDriver.getLastLibErrCode();

    if (success) {
        Debug::infoln("DxlServo[" + String(_id) + "]: Ping successful", DXL_DEBUG);
    } else {
        Debug::errorln("DxlServo[" + String(_id) + "]: Ping failed", DXL_DEBUG);
    }

    return success;
}


bool DxlServo::setPosition(float position) {
    float clamped = Utils::clamp(position, _minAngle, _maxAngle);
    bool ok = dxlDriver.setGoalPosition(_id, clamped, UNIT_DEGREE);
    _lastErrorCode = dxlDriver.getLastLibErrCode();

    if (!ok) {
        Debug::errorln("DxlServo[" + String(_id) + "]: Failed to set position to " + String(position), DXL_DEBUG);
        return false;
    }

    Debug::infoln("DxlServo[" + String(_id) + "]: Position set to " + String(clamped), DXL_DEBUG);
    return true;
}

bool DxlServo::setVelocity(float velocityDegPerSec) {
    uint32_t rawVelocity = convertDegPerSecToRaw(velocityDegPerSec);
    bool ok = dxlDriver.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, _id, rawVelocity);
    _lastErrorCode = dxlDriver.getLastLibErrCode();

    if (!ok) {
        Debug::errorln("DxlServo[" + String(_id) + "]: Failed to set velocity to " + String(velocityDegPerSec) + " deg/s", DXL_DEBUG);
        return false;
    }

    Debug::infoln("DxlServo[" + String(_id) + "]: Velocity set to " + String(velocityDegPerSec) + " deg/s (raw: " + String(rawVelocity) + ")", DXL_DEBUG);
    return true;
}

float DxlServo::getPosition() {
    float position = dxlDriver.getPresentPosition(_id, UNIT_DEGREE);
    _lastErrorCode = dxlDriver.getLastLibErrCode();

    if (_lastErrorCode != DXL_LIB_OK) {
        Debug::errorln("DxlServo[" + String(_id) + "]: Failed to read position", DXL_DEBUG);
    }
    else {
        Debug::infoln("DxlServo[" + String(_id) + "]: Current position is " + String(position), DXL_DEBUG);
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
    float rpm = velocityDegPerSec * (60.0f / 360.0f);  // deg/s → rpm
    float raw = rpm / _velocityUnitScale;
    return static_cast<uint32_t>(raw);
}

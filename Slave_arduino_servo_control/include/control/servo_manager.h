#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <stdint.h>

#include "control/dxl_servo_class.h"
#include "control/analog_servo_class.h"
#include "shared/shared_array_with_flags.h"
#include "utils/array_utils.h"

constexpr bool MANAGER_DEBUG = true;

/**
 * @brief Manages a fixed set of Dynamixel and analog servos.
 * 
 * Designed for use with static memory and external data buffers (e.g., SharedServoData).
 * Supports setting positions, velocities, reading states, and batching communication with minimal overhead.
 * 
 * @tparam sizeDXL Number of Dynamixel servos
 * @tparam sizeAnalog Number of analog servos
 */
template<uint8_t sizeDXL, uint8_t sizeAnalog>
class ServoManager
{
public:
    /**
     * @brief Construct a new ServoManager with static arrays of servos.
     * 
     * @param dxlServos Array of preconfigured DxlServo objects
     * @param analogServos Array of preconfigured AnalogServo objects
     */
    ServoManager(const DxlServo (&dxlServos)[sizeDXL], const AnalogServo (&analogServos)[sizeAnalog]);

    static bool initServoLibraries();

    /**
     * @brief Initializes a single servo by ID (calls ping, sets mode, enables torque).
     * 
     * @param id Servo ID (DXL or analog channel)
     * 
     * @return True if the servo was initialized successfully
     */
    bool init(uint8_t id);

    /**
     * @brief Sends a ping to a specific servo to check connectivity.
     * 
     * @param id Servo ID
     * 
     * @return True if ping was successful
     */
    bool ping(uint8_t id);

    /**
     * @brief Sets the position of a servo.
     * 
     * @param id Servo ID
     * @param degrees Target angle in degrees
     * 
     * @return True if the command was successfully sent
     */
    bool setGoalPosition(uint8_t id, float degrees);

    /**
     * @brief Sets the velocity of a servo.
     * 
     * @param id Servo ID
     * @param velocityDegPerSec Target velocity in velocity degree per second
     * 
     * @return True if the command was successfully sent
     */
    bool setGoalVelocity(uint8_t id, float velocityDegPerSec);

    /**
     * @brief Gets the current position of a Dynamizel servo.
     * 
     * @param id Servo ID
     * 
     * @return Current angle in degrees, or 0 may mean not found for dynamiexel, -1.0f means unsupported
     */
    float getCurrentPosition(uint8_t id);

    /**
     * @brief Gets the last known error code for a DXL servo.
     * 
     * @param id Servo ID
     * 
     * @return DXLLibErrorCode_t error code (DXL only; analog servos return 0)
     */
    DXLLibErrorCode_t getError(uint8_t id);

    /**
     * @brief Checks whether a target position is within the servo's allowed range.
     * 
     * @param id Servo ID
     * @param position Target position in degrees
     * 
     * @return True if within allowed bounds
     */
    bool checkPositionInAllowedRange(uint8_t id, float position);

    /**
     * @brief Initializes all dynamixel servos (ping, mode, torque).
     * 
     * @return True if all servos responded and were configured
     */
    bool initAll();

    /**
     * @brief Sets goal positions for all servos from an array.
     * 
     * @param goalPositions Array of goal positions
     * 
     * @return true if all values were set successfully
     */
    bool setGoalPositions(const float (&goalPositions)[sizeDXL + sizeAnalog]);

    /**
     * @brief Sets velocity values for all DXL servos from an array.
     * 
     * @param goalVelocities Array of velocity values in deg/s
     * 
     * @return true if all velocities were applied
     */
    bool setGoalVelocities(const float (&goalVelocities)[sizeDXL]);

    /**
     * @brief Reads current positions of all DXL servos into a float array.
     * 
     * @param out Array to store values
     * 
     * @return true if all reads succeeded
     */
    bool getCurrentPositions(float (&out)[sizeDXL]);

    /**
     * @brief Gets error codes for DXL servos.
     * 
     * @param out Output buffer for errors
     * @param size Number of entries to fill (must not exceed sizeDXL - startIndex)
     * @param startIndex Offset into the DXL servo list
     * 
     * @return true if all errors retrieved
     */
    bool getErrors(DXLLibErrorCode_t* out, uint8_t size, uint8_t startIndex = 0);

    /**
     * @brief Pings all DXL servos and returns true if all respond.
     */
    bool pingAll();

    constexpr uint8_t getTotalAmount() const { return totalSize; }
    constexpr uint8_t getDXLAmount() const { return sizeDXL; }
    constexpr uint8_t getAnalogAmount() const { return sizeAnalog; }

private:
    DxlServo _dxlServos[sizeDXL];
    AnalogServo _analogServos[sizeAnalog];
    SemaphoreHandle_t _mutex;

    /// Total number of servos managed
    static constexpr uint8_t totalSize = sizeDXL + sizeAnalog;
    static_assert(totalSize <= 255, "Total size can't be greater than 255");
};

template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::initServoLibraries() {
    bool dxlOk = DxlServo::initDxlServoDriver();
    bool analogOk = AnalogServo::initAnalogServoDriver();

    return dxlOk && analogOk;
}


template<uint8_t sizeDXL, uint8_t sizeAnalog>
ServoManager<sizeDXL, sizeAnalog>::ServoManager(
    const DxlServo (&dxlServos)[sizeDXL],
    const AnalogServo (&analogServos)[sizeAnalog]) {

    _mutex = xSemaphoreCreateMutex();

    for (uint8_t i = 0; i < sizeDXL; ++i) {
        _dxlServos[i] = dxlServos[i];
    }
    for (uint8_t i = 0; i < sizeAnalog; ++i) {
        _analogServos[i] = analogServos[i];
    }
}

template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::init(uint8_t id) {
    if (id >= sizeDXL) {
        Debug::errorln("Tried to init out of DXL range");
        return false;
    }

    ScopedLock lock(_mutex);
    if (lock.isLocked()) {
        return _dxlServos[id].init();
    }

    return false;
}


template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::ping(uint8_t id) {
    if (id >= sizeDXL) {
        Debug::errorln("Tried to ping out of DXL range");
        return false;
    }

    ScopedLock lock(_mutex);
    if (lock.isLocked()) {
        return _dxlServos[id].ping();
    }

    return false;
}

template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::setGoalPosition(uint8_t id, float degrees) {
    ScopedLock lock(_mutex);
    if (!lock.isLocked()) return false;

    if (id < sizeDXL) {
        return _dxlServos[id].setPosition(degrees);
    } else if (id < sizeDXL + sizeAnalog) {
        return _analogServos[id - sizeDXL].setToPosition(degrees);
    }

    Debug::errorln("Tried to set position out of total range");
    return false;
}

template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::setGoalVelocity(uint8_t id, float velocityDegPerSec) {
    ScopedLock lock(_mutex);
    if (!lock.isLocked()) return false;

    if (id < sizeDXL) {
        return _dxlServos[id].setVelocity(velocityDegPerSec);
    }

    Debug::errorln("Tried to set velocity out of dxl range");
    return false;
}

template<uint8_t sizeDXL, uint8_t sizeAnalog>
float ServoManager<sizeDXL, sizeAnalog>::getCurrentPosition(uint8_t id) {
    ScopedLock lock(_mutex);
    if (!lock.isLocked()) return -1.0f;

    if (id < sizeDXL) {
        return _dxlServos[id].getPosition();
    }

    Debug::errorln("Tried to get position out of DXL range");
    return -1.0f;
}

template<uint8_t sizeDXL, uint8_t sizeAnalog>
DXLLibErrorCode_t ServoManager<sizeDXL, sizeAnalog>::getError(uint8_t id) {
    ScopedLock lock(_mutex);
    if (!lock.isLocked()) return DXL_LIB_OK;

    if (id < sizeDXL) {
        return _dxlServos[id].getLastError();
    }

    Debug::errorln("Tried to get error out of DXL range");
    return DXL_LIB_OK; // Return OK for analog
}

template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::checkPositionInAllowedRange(uint8_t id, float position) {
    if (id < sizeDXL) {
        return _dxlServos[id].checkPositionInAllowedRange(position);
    } else if (id < sizeDXL + sizeAnalog) {
        return _analogServos[id - sizeDXL].checkPositionInAllowedRange(position);
    }
    Debug::errorln("Tried to check position range out of total range");
    return false;
}

template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::initAll() {
    bool allOk = true;
    ScopedLock lock(_mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < sizeDXL; ++i) {
            if (!_dxlServos[i].initWithRetry()) allOk = false;
        }
    } else {
        return false;
    }
    return allOk;
}

template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::pingAll() {
    bool allOk = true;
    for (uint8_t id = 0; id < sizeDXL; ++id) {
        if (!ping(id)) allOk = false;
    }
    return allOk;
}

template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::setGoalPositions(const float (&goalPositions)[sizeDXL + sizeAnalog]) {

    ScopedLock lock(_mutex);
    if (!lock.isLocked()) return false;

    // Use syncWrite for all DXLs
    bool dxlOk = DxlServo::syncSetPositions<sizeDXL>(_dxlServos, sliceFirst<float, sizeDXL>(goalPositions));

    // Set analog servos
    bool analogOk = true;
    for (uint8_t i = 0; i < sizeAnalog; ++i) {
        if (!_analogServos[i].setToPosition(goalPositions[sizeDXL + i])) {
            analogOk = false;
        }
    }

    return dxlOk && analogOk;
}


template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::setGoalVelocities(const float (&goalVelocities)[sizeDXL]) {

    ScopedLock lock(_mutex);
    if (!lock.isLocked()) return false;

    return DxlServo::syncSetVelocities(_dxlServos, goalVelocities);
}


template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::getCurrentPositions(float (&out)[sizeDXL]) {

    ScopedLock lock(_mutex);
    if (!lock.isLocked()) return false;

    return DxlServo::syncGetPositions(_dxlServos, out);
}


template<uint8_t sizeDXL, uint8_t sizeAnalog>
bool ServoManager<sizeDXL, sizeAnalog>::getErrors(DXLLibErrorCode_t* out, uint8_t size, uint8_t startIndex) {
    if (startIndex + size > sizeDXL) return false;

    ScopedLock lock(_mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size; ++i) {
            out[i] = _dxlServos[startIndex + i].getLastError();
        }
    } else {
        return false;
    }
    return true;
}

#endif // SERVO_MANAGER_H

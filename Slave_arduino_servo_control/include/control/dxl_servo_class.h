#ifndef DXL_SERVO_CLASS_H
#define DXL_SERVO_CLASS_H

#include <stdint.h>
#include <DynamixelShield.h>

/**
 * @brief Represents a single Dynamixel servo on the bus.
 * 
 * This class handles unit conversion, clamping, and writing position/velocity commands
 * for a specific servo. All communication goes through a shared DynamixelShield object
 * defined in the implementation file.
 */
class DxlServo
{
public:
    /**
     * @brief Construct a new DxlServo object
     * 
     * @param id ID of the servo on the bus
     * @param minAngle Minimum allowed angle in degrees (for clamping)
     * @param maxAngle Maximum allowed angle in degrees (for clamping)
     * @param velocityUnitScale Conversion factor from raw velocity units to RPM.
     *                          Example: XM430 uses 0.229 RPM/unit
     */
    DxlServo(uint8_t id, float minAngle = 0.0f, float maxAngle = 360.0f, float velocityUnitScale = 0.229f);

    DxlServo();

    /**
     * @brief Initialize the servo by applying checking its status and setting proper configuration.
     * 
     * Must be called after initDxlServoDriver(), before using the servo.
     */
    bool init();


    /**
     * @brief Attempts to initialize the servo multiple times (ping, mode set, torque).
     * 
     * If the initialization fails due to communication issues (e.g., CRC, checksum),
     * the function retries up to the specified number of attempts.
     * 
     * @param maxAttempts Number of retry attempts before failing
     * @return true if initialized successfully, false if all attempts failed
     */
    bool initWithRetry(uint8_t maxAttempts = 3);


    /**
     * @brief Pings the servo to check if it responds on the bus.
     * 
     * Updates internal error state.
     * 
     * @return true if ping was successful
     */
    bool ping();


    /**
     * @brief Sets the servo to a target position (in degrees).
     * 
     * The position is clamped between the configured min and max angles.
     * 
     * @param position Angle in degrees
     * 
     * @return True if the position was sent successfully. 
     */
    bool setPosition(float position);


    /**
     * @brief Checks if the given position is within the configured angle bounds.
     * 
     * @param position Desired position in degrees
     * @return True if within [_minAngle, _maxAngle]
     */
    bool checkPositionInAllowedRange(float position) const;


    /**
     * @brief Sets the target velocity in degrees per second.
     * 
     * Internally converts to raw units based on the velocityUnitScale.
     * 
     * @param velocityDegPerSec Desired speed in degrees per second
     * 
     * @return True if the velocity was written successfully
     */
    bool setVelocity(float velocityDegPerSec);


    /**
     * @brief Reads the current position of the servo in degrees.
     * 
     * @return Float Angle in degrees, will return 0 if failed (if error, this )
     */
    float getPosition();


    /**
     * @brief Gets the Dynamixel servo ID.
     */
    uint8_t getID() const;


    /**
     * @brief Gets the last communication error code from the last operation.
     * 
     * Call immediately after a failed set/get to inspect the cause.
     * 
     * @return DXLLibErrorCode_t Error code from DynamixelShield
     */
    DXLLibErrorCode_t getLastError() const { return _lastErrorCode; }

private:
    uint8_t _id;
    float _minAngle;
    float _maxAngle;
    float _velocityUnitScale;  // Conversion factor from raw unit to RPM

    DXLLibErrorCode_t _lastErrorCode = DXLLibErrorCode::DXL_LIB_OK;  ///< Last result from any operation


    /**
     * @brief Converts a velocity in degrees per second to raw Dynamixel value.
     * 
     * Conversion:
     *   deg/s → rpm → raw value.
     *   Raw = (degPerSec * 60 / 360) / velocityUnitScale
     * 
     * @param velocityDegPerSec Input velocity in degrees per second
     * 
     * @return uint32_t Raw value for the PROFILE_VELOCITY register
     */
    uint32_t convertDegPerSecToRaw(float velocityDegPerSec) const;
};


/**
 * @brief Initializes the shared Dynamixel driver (DynamixelShield).
 * 
 * Must be called once in setup() before using any DxlServo objects.
 * 
 * @return true if the driver was initialized successfully
 */
bool initDxlServoDriver();

#endif // DXL_SERVO_CLASS_H

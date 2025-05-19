#ifndef DXL_SERVO_CLASS_H
#define DXL_SERVO_CLASS_H

#include <stdint.h>
#include <DynamixelShield.h>

#include "utils/math_utils.h"

#include "utils/debug_utils.h"

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
    * @brief Initializes the shared Dynamixel driver (DynamixelShield).
    * 
    * Must be called once in setup() before using any DxlServo objects.
    * 
    * @return True if the driver was initialized successfully
    */
   static bool initDxlServoDriver();

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
     * @return True if initialized successfully, false if all attempts failed
     */
    bool initWithRetry(uint8_t maxAttempts = 3);


    /**
     * @brief Pings the servo to check if it responds on the bus.
     * 
     * Updates internal error state.
     * 
     * @return True if ping was successful
     */
    bool ping();

    /**
     * @brief Checks if the given position is within the configured angle bounds.
     * 
     * @param position Desired position in degrees
     * @return True if within [_minAngle, _maxAngle]
     */
    bool checkPositionInAllowedRange(float position) const;


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
     * @brief Sends goal positions to multiple servos simultaneously using syncWrite.
     * 
     * This function clamps each target position to its corresponding servo's min/max angle limits,
     * converts the value from degrees to raw Dynamixel units, and sends a single syncWrite packet
     * to update all specified servos at once. It does NOT wait for a response.
     * 
     * This method assumes that the 'DxlServo::init()' has been called for each servo.
     * 
     * @tparam DXLAmount Number of servos in the array
     * 
     * @param servos Pointer to an array of 'DxlServo' objects to command
     * @param positions Pointer to an array of target angles (in degrees) for each servo
     * 
     * @return True if the syncWrite packet was sent successfully, false otherwise
     */
    template <uint8_t DXLAmount>
    static bool syncSetPositions(const DxlServo (&servos)[DXLAmount], const float (&positions)[DXLAmount]);


    /**
     * @brief Sets the target velocity in degrees per second.
     * 
     * Internally converts to raw units based on the velocityUnitScale.
     * 
     * @param velocityDegPerSec Desired speed in degrees per second
     * 
     * @return True if the velocity was sent successfully
     */
    bool setVelocity(float velocityDegPerSec);

    /**
     * @brief Sends target velocities to multiple servos simultaneously using syncWrite.
     * 
     * This function converts each target velocity from degrees per second to raw Dynamixel units,
     * based on the velocity scale configured in each 'DxlServo' instance. It then sends a single 
     * syncWrite packet to update all specified servos. It does NOT wait for a response.
     * 
     * This method assumes that the 'DxlServo::init()' has been called for each servo.
     * 
     * @tparam DXLAmount Number of servos in the array
     * 
     * @param servos Pointer to an array of 'DxlServo' objects to command
     * @param velocityDegPerSec Pointer to an array of target velocities (in degrees per second) for each servo
     * 
     * @return True if the syncWrite packet was sent successfully, false otherwise
     */
    template <uint8_t DXLAmount>
    static bool syncSetVelocities(const DxlServo (&servos)[DXLAmount], const float (&velocityDegPerSec)[DXLAmount]);


    /**
     * @brief Reads the current position of the servo in degrees.
     * 
     * @return Float Angle in degrees, will return 0 if failed
     */
    float getPosition();

    /**
     * @brief Reads the current positions of multiple servos using bulkRead.
     * 
     * This function performs a bulkRead request for all specified servos and fills the provided
     * array with the current position of each servo (in degrees). Communication results and
     * per-servo error states are stored internally in each 'DxlServo' instance via '_lastErrorCode'.
     * 
     * This method assumes that the 'DxlServo::init()' has been called for each servo.
     * 
     * @tparam DXLAmount Number of servos in the array
     * 
     * @param servos Pointer to an array of 'DxlServo' objects to read from
     * @param positions Pointer to an array to be filled with current positions (in degrees)
     * 
     * @return True if at all servos responded successfully, false otherwise
     */
    template <uint8_t DXLAmount>
    static bool syncGetPositions(DxlServo (&servos)[DXLAmount], float (&positions)[DXLAmount]);


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

    DXLLibErrorCode_t _lastErrorCode = DXLLibErrorCode::DXL_LIB_OK;  // Last result from any operation

    static DynamixelShield dxl;

    /**
     * @brief Converts a velocity in degrees per second to raw Dynamixel value.
     * 
     * Conversion:
     *   deg/s -> rpm -> raw value.
     *   Raw = (degPerSec * 60 / 360) / velocityUnitScale
     * 
     * @param velocityDegPerSec Input velocity in degrees per second
     * 
     * @return uint32_t Raw value for the PROFILE_VELOCITY register
     */
    uint32_t convertDegPerSecToRaw(float velocityDegPerSec) const;
};

template <uint8_t DXLAmount>
bool DxlServo::syncSetPositions(const DxlServo (&servos)[DXLAmount], const float (&positions)[DXLAmount]) {

    // Holds raw positions to be sent (int32 = 4 bytes)
    int32_t goal_positions[DXLAmount];
    DYNAMIXEL::XELInfoSyncWrite_t info_xels[DXLAmount];

    for (uint8_t i = 0; i < DXLAmount; ++i) {
        // Clamp and convert to raw position
        float clamped = Utils::clamp(positions[i], servos[i]._minAngle, servos[i]._maxAngle);
        goal_positions[i] = static_cast<int32_t>((clamped / 360.0f) * 4095.0f);

        // Fill out per-servo syncWrite info
        info_xels[i].id = servos[i]._id;
        info_xels[i].p_data = reinterpret_cast<uint8_t*>(&goal_positions[i]);
    }

    // Fill out syncWrite packet struct
    DYNAMIXEL::InfoSyncWriteInst_t sync_info;
    sync_info.addr = 116;   // Start address of Goal position = 116
    sync_info.addr_length = 4;
    sync_info.p_xels = info_xels;
    sync_info.xel_count = DXLAmount;
    sync_info.is_info_changed = true;
    sync_info.packet.p_buf = nullptr;        // Let SDK manage internal buffer
    sync_info.packet.is_completed = false;

    return dxl.syncWrite(&sync_info);
}


template <uint8_t DXLAmount>
bool DxlServo::syncSetVelocities(const DxlServo (&servos)[DXLAmount], const float (&velocityDegPerSec)[DXLAmount]) {

    uint32_t velocity_raw[DXLAmount];
    DYNAMIXEL::XELInfoSyncWrite_t info_xels[DXLAmount];

    for (uint8_t i = 0; i < DXLAmount; ++i) {
        velocity_raw[i] = servos[i].convertDegPerSecToRaw(velocityDegPerSec[i]);

        #ifdef DEBUG
            Debug::infoln("Converted degpersec to raw: " + String(velocity_raw[i]));
        #endif

        info_xels[i].id = servos[i]._id;
        info_xels[i].p_data = reinterpret_cast<uint8_t*>(&velocity_raw[i]);
    }

    DYNAMIXEL::InfoSyncWriteInst_t sync_info;
    sync_info.addr = 112;  // Start address of Profile Velocity = 112
    sync_info.addr_length = 4;
    sync_info.p_xels = info_xels;
    sync_info.xel_count = DXLAmount;
    sync_info.is_info_changed = true;
    sync_info.packet.p_buf = nullptr;
    sync_info.packet.is_completed = false;

    return dxl.syncWrite(&sync_info);
}


template <uint8_t DXLAmount>
bool DxlServo::syncGetPositions(DxlServo (&servos)[DXLAmount], float (&positions)[DXLAmount]) {

    typedef struct sr_data{
        int32_t present_position;
    } __attribute__((packed)) sr_data_t;

    sr_data_t sr_data[DXLAmount]; // Array to store raw position data
    DYNAMIXEL::InfoSyncReadInst_t sr_infos;
    DYNAMIXEL::XELInfoSyncRead_t info_xels[DXLAmount];

    // Create the packet buffer
    uint8_t user_pkt_buf[128]; // Buffer to store the read data
    uint8_t user_pkt_buf_cap = sizeof(user_pkt_buf);

    // Prepare the syncRead packet information
    sr_infos.packet.p_buf = user_pkt_buf;
    sr_infos.packet.buf_capacity = user_pkt_buf_cap;
    sr_infos.packet.is_completed = false;
    sr_infos.addr = 132;  // Start address for PRESENT_POSITION
    sr_infos.addr_length = 4;  // 4 bytes per servo
    sr_infos.p_xels = info_xels;
    sr_infos.xel_count = DXLAmount;

    // Set up the syncRead configuration for each servo
    for (uint8_t i = 0; i < DXLAmount; ++i) {
        info_xels[i].id = servos[i]._id;
        info_xels[i].p_recv_buf = (uint8_t*)(&sr_data[i]);
    }
    sr_infos.is_info_changed = true;

    // Perform the syncRead
    uint8_t receivedCount = dxl.syncRead(&sr_infos);

    #ifdef DEBUG
        Debug::infoln("Received count: " + String(receivedCount));
    #endif

    bool allSuccess = true;
    
    // Loop through each servo and process the data
    if (receivedCount == DXLAmount) {
        for (uint8_t i = 0; i < DXLAmount; ++i) {
            servos[i]._lastErrorCode = sr_infos.p_xels[i].error;
    
            if (sr_infos.p_xels[i].error == 0) {
                // Convert raw data to degrees (0-360 scale)
                float degrees = static_cast<float>(sr_data[i].present_position) * 360.0f / 4095.0f;
                positions[i] = degrees;
    
                #ifdef DEBUG
                    Debug::infoln("Received position: " + String(degrees));
                #endif
            } else {
                // Handle error by setting position to 0 and marking failure
                positions[i] = 0.0f;
                allSuccess = false;
            }
        }
    }

    else {
        allSuccess = false;

        for (uint8_t i = 0; i < DXLAmount; ++i) {
            positions[i] = 0.0f;
            servos[i]._lastErrorCode = DXL_LIB_ERROR_TIMEOUT;
        }

        #ifdef DEBUG
            Debug::errorln("[SyncRead] Fail, Lib error code: " + String(dxl.getLastLibErrCode()));
        #endif
    }

    return allSuccess;
}


#endif // DXL_SERVO_CLASS_H

#ifndef SHAREDSERVODATA_H
#define SHAREDSERVODATA_H

#include <DynamixelShield.h>


void InitServoDataMutexes();

/**
 * @brief Sets the goal servo position in an array, will also set the goal positions flag to true | this has to manually be turned to false again
 * @param id The id of the array you want to get, the id is the equvalent to the servo order, 0 will return doing nothing.
 * @param position The position to be set in the array, in degrees, between 0 and 360
 */
void SetGoalPosition(uint8_t id, float position);

/**
 * @brief Gets the current servo position in an array
 * @param id The id of the array you want to get the id is the equvalent to the servo order, servo ids start at 1, 0 will return -1.0f
 * @return The degree between 0 and 360, returns -1.0f if the id is too high
 */
float GetGoalPosition(uint8_t id);

/**
 * @brief Gets the flag for goal position for the id
 * @param id The id of the array you want to get the id is the equvalent to the servo order, servo ids start at 1, 0 will return false
 * @return The flag of the id
 */
bool GetGoalPositionFlag(uint8_t id);

/**
 * @brief Gets the flag for goal position for the id
 * @param id The id of the array you want to get the id is the equvalent to the servo order, servo ids start at 1, 0 will return doing nothing
 */
void SetGoalPositionFlag(uint8_t id, bool flag);

/**
 * @brief Sets the current servo position in an array
 * @param id The id of the array you want to get the id is the equvalent to the servo order except the gripper, 0 will return doing nothing
 * @param position The position to be set in the array, in degrees, between 0 and 360
 */
void SetCurrentPosition(uint8_t id, float position);

/**
 * @brief Gets the current servo position in an array
 * @param id The id of the array you want to get the id is the equvalent to the servo order except the gripper, 0 will return -1.0f
 * @return The degree between 0 and 360, returns -1.0f if the id is too high
 */
float GetCurrentPosition(uint8_t id);


/**
 * @brief Sets the current error code in an array
 * @param id The id of the array you want to get the id is the equvalent to the servo order except the gripper, 0 will return doing nothing
 * @param position The position to be set in the array, in degrees, between 0 and 360
 */
void SetCurrentErrorCode(uint8_t id, DXLLibErrorCode_t errorCode);

/**
 * @brief Gets the current servo position in an array
 * @param id The id of the array you want to get the id is the equvalent to the servo order except the gripper, 0 will return DXL_LIB_ERROR_NULLPTR
 * @return The current servo error, DXL_LIB_OK = 0 = means its OK
 */
DXLLibErrorCode_t GetCurrentErrorCode(uint8_t id);

#endif
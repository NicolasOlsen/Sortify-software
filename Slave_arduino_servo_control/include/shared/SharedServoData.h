#ifndef SHAREDSERVODATA_H
#define SHAREDSERVODATA_H

#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

extern DynamixelShield dxl;

extern float goalPositions[5];

extern float currentPositions[4];

extern DXLLibErrorCode_t servoErrors[4];

void InitServoDataMutexes();

/**
 * @brief Sets the goal servo position in an array
 * @param index The index of the array you want to get, the index is the equvalent to the servo order, 0 will return doing nothing.
 * @param position The position to be set in the array, in degrees, between 0 and 360
 */
void SetGoalPosition(uint8_t index, float position);

/**
 * @brief Gets the current servo position in an array
 * @param index The index of the array you want to get the index is the equvalent to the servo order, servo ids start at 1, 0 will return error
 * @return The degree between 0 and 360, returns -1.0f if the index is too high
 */
float GetGoalPosition(uint8_t index);


/**
 * @brief Sets the current servo position in an array
 * @param index The index of the array you want to get the index is the equvalent to the servo order except the gripper, 0 will return doing nothing
 * @param position The position to be set in the array, in degrees, between 0 and 360
 */
void SetCurrentPosition(uint8_t index, float position);

/**
 * @brief Gets the current servo position in an array
 * @param index The index of the array you want to get the index is the equvalent to the servo order except the gripper, 0 will return error
 * @return The degree between 0 and 360, returns -1.0f if the index is too high
 */
float GetCurrentPosition(uint8_t index);


/**
 * @brief Sets the current error code in an array
 * @param index The index of the array you want to get the index is the equvalent to the servo order except the gripper, 0 will return doing nothing
 * @param position The position to be set in the array, in degrees, between 0 and 360
 */
void SetCurrentErrorCode(uint8_t index, DXLLibErrorCode_t errorCode);

/**
 * @brief Gets the current servo position in an array
 * @param index The index of the array you want to get the index is the equvalent to the servo order except the gripper, 0 will return error
 * @return The current servo error, DXL_LIB_OK = 0 = means its OK
 */
DXLLibErrorCode_t GetCurrentErrorCode(uint8_t index);

#endif
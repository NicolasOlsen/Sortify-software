#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "shared/SharedServoData.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr uint8_t goalPositionsSize = 5;        // Amount of servos
constexpr uint8_t currentPositionsSize = 4;     // Amount of readable servos wit position | this case same as dynamixel
constexpr uint8_t servoErrorsSize = 4;          // Amount of dynamixel servos

float goalPositions[goalPositionsSize] = {0};           // Servo setter uses these values
bool goalPositionsFlags[goalPositionsSize] = {0};       // Flags for reading wether the positions has changed

float currentPositions[currentPositionsSize] = {0};
DXLLibErrorCode_t servoErrors[4] = {0};

SemaphoreHandle_t xGoalMutex;
SemaphoreHandle_t xCurrentMutex;
SemaphoreHandle_t xErrorMutex;

void InitServoDataMutexes() {
    xGoalMutex = xSemaphoreCreateMutex();
    xCurrentMutex = xSemaphoreCreateMutex();
    xErrorMutex = xSemaphoreCreateMutex();
}


void SetGoalPosition(uint8_t id, float position) {
    id--;    // To follow the servo id
    if (id >= goalPositionsSize) return;
    if (xGoalMutex && xSemaphoreTake(xGoalMutex, portMAX_DELAY)) {
        goalPositions[id] = position;
        goalPositionsFlags[id] = true;      // The position has changed and will therefore mark it as true
        Debug::infoln(String(id+1) + " set goal pos " + String(position), DEBUG_MODE);
        xSemaphoreGive(xGoalMutex);
    }
}

float GetGoalPosition(uint8_t id) {
    id--;
    if (id >= goalPositionsSize) return -1.0f;     // -1.0f signals something went wrong
    float position = -1.0f;
    if (xGoalMutex && xSemaphoreTake(xGoalMutex, portMAX_DELAY)) {
        position = goalPositions[id];
        Debug::infoln(String(id+1) + " got goal pos " + String(position), DEBUG_MODE);
        xSemaphoreGive(xGoalMutex);
    }
    return position;
}

bool GetGoalPositionFlag(uint8_t id) {
    id--;    // To follow the servo id
    if (id >= goalPositionsSize) return false;
    bool flag = false;
    if (xGoalMutex && xSemaphoreTake(xGoalMutex, portMAX_DELAY)) {
        flag = goalPositionsFlags[id];
        Debug::infoln(String(id+1) + " get flag " + String(flag), DEBUG_MODE);
        xSemaphoreGive(xGoalMutex);
    }
    return flag;
}

void SetGoalPositionFlag(uint8_t id, bool flag) {
    id--;    // To follow the servo id
    if (id >= goalPositionsSize) return;
    if (xGoalMutex && xSemaphoreTake(xGoalMutex, portMAX_DELAY)) {
        goalPositionsFlags[id] = flag;      // The position has changed and will therefore mark it as true
        Debug::infoln(String(id+1) + " set flag " + String(flag), DEBUG_MODE);
        xSemaphoreGive(xGoalMutex);
    }
}


void SetCurrentPosition(uint8_t id, float position) {
    id--;
    if (id >= currentPositionsSize) return;
    if (xCurrentMutex && xSemaphoreTake(xCurrentMutex, portMAX_DELAY)) {
        currentPositions[id] = position;
        Debug::infoln(String(id+1) + " set current pos " + String(position), DEBUG_MODE);
        xSemaphoreGive(xCurrentMutex);
    }
}

float GetCurrentPosition(uint8_t id) {
    id--;
    if (id >= currentPositionsSize) return -1.0f;     // -1.0f signals something went wrong
    float position = -1.0f;
    if (xCurrentMutex && xSemaphoreTake(xCurrentMutex, portMAX_DELAY)) {
        position = currentPositions[id];
        Debug::infoln(String(id+1) + " get current pos " + String(position), DEBUG_MODE);
        xSemaphoreGive(xCurrentMutex);
    }
    return position;
}


void SetCurrentErrorCode(uint8_t id, DXLLibErrorCode_t errorCode) {
    id--;
    if (id >= servoErrorsSize) return;

    if (xErrorMutex && xSemaphoreTake(xErrorMutex, portMAX_DELAY)) {
        servoErrors[id] = errorCode;
        Debug::infoln(String(id+1) + " set error " + String(errorCode), DEBUG_MODE);
        xSemaphoreGive(xErrorMutex);
    }
}

DXLLibErrorCode_t GetCurrentErrorCode(uint8_t id) {
    id--;
    if (id >= servoErrorsSize) return DXL_LIB_ERROR_NULLPTR;

    DXLLibErrorCode_t errorCode = DXL_LIB_ERROR_NULLPTR;

    if (xErrorMutex && xSemaphoreTake(xErrorMutex, portMAX_DELAY)) {
        errorCode = servoErrors[id];
        Debug::infoln(String(id+1) + " get error " + String(errorCode), DEBUG_MODE);
        xSemaphoreGive(xErrorMutex);
    }
    return errorCode;
}
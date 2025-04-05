#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "shared/SharedServoData.h"

DynamixelShield dxl;

constexpr uint8_t goalPositionsSize = 5;        // Amount of servos
constexpr uint8_t currentPositionsSize = 4;     // Amount of readable servos | this case same as dynamixel
constexpr uint8_t servoErrorsSize = 4;          // Amount of dynamixel servos

float goalPositions[goalPositionsSize] = {0};
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
        xSemaphoreGive(xGoalMutex);
  }
}

float GetGoalPosition(uint8_t id) {
    id--;
    if (id >= goalPositionsSize) return -1.0f;     // -1.0f signals something went wrong
    float pos = -1.0f;
    if (xGoalMutex && xSemaphoreTake(xGoalMutex, portMAX_DELAY)) {
        pos = goalPositions[id];
        xSemaphoreGive(xGoalMutex);
    }
    return pos;
}


void SetCurrentPosition(uint8_t id, float position) {
    id--;
    if (id >= currentPositionsSize) return;
    if (xCurrentMutex && xSemaphoreTake(xCurrentMutex, portMAX_DELAY)) {
        currentPositions[id] = position;
        xSemaphoreGive(xCurrentMutex);
    }
}

float GetCurrentPosition(uint8_t id) {
    id--;
    if (id >= currentPositionsSize) return -1.0f;     // -1.0f signals something went wrong
    float pos = -1.0f;
    if (xCurrentMutex && xSemaphoreTake(xCurrentMutex, portMAX_DELAY)) {
        pos = currentPositions[id];
        xSemaphoreGive(xCurrentMutex);
    }
    return pos;
}


void SetCurrentErrorCode(uint8_t id, DXLLibErrorCode_t errorCode) {
    id--;
    if (id >= servoErrorsSize) return;

    if (xErrorMutex && xSemaphoreTake(xErrorMutex, portMAX_DELAY)) {
        servoErrors[id] = errorCode;
        xSemaphoreGive(xErrorMutex);
    }
}

DXLLibErrorCode_t GetCurrentErrorCode(uint8_t id) {
    id--;
    if (id >= servoErrorsSize) return DXL_LIB_ERROR_NULLPTR;

    DXLLibErrorCode_t errorCode = DXL_LIB_ERROR_NULLPTR;

    if (xErrorMutex && xSemaphoreTake(xErrorMutex, portMAX_DELAY)) {
        errorCode = servoErrors[id];
        xSemaphoreGive(xErrorMutex);
    }
    return errorCode;
}
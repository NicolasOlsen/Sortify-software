#include "shared/System_status.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

using namespace Com_code;

SemaphoreHandle_t mutex;

StatusCode systemState = StatusCode::INITIALIZING;

namespace System_status
{
    void InitSystemStatusMutex() {
        mutex = xSemaphoreCreateMutex();
    }

    StatusCode GetSystemState() {
        StatusCode state = StatusCode::FAULT;
        if (mutex && xSemaphoreTake(mutex, portMAX_DELAY)) {
            state = systemState;
            xSemaphoreGive(mutex);
        }
        return state;
    }

    void SetSystemState(StatusCode state) {
        if (mutex && xSemaphoreTake(mutex, portMAX_DELAY)) {
            systemState = state;
            xSemaphoreGive(mutex);
        }
    }
}
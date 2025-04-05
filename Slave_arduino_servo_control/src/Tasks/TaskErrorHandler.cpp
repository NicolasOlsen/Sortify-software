#include <Arduino_FreeRTOS.h>

#include "tasks/TaskErrorHandler.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr UBaseType_t task_priority = 1;  // Low priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(400);  // Periodic polling interval

static void TaskErrorHandler(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        Debug::infoln("[ErrorHnadler] Running...", DEBUG_MODE);

        vTaskDelayUntil(&lastWakeTime, TASK_PERIOD);
    }
  }

void createTaskErrorHandler() {
    xTaskCreate(
        TaskErrorHandler,
        "Tuner",
        256,
        NULL,
        task_priority,
        NULL
    );
  }
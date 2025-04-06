#include <Arduino_FreeRTOS.h>

#include "tasks/TaskThink.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr UBaseType_t task_priority = 0;  // Lowest priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(1000); // Periodic polling interval

static void TaskThink(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    Debug::infoln("[T_Think] started", DEBUG_MODE);

    for (;;) {
        Debug::infoln("[T_Think]", DEBUG_MODE);

        vTaskDelayUntil(&lastWakeTime, TASK_PERIOD);
    }
  }

void createTaskThink() {
    xTaskCreate(
        TaskThink,
        "Think",
        256,
        NULL,
        task_priority,
        NULL
    );
  }
#include <Arduino_FreeRTOS.h>

#include "tasks/TaskServoTuner.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr UBaseType_t task_priority = 0;  // Lowest priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(1000); // Periodic polling interval

static void TaskServoTuner(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        Debug::infoln("[Tuner] Running...", DEBUG_MODE);

        vTaskDelayUntil(&lastWakeTime, TASK_PERIOD);
    }
  }

void createTaskServoTuner() {
    xTaskCreate(
        TaskServoTuner,
        "Tuner",
        256,
        NULL,
        task_priority,
        NULL
    );
  }
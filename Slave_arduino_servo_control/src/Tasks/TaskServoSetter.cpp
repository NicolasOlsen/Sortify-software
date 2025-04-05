#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

#include "tasks/TaskServoSetter.h"
#include "shared/SharedServoData.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr UBaseType_t task_priority = 2;                // Medium priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(2000); // Periodic polling interval

static void TaskServoSetter(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  bool toggle = true;

  for (;;) {
    for (size_t id = 1; id <= 4; id++) {
      if (toggle) {
        dxl.setGoalPosition(id, GetGoalPosition(id), UNIT_DEGREE);
      }
      else {
        dxl.setGoalPosition(id, 360.0, UNIT_DEGREE);
      }
    }

    toggle = !toggle;

    vTaskDelayUntil(&lastWakeTime, TASK_PERIOD);
  }
}

void createTaskServoSetter() {
    xTaskCreate(
        TaskServoSetter,
        "Setter",
        256,
        NULL,
        task_priority,   
        NULL
    );
  }
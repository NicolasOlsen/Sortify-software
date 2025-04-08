#include <Arduino_FreeRTOS.h>

#include "tasks/TaskServoReader.h"
#include "shared/SharedServoState.h"
#include "control/ServoControl.h"

#include "utils/Debug.h"

using namespace ControlTableItem;

constexpr bool LOCAL_DEBUG = true;

constexpr UBaseType_t task_priority = 2;                // High priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(600);  // Periodic polling interval

static void TaskServoReader(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  Debug::infoln("[T_Reader] started", LOCAL_DEBUG);

  for (;;) {
    Debug::infoln("[T_Reader]", LOCAL_DEBUG);

    ServoControl::StoreCurrentServoPositions(currentPositions);

    vTaskDelayUntil(&lastWakeTime, TASK_PERIOD);
  }
}

void createTaskServoReader() {
    xTaskCreate(
        TaskServoReader,
        "Reader",
        256,
        NULL,
        task_priority,
        NULL
    );
  }


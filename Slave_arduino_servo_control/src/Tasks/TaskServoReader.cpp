#include <Arduino_FreeRTOS.h>

#include "tasks/TaskServoReader.h"
#include "shared/SharedServoData.h"

#include "utils/Debug.h"

using namespace ControlTableItem;

constexpr bool DEBUG_MODE = true;

constexpr UBaseType_t task_priority = 3;                // High priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(400);  // Periodic polling interval

static void TaskServoReader(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  float currentPosition;
  DXLLibErrorCode_t lastError;

  for (;;) {
    for (size_t id = 1; id <= 4; id++) {

      currentPosition = dxl.getPresentPosition(id, UNIT_DEGREE);

      if (dxl.getLastLibErrCode() != DXL_LIB_OK) {
          lastError = dxl.getLastLibErrCode();
          SetCurrentErrorCode(id, lastError);
      } else {
          SetCurrentPosition(id, currentPosition);
      }
    }

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


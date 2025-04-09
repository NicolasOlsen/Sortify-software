#include <Arduino_FreeRTOS.h>

#include "rtos_tasks/task_servo_reader.h"
#include "config/task_config.h"
#include "shared/shared_servo_state.h"
#include "control/servo_control.h"

#include "utils/debug.h"

using namespace ControlTableItem;

constexpr bool LOCAL_DEBUG = true;

static void TaskServoReader(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  Debug::infoln("[T_Reader] started", LOCAL_DEBUG);

  for (;;) {
    Debug::infoln("[T_Reader]", LOCAL_DEBUG);

    ServoControl::StoreCurrentServoPositions(currentPositions);

    vTaskDelayUntil(&lastWakeTime, READ_TASK.period);
  }
}

void createTaskServoReader() {
    xTaskCreate(
        TaskServoReader,
        "Reader",
        READ_TASK.stackSize,
        NULL,
        READ_TASK.priority,
        NULL
    );
  }


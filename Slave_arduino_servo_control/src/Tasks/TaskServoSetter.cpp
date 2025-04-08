#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

#include "tasks/TaskServoSetter.h"
#include "control/ServoControl.h"
#include "shared/System_status.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr UBaseType_t task_priority = 1;                // Medium priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(1000); // Periodic polling interval

static void TaskServoSetter(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  Debug::infoln("[T_Setter] started", DEBUG_MODE);

  for (;;) {
    Debug::infoln("[T_Setter]", DEBUG_MODE);

    ServoControl::SetServosToGoalPosition();

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


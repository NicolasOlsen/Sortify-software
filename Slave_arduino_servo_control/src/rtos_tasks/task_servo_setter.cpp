#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

#include "rtos_tasks/task_servo_setter.h"
#include "config/task_config.h"
#include "control/servo_control.h"
#include "shared/shared_servo_state.h"

#include "utils/Debug.h"

constexpr bool LOCAL_DEBUG = true;

static void TaskServoSetter(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  Debug::infoln("[T_Setter] started", LOCAL_DEBUG);

  for (;;) {
    Debug::infoln("[T_Setter]", LOCAL_DEBUG);

    ServoControl::SetServosToPosition(goalPositions);

    vTaskDelayUntil(&lastWakeTime, SET_TASK.period);
  }
}

void createTaskServoSetter() {
  xTaskCreate(
      TaskServoSetter,
      "Setter",
      SET_TASK.stackSize,
      NULL,
      SET_TASK.priority,   
      NULL
  );
}


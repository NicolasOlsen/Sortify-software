#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

#include "rtos_tasks/task_servo_setter.h"
#include "config/task_config.h"
#include "shared/shared_objects.h"

#include "utils/Debug.h"

constexpr bool LOCAL_DEBUG = true;

static void TaskServoSetter(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  auto& manager = Shared::servoManager;

  float tempGoalPositions[manager.getTotalAmount()];

  Debug::infoln("[T_Setter] started", LOCAL_DEBUG);

  for (;;) {
    Debug::infoln("[T_Setter]", LOCAL_DEBUG);

    // Applies goal positions to servos only if their update flag is set.
    // This avoids redundant communication with unchanged servos.
    Shared::goalPositions.Get(tempGoalPositions, manager.getTotalAmount());
    for (uint8_t id = 0; id < manager.getTotalAmount(); id++) {
      if (Shared::goalPositions.GetFlag(id)) {
        if (manager.setGoalPosition(id, tempGoalPositions[id])) {
          Shared::goalPositions.SetFlag(id, false);
        }
      }
    }    

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


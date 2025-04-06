#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

#include "tasks/TaskServoSetter.h"
#include "shared/SharedServoData.h"
#include "control/ServoControl.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr UBaseType_t task_priority = 1;                // Medium priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(1000); // Periodic polling interval

constexpr uint8_t smartServos = 4;  // Note: this is assuming every servo 4 and less is smart servos
constexpr uint8_t totalServos = 5;  // Assuming the rest is analog servos

static void TaskServoSetter(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  Debug::infoln("[T_Setter] started", DEBUG_MODE);

  for (;;) {
    Debug::infoln("[T_Setter]", DEBUG_MODE);

    for (uint8_t id = 1; id <= totalServos; id++) {
      if (id <= smartServos) {
        SetSmartServoPosition(id); 
      }
      else {
        SetAnalogServoPosition(id);
      }
    }

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


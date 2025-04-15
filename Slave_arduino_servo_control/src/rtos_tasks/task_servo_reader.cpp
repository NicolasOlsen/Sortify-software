#include <Arduino_FreeRTOS.h>

#include "rtos_tasks/task_servo_reader.h"
#include "config/task_config.h"
#include "shared/shared_objects.h"

#include "utils/debug.h"

constexpr bool LOCAL_DEBUG = true;

static void TaskServoReader(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  Debug::infoln("[T_Reader] started", LOCAL_DEBUG);

  auto& manager = Shared::servoManager;

  float tempCurrentPositions[manager.getTotalAmount()];

  DXLLibErrorCode_t tempErrors[manager.getDXLAmount()];

  for (;;) {
    Debug::infoln("[T_Reader]", LOCAL_DEBUG);

    // Read positions from DXL servos
    manager.getCurrentPositions(
      tempCurrentPositions, 
      manager.getDXLAmount());

    // Note: If reading a DXL servo fails, the Dynamixel library returns 0.0f.
    // The value is still written to Shared::currentPositions, and the error is tracked separately.
    // The system will handle errors based on the Shared::servoErrors.

    // Read get errors from DXL servos
    manager.getErrors(
      tempErrors,
      manager.getDXLAmount());

    // Analog servos cant be read directly, use goal positions as their "current state"
    Shared::goalPositions.Get(
      &tempCurrentPositions[manager.getDXLAmount()], 
      manager.getAnalogAmount(),
      manager.getDXLAmount()
    );

    // Store combined current positions into shared memory
    Shared::currentPositions.Set(tempCurrentPositions, manager.getTotalAmount());

    // Store errors from dxl servos
    Shared::servoErrors.Set(tempErrors, manager.getDXLAmount());

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

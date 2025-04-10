#include <Arduino_FreeRTOS.h>

#include "rtos_tasks/task_think.h"
#include "config/task_config.h"
#include "shared/shared_servo_state.h"
#include "shared/System_status.h"

#include "utils/Debug.h"

constexpr bool LOCAL_DEBUG = true;

void CheckAndChangeSystemState();
void CheckMovingStatus();

static void TaskThink(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	Debug::infoln("[T_Think] started", LOCAL_DEBUG);

	for (;;) {
		Debug::infoln("[T_Think]", LOCAL_DEBUG);

		CheckAndChangeSystemState();
		
		vTaskDelayUntil(&lastWakeTime, THINK_TASK.period);
	}
  }

void createTaskThink() {
	xTaskCreate(
		TaskThink,
		"Think",
		THINK_TASK.stackSize,
		NULL,
		THINK_TASK.priority,
		NULL
	);
}

void CheckAndChangeSystemState() {
	bool isError = false;

	for (uint8_t id = 1; id <= SMART_SERVO_COUNT; id++)
	{
		DXLLibErrorCode_t error = servoErrors.Get(id);
		if (error != DXL_LIB_OK) {
			System_status::systemState.Set(StatusCode::FAULT);
			isError = true;
		break;
		}
	}

	if (isError) {
		for(uint8_t id = 1; id <= SMART_SERVO_COUNT; id++) {
			goalPositions.Set(id, currentPositions.Get(id));
		}
	}
	else {
		System_status::systemState.Set(StatusCode::IDLE);
	}
}
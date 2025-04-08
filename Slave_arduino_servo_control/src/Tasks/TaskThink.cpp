#include <Arduino_FreeRTOS.h>

#include "tasks/TaskThink.h"
#include "shared/SharedServoState.h"
#include "shared/System_status.h"

#include "utils/Debug.h"

constexpr bool LOCAL_DEBUG = true;

constexpr UBaseType_t task_priority = 0;  // Lowest priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(1000); // Periodic polling interval

constexpr uint8_t smartServos = 4;      // To go through all the smart servo errors

void CheckAndChangeSystemState();
void CheckMovingStatus();

static void TaskThink(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	Debug::infoln("[T_Think] started", LOCAL_DEBUG);

	for (;;) {
		Debug::infoln("[T_Think]", LOCAL_DEBUG);

		CheckAndChangeSystemState();
		
		vTaskDelayUntil(&lastWakeTime, TASK_PERIOD);
	}
  }

void createTaskThink() {
	xTaskCreate(
		TaskThink,
		"Think",
		256,
		NULL,
		task_priority,
		NULL
	);
}

void CheckAndChangeSystemState() {
	bool isError = false;

	for (uint8_t id = 1; id < smartServos; id++)
	{
		DXLLibErrorCode_t error = servoErrors.Get(id);
		if (error != DXL_LIB_OK) {
		System_status::SetSystemState(StatusCode::FAULT);
		isError = true;
		break;
		}
	}

	if (isError) {
		for(uint8_t id = 1; id <= smartServos; id++) {
			goalPositions.Set(id, currentPositions.Get(id));
		}
	}
	else {
		System_status::SetSystemState(StatusCode::IDLE);
	}
}
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
		auto timer = millis();
		Debug::infoln("[T_Reader]", LOCAL_DEBUG);

		StatusCode tempStatus = Shared::systemState.Get();

		if (tempStatus == StatusCode::FAULT) {
			// Zero out entire array to prevent garbage values for DXL section
			memset(tempCurrentPositions, 0, sizeof(tempCurrentPositions));
			
			// Ping DXL servos to keep them responsive and detect recovery,
			// but avoid full reads to minimize task time in FAULT mode
			manager.pingAll();

			// Use last-known goals as fallback "current" for analog servos
			Shared::goalPositions.Get(
				&tempCurrentPositions[manager.getDXLAmount()],
				manager.getAnalogAmount(),
				manager.getDXLAmount()
			);

			// Update current positions with partial valid data
			Shared::currentPositions.Set(tempCurrentPositions, manager.getTotalAmount());
		}
		else {
			// Normal operation: fetch real positions and error codes
			manager.getCurrentPositions(tempCurrentPositions, manager.getDXLAmount());
			manager.getErrors(tempErrors, manager.getDXLAmount());

			// Add analog servo positions based on goal fallback
			Shared::goalPositions.Get(
				&tempCurrentPositions[manager.getDXLAmount()],
				manager.getAnalogAmount(),
				manager.getDXLAmount()
			);

			Shared::currentPositions.Set(tempCurrentPositions, manager.getTotalAmount());
			Shared::servoErrors.Set(tempErrors, manager.getDXLAmount());
		}

		// Adjust task frequency based on current state
		TickType_t delayTicks = (tempStatus == StatusCode::FAULT) 
			? READ_TASK.period * 2
			: READ_TASK.period;

		Serial.println("Re Timer: " + String(millis() - timer));

		vTaskDelayUntil(&lastWakeTime, delayTicks);
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

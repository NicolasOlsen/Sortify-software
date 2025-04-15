#include <stdint.h>
#include <Arduino_FreeRTOS.h>

#include "rtos_tasks/task_think.h"
#include "config/task_config.h"
#include "shared/shared_objects.h"

#include "utils/Debug.h"

constexpr bool LOCAL_DEBUG = true;

static void TaskThink(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	auto& manager = Shared::servoManager;

	// Temporary buffers for current errors and positions
	DXLLibErrorCode_t tempErrors[manager.getDXLAmount()] = {0};
	float tempPositions[manager.getTotalAmount()] = {0};
	uint8_t errorCount[manager.getDXLAmount()] = {0};

	bool systemStateOK = true;

	Debug::infoln("[T_Think] started", LOCAL_DEBUG);

	for (;;) {
		Debug::infoln("[T_Think]", LOCAL_DEBUG);

		// Read all error codes from shared buffer
		Shared::servoErrors.Get(
			tempErrors, 
			manager.getDXLAmount());

		systemStateOK = true;
		
		// Check errors for each Dynamixel servo
		for (uint8_t id = 0; id < manager.getDXLAmount(); id++) {
			if (tempErrors[id] == DXL_LIB_OK) {
				errorCount[id] = 0;  // Reset on successful status
			}
			else if (tempErrors[id] == DXL_LIB_ERROR_CHECK_SUM ||
				     tempErrors[id] == DXL_LIB_ERROR_CRC) {
				
				// If the same transient error happens too many times, consider it faulty
				if (errorCount[id] > MAX_ERRORS) {
					systemStateOK = false;
					
					// Disable further updates for this servo (prevents flooding with retries)
					Shared::goalPositions.SetFlag(id, false);

					Debug::warnln("Servo " + String(id) + " exceeded max retries. Disabling goal updates.");
				} else {
					++errorCount[id];  // Increment retry attempt
				}
			}
			else {
				// Any other (more serious) error results in immediate fault state
				Shared::goalPositions.SetFlag(id, false);
				systemStateOK = false;
				Debug::errorln("Critical error on servo ID: " + String(id), LOCAL_DEBUG);
			}
		}

		// Enter FAULT state if errors are detected and we are not already in fault mode
		if (!systemStateOK && Shared::systemState.Get() != StatusCode::FAULT) {
			Debug::errorln("[T_Think] Switching to FAULT state", LOCAL_DEBUG);

			// Copy current positions to goal buffer to freeze movement
			Shared::currentPositions.Get(
				tempPositions, 
				manager.getTotalAmount());
			Shared::goalPositions.Set(
				tempPositions, 
				manager.getTotalAmount());

			Shared::systemState.Set(StatusCode::FAULT);
		}

		// Wait until the next period
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

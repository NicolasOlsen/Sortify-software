#include <stdint.h>
#include <Arduino_FreeRTOS.h>
#include <math.h>

#include "rtos_tasks/task_think.h"
#include "config/task_config.h"
#include "shared/shared_objects.h"

#include "utils/Debug.h"

constexpr bool LOCAL_DEBUG = true;

auto& manager = Shared::servoManager;

float tempCurrentPositions[manager.getTotalAmount()] = {0};
float tempGoalPositions[manager.getTotalAmount()] = {0};

constexpr float tolerance = 0.2f;

void checkForErrors();
void checkForMovement();

static void TaskThink(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	Debug::infoln("[T_Think] started", LOCAL_DEBUG);

	auto timer = millis();

	for (;;) {
	  	timer = millis();

	  	Debug::infoln("[T_Think]", LOCAL_DEBUG);

		checkForErrors();
		
		if (Shared::systemState.Get() != StatusCode::FAULT) {
			checkForMovement();
		}

		Serial.println("Th Timer: " + String(millis() - timer));

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

void checkForMovement() {
	// Retrieve current servo positions and goal positions from shared memory
	Shared::currentPositions.Get(
		tempCurrentPositions, 
		manager.getTotalAmount());
	Shared::goalPositions.Get(
		tempGoalPositions, 
		manager.getTotalAmount());

	bool idle = true;

	// Compare each servo's current position to its goal position
	// If any servo is outside the allowed movement tolerance, the system is still moving
	for (uint8_t id = 0; id < manager.getTotalAmount(); ++id) {
		if (fabsf(tempCurrentPositions[id] - tempGoalPositions[id]) > tolerance) {
			idle = false;
			break;  // Early exit if any servo is still in motion
		}
	}

	// Set system state based on movement status
	if (idle) {
		Shared::systemState.Set(StatusCode::IDLE);
		Debug::infoln("[T_Think] Switching to IDLE state", LOCAL_DEBUG);
	}
	else {
		Shared::systemState.Set(StatusCode::MOVING);
		Debug::infoln("[T_Think] Switching to MOVING state", LOCAL_DEBUG);
	}
}


void checkForErrors() {
	// Temporary buffer for DXL error codes
	DXLLibErrorCode_t tempErrors[manager.getDXLAmount()] = {0};

	// Persistent error count for each DXL servo across task cycles
	static uint8_t errorCount[manager.getDXLAmount()] = {0};

	// Read latest servo error states from shared memory
	Shared::servoErrors.Get(
		tempErrors, 
		manager.getDXLAmount());

	// Used to determine if system should remain operational or enter FAULT
	bool systemStateOK = true;

	// Temporary copy of update flags for goal positions
	bool updatedFlags[manager.getTotalAmount()];
	Shared::goalPositions.GetFlags(updatedFlags, manager.getTotalAmount());

	// Loop through all Dynamixel servos to check their error status
	for (uint8_t id = 0; id < manager.getDXLAmount(); id++) {
		if (tempErrors[id] == DXL_LIB_OK) {
			// Reset error counter on successful communication
			errorCount[id] = 0;
		}
		else if (tempErrors[id] == DXL_LIB_ERROR_CHECK_SUM ||
				 tempErrors[id] == DXL_LIB_ERROR_CRC) {
			
			// Handle recoverable/transient errors (communication noise)
			if (errorCount[id] > MAX_ERRORS) {
				systemStateOK = false;

				// Disable goal updates for this servo to stop retry spam
				updatedFlags[id] = false;

				Debug::warnln("Servo " + String(id) + " exceeded max retries. Disabling goal updates.");
			} else {
				// Increment retry counter
				++errorCount[id];
			}
		}
		else {
			// Unrecoverable or unexpected error (hardware fault)
			updatedFlags[id] = false;
			systemStateOK = false;

			Debug::errorln("Critical error on servo ID: " + String(id), LOCAL_DEBUG);
		}
	}

	// Write updated flags back to shared buffer all at once, mutex-safe
	Shared::goalPositions.SetFlags(updatedFlags, manager.getTotalAmount());

	// If the system is NOT OK
	if (!systemStateOK) {
		Debug::errorln("[T_Think] Switching to FAULT state", LOCAL_DEBUG);

		// Freeze all movement by copying current positions as new goals
		Shared::currentPositions.Get(
			tempCurrentPositions, 
			manager.getTotalAmount());
		Shared::goalPositions.Set(
			tempCurrentPositions, 
			manager.getTotalAmount(), 0, false);

		// System goes into FAULT mode
		Shared::systemState.Set(StatusCode::FAULT);
	}
}

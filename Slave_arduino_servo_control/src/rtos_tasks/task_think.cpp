#include <stdint.h>
#include <Arduino_FreeRTOS.h>
#include <math.h>

#include "rtos_tasks/task_think.h"
#include "config/task_config.h"
#include "shared/shared_objects.h"

#include "utils/task_timer.h"
#include "utils/debug_utils.h"

using namespace COMM_CODE;

#ifdef TIMING_MODE
    static TaskTimingStats commTiming;
#endif

auto& manager = Shared::servoManager;

float tempCurrentPositions[manager.getTotalAmount()];
float tempGoalPositions[manager.getTotalAmount()];

constexpr float idleTolerance = 1.0f;

void checkForErrors(StatusCode status);
void checkForMovement();

static void TaskThink(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	Debug::infoln("[T_Think] started");

	for (;;) {
	  	Debug::infoln("[T_Think]");

		#ifdef TIMING_MODE
		  uint32_t startMicros = micros();
	  	#endif

		StatusCode status = Shared::systemState.Get();
		if (status != StatusCode::FAULT_INIT) {
			checkForErrors(status);
		}

		switch(Shared::systemState.Get()) {
			case StatusCode::INITIALIZING: {
				break;
			}
			case StatusCode::IDLE: {
				checkForMovement();
				break;
			}

			case StatusCode::MOVING: {
				checkForMovement();
				break;
			}

			case StatusCode::FAULT_INIT: {
				break;
			}

			case StatusCode::FAULT_RUNTIME: {
				break;
			}
		}

		#ifdef TIMING_MODE
            uint32_t duration = micros() - startMicros;
            commTiming.update(duration);
        
            if (commTiming.runCount >= TIMING_SAMPLE_COUNT) {
                commTiming.printTimingStats("Thinker");
                commTiming.reset();

                // Long delay to simulate less frequent task execution in TIMING_MODE
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TIMING_DELAY_TASKS));
            }
        #else
            // Wait until the next period
			vTaskDelayUntil(&lastWakeTime, THINK_TASK.period);
        #endif
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
		if (fabsf(tempCurrentPositions[id] - tempGoalPositions[id]) > idleTolerance) {
			idle = false;
			break;  // Early exit if any servo is still in motion
		}
	}

	// Set system state based on movement status
	if (idle) {
		Shared::systemState.Set(StatusCode::IDLE);
		Debug::infoln("[T_Think] Switching to IDLE state");
	}
	else {
		Shared::systemState.Set(StatusCode::MOVING);
		Debug::infoln("[T_Think] Switching to MOVING state");
	}
}


void checkForErrors(StatusCode status) {
	// Temporary buffers for DXL error codes and flags
	DXLLibErrorCode_t tempErrors[manager.getDXLAmount()];
	bool tempErrorFlags[manager.getDXLAmount()];

	// Persistent error count for each DXL servo across task cycles
	static uint8_t errorCount[manager.getDXLAmount()];

	// Read latest servo error states from shared memory
	Shared::currentErrors.Get(tempErrors);

	Shared::currentErrors.GetFlags(tempErrorFlags);

	// Used to determine if system should remain operational or enter FAULT
	bool systemStateOK = true;

	// Loop through all Dynamixel servos to check their error status
	for (uint8_t id = 0; id < manager.getDXLAmount(); id++) {
		// Check flags if changed
		if (tempErrorFlags[id]) {
			tempErrorFlags[id] = false;
		}
		else {	// Dont process error, its not been updated
			continue;
		}

		if (tempErrors[id] == DXL_LIB_OK) {
			// Reset error counter on successful communication
			errorCount[id] = 0;
		}
		else if (tempErrors[id] == DXL_LIB_ERROR_CHECK_SUM ||
				 tempErrors[id] == DXL_LIB_ERROR_CRC) {
			
			// Handle recoverable/transient errors (communication noise)
			if (errorCount[id] > MAX_ERRORS) {
				systemStateOK = false;

				Debug::warnln("Servo " + String(id) + " exceeded max retries. Disabling goal updates.");
			} else {
				// Increment retry counter
				++errorCount[id];
			}
		}
		else {
			// Unrecoverable or unexpected error (hardware fault)
			systemStateOK = false;

			Debug::errorln("Critical error on servo ID: " + String(id));
		}
	}

	Shared::currentErrors.SetFlags(tempErrorFlags);

	if (systemStateOK) {
		if (status == StatusCode::FAULT_RUNTIME) {
			// Shared::systemState.Set(StatusCode::INITIALIZING);
		}
	}
	else {
		Debug::errorln("[T_Think] Switching to FAULT state");

		// Freeze all movement by copying current positions as new goals
		Shared::currentPositions.Get(
			tempCurrentPositions, 
			manager.getTotalAmount());
		Shared::goalPositions.Set(
			tempCurrentPositions, 
			manager.getTotalAmount());

		// System goes into FAULT mode and saves errors if not already
		if (status != StatusCode::FAULT_INIT ||
		status != StatusCode::FAULT_RUNTIME) {
			Shared::systemState.Set(StatusCode::FAULT_RUNTIME);
			Shared::lastErrors.Set(tempErrors);
		}
	}
}

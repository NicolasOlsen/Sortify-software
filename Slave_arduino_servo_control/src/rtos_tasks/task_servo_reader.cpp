#include <Arduino_FreeRTOS.h>

#include "rtos_tasks/task_servo_reader.h"
#include "config/task_config.h"
#include "shared/shared_objects.h"

#include "utils/task_timer.h"
#include "utils/debug_utils.h"

using namespace COMM_CODE;

#ifdef TIMING_MODE
    static TaskTimingStats commTiming;
#endif

static void TaskServoReader(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	Debug::infoln("[T_Reader] started");

	auto& manager = Shared::servoManager;

	static float tempCurrentPositions[manager.getTotalAmount()];
	static DXLLibErrorCode_t tempErrors[manager.getDXLAmount()];

	for (;;) {
		Debug::infoln("[T_Reader]");

		#ifdef TIMING_MODE
			uint32_t startMicros = micros();
		#endif

		StatusCode systemState = Shared::systemState.Get();
		if (systemState == StatusCode::FAULT_INIT ||
			systemState == StatusCode::FAULT_RUNTIME) {			
			// Ping DXL servos to keep them responsive and detect recovery for logging,
			// but avoid full reads to minimize task time in FAULT mode
			manager.pingAll();

			manager.getErrors(tempErrors, manager.getDXLAmount());
			Shared::servoErrors.Set(tempErrors, manager.getDXLAmount());
		}
		else {
			// Normal operation: fetch real positions and error codes
			manager.getCurrentPositions(sliceFirst<float, manager.getDXLAmount()>(tempCurrentPositions));
			manager.getErrors(tempErrors, manager.getDXLAmount());

			// Add analog servo positions based on goal fallback
			Shared::goalPositions.Get(
				&tempCurrentPositions[manager.getDXLAmount()],
				manager.getAnalogAmount(),
				manager.getDXLAmount()
			);

			// Check if all of the servos got read succesfully
			bool allSuccess = true;
			for (uint8_t id = 0; id < manager.getDXLAmount(); id++) {
				if (tempErrors[id] != DXL_LIB_OK) {
					allSuccess = false;
					break;
				}
			}

			// Only save the positions if succesfull, since the syncRead only returns data if all where succesful
			if (allSuccess) {
				Shared::currentPositions.Set(tempCurrentPositions, manager.getTotalAmount());
			} else {
                Debug::warnln("Some servos failed to respond, not updating positions.");
            }

			// Update the servo errors regardless of success
			Shared::servoErrors.Set(tempErrors, manager.getDXLAmount());
		}

		#ifdef TIMING_MODE
            uint32_t duration = micros() - startMicros;
            commTiming.update(duration);
        
            if (commTiming.runCount >= TIMING_SAMPLE_COUNT) {
                commTiming.printTimingStats("Reader");
                commTiming.reset();

                // Long delay to simulate less frequent task execution in TIMING_MODE
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TIMING_DELAY_TASKS));
            }
        #else
            // Wait until the next period
			vTaskDelayUntil(&lastWakeTime, READ_TASK.period);
        #endif
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

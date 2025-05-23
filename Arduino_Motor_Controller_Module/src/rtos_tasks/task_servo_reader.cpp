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

	static uint8_t id = 0;

	for (;;) {
		Debug::infoln("[T_Reader]");

		#ifdef TIMING_MODE
			uint32_t startMicros = micros();
		#endif

		StatusCode systemState = Shared::systemState.Get();
		if (systemState == StatusCode::FAULT_INIT ||
			systemState == StatusCode::FAULT_RUNTIME) {			
			// Ping Dxl servos for diagnostics and logging,
			// avoids full reads to minimize task time in FAULT mode
			manager.ping(id);

			tempErrors[id] = manager.getError(id);
			Shared::currentErrors.Set(id, tempErrors[id]);

			id = (id + 1) % manager.getDXLAmount();
		}
		else {
			// Normal operation, get real positions and error codes
			manager.getCurrentPositions(sliceFirst<float, manager.getDXLAmount()>(tempCurrentPositions));
			manager.getErrors(tempErrors, manager.getDXLAmount());

			// Add analog servo positions based on goal position
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

			// Only save the positions if succesfull, since sync-read only returns data if all where succesful
			if (allSuccess) {
				Shared::currentPositions.Set(tempCurrentPositions, manager.getTotalAmount());
			} else {
                Debug::warnln("Some servos failed to respond, not updating positions.");
            }

			// Update the servo errors regardless of success
			Shared::currentErrors.Set(tempErrors, manager.getDXLAmount());
		}

		#ifdef TIMING_MODE
			uint32_t duration = micros() - startMicros;
			commTiming.update(duration);

			#ifdef INDIVIDUAL_TIMING_MODE
				if (commTiming.runCount >= TIMING_SAMPLE_COUNT) {
					commTiming.printTimingStats("Reader");
					commTiming.reset();
					vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TIMING_DELAY_TASKS));
				}
			#else
				if (commTiming.runCount >= TIMING_SAMPLE_COUNT) {
					commTiming.printTimingStats("Reader");
					commTiming.reset();
				}
		
				vTaskDelayUntil(&lastWakeTime, READ_TASK.period);  // Regular periodic delay
			#endif
		
		#else
			vTaskDelayUntil(&lastWakeTime, READ_TASK.period);      // Not in timing mode at all
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

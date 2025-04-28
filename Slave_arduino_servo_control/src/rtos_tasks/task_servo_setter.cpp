#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

#include "rtos_tasks/task_servo_setter.h"
#include "config/task_config.h"
#include "shared/shared_objects.h"

#include "utils/task_timer.h"
#include "utils/debug_utils.h"

#ifdef TIMING_MODE
    static TaskTimingStats commTiming;
#endif

using namespace COMM_CODE;

static void TaskServoSetter(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	auto& manager = Shared::servoManager;

	float tempGoalPositions[manager.getTotalAmount()];
	float tempGoalVelocities[manager.getDXLAmount()];

	Debug::infoln("[T_Setter] started");

	for (;;) {
		Debug::infoln("[T_Setter]");

		#ifdef TIMING_MODE
			uint32_t startMicros = micros();
		#endif

		auto systemState = Shared::systemState.Get();
		if (systemState != StatusCode::FAULT_INIT) {
			// Safe to set servo goals even during runtime faults
			Shared::goalVelocities.Get(tempGoalVelocities);
			Shared::goalPositions.Get(tempGoalPositions);
		
			manager.setGoalVelocities(tempGoalVelocities);
			manager.setGoalPositions(tempGoalPositions);  
		}

		#ifdef TIMING_MODE
            uint32_t duration = micros() - startMicros;
            commTiming.update(duration);
        
            if (commTiming.runCount >= TIMING_SAMPLE_COUNT) {
                commTiming.printTimingStats("Setter");
                commTiming.reset();

                // Long delay to simulate less frequent task execution in TIMING_MODE
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TIMING_DELAY_TASKS));
            }
        #else
            // Wait until the next period
			vTaskDelayUntil(&lastWakeTime, SET_TASK.period);
        #endif
	}
}

void createTaskServoSetter() {
	xTaskCreate(
			TaskServoSetter,
			"Setter",
			SET_TASK.stackSize,
			NULL,
			SET_TASK.priority,   
			NULL
	);
}


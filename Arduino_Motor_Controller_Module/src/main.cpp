#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#include "control/system_init.h"

#include "rtos_tasks/task_communication.h"
#include "rtos_tasks/task_servo_reader.h"
#include "rtos_tasks/task_servo_setter.h"
#include "rtos_tasks/task_think.h"

void setup() {
	InitSystem();

	// Start all tasks
	createTaskCommunication();
	createTaskThink();
	createTaskServoSetter();
	createTaskServoReader();

	vTaskStartScheduler();
}

void loop() {
	// Not used with FreeRTOS
}

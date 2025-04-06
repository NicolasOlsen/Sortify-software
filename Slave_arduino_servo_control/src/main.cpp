#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

#include "shared/SharedServoData.h"
#include "control/SystemInit.h"

#include "tasks/TaskCommunication.h"
#include "tasks/TaskServoReader.h"
#include "tasks/TaskServoSetter.h"
#include "tasks/TaskThink.h"

void setup() {
    InitSystem();

    // Start all tasks
    createTaskCommunication();
    createTaskServoReader();
    createTaskServoSetter();
    createTaskThink();

    vTaskStartScheduler();
}

void loop() {
  // Not used with FreeRTOS
}

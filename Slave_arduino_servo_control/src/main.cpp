#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

#include "shared/SharedServoData.h"
#include "control/SystemInit.h"

#include "Tasks/TaskCommunication.h"
#include "Tasks/TaskServoReader.h"
#include "Tasks/TaskServoSetter.h"
#include "Tasks/TaskThink.h"

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

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <DynamixelShield.h>

#include "shared/SharedServoData.h"
#include "control/ServoInit.h"

#include "tasks/taskCommunication.h"
#include "tasks/taskServoReader.h"
#include "tasks/taskServoSetter.h"
#include "tasks/taskErrorHandler.h"
#include "tasks/taskServoTuner.h"

#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr uint32_t baudrate = 1000000;

void setup() {
    Debug::init(baudrate);
    while (!Serial1);
    Debug::infoln("Initializing", DEBUG_MODE);

    InitServoDataMutexes();
    InitServoSystem();

    // Start all tasks
    createTaskCommunication();
    createTaskServoReader();
    // createTaskServoSetter();
    createTaskErrorHandler();
    createTaskServoTuner();

    vTaskStartScheduler();
}

void loop() {
  // Not used with FreeRTOS
}

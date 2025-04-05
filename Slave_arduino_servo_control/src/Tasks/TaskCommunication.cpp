#include <Arduino_FreeRTOS.h>
#include <queue.h>

#include "shared/SharedServoData.h"
#include "tasks/TaskCommunication.h"
#include "comms/UART_communication.h"
#include "utils/Debug.h"

constexpr bool DEBUG_MODE = true;

constexpr UBaseType_t task_priority = 4;                // High priority
constexpr TickType_t TASK_PERIOD = pdMS_TO_TICKS(200);  // Periodic polling interval

constexpr uint16_t QUEUE_SIZE = 128;                    // Enough for several packets

QueueHandle_t serial1Queue;  // Global queue handle

// Communication Task
static void TaskCommunication(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    uint8_t byte;

    float currentPosition;
    DXLLibErrorCode_t currentError;

    Debug::infoln("[Comm] Task started", DEBUG_MODE);

    for (;;) {
        Debug::infoln("[Comm] Running...");

        for (uint8_t id = 1; id <= 4; id++) {
            Debug::print(String(id), DEBUG_MODE);
            Debug::print(" ", DEBUG_MODE);
      
            currentPosition = GetCurrentPosition(id);

            currentError = GetCurrentErrorCode(id);
            Debug::print(String(currentPosition) + " error " + String(currentError) + "\n");
          }

        vTaskDelayUntil(&lastWakeTime, TASK_PERIOD);  // Keeps execution periodic
    }
}


// Initialization function to create task and queue
void createTaskCommunication() {
    serial1Queue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
    if (serial1Queue == NULL) {
        Debug::errorln("[Comm] Failed to create serial queue");
        while (true);  // Halt system
    }

    xTaskCreate(
        TaskCommunication,
        "Communication",
        256,
        NULL,
        task_priority,    
        NULL
    );
}
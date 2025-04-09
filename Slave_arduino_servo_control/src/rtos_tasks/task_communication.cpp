#include <Arduino_FreeRTOS.h>
#include <queue.h>

#include "rtos_tasks/task_communication.h"
#include "config/task_config.h"
#include "shared/shared_servo_state.h"
#include "comms/UART_communication.h"

#include "utils/Debug.h"

constexpr bool LOCAL_DEBUG = true;

constexpr uint16_t QUEUE_SIZE = 128;                    // Enough for several packets

QueueHandle_t serial1Queue;  // Global queue handle

// Communication Task
static void TaskCommunication(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    Debug::infoln("[T_Comm] started", LOCAL_DEBUG);

    for (;;) {
        Debug::infoln("[T_Comm]");

        for (uint8_t id = 1; id <= 5; id++) {      
            goalPositions.Set(id, 180.0f);
        }
        for (uint8_t id = 1; id <= 4; id++) {      
            currentPositions.Get(id);
        }

        // receiveUARTData();

        vTaskDelayUntil(&lastWakeTime, COMM_TASK.period);  // Keeps execution periodic
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
        COMM_TASK.stackSize,
        NULL,
        COMM_TASK.priority,    
        NULL
    );
}
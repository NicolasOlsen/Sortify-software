#include <Arduino_FreeRTOS.h>
#include <queue.h>

#include "rtos_tasks/task_communication.h"
#include "config/task_config.h"
#include "shared/shared_servo_state.h"
#include "comms/UART_communication.h"

#include "utils/Debug.h"

constexpr bool LOCAL_DEBUG = true;

QueueHandle_t uartPacketQueue;  // Global queue handle

// Communication Task
static void TaskCommunication(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    UARTPacket packet;

    Debug::infoln("[T_Comm] started", LOCAL_DEBUG);

    for (;;) {
        Debug::infoln("[T_Comm]");

        receiveUARTData();

        // Check if any complete packets have been received
        while (xQueueReceive(uartPacketQueue, &packet, 0) == pdTRUE) {
            processReceivedPacket(packet.data, packet.length);
        }

        vTaskDelayUntil(&lastWakeTime, COMM_TASK.period);  // Keeps execution periodic
    }
}


// Initialization function to create task and queue
void createTaskCommunication() {
    uartPacketQueue = xQueueCreate(QUEUE_SIZE, sizeof(UARTPacket));
    if (uartPacketQueue == NULL) {
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
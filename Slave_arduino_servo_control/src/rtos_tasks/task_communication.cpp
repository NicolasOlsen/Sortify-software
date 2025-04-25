#include <Arduino_FreeRTOS.h>
#include <queue.h>

#include "rtos_tasks/task_communication.h"
#include "config/task_config.h"
#include "shared/shared_objects.h"
#include "comms/uart_receive.h"
#include "comms/packet_processing.h"

#include "utils/task_timer.h"
#include "utils/debug_utils.h"

#ifdef TIMING_MODE
    static TaskTimingStats commTiming;
#endif


namespace UART_COMM {
    QueueHandle_t uartPacketQueue;
}

// Communication Task
static void TaskCommunication(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    UART_COMM::UARTPacket packet;

    Debug::infoln("[T_Comm] started");

    for (;;) {
        Debug::infoln("[T_Comm]");

        #ifdef TIMING_MODE
            uint32_t startMicros = micros();
        #endif
        
        UART_COMM::receiveUARTData();
    
        while (xQueueReceive(UART_COMM::uartPacketQueue, &packet, 0) == pdTRUE) {
            UART_COMM::processReceivedPacket(packet.data, packet.length);
        }
        
        #ifdef TIMING_MODE
            uint32_t duration = micros() - startMicros;
            commTiming.update(duration);
        
            if (commTiming.runCount >= TIMING_SAMPLE_COUNT) {
                commTiming.printTimingStats("Comm");
                commTiming.reset();

                // Long delay to simulate less frequent task execution in TIMING_MODE
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TIMING_DELAY_TASKS));
            }
        #else
            vTaskDelayUntil(&lastWakeTime, COMM_TASK.period);  // Keeps execution periodic
        #endif
    }
}


// Initialization function to create task and queue
void createTaskCommunication() {
    UART_COMM::uartPacketQueue = xQueueCreate(QUEUE_SIZE, sizeof(UART_COMM::UARTPacket));
    if (UART_COMM::uartPacketQueue == NULL) {
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
#ifndef UART_RECEIVE_H
#define UART_RECEIVE_H

#include <stdint.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

#include "config/communication_config.h"

namespace UART_COMM {

struct UARTPacket {
    uint8_t data[UART_BUFFER_SIZE];
    uint8_t length;
    };

extern QueueHandle_t uartPacketQueue;

/**
 * @brief Initializes the UART peripheral for communication.
 * @param baudRate The baud rate for UART communication.
 */
void UART_init(uint32_t baudRate);

/**
 * @brief Reads a full UART packet from the serial buffer using a state machine.
 */
void receiveUARTData();

} // namespace UART_COMM

#endif // UART_RECEIVE_H

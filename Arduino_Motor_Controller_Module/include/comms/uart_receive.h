#ifndef UART_RECEIVE_H
#define UART_RECEIVE_H

#include <stdint.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

#include "config/communication_config.h"

namespace UART_COMM {

// Packet to be saved inside of a FreeRTOS queue
struct UARTPacket {
    uint8_t data[UART_BUFFER_SIZE];
    uint8_t length;
    };


// FreeRTOS queue for UARTpackets, usefull for integration with FreeRTOS tasks.
extern QueueHandle_t uartPacketQueue;

/**
 * @brief Initializes the UART peripheral for communication.
 * @param baudRate The baud rate for UART communication.
 */
void UART_init(uint32_t baudRate);

/**
 * @brief Reads from the serial buffer and adds it to a packet.
            Handles timeouts, CRC-16 validation, and adds it to a FreeRTOS queue when the packet is complete.
 */
void receiveUARTData();
/*
Potential upgrade that was considered, make a custom handler for UART buffer
that gets packet state and pushes to FreeRTOS queue when the packet is complete.
Was not experimented, because of lack of time and risk (interrupts is heavily timing constrained).
*/ 

} // namespace UART_COMM

#endif // UART_RECEIVE_H

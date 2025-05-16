#ifndef UART_RECEIVE_H
#define UART_RECEIVE_H

#include <stdint.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

#include "config/communication_config.h"

/*
 * ========================================================
 * UART Communication Packet Structure: Master → Arduino
 * ========================================================
 * This structure defines how the SBC (e.g., Raspberry Pi) sends commands
 * to the Arduino controller over UART to manipulate and monitor the robotic arm.
 * 
 * ========================================================
 * Packet Format (Byte Order)
 * ========================================================
 * | Byte #  | Field Name     | Size (bytes) | Description |
 * |---------|----------------|--------------|-------------|
 * | 0       | Start Byte 1    | 1            | Always `0xAA` (indicates packet start) |
 * | 1       | Start Byte 2    | 1            | Always `0x55` (ensures message framing) |
 * | 2       | Packet Length   | 1            | Total length of the packet, including this byte but excluding the start bytes |
 * | 3       | Command ID      | 1            | Identifies the type of command (see `MainCommand` enum) |
 * | 4 - N   | Payload         | Variable     | Command-specific data (optional) |
 * | Last-2  | CRC-16 (Low)    | 1            | Lower byte of CRC-16 checksum |
 * | Last-1  | CRC-16 (High)   | 1            | Upper byte of CRC-16 checksum |
 * 
 * ========================================================
 * Notes
 * ========================================================
 * - No error byte is included in packets sent **to** the Arduino.
 * - The CRC-16 checksum is calculated over **all bytes starting from the length byte up to the last payload byte**.
 * - Packet length **includes** itself, command ID, payload, and CRC bytes — but **excludes** start bytes.
 */


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

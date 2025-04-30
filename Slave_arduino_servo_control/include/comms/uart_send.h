#ifndef UART_SEND_H
#define UART_SEND_H

#include <stdint.h>
#include "Communication_code.h"

/*
 * ========================================================
 * UART Communication Packet Structure: Arduino → Master
 * ========================================================
 * This structure defines how the Arduino sends responses or status updates
 * back to the SBC (e.g., Raspberry Pi) over UART, following command execution.
 * 
 * ========================================================
 * Packet Format (Byte Order)
 * ========================================================
 * | Byte #  | Field Name     | Size (bytes) | Description |
 * |---------|----------------|--------------|-------------|
 * | 0       | Start Byte 1    | 1            | Always `0xAA` (indicates packet start) |
 * | 1       | Start Byte 2    | 1            | Always `0x55` (ensures message framing) |
 * | 2       | Packet Length   | 1            | Total length of the packet, including this byte but excluding the start bytes |
 * | 3       | Command ID      | 1            | Echo of command or response type |
 * | 4 - N   | Payload         | Variable     | Response-specific data (if applicable) |
 * | Last-3  | System State    | 1            | Current Arduino system status code |
 * | Last-2  | CRC-16 (Low)    | 1            | Lower byte of CRC-16 checksum |
 * | Last-1  | CRC-16 (High)   | 1            | Upper byte of CRC-16 checksum |
 * 
 * ========================================================
 * Notes
 * ========================================================
 * - Every response includes a **system state** byte before the CRC to report Arduino's current status (e.g., OK, error).
 * - The CRC-16 checksum is calculated over **all bytes starting from the length byte up to and including the system state byte**.
 * - Packet length **includes** itself, command ID, payload, system state, and CRC — but **excludes** start bytes.
 */


namespace UART_COMM {

/**
 * @brief Sends a structured UART packet.
 * 
 * @param command The command ID from MainCommand enum.
 * @param payload Pointer to the payload data.
 * @param payloadLength Number of bytes in the payload.
 */
void sendPacket(COMM_CODE::MainCommand command, const uint8_t* payload, uint8_t payloadLength);

/**
 * @brief Stores the packet in case of communication error
 * 
 * @param packet The packet to be copied
 * @param size The size of the packet
 */
void storePreviousPacket(const uint8_t* packet, uint8_t size);

/**
 * @brief Resends the previously transmitted packet (used after receiving NACK).
 */
void sendPrevPacket();

/**
 * @brief Sends a NACK packet with the given communication error code.
 * 
 * @param error The error code to send.
 */
void sendNACK(COMM_CODE::ComErrorCode error);

/**
 * @brief Sends a simple ACK packet indicating successful processing.
 * 
 * @param command The command to respond with as acknowledgement
 */
void sendACK(COMM_CODE::MainCommand command);

} // namespace UART_COMM

#endif // UART_SEND_H

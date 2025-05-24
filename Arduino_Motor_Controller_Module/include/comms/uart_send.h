#ifndef UART_SEND_H
#define UART_SEND_H

#include <stdint.h>
#include "Communication_code.h"


namespace UART_COMM {

/**
 * @brief Makes and sends a packet.
 * 
 * @param command The MainCommand of the packet
 * @param payload Pointer to the payload data
 * @param payloadLength Number of bytes in the payload
 */
void sendPacket(COMM_CODE::MainCommand command, const uint8_t* payload, uint8_t payloadLength);

/**
 * @brief Stores the packet in case of communication error.
 * 
 * @param packet The packet to be copied
 * @param size The size of the packet
 */
void storePreviousPacket(const uint8_t* packet, uint8_t size);

/**
 * @brief Resends the previously transmitted packet.
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

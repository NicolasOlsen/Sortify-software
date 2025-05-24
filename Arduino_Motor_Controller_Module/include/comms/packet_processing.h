#ifndef PACKET_PROCESSING_H
#define PACKET_PROCESSING_H

#include <stdint.h>

namespace UART_COMM
{

/**
 * @brief Processes a complete packet received via Arduino Serial.
 * @param packet The packet to process
 * @param length The length of the packet
 */
void processReceivedPacket(const uint8_t* packet, uint8_t length);
    
} // namespace UART_COMM

#endif
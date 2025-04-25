#ifndef PACKET_UTILS_H
#define PACKET_UTILS_H

#include <stdint.h>
#include <string.h>

#include "comms/Communication_code.h"
#include "config/communication_config.h"

namespace PACKET_UTILS {

/**
 * @brief This function converts the bytes stored in a communication packet to the proper type and adds it in a array
 * @tparam T The type to convert the bytes to
 * 
 * @param byteData The array to copy from
 * @param count The size of the output array
 * @param outputArray The array to save the byte data
 */
template<typename T>
void convertBytesToTypedArray(const uint8_t* byteData, uint8_t count, T* outputArray) {
    for (uint8_t i = 0; i < count; ++i) {    // Count is the amount of the data type
        memcpy(&outputArray[i], &byteData[i * sizeof(T)], sizeof(T));   // Copies from the bytedata, with the size of the data type at a time
    }
}

/**
 * @brief Makes the CRC and adds it to the packet to be sent
 * 
 * @param packet The packet to be send
 * @param size The size of the packet
 */
void makePacketCRC(uint8_t* packet, uint8_t packetSize);

/**
 * @brief Validates the CRC in the received packet
 * 
 * @param packet The packet to be validated
 * @param size The size of the packet
 */
bool validatePacketCRC(const uint8_t* packet, uint8_t packetSize);

/**
 * @brief Calculates the CRC16 used for both generating and validating packets.
 * 
 * @param packet The packet to calculate CRC16
 * @param size The size of the packet
 */
uint16_t calculateCRC16(const uint8_t* packet, uint8_t packetSize);

/**
 * @brief Checks if the packet size is within the bound, if not sends a communication error
 * 
 * @param packetSize The size of the recieving packet
 * @param expectedSize The size of the expected packet size
 * 
 * @return If the packet is the expected size
 */
bool packetExpectedSize(uint8_t packetSize, uint8_t expectedSize);

}

#endif
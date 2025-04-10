#include <stdint.h>
#include <CRC16.h>

#include "comms/communication_helper.h"

#include "config/communication_config.h"
#include "shared/System_status.h"

#include "utils/Debug.h"

namespace Com_helper {

constexpr bool LOCAL_DEBUG = true;

CRC16 crc16;


bool CheckFault() { return System_status::systemState.Get() == StatusCode::FAULT; }


// Appends a CRC-16 checksum to the last two bytes of the packet
void makePacketCRC(uint8_t* packet, uint8_t packetSize) {
    // Basic null and size validation
    if (packet == nullptr || packetSize < MIN_PACKET_SIZE) {
        Debug::errorln("Invalid packet or length in makePacketCRC!", LOCAL_DEBUG);
        return;
    }

    uint16_t crc = calculateCRC16(packet, packetSize);

    // Append CRC to the end of the packet (Little Endian: LSB first)
    packet[packetSize - 2] = crc & 0xFF;
    packet[packetSize - 1] = (crc >> 8) & 0xFF;

    // Debug output of generated CRC
    Debug::infoln("Generated CRC Bytes: " + 
        String(packet[packetSize - 2], HEX) + " " + 
        String(packet[packetSize - 1], HEX), LOCAL_DEBUG);
}


// Validates the CRC-16 checksum of a received packet
bool validatePacketCRC(const uint8_t* packet, uint8_t packetSize) {
    if (packet == nullptr || packetSize < MIN_PACKET_SIZE) {
        Debug::errorln("Invalid packet or length in validatePacketCRC!", LOCAL_DEBUG);
        return false;
    }

    // Extract CRC from the last two bytes of the packet (Little Endian)
    uint16_t receivedCRC = (packet[packetSize - 2]) |
                           (packet[packetSize - 1] << 8);

    // Recompute CRC over the same region used in makePacketCRC()
    uint16_t computedCRC = calculateCRC16(packet, packetSize);

    Debug::infoln("Computed CRC Bytes: " + 
        String(computedCRC & 0xFF, HEX) + " " + 
        String((computedCRC >> 8) & 0xFF, HEX) + 
        " | Received CRC Bytes: " + 
        String(receivedCRC & 0xFF, HEX) + " " + 
        String((receivedCRC >> 8) & 0xFF, HEX), LOCAL_DEBUG);

    return computedCRC == receivedCRC;
}


// Calculates CRC-16 over the packet excluding START_BYTES and CRC field itself
uint16_t calculateCRC16(const uint8_t* packet, uint8_t packetSize) {
    // Compute CRC starting from LENGTH field up to the byte before the CRC
    // i.e., skip START_BYTES and exclude the last 2 CRC bytes
    crc16.reset();
    crc16.add(&packet[START_BYTES_SIZE], packetSize - (START_BYTES_SIZE + 2));
    return crc16.calc();
}

}
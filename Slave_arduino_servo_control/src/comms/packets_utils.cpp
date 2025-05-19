#include <stdint.h>
#include <CRC16.h>

#include "comms/packet_utils.h"
#include "comms/uart_send.h"
#include "config/communication_config.h"
#include "shared/shared_objects.h"

#include "utils/debug_utils.h"

using namespace COMM_CODE;

CRC16 crc16;

namespace PACKET_UTILS {

// Appends a CRC-16 checksum to the last two bytes of the packet
void makePacketCRC(uint8_t* packet, uint8_t packetSize) {
    // Basic null and size validation
    if (packet == nullptr || packetSize < MIN_PACKET_SIZE) {
        Debug::errorln("Invalid packet or length in makePacketCRC!");
        return;
    }

    uint16_t crc = calculateCRC16(packet, packetSize);

    // Append CRC to the end of the packet (Little Endian: LSB first)
    packet[packetSize - 2] = crc & 0xFF;
    packet[packetSize - 1] = (crc >> 8) & 0xFF;

    #ifdef DEBUG
        // Debug output of generated CRC
        Debug::infoln("Generated CRC Bytes: " + 
            String(packet[packetSize - 2], HEX) + " " + 
            String(packet[packetSize - 1], HEX));
    #endif
}


// Validates the CRC-16 checksum of a received packet
bool validatePacketCRC(const uint8_t* packet, uint8_t packetSize) {
    if (packet == nullptr || packetSize < MIN_PACKET_SIZE) {
        Debug::errorln("Invalid packet or length in validatePacketCRC!");
        return false;
    }

    // Extract CRC from the last two bytes of the packet (Little Endian)
    uint16_t receivedCRC = (packet[packetSize - 2]) |
                           (packet[packetSize - 1] << 8);

    // Recompute CRC
    uint16_t computedCRC = calculateCRC16(packet, packetSize);

    #ifdef DEBUG
        Debug::infoln("Computed CRC Bytes: " + 
            String(computedCRC & 0xFF, HEX) + " " + 
            String((computedCRC >> 8) & 0xFF, HEX) + 
            " | Received CRC Bytes: " + 
            String(receivedCRC & 0xFF, HEX) + " " + 
            String((receivedCRC >> 8) & 0xFF, HEX));
    #endif

    return computedCRC == receivedCRC;
}


// Calculates CRC-16 over the packet excluding START_BYTES and CRC field itself
uint16_t calculateCRC16(const uint8_t* packet, uint8_t packetSize) {
    // Compute CRC starting from LENGTH field up to the byte before the CRC
    crc16.reset();
    crc16.add(&packet[START_BYTES_SIZE], packetSize - (START_BYTES_SIZE + 2));  // skip START_BYTES and exclude the last 2 CRC bytes
    return crc16.calc();
}

// Checks if the packetsize is smaller than the expceted packetsize
bool packetExpectedSize(uint8_t packetSize, uint8_t expectedSize) {
    if (packetSize < expectedSize) {
        #ifdef DEBUG
            Debug::errorln("Packet is less than expected size, expected " + 
                String(expectedSize) + " got " + String(packetSize));
        #endif
        UART_COMM::sendNACK(ComErrorCode::INVALID_PAYLOAD_SIZE);
        return false;
    }
    return true;
}

bool isSystemStateFault() {
    auto systemState = Shared::systemState.Get();
    if (systemState == StatusCode::FAULT_INIT ||
        systemState == StatusCode::FAULT_RUNTIME) {
        Debug::warnln("System is in fault mode");
        UART_COMM::sendNACK(ComErrorCode::SYSTEM_FAULT);
        return true;
    }

    return false;
}

}
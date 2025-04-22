#include "comms/uart_send.h"
#include "config/communication_config.h"
#include "comms/packet_utils.h"

#include "shared/shared_objects.h"

#include "utils/debug_utils.h"

using namespace COMM_CODE;

// Buffer for storing the previously sent packet
static uint8_t prevPacket[UART_BUFFER_SIZE] = { 0 };
static uint8_t prevPacketSize = 0;

namespace UART_COMM {

void sendPacket(MainCommand command, const uint8_t* payload, uint8_t payloadLength) {
    // +1 byte for system state
    uint8_t adjustedPayloadLength = payloadLength + 1;

    // Total packet size = MIN_PACKET_SIZE + additional payload bytes (if any)
    uint8_t packetSize = MIN_PACKET_SIZE + adjustedPayloadLength;

    if (packetSize > UART_BUFFER_SIZE) {
        sendNACK(ComErrorCode::BUFFER_OVERFLOW);
        return;
    }

    if (payload == nullptr && payloadLength > 0) {
        Debug::errorln("Attempted to send non-zero payload with nullptr");
        return;
    }

    uint8_t packet[UART_BUFFER_SIZE];

    // Add start bytes
    for (uint8_t i = 0; i < START_BYTES_SIZE; i++) {
        packet[i] = START_BYTES[i];
    }

    packet[LENGTH_INDEX]  = packetSize - START_BYTES_SIZE;
    packet[COMMAND_INDEX] = static_cast<uint8_t>(command);

    // Copy payload
    if (payloadLength > 0) {
        memcpy(&packet[PAYLOAD_INDEX], payload, payloadLength);
    }

    // Append system state just before CRC
    uint8_t systemState = static_cast<uint8_t>(Shared::systemState.Get());
    packet[PAYLOAD_INDEX + payloadLength] = systemState;

    // Append CRC
    PACKET_UTILS::makePacketCRC(packet, packetSize);

    #ifdef DEBUG
        Debug::info("Sent: ");
        Debug::printHex(packet, packetSize);
        Debug::print("\n");
    #endif

    // Store for potential retransmission
    if (command != MainCommand::NACK) {
        storePreviousPacket(packet, packetSize);
    }

    COMM_SERIAL.write(packet, packetSize);
}

void storePreviousPacket(const uint8_t packet[UART_BUFFER_SIZE], uint8_t size) {
    if (size > UART_BUFFER_SIZE) {
        Debug::errorln("Packet size exceeds buffer in storePreviousPacket");
        return;
    }

    prevPacketSize = size;
    memcpy(prevPacket, packet, size);
}

void sendPrevPacket() {
    if (prevPacketSize > 0 && prevPacketSize <= UART_BUFFER_SIZE) {
        COMM_SERIAL.write(prevPacket, prevPacketSize);
    } else {
        Debug::warnln("No valid previous packet to send");
    }
}

void sendACK() {
    sendPacket(MainCommand::ACK, nullptr, 0);
}

void sendNACK(ComErrorCode error) {
    uint8_t ComErrorCode = static_cast<uint8_t>(error);
    sendPacket(MainCommand::NACK, &ComErrorCode, 1);
}

} // namespace UART_COMM
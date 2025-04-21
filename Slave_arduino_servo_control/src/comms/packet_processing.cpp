#include "comms/packet_processing.h"
#include "config/communication_config.h"
#include "comms/Communication_code.h"
#include "comms/uart_send.h"
#include "comms/packet_utils.h"

#include "shared/shared_objects.h"

#include "utils/debug_utils.h"

using namespace COMM_CODE;

namespace UART_COMM {

namespace {
    void handleReadPositionRange(const uint8_t* packet, uint8_t packetSize);
    void handleReadErrorRange(const uint8_t* packet, uint8_t packetSize);
    void handleWritePositionRange(const uint8_t* packet, uint8_t packetSize);
    void handleWriteVelocityRange(const uint8_t* packet, uint8_t packetSize);
}    

void processReceivedPacket(const uint8_t* packet, uint8_t packetSize) {
    if(!PACKET_UTILS::packetExpectedSize(packetSize, MIN_PACKET_SIZE)) return;    // If packet isnt the minimum size

    Debug::info("Processing packet: ");
    Debug::printHex(packet, packetSize);
    Debug::print("\n");

    MainCommand command = static_cast<MainCommand>(packet[COMMAND_INDEX]);

    switch (command) {

        case MainCommand::HEARTBEAT: {
            Debug::infoln("HB received");
            sendACK();
            break;
        }

        case MainCommand::ACK: {
            Debug::infoln("ACK received");
            break;
        }

        case MainCommand::NACK: {
            Debug::infoln("NACK received");
            sendPrevPacket();
            break;
        }
        
        // === Position Commands ===
        case MainCommand::READ_POSITION_RANGE: {
            Debug::infoln("RPR received");
            handleReadPositionRange(packet, packetSize);
            break;
        }
            
        case MainCommand::WRITE_POSITION_RANGE: {
            Debug::infoln("WPR received");
            handleWritePositionRange(packet, packetSize);
            break;
        }
        
        // === Velocity Commands ===
        case MainCommand::WRITE_VELOCITY_RANGE: {
            Debug::infoln("WVR received");
            handleWriteVelocityRange(packet, packetSize);
            break;
        }
        
        // === Error Commands ===
        case MainCommand::READ_ERROR_RANGE: {
            Debug::infoln("RER received");
            handleReadErrorRange(packet, packetSize);
            break;
        }

        default: {
            Debug::infoln("Unknown command received");
            sendNACK(ComErrorCode::UNKNOWN_COMMAND);
            break;
        }
    }
}

namespace {

constexpr uint8_t rangeMetaSize = 2; // [start_id][count] in payload

// Generic read handler for any SharedServoData type
template<typename T, uint8_t SIZE>
void handleReadRangeTemplate(SharedServoData<T, SIZE>& source, MainCommand responseCommand, const uint8_t* packet, uint8_t packetSize) {
    if (!PACKET_UTILS::packetExpectedSize(packetSize, MIN_PACKET_SIZE + rangeMetaSize)) return;

    uint8_t startId = packet[PAYLOAD_INDEX];
    uint8_t count   = packet[PAYLOAD_INDEX + 1];

    if (startId + count > SIZE) {
        sendNACK(ComErrorCode::ID_OUT_OF_RANGE);
        return;
    }

    T values[SIZE];
    source.Get(values, count, startId);

    sendPacket(responseCommand, reinterpret_cast<uint8_t*>(values), count * sizeof(T));
}

// Generic write handler for any SharedServoData type
template<typename T, uint8_t SIZE>
void handleWriteRangeTemplate(SharedServoData<T, SIZE>& target, const uint8_t* packet, uint8_t packetSize) {
    if (!PACKET_UTILS::packetExpectedSize(packetSize, MIN_PACKET_SIZE + rangeMetaSize)) return;

    uint8_t startId = packet[PAYLOAD_INDEX];
    uint8_t count   = packet[PAYLOAD_INDEX + 1];

    uint8_t expectedPayloadSize = rangeMetaSize + count * sizeof(T);
    if (!PACKET_UTILS::packetExpectedSize(packetSize, MIN_PACKET_SIZE + expectedPayloadSize)) return;
    if (startId + count > SIZE) {
        sendNACK(ComErrorCode::ID_OUT_OF_RANGE);
        return;
    }

    T values[SIZE];

    if (Shared::systemState.Get() == StatusCode::FAULT) {
        Debug::warnln("System is in fault mode");
        UART_COMM::sendACK();
        return;
    }

    PACKET_UTILS::convertBytesToTypedArray<T>(&packet[PAYLOAD_INDEX + rangeMetaSize], count, values);

    target.Set(values, count, startId);

    sendACK();
}

void handleReadPositionRange(const uint8_t* packet, uint8_t packetSize) {
    handleReadRangeTemplate(Shared::currentPositions, MainCommand::READ_POSITION_RANGE, packet, packetSize);
}

void handleReadErrorRange(const uint8_t* packet, uint8_t packetSize) {
    handleReadRangeTemplate(Shared::servoErrors, MainCommand::READ_ERROR_RANGE, packet, packetSize);
}

void handleWritePositionRange(const uint8_t* packet, uint8_t packetSize) {
    handleWriteRangeTemplate(Shared::goalPositions, packet, packetSize);
}

void handleWriteVelocityRange(const uint8_t* packet, uint8_t packetSize) {
    handleWriteRangeTemplate(Shared::goalVelocities, packet, packetSize);
}

} // anonymous namespace

} // namespace UART_COMM
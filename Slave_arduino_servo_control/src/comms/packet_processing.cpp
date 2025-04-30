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

    #ifdef DEBUG
        Debug::info("Processing packet: ");
        Debug::printHex(packet, packetSize);
        Debug::print("\n");
    #endif

    MainCommand command = static_cast<MainCommand>(packet[COMMAND_INDEX]);

    switch (command) {

        case MainCommand::PING_: {
            Debug::infoln("Ping received");
            sendACK(command);
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

        case MainCommand::STOP_MOVEMENT: {
            Debug::infoln("SM received");

            auto systemState = Shared::systemState.Get();
            if (systemState == StatusCode::FAULT_INIT ||
                systemState == StatusCode::FAULT_RUNTIME) {
                Debug::warnln("System is in fault mode");
                UART_COMM::sendNACK(ComErrorCode::SYSTEM_FAULT);
                return;
            }
            else {
                float tempPositions[Shared::servoManager.getDXLAmount()];
                Shared::currentPositions.Get(tempPositions, Shared::servoManager.getDXLAmount());
                Shared::goalPositions.Set(tempPositions, Shared::servoManager.getDXLAmount());
                sendACK(command);
            }
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

// Generic read handler for any SharedArrayWithFlags type
template<typename T, uint8_t SIZE>
void handleReadRangeTemplate(SharedArrayWithFlags<T, SIZE>& source, MainCommand responseCommand, const uint8_t* packet, uint8_t packetSize) {
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

// Generic read handler for any SharedArray type
template<typename T, uint8_t SIZE>
void handleReadRangeTemplate(SharedArray<T, SIZE>& source, MainCommand responseCommand, const uint8_t* packet, uint8_t packetSize) {
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

// Generic write handler for any SharedArray type
template<typename T, uint8_t SIZE>
void handleWriteRangeTemplate(SharedArray<T, SIZE>& target, const uint8_t* packet, uint8_t packetSize) {
    if (!PACKET_UTILS::packetExpectedSize(packetSize, MIN_PACKET_SIZE + rangeMetaSize)) return;

    MainCommand command = static_cast<MainCommand>(packet[COMMAND_INDEX]);
    uint8_t startId = packet[PAYLOAD_INDEX];
    uint8_t count   = packet[PAYLOAD_INDEX + 1];

    uint8_t expectedPayloadSize = rangeMetaSize + count * sizeof(T);
    if (!PACKET_UTILS::packetExpectedSize(packetSize, MIN_PACKET_SIZE + expectedPayloadSize)) return;
    if (startId + count > SIZE) {
        sendNACK(ComErrorCode::ID_OUT_OF_RANGE);
        return;
    }

    T values[SIZE];

    auto systemState = Shared::systemState.Get();
    if (systemState == StatusCode::FAULT_INIT ||
        systemState == StatusCode::FAULT_RUNTIME) {
        Debug::warnln("System is in fault mode");
        UART_COMM::sendNACK(ComErrorCode::SYSTEM_FAULT);
        return;
    }

    PACKET_UTILS::convertBytesToTypedArray<T>(&packet[PAYLOAD_INDEX + rangeMetaSize], count, values);

    if (command == MainCommand::WRITE_POSITION_RANGE) {
        for (uint8_t i = 0; i < count; ++i) {
            uint8_t servoId = startId + i;
            if (!Shared::servoManager.checkPositionInAllowedRange(servoId, values[i])) {
                UART_COMM::sendNACK(ComErrorCode::POSITION_OUT_OF_RANGE);
                return;
            }
        }        
    }

    target.Set(values, count, startId);

    sendACK(command);
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
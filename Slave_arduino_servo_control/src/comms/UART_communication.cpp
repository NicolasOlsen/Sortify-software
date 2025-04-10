#include <Arduino.h>
#include <stdint.h>
#include <CRC16.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

#include "comms/UART_communication.h"
#include "config/communication_config.h"
#include "comms/communication_helper.h"
#include "comms/Communication_code.h"

#include "shared/System_status.h"
#include "shared/shared_servo_state.h"

#include "utils/Debug.h"

using namespace Com_code;

// Buffer for storing the previously sent packet
static uint8_t prevPacket[UART_BUFFER_SIZE] = { 0 };
static uint8_t prevPacketSize = 0;

// Debug flag for local debugging
constexpr bool LOCAL_DEBUG = true;

constexpr uint8_t SYSTEM_STATE_SIZE = sizeof(uint8_t);
constexpr uint8_t ID_SIZE = sizeof(uint8_t);
constexpr uint8_t SERVO_POSITION_SIZE = sizeof(float);
constexpr uint8_t SERVO_VELOCITY_SIZE = sizeof(float);
constexpr uint8_t ALL_POSITION_PAYLOAD_SIZE = SMART_SERVO_COUNT * SERVO_POSITION_SIZE;
constexpr uint8_t ALL_VELOCITY_PAYLOAD_SIZE = SMART_SERVO_COUNT * SERVO_VELOCITY_SIZE;

// Final expected size for SET_SERVO_POSITION
constexpr uint8_t SSP_TOTAL_SIZE = MIN_PACKET_SIZE + ID_SIZE + SERVO_POSITION_SIZE + SYSTEM_STATE_SIZE;

// Final expected size for SET_ALL_POSITIONS
constexpr uint8_t SAP_TOTAL_SIZE = MIN_PACKET_SIZE + ALL_POSITION_PAYLOAD_SIZE + SYSTEM_STATE_SIZE;

// Final expected size for SET_SERVO_GOAL_VELOCITY
constexpr uint8_t SSGS_TOTAL_SIZE = MIN_PACKET_SIZE + ID_SIZE + SERVO_VELOCITY_SIZE + SYSTEM_STATE_SIZE;

// Final expected size for SET_ALL_GOAL_VELOCITY
constexpr uint8_t SAGV_TOTAL_SIZE = MIN_PACKET_SIZE + ALL_VELOCITY_PAYLOAD_SIZE + SYSTEM_STATE_SIZE;

void UART_init(uint32_t baudRate) {
    COMM_SERIAL.begin(baudRate);
}

void receiveUARTData() {
    // State machine to parse UART packets in phases: SYNC -> LENGTH -> DATA
    static enum class ReceiveState { SYNC, LENGTH, DATA } state = ReceiveState::SYNC;

    static uint8_t expectedLength = 0;         // Total expected packet length
    static unsigned long packetStartTime = 0;  // Timestamp when packet reception began

    static uint8_t localBuffer[UART_BUFFER_SIZE]; // Local buffer to hold incoming packet data
    static uint8_t localIndex = 0;                // Current write index in the buffer

    // Check for packet timeout (prevents hanging in mid-packet if something went wrong)
    if (state != ReceiveState::SYNC && (millis() - packetStartTime > PACKET_TIMEOUT)) {
        Debug::printHex(localBuffer, localIndex, LOCAL_DEBUG);
        Debug::print("\n", LOCAL_DEBUG);
        Debug::warnln("Packet Timeout - Resetting State Machine", LOCAL_DEBUG);

        state = ReceiveState::SYNC;
        localIndex = 0;
        expectedLength = 0;
    }

    // Read all available bytes from the UART buffer
    while (COMM_SERIAL.available()) {
        uint8_t incomingByte = COMM_SERIAL.read();

        switch (state) {
            // Wait for sync bytes at the start of every packet
            case ReceiveState::SYNC:
                if (localIndex < START_BYTES_SIZE && incomingByte == START_BYTES[localIndex]) {
                    localBuffer[localIndex++] = incomingByte;

                    // Sync complete: move to length phase
                    if (localIndex == START_BYTES_SIZE) {
                        state = ReceiveState::LENGTH;
                        packetStartTime = millis();
                        Debug::info("Receiving: ", LOCAL_DEBUG);
                    }
                } else {
                    // Sync error: reset sync state
                    if constexpr (LOCAL_DEBUG) {
                        if (localIndex > 0) {
                            Debug::printHex(localBuffer, localIndex);
                            Debug::print("\n");
                        }
                        Debug::warnln("Out of sync");
                    }
                    localIndex = 0;
                }
                break;

            // Read length byte and validate it
            case ReceiveState::LENGTH:
                expectedLength = incomingByte + START_BYTES_SIZE;  // Full packet length including headers

                // Validate packet length
                if (expectedLength > sizeof(localBuffer) ||
                    expectedLength < MIN_PACKET_SIZE - START_BYTES_SIZE) {
                    
                    Debug::printHex(localBuffer, localIndex, LOCAL_DEBUG);
                    Debug::print("\n", LOCAL_DEBUG);
                    Debug::warnln("Length is either too long or too short", LOCAL_DEBUG);

                    sendCommunicationError(ComErrorCode::BUFFER_OVERFLOW);

                    state = ReceiveState::SYNC;
                    localIndex = 0;
                    return;
                }

                localBuffer[localIndex++] = incomingByte;  // Store the length byte
                state = ReceiveState::DATA;                // Move to data reading
                break;

            // Read remaining data bytes of the packet
            case ReceiveState::DATA:
                localBuffer[localIndex++] = incomingByte;

                // Full packet received
                if (localIndex == expectedLength) {
                    Debug::printHex(localBuffer, localIndex, LOCAL_DEBUG);
                    Debug::print("\n", LOCAL_DEBUG);

                    if (Com_helper::validatePacketCRC(localBuffer, expectedLength)) {
                        // Valid packet: send to queue
                        UARTPacket packet;
                        memcpy(packet.data, localBuffer, expectedLength);
                        packet.length = expectedLength;

                        if (xQueueSend(uartPacketQueue, &packet, 0) != pdPASS) {
                            Debug::warnln("UART Queue Full", LOCAL_DEBUG);
                            sendCommunicationError(ComErrorCode::QUEUE_FULL);
                        }
                    } else {
                        // CRC check failed
                        Debug::warnln("Checksum error", LOCAL_DEBUG);
                        sendCommunicationError(ComErrorCode::CHECKSUM_ERROR);
                    }

                    // Reset state machine after complete packet
                    state = ReceiveState::SYNC;
                    localIndex = 0;
                    expectedLength = 0;
                    packetStartTime = 0;
                    return;
                }

                // Defensive: if somehow too much data was read
                else if (localIndex > expectedLength) {
                    Debug::printHex(localBuffer, localIndex, LOCAL_DEBUG);
                    Debug::print("\n", LOCAL_DEBUG);
                    Debug::warnln("Buffer overflow (too much data)", LOCAL_DEBUG);

                    sendCommunicationError(ComErrorCode::BUFFER_OVERFLOW);

                    state = ReceiveState::SYNC;
                    localIndex = 0;
                    expectedLength = 0;
                    packetStartTime = 0;
                    return;
                }
                break;
        }
    }
}


void processReceivedPacket(const uint8_t* packet, uint8_t packetSize) {
    if(!packetExpectedSize(packetSize, MIN_PACKET_SIZE)) return;    // If packet isnt the minimum size

    Debug::info("Processing packet: ", LOCAL_DEBUG);
    Debug::printHex(packet, packetSize, LOCAL_DEBUG);
    Debug::print("\n", LOCAL_DEBUG);

    MainCommand command = static_cast<MainCommand>(packet[COMMAND_INDEX]);

    if (command > MainCommand::COMMUNICATION_ERROR) {
        sendCommunicationError(ComErrorCode::UNKNOWN_COMMAND);
        return;
    }

    switch (command) {

        case MainCommand::HEARTBEAT: {
            Debug::infoln("HB received");
            sendAcknowledgement();
            break;
        }
        
        case MainCommand::REQUEST_SERVO_POSITIONS: {
            Debug::infoln("RSP received");
        
            float current[SMART_SERVO_COUNT];
            currentPositions.Get(current);  // Thread-safe copy of floats
        
            constexpr uint8_t payloadBytes = SMART_SERVO_COUNT * sizeof(float);
            uint8_t payload[payloadBytes];
        
            // Convert float array to byte buffer
            memcpy(payload, current, payloadBytes);
        
            sendPacket(MainCommand::RESPOND_SERVO_POSITIONS, payload, sizeof(payload));
            break;
        }            

        case MainCommand::SET_SERVO_POSITION: {
            Debug::infoln("SSP received");
            if (Com_helper::CheckFault()) {
                sendAcknowledgement();
                return;
            }
            if (!packetExpectedSize(packetSize, SSP_TOTAL_SIZE)) return;
            Com_helper::handleSetSingleValue(packet, goalPositions);
            sendAcknowledgement();
            break;
        }            

        case MainCommand::SET_ALL_POSITIONS: {
            Debug::infoln("SAP received");
            if (Com_helper::CheckFault()) {
                sendAcknowledgement();
                return;
            }
            if (!packetExpectedSize(packetSize, SAP_TOTAL_SIZE)) return;
            Com_helper::handleSetAllValues(packet, goalPositions);
            sendAcknowledgement();
            break;
        }

        case MainCommand::SET_SERVO_GOAL_VELOCITY: {
            Debug::infoln("SSGS received");
            if (Com_helper::CheckFault()) {
                sendAcknowledgement();
                return;
            }
            if (!packetExpectedSize(packetSize, SSGS_TOTAL_SIZE)) return;
            Com_helper::handleSetSingleValue(packet, goalVelocities);
            sendAcknowledgement();
            break;
        }

        case MainCommand::SET_ALL_GOAL_VELOCITY: {
            Debug::infoln("SAGS received");
            if (Com_helper::CheckFault()) {
                sendAcknowledgement();
                return;
            }
            if (!packetExpectedSize(packetSize, SAGV_TOTAL_SIZE)) return;
            Com_helper::handleSetAllValues(packet, goalVelocities);
            sendAcknowledgement();
            break;
        }

        case MainCommand::STOP_MOVEMENT:
            Debug::infoln("SM received");
            if (Com_helper::CheckFault()) {
                sendAcknowledgement();
                return;
            }
            float current[SMART_SERVO_COUNT];
            currentPositions.Get(current);

            float goal[TOTAL_SERVO_COUNT];
            goalPositions.Get(goal);
            for (uint8_t i = 0; i < SMART_SERVO_COUNT; ++i) {
                goal[i] = current[i];
            }
            goalPositions.Set(goal);
            System_status::systemState.Set(StatusCode::IDLE);
            sendAcknowledgement();
            break;

        case MainCommand::REQUEST_ERROR_STATUS: {
            Debug::infoln("RES received");
        
            DXLLibErrorCode_t flags[SMART_SERVO_COUNT];
            servoErrors.Get(flags);  // Get actual 32-bit error codes
        
            constexpr size_t payloadSize = SMART_SERVO_COUNT * sizeof(DXLLibErrorCode_t);
            uint8_t payload[payloadSize];
        
            memcpy(payload, flags, payloadSize);
        
            sendPacket(MainCommand::RESPOND_ERROR_STATUS, payload, payloadSize);
            break;
        }
            

        case MainCommand::COMMUNICATION_ERROR:
            Debug::infoln("CE received");
            sendPrevPacket();
            break;

        default:
            sendCommunicationError(ComErrorCode::UNKNOWN_COMMAND);
            break;
    }
}


void sendPacket(MainCommand command, const uint8_t* payload, uint8_t payloadLength) {
    // +1 byte for system state
    uint8_t adjustedPayloadLength = payloadLength + 1;

    // Total packet size = MIN_PACKET_SIZE + additional payload bytes (if any)
    uint8_t packetSize = MIN_PACKET_SIZE + adjustedPayloadLength;

    if (packetSize > UART_BUFFER_SIZE) {
        sendCommunicationError(ComErrorCode::BUFFER_OVERFLOW);
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
    uint8_t systemState = static_cast<uint8_t>(System_status::systemState.Get());
    packet[PAYLOAD_INDEX + payloadLength] = systemState;

    // Append CRC
    Com_helper::makePacketCRC(packet, packetSize);

    // Debug
    Debug::info("Sent: ", LOCAL_DEBUG);
    Debug::printHex(packet, packetSize, LOCAL_DEBUG);
    Debug::print("\n");

    // Store for potential retransmission
    if (command != MainCommand::COMMUNICATION_ERROR) {
        storePreviousPacket(packet, packetSize);
    }

    COMM_SERIAL.write(packet, packetSize);
}

void sendAcknowledgement() {
    uint8_t statusCode = static_cast<uint8_t>(System_status::systemState.Get());
    sendPacket(MainCommand::ACKNOWLEDGE, &statusCode, 1);
}

void sendCommunicationError(ComErrorCode error) {
    uint8_t ComErrorCode = static_cast<uint8_t>(error);
    sendPacket(MainCommand::COMMUNICATION_ERROR, &ComErrorCode, 1);
}

// Resends the previously stored packet (used after a COMMUNICATION_ERROR is received)
void sendPrevPacket() {
    if (prevPacketSize > 0 && prevPacketSize <= UART_BUFFER_SIZE) {
        COMM_SERIAL.write(prevPacket, prevPacketSize);
    } else {
        Debug::warnln("No valid previous packet to send", LOCAL_DEBUG);
    }
}

// Stores the most recently sent packet for potential retransmission
void storePreviousPacket(const uint8_t packet[UART_BUFFER_SIZE], uint8_t size) {
    if (size > UART_BUFFER_SIZE) {
        Debug::errorln("Packet size exceeds buffer in storePreviousPacket", LOCAL_DEBUG);
        return;
    }

    prevPacketSize = size;
    memcpy(prevPacket, packet, size);
}

// Checks if the packetsize is smaller than the expceted packetsize
bool packetExpectedSize(uint8_t packetSize, uint8_t expectedSize) {
    if (packetSize < expectedSize) {
        sendCommunicationError(ComErrorCode::INVALID_PAYLOAD_SIZE);
        Debug::errorln("Packet too small");
        return false;
    }
    return true;
}
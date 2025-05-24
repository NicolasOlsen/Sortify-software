#include "comms/uart_receive.h"
#include "config/communication_config.h"
#include "comms/uart_send.h"
#include "comms/packet_utils.h"
#include "utils/debug_utils.h"

using namespace COMM_CODE;

namespace UART_COMM {

void UART_init(uint32_t baudRate) {
    COMM_SERIAL.begin(baudRate);
}

void receiveUARTData() {
    // State machine to control the recieving flow
    static enum class ReceiveState { SYNC, LENGTH, DATA } state = ReceiveState::SYNC;

    static uint8_t expectedLength = 0;         // Total expected packet length
    static unsigned long packetStartTime = 0;  // Timestamp when packet reception began

    static uint8_t localBuffer[UART_BUFFER_SIZE]; // Local buffer to hold incoming packet data
    static uint8_t localIndex = 0;                // Current write index in the buffer

    // Lambda to reset state machine cleanly
    auto resetState = [&]() {
        state = ReceiveState::SYNC;
        localIndex = 0;
        expectedLength = 0;
        packetStartTime = 0;
    };

    // Check for packet timeout
    if (state != ReceiveState::SYNC && (millis() - packetStartTime > PACKET_TIMEOUT)) {
        Debug::printHex(localBuffer, localIndex);
        Debug::print("\n");
        Debug::warnln("Packet Timeout - Resetting State Machine");

        sendNACK(ComErrorCode::COMM_TIMEOUT);
        resetState();
    }

    // Read all available bytes from the UART buffer
    while (COMM_SERIAL.available()) {
        uint8_t incomingByte = COMM_SERIAL.read();

        switch (state) {
            // Wait for sync bytes at the start of every packet
            case ReceiveState::SYNC:
                if (localIndex < START_BYTES_SIZE && incomingByte == START_BYTES[localIndex]) {
                    localBuffer[localIndex++] = incomingByte;

                    // Sync complete, move to length phase
                    if (localIndex == START_BYTES_SIZE) {
                        state = ReceiveState::LENGTH;
                        packetStartTime = millis();
                        Debug::info("Receiving: ");
                    }
                } else {
                    // Sync error, reseting sync state
                    resetState();
                    #ifdef DEBUG
                        if (localIndex > 0) {
                            Debug::printHex(localBuffer, localIndex);
                            Debug::println("\n");
                        }
                        Debug::warn("Out of sync. Unexpected byte: 0x");
                        Debug::printHex(incomingByte);
                        Debug::print("\n");
                    #endif
                }
                break;

            // Read length byte and validate it
            case ReceiveState::LENGTH:
                expectedLength = incomingByte + START_BYTES_SIZE;  // Full packet length including headers

                // Validate packet length
                if (expectedLength > sizeof(localBuffer) ||
                    expectedLength < MIN_PACKET_SIZE - START_BYTES_SIZE) {
                    
                    #ifdef DEBUG
                        Debug::printHex(localBuffer, localIndex);
                        Debug::print("\n");
                        Debug::warnln("Length is either too long or too short");
                    #endif

                    sendNACK(ComErrorCode::BUFFER_OVERFLOW);
                    resetState();
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
                    Debug::printHex(localBuffer, localIndex);
                    Debug::print("\n");

                    if (PACKET_UTILS::validatePacketCRC(localBuffer, expectedLength)) {
                        // Valid packet, send to queue
                        UARTPacket packet;
                        memcpy(packet.data, localBuffer, expectedLength);
                        packet.length = expectedLength;

                        if (xQueueSend(uartPacketQueue, &packet, 0) != pdPASS) {
                            Debug::warnln("UART Queue Full");
                            sendNACK(ComErrorCode::QUEUE_FULL);
                        }
                    } else {
                        // CRC check failed
                        Debug::warnln("Checksum error");
                        sendNACK(ComErrorCode::CHECKSUM_ERROR);
                    }

                    // Reset state machine after complete packet
                    resetState();
                    return;
                }

                // Defensive: if somehow too much data was read
                else if (localIndex > expectedLength) {
                    #ifdef DEBUG
                        Debug::printHex(localBuffer, localIndex);
                        Debug::print("\n");
                        Debug::warnln("Buffer overflow (too much data)");
                    #endif

                    sendNACK(ComErrorCode::BUFFER_OVERFLOW);
                    resetState();
                    return;
                }
                break;
        }
    }
}

} // namespace UART_COMM

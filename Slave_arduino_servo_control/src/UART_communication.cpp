#include "UART_communication.h"
#include "System_status.h"
#include <CRC16.h>

namespace UART_communication {

// UART buffer settings
const unsigned UART_BUFFER_SIZE { 64 };

// Const values for packet
const uint8_t Header1 { 0xAA };
const uint8_t Header2 { 0x55 };
const uint8_t minPacketSize { 6 };
const uint8_t PACKET_TIMEOUT { 100 };

// Debug flag for serial output
constexpr bool DEBUG_MODE { true };

// Copy last packet sent, in case of communication error
uint8_t prevPacket[UART_BUFFER_SIZE] {};
uint8_t prevPacketSize { 0 };

CRC16 crc16;


void UART_init(uint32_t baudRate) {
    Serial1.begin(baudRate);
}

void receiveUARTData() {
    static enum class ReceiveState { SYNC, LENGTH, DATA } state = ReceiveState::SYNC;
    static uint8_t expectedLength = 0; // Expected packet length
    static unsigned long lastByteTime = 0; // Track last byte received time
    static int errorCount = 0; // Track consecutive errors, resets periodically

    static uint8_t localBuffer[UART_BUFFER_SIZE]; // Store per-packet data
    uint8_t localIndex = 0; // Local index per packet

    if (DEBUG_MODE) {
        Serial.print("Recieving: ");
    }

    while (Serial1.available()) {
        uint8_t incomingByte = Serial1.read();
        lastByteTime = millis(); // Update last received timestamp

        if (DEBUG_MODE) {
            Serial.print(incomingByte, HEX);
            Serial.print(" ");
        }

        switch (state) {
            case ReceiveState::SYNC:
                if (incomingByte == Header1 && localIndex == 0) {
                    localIndex++;
                } 
                else if (incomingByte == Header2 && localIndex == 1) {
                    localIndex++;  // Reset index; real data starts after this
                    state = ReceiveState::LENGTH;
                } 
                else {
                    localIndex = 0;  // Reset if bytes are out of sync
                }
                break;

            case ReceiveState::LENGTH:
                expectedLength = incomingByte + 2;  // Read packet length byte and add 2 for headers

                // Validate length
                if (static_cast<size_t>(expectedLength) > sizeof(localBuffer) || expectedLength < minPacketSize) { 
                    sendCommunicationError(ErrorCode::BUFFER_OVERFLOW);

                    errorCount++;
                    if (errorCount > 5) {  // If more than 5 bad packets in a row, flush to resync
                        if (DEBUG_MODE) Serial.println("[WARNING] Too many errors, flushing UART buffer");
                        while (Serial1.available()) Serial1.read();
                        errorCount = 0;
                    }
                    state = ReceiveState::SYNC;
                    return;
                }

                localBuffer[localIndex++] = incomingByte;  // Store length byte
                state = ReceiveState::DATA;
                break;

            case ReceiveState::DATA:
                if (localIndex < expectedLength) {  // Prevent buffer overflow
                    localBuffer[localIndex++] = incomingByte;
                } else {
                    sendCommunicationError(ErrorCode::BUFFER_OVERFLOW);
                    state = ReceiveState::SYNC;
                    return;
                }

                // Process the packet when full length is received
                if (localIndex >= expectedLength) {
                    if (validatePacketCRC(localBuffer, expectedLength)) { 
                        processReceivedPacket(localBuffer, expectedLength);
                        errorCount = 0;  // Reset error count on successful reception
                    } else {
                        Serial.print("Error here");
                        sendCommunicationError(ErrorCode::CHECKSUM_ERROR);
                    }
                    state = ReceiveState::SYNC;
                    return;
                }
                break;
        }
    }

    // Timeout Handling - Prevent stuck state if bytes stop arriving
    if (state != ReceiveState::SYNC && millis() - lastByteTime > PACKET_TIMEOUT) {
        if (DEBUG_MODE) Serial.println("[ERROR] Packet Timeout - Resetting State Machine");
        state = ReceiveState::SYNC;
        localIndex = 0;
        expectedLength = 0;
        errorCount = 0;  // Reset error count on timeout
    }
}

void processReceivedPacket(const uint8_t* packet, uint8_t packetSize) {
    if (packetSize < minPacketSize) return;  // Minimum packet size check

    if (DEBUG_MODE) Serial.print("Processing packet: ");

    // Extract command and process
    MainCommand command = static_cast<MainCommand>(packet[3]);
    switch (command) {
        case MainCommand::REQUEST_STATUS:
            sendRespondStatus();
            break;

        case MainCommand::RESPOND_STATUS:
            // Respond code for Arduino
            break;
        
        case MainCommand::REQUEST_SERVO_POSITIONS:
            sendServoPositions();   // Master asks for servo positions
            break;

        case MainCommand::RESPOND_SERVO_POSITIONS:
            // Respond with current servo positions
            break;

        case MainCommand::SET_SERVO_POSITION:
            // Function to set a single servo to a position
            break;

        case MainCommand::SET_ALL_POSITIONS:
            // Function to set all arm movement servos [Base, Shoulder, Elbow, Wrist]
            break;

        case MainCommand::SET_MAX_SPEED:
            // Function to set max speed
            break;

        case MainCommand::SYSTEM_CONTROL:
            // Master sends system control
            break;

        case MainCommand::REQUEST_ERROR_REPORT:
            // Master requests error report
            break;

        case MainCommand::RESPOND_ERROR_REPORT:
            sendRespondErrorReport();
            break;

        case MainCommand::COMMUNICATION_ERROR:
            sendPrevPacket();
            break;
        
        default:
            sendCommunicationError(ErrorCode::UNKNOWN_COMMAND);
            break;
    }
}

// Sends a formatted packet
void sendPacket(MainCommand command, const uint8_t* payload, uint8_t payloadLength) {
    // Ensure the packet size does not exceed buffer size
    uint8_t packetSize = minPacketSize + payloadLength;
    if (packetSize > UART_BUFFER_SIZE) {
        sendCommunicationError(ErrorCode::BUFFER_OVERFLOW);
        return;
    }

    // Packet buffer (no static)
    uint8_t packet[UART_BUFFER_SIZE];

    // Construct packet
    packet[0] = Header1; 
    packet[1] = Header2;
    packet[2] = packetSize - 2;
    packet[3] = static_cast<uint8_t>(command);

    // Copy payload if present
    if (payloadLength > 0) {
        memcpy(&packet[4], payload, payloadLength);
    }

    // Generate CRC correctly
    makePacketCRC(packet, packetSize);

    // Debug output (single print for efficiency)
    if (DEBUG_MODE) {
        Serial.print("\nSent: ");
        for (size_t i = 0; i < packetSize; i++) {
            Serial.print(packet[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Store last sent packet for potential retransmission
    storePreviousPacket(packet, packetSize);

    // Transmit packet
    Serial1.write(packet, packetSize);
}




void sendRespondStatus() {
    MainCommand command = MainCommand::RESPOND_STATUS;
    uint8_t statusCode = static_cast<uint8_t>(System_status::currentStatus);
    sendPacket(command, &statusCode, 1);
}

void sendServoPositions() {

}


void sendRespondErrorReport() {

}


void sendCommunicationError(ErrorCode error) {
    uint8_t errorCode = static_cast<uint8_t>(error);
    sendPacket(MainCommand::COMMUNICATION_ERROR, &errorCode, 1);
}


void sendPrevPacket() {
    Serial1.write(prevPacket, prevPacketSize);
}

void storePreviousPacket(const uint8_t packet[UART_BUFFER_SIZE], uint8_t size) {
    prevPacketSize = size;
    memcpy(prevPacket, packet, prevPacketSize);
}

void makePacketCRC(uint8_t* packet, uint8_t packetSize) {
    if (packet == nullptr || packetSize < minPacketSize) {
        if (DEBUG_MODE) Serial.println("[ERROR] Invalid packet or length in makePacketCRC!");
        return;
    }

    // Compute CRC-16 from index 2 (excluding headers, including length byte)
    crc16.reset();
    crc16.add(&packet[2], packetSize - 4);
    uint16_t crc = crc16.calc();

    // Append CRC to the end of the packet
    packet[packetSize - 2] = crc & 0xFF;
    packet[packetSize - 1] = (crc >> 8) & 0xFF;

    if (DEBUG_MODE) {
        Serial.print("Generated CRC: ");
        Serial.print(crc, HEX);
        Serial.println();
    }
}

bool validatePacketCRC(const uint8_t* packet, uint8_t packetSize) {
    if (packet == nullptr || packetSize < minPacketSize) {
        if (DEBUG_MODE) Serial.println("[ERROR] Invalid packet or length in validatePacketCRC!");
        return false;
    }

    // Extract received CRC from the last two bytes
    uint16_t receivedCRC = (packet[packetSize - 2] | (packet[packetSize - 1] << 8));

    // Compute CRC-16 from index 2 (excluding headers, including length byte)
    crc16.reset();
    crc16.add(&packet[2], packetSize - 4);
    uint16_t computedCRC = crc16.calc();

    if (DEBUG_MODE) {
        Serial.print("\nComputed CRC: ");
        Serial.print(computedCRC, HEX);
        Serial.print(" | Received CRC: ");
        Serial.println(receivedCRC, HEX);
    }

    return computedCRC == receivedCRC;
}


} // namespace UART_communication

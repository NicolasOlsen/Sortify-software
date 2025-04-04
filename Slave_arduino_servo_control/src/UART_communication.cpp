#include <CRC16.h>

#include "comms/UART_communication.h"
#include "shared/System_status.h"
#include "utils/Debug.h"

using namespace Com_code;


// UART buffer settings
constexpr unsigned UART_BUFFER_SIZE { 64 };

// Const values for packet
constexpr uint8_t startBytes[] { 0xAA, 0x55 };  // Can easily add more start bytes, Has to have at least one
constexpr uint8_t startBytesSize = sizeof(startBytes) / sizeof(startBytes[0]);
static_assert(startBytesSize > 0, "StartBytesSize must be greater than 0");

constexpr uint8_t minPacketSize = startBytesSize + 4;
constexpr uint8_t PACKET_TIMEOUT = 100; // ms

// Debug flag for local debugging
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
    static unsigned long packetStartTime = 0;  // Time started receiving the packet

    static uint8_t localBuffer[UART_BUFFER_SIZE]; // Store per-packet data
    static uint8_t localIndex = 0;  // Tracks position in sync sequence across calls


    // Check for packet timeout
    if (state != ReceiveState::SYNC && (millis() - packetStartTime > PACKET_TIMEOUT)) {
        Debug::printHex(localBuffer, localIndex, DEBUG_MODE);
        Debug::print("\n", DEBUG_MODE);
        Debug::warnln("Packet Timeout - Resetting State Machine", DEBUG_MODE);

        state = ReceiveState::SYNC;
        localIndex = 0;
        expectedLength = 0;
    }

    while (Serial1.available()) {
        uint8_t incomingByte = Serial1.read();

        switch (state) {
            case ReceiveState::SYNC:
                if (localIndex < startBytesSize && incomingByte == startBytes[localIndex]) {
                    localBuffer[localIndex++] = incomingByte;
                
                    if (localIndex == startBytesSize) {
                        state = ReceiveState::LENGTH;
                        packetStartTime = millis();
                        Debug::info("Receiving: ", DEBUG_MODE);
                    }
                } else {
                    if constexpr (DEBUG_MODE) {
                        if (localIndex > 0) {
                            Debug::printHex(localBuffer, localIndex);
                            Debug::print("\n");
                        }
                        Debug::warnln("Out of sync");
                    }
                    localIndex = 0;
                }                     
                break;

            case ReceiveState::LENGTH:
                expectedLength = incomingByte + startBytesSize;  // Read packet length byte and add for headers
                
                if (static_cast<size_t>(expectedLength) > sizeof(localBuffer) || // Validate length
                    expectedLength < minPacketSize - startBytesSize) {

                    Debug::printHex(localBuffer, localIndex, DEBUG_MODE);
                    Debug::print("\n", DEBUG_MODE); 
                    Debug::warnln("Length is either too long or too short", DEBUG_MODE);

                    sendCommunicationError(ErrorCode::BUFFER_OVERFLOW);

                    state = ReceiveState::SYNC;
                    return;
                }

                localBuffer[localIndex++] = incomingByte;  // Store length byte
                state = ReceiveState::DATA;
                break;

            case ReceiveState::DATA:
                localBuffer[localIndex++] = incomingByte;

                // Check if we've reached or exceeded expected length
                if (localIndex == expectedLength) {
                    Debug::printHex(localBuffer, localIndex, DEBUG_MODE);
                    Debug::print("\n", DEBUG_MODE); 
                
                    if (validatePacketCRC(localBuffer, expectedLength)) {
                        processReceivedPacket(localBuffer, expectedLength);
                    } else {
                        Debug::warnln("Checksum error", DEBUG_MODE);
                        sendCommunicationError(ErrorCode::CHECKSUM_ERROR);
                    }
                
                    // Reset packet tracking
                    state = ReceiveState::SYNC;
                    localIndex = 0;
                    expectedLength = 0;
                    packetStartTime = 0; // Reset timeout timer
                    return;
                
                } else if (localIndex > expectedLength) {
                    Debug::printHex(localBuffer, localIndex, DEBUG_MODE);
                    Debug::print("\n", DEBUG_MODE); 
                    Debug::warnln("Buffer overflow (too much data)", DEBUG_MODE);
                
                    sendCommunicationError(ErrorCode::BUFFER_OVERFLOW);

                    // Reset on overflow
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
    if (packetSize < minPacketSize) return;  // Minimum packet size check

    Debug::infoln("Processing packet: ", DEBUG_MODE);

    // Extract command and process
    MainCommand command = static_cast<MainCommand>(packet[startBytesSize + 1]); // start bytes, length and then main command
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
            // Master sends system control command
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

    uint8_t packet[UART_BUFFER_SIZE];

    // Construct packet
    for (uint8_t i = 0; i < startBytesSize; i++) {
        packet[i] = startBytes[i];
    }    
    packet[startBytesSize] = packetSize - startBytesSize;           // Add the length right after start bytes
    packet[startBytesSize + 1] = static_cast<uint8_t>(command);     // Add the command next

    // Copy payload if present
    if (payloadLength > 0) {
        memcpy(&packet[(startBytesSize + 2)], payload, payloadLength);  // payload copied in after startbytes, length and command
    }

    // Generate CRC correctly
    makePacketCRC(packet, packetSize);

    // Debug output
    Debug::info("Sent: ", DEBUG_MODE);
    Debug::printHex(packet, packetSize, DEBUG_MODE);
    Debug::print("\n");

    // Store last sent packet for potential retransmission, unless communication error
    if (command != MainCommand::COMMUNICATION_ERROR) {
        storePreviousPacket(packet, packetSize);
    }

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
        Debug::errorln("Invalid packet or length in makePacketCRC!", DEBUG_MODE);
        return;
    }
    
    uint16_t crc = calculateCRC16(packet, packetSize);

    // Append CRC to the end of the packet
    packet[packetSize - 2] = crc & 0xFF;
    packet[packetSize - 1] = (crc >> 8) & 0xFF;

    Debug::infoln("Generated CRC Bytes: " + 
        String(packet[packetSize - 2], HEX) + " " + 
        String(packet[packetSize - 1], HEX), DEBUG_MODE);   // Look at CRC to the end of the packet

}

bool validatePacketCRC(const uint8_t* packet, uint8_t packetSize) {
    if (packet == nullptr || packetSize < minPacketSize) {
        Debug::errorln("Invalid packet or length in validatePacketCRC!", DEBUG_MODE);
        return false;
    }

    // Extract received CRC from the last two bytes
    uint16_t receivedCRC = (packet[packetSize - 2] | (packet[packetSize - 1] << 8));

    uint16_t computedCRC = calculateCRC16(packet, packetSize);

    Debug::infoln("Computed CRC Bytes: " + 
        String(computedCRC & 0xFF, HEX) + " " + 
        String((computedCRC >> 8) & 0xFF, HEX) + 
        " | Received CRC Bytes: " + 
        String(receivedCRC & 0xFF, HEX) + " " + 
        String((receivedCRC >> 8) & 0xFF, HEX), DEBUG_MODE);

    return computedCRC == receivedCRC;
}

uint16_t calculateCRC16(const uint8_t* packet, uint8_t packetSize) {
    // Compute CRC-16 (excluding headers, including length byte)
    crc16.reset();
    crc16.add(&packet[startBytesSize], packetSize - (startBytesSize + 2));
    return crc16.calc();
}

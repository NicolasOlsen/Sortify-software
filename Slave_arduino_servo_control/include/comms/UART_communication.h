#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include <Arduino.h>
#include <stdint.h>

#include "Communication_code.h"

/*
 * ========================================================
 * UART Communication Packet Structure for Robot Arm Control
 * ========================================================
 * This protocol is used for structured communication between an SBC 
 * (such as a Raspberry Pi) and an Arduino over UART. The Arduino controls 
 * a robotic arm, and this protocol facilitates sending commands, receiving 
 * status updates, and ensuring data integrity.
 * 
 * 
 * ========================================================
 * Packet Format (Byte Order)
 * ========================================================
 * | Byte #  | Field Name     | Size (bytes) | Description |
 * |---------|----------------|--------------|-------------|
 * | 0       | Start Byte 1   | 1            | Always `0xAA` (indicates packet start) |
 * | 1       | Start Byte 2   | 1            | Always `0x55` (ensures message framing) |
 * | 2       | Packet Length  | 1            | Total length of the packet **including this byte** but excluding the start bytes |
 * | 3       | Command ID     | 1            | Identifies the command type (see `MainCommand` enum in communication_code.h) |
 * | 4 - N   | Payload        | Variable     | Command-specific data (if applicable) |
 * | Last-1  | CRC-16 (Low)   | 1            | Lower byte of CRC-16 checksum (error detection) |
 * | Last    | CRC-16 (High)  | 1            | Upper byte of CRC-16 checksum |
 * 
 * ========================================================
 * Payload Details for Position Commands
 * ========================================================
 * - **SET_SERVO_POSITION** (`0x05`):
 *   - Payload Structure:
 *     - Byte 4: `Servo ID` (1 byte)
 *     - Bytes 5-8: `Position Value` (4 bytes, little-endian)
 *   - Description: Sets the position of a specific servo.
 * 
 * - **SET_ALL_POSITIONS** (`0x06`):
 *   - Payload Structure:
 *     - Bytes 4-7: `Base Position` (4 bytes)
 *     - Bytes 8-11: `Shoulder Position` (4 bytes)
 *     - Bytes 12-15: `Elbow Position` (4 bytes)
 *     - Bytes 16-19: `Wrist Position` (4 bytes)
 *   - Description: Sets the positions for all servos except the gripper.
 * 
 * - **SEND_SERVO_POSITIONS** (`0x04`):
 *   - Payload Structure:
 *     - Bytes 4-7: `Base Position` (4 bytes)
 *     - Bytes 8-11: `Shoulder Position` (4 bytes)
 *     - Bytes 12-15: `Elbow Position` (4 bytes)
 *     - Bytes 16-19: `Wrist Position` (4 bytes)
 *   - Description: Reports the current positions of all servos except the gripper.
 * 
 * ========================================================
 * Example Packet: Setting Servo Position
 * ========================================================
 * Master sends the following packet to set the base servo to position `0x001E8480` (2,000,000 in decimal):
 * ```
 * | 0xAA | 0x55 | 0x09 | 0x05 | 0x01 | 0x80 | 0x84 | 0x1E | 0x00 | 0xB2 | 0x3F |
 * ```
 * - `0xAA 0x55` → Start bytes
 * - `0x09` → Packet length (9 bytes: includes itself, command, servo ID, position value, and CRC)
 * - `0x05` → `SET_SERVO_POSITION` command
 * - `0x01` → Servo ID (Base)
 * - `0x80 0x84 0x1E 0x00` → Position value (2,000,000 in little-endian format)
 * - `0xB2 0x3F` → CRC-16 checksum
 * 
 * ========================================================
 * Notes
 * ========================================================
 * - The CRC-16 checksum is calculated over **all bytes except the start bytes**.
 * - The packet length **includes itself**, meaning the byte at position `2` counts towards the total.
 * - This structure ensures **efficient and reliable UART communication** between the SBC and the Arduino controlling the robotic arm.
 */

// =====================
// Function Declarations
// =====================
/**
 * @brief Initializes UART and configures GPIO interrupts for bidirectional communication.
 * @param baudRate The baud rate for UART communication.
 */
void UART_init(uint32_t baudRate);

/**
 * @brief Reads a full UART packet from the buffer.
 */
void receiveUARTData();

/**
 * @brief Processes a complete packet received via UART.
 * @param packet The packet to process
 * @param length The langth of the packet
 */
void processReceivedPacket(const uint8_t* packet, uint8_t length);

/**
 * @brief Sends a structured UART packet.
 * @param command The command ID from MainCommand enum.
 * @param payload Pointer to the payload data.
 * @param payloadLength Number of bytes in the payload.
 */
void sendPacket(Com_code::MainCommand, const uint8_t* payload, uint8_t payloadLength);

/**
 * @brief Sends the current system status
 */
void sendRespondStatus();

/**
 * @brief Sends the current servo positions in one package (Excluding the gripper)
 */
void sendServoPositions();

/**
 * @brief Sends the latest recorded system or servo error
 */
void sendRespondErrorReport();

/**
 * @brief Sends notice of communication error and type, so recieving device can try again
 */
void sendCommunicationError(Com_code::ErrorCode error);

/**
 * @brief Send the exact copy of the previous packet, used in case of communication error
*/
void sendPrevPacket();

/**
 * @brief Stores the packet in case of communication error
 * @param packet The packet to be copied
 * @param size The size of the packet
 */
void storePreviousPacket(const uint8_t* packet, uint8_t size);

/**
 * @brief Makes the CRC and adds it to the packet to be sent
 * @param packet The packet to be send
 * @param size The size of the packet
 */
void makePacketCRC(uint8_t* packet, uint8_t packetSize);

/**
 * @brief Validates the CRC in the recieved packet
 * @param packet The packet to be validated
 * @param size The size of the packet
 */
bool validatePacketCRC(const uint8_t* packet, uint8_t packetSize);

/**
 * @brief Helper function for other CRC functions, to keep consistent
 * @param packet The packet to calculate CRC16
 * @param size The size of the packet
 */
uint16_t calculateCRC16(const uint8_t* packet, uint8_t packetSize);


#endif // UART_COMMUNICATION_H
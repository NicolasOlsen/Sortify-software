#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include <stdint.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

#include "Communication_code.h"
#include "config/communication_config.h"

/*
 * ========================================================
 * UART Communication Packet Structure for Robot Arm Control
 * ========================================================
 * This protocol is used for structured communication between an SBC 
 * (such as a Raspberry Pi) and an Arduino over UART. The Arduino controls 
 * a robotic arm, and this protocol facilitates sending commands, receiving 
 * status updates, and ensuring data integrity.
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
 * | Last-3  | System state   | 1            | The system state of the Arduino (0x00 = no error) |
 * | Last-2  | CRC-16 (Low)   | 1            | Lower byte of CRC-16 checksum (error detection) |
 * | Last-1  | CRC-16 (High)  | 1            | Upper byte of CRC-16 checksum |
 * 
 * ========================================================
 * Payload Details for Position Commands
 * ========================================================
 * - **SET_SERVO_POSITION** (`0x09`):
 *   - Payload Structure:
 *     - Byte 4: `Servo ID` (1 byte)
 *     - Bytes 5-8: `Position Value` (4 bytes, little-endian)
 *   - Description: Sets the position of a specific servo.
 * 
 * - **SET_ALL_POSITIONS** (`0x0A`):
 *   - Payload Structure:
 *     - Bytes 4-7: `Base Position` (4 bytes)
 *     - Bytes 8-11: `Shoulder Position` (4 bytes)
 *     - Bytes 12-15: `Elbow Position` (4 bytes)
 *     - Bytes 16-19: `Wrist Position` (4 bytes)
 *   - Description: Sets the positions for all servos except the gripper.
 * 
 * - **RESPOND_SERVO_POSITIONS** (`0x06`):
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
 * | 0xAA | 0x55 | 0x0A | 0x09 | 0x01 | 0x80 | 0x84 | 0x1E | 0x00 | 0x00 | 0xB2 | 0x3F |
 * ```
 * - `0xAA 0x55` → Start bytes
 * - `0x0A` → Packet length (10 bytes total: length byte, command, ID, 4-byte position, error code, CRC)
 * - `0x09` → `SET_SERVO_POSITION` command
 * - `0x01` → Servo ID (Base)
 * - `0x80 0x84 0x1E 0x00` → Position value (2,000,000 in little-endian)
 * - `0x00` → Last error code (0x00 = no error)
 * - `0xB2 0x3F` → CRC-16 checksum
 * 
 * ========================================================
 * Notes
 * ========================================================
 * - The CRC-16 checksum is calculated over **all bytes starting from the length byte up to and including the error byte**.
 * - The packet length **includes itself**, but **not the start bytes**.
 * - The last error byte is always appended before the CRC to provide context about the Arduino’s internal state.
 * - This structure ensures **efficient and reliable UART communication** between the SBC and the Arduino controlling the robotic arm.
 */



#endif // UART_COMMUNICATION_H
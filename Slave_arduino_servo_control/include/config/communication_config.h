#ifndef COMMUNICATION_CONFIG_H
#define COMMUNICATION_CONFIG_H

#include <stdint.h>

// === UART Settings ===
constexpr uint32_t BAUDRATE_COMM = 1000000;
#define COMM_SERIAL Serial1

// === FreeRTOS Queue Settings ===
constexpr uint16_t QUEUE_SIZE = 4; // Holds up to 4 full packets

// === Packet Format ===
constexpr uint8_t UART_BUFFER_SIZE = 128;
constexpr uint8_t PACKET_TIMEOUT = 100; // ms
constexpr uint8_t START_BYTES[] = { 0xAA, 0x55 };
constexpr uint8_t START_BYTES_SIZE = sizeof(START_BYTES);
static_assert(START_BYTES_SIZE > 0, "START_BYTES_SIZE must be greater than 0");


 // Packet structure: [START][LEN][CMD][PAYLOAD][SYSTEM_STATE][CRC_L][CRC_H]
constexpr uint8_t MIN_PACKET_SIZE = START_BYTES_SIZE + 1 + 1 + 2;  // start + len + cmd + CRC   |   recieving bytes doesnt have system state

constexpr uint8_t LENGTH_INDEX  = START_BYTES_SIZE;
constexpr uint8_t COMMAND_INDEX = START_BYTES_SIZE + 1;     // Start bytes, then length byte, then main command byte
constexpr uint8_t PAYLOAD_INDEX = COMMAND_INDEX + 1;


#endif // COMMUNICATION_CONFIG_H

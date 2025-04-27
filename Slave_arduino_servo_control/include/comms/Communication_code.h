#ifndef COMMUNICATION_CODE_H
#define COMMUNICATION_CODE_H

#include <stdint.h>

namespace COMM_CODE {
    // Main Command List
    // These commands are used for communication between the SBC (such as a Raspberry Pi) and the Arduino.
    // The SBC sends these commands to request information or control the servos.
    enum class MainCommand : uint8_t {
        // Status and Health
        HEARTBEAT       = 0x01,     // Master checks if Arduino is alive (Arduino responds with ACK)
        ACK,                        // Generic ACK when no specific data is returned
        NACK,                       // Negative Acknowledge, payload contains ComErrorCode
    
        // Flexible Position Commands
        READ_POSITION_RANGE,        // Read positions from [start_id, count]
        WRITE_POSITION_RANGE,       // Write positions for [start_id, count]

        // Flexible Velocity Command
        WRITE_VELOCITY_RANGE,       // Write velocities for [start_id, count]

        // Flexible Error Reporting
        READ_ERROR_RANGE            // Read error status from [start_id, count]
    };    

    // System Status Codes
    // Arduino sends one of these status codes in response to REQUEST_STATUS.
    enum class StatusCode : uint8_t {
        INITIALIZING = 0x01,   // System is booting up, hardware not ready
        IDLE,                  // System initialized and waiting for commands
        MOVING,                // Servos are actively moving to target positions
        FAULT_INIT,            // Initialization failure (e.g., servo not found, UART setup failed)
        FAULT_RUNTIME          // Failure during operation (e.g., overheat, disconnection)
    };

    // Error Codes
    // These errors are reported with a NACK payload or in error reports.
    enum class ComErrorCode : uint8_t {
        COMM_TIMEOUT = 0x01,        // Timeout waiting for full packet
        CHECKSUM_ERROR,             // CRC16 mismatch
        UNKNOWN_COMMAND,            // Unrecognized command byte
        BUFFER_OVERFLOW,            // Packet too large for buffer
        QUEUE_FULL,                 // FreeRTOS queue full
        INVALID_PAYLOAD_SIZE,       // Payload size does not match expected for given command
        ID_OUT_OF_RANGE             // The requested id is out of range
    };

    // Servo Identifiers
    // Used to reference a specific servo motor when setting positions or stopping individual servos.
    enum class ServoId : uint8_t {
        BASE     = 0x00,  // Base rotation servo
        SHOULDER = 0x01,  // Shoulder joint servo
        ELBOW    = 0x02,  // Elbow joint servo
        WRIST    = 0x03,  // Wrist joint servo
        GRIPPER  = 0x04   // Gripper (NOT INCLUDED in SET_ALL_POSITIONS)
    };
}

#endif // COMMUNICATION_CODE_H

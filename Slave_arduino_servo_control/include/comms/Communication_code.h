#ifndef COMMUNICATION_CODE_H
#define COMMUNICATION_CODE_H

#include <stdint.h>

namespace COMM_CODE {
    // Main Command List
    // These commands are used for communication between the SBC (such as a Raspberry Pi) and the Arduino.
    // The SBC sends these commands to request information or control the servos.
    enum class MainCommand : uint8_t {
        // Status and Health
        PING_ = 0x01,                 // Master checks if Arduino is alive (Arduino responds with PING + system state)
        NACK,                        // Negative Acknowledge (command failed), payload = ComErrorCode
    
        // Position Control
        READ_POSITION_RANGE,        // Read current positions from [start_id, count]
        WRITE_POSITION_RANGE,       // Set target positions for [start_id, count]
        STOP_MOVEMENT,              // Immediately stops all servo movement
    
        // Velocity Control
        WRITE_VELOCITY_RANGE,       // Set velocities for [start_id, count]
    
        // Error Reporting
        READ_CURRENT_ERROR_RANGE,           // Read current error from [start_id, count]
        READ_LAST_ERROR_RANGE       // Read last error from [start_id, count]
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
        SYSTEM_FAULT = 0x01,        // System is in fault mode
        COMM_TIMEOUT,               // Timeout waiting for full packet
        CHECKSUM_ERROR,             // CRC16 mismatch
        UNKNOWN_COMMAND,            // Unrecognized command byte
        INVALID_PAYLOAD_SIZE,       // The payload size is invalid
        BUFFER_OVERFLOW,            // Packet too large for buffer
        QUEUE_FULL,                 // FreeRTOS queue full
        ID_OUT_OF_RANGE,            // The requested id is out of range
        POSITION_OUT_OF_RANGE       // The position is out of range
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

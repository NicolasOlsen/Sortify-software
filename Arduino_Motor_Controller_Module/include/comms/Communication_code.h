#ifndef COMMUNICATION_CODE_H
#define COMMUNICATION_CODE_H

#include <stdint.h>

namespace COMM_CODE {
    // Main Command List
    enum class MainCommand : uint8_t {
        // Status and Health
        PING_ = 0x01,               // To only get system state
        NACK,                       // Negative Acknowledge (response command if packet processing failed), 
                                    //  params: ComErrorCode
    
        // Position Control
        READ_POSITION_RANGE,        // Read current positions, params: [start_id, count]
        WRITE_POSITION_RANGE,       // Set target positions, params: [start_id, count]
        STOP_MOVEMENT,              // Stop all servo movement. Usefull for an easy or emergency stop
    
        // Velocity Control
        WRITE_VELOCITY_RANGE,       // Set velocities, params: [start_id, count]. Usefull for synced movement
    
        // Error Reporting
        READ_CURRENT_ERROR_RANGE,   // Read current error, params: [start_id, count]. Usefull for debugging/logging
        READ_LAST_ERROR_RANGE       // Read last error, params: [start_id, count]. Usefull for debugging/logging
    };       

    // System Status Codes
    enum class StatusCode : uint8_t {
        INITIALIZING = 0x01,   // System is initializing up
        IDLE,                  // No servos are moving
        MOVING,                // At least one servo is moving
        FAULT_INIT,            // Failure duringInitialization
        FAULT_RUNTIME          // Failure during operation
    };

    // Error Codes
    // These errors are sent with a NACK payload.
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
}
#endif // COMMUNICATION_CODE_H

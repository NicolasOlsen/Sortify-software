#ifndef COMMUNICATION_CODE_H
#define COMMUNICATION_CODE_H

#include <stdint.h>

namespace Com_code {
    // Main Command List
    // These commands are used for communication between the SBC (such as a Raspberry Pi) and the Arduino.
    // The SBC sends these commands to request information or control the servos.
    enum class MainCommand : uint8_t {
        // Status and Health
        HEARTBEAT       = 0x01,     // Master checks if Arduino is alive (Arduino responds with ACKNOWLEDGE)
        ACKNOWLEDGE,                // Generic ACK when no specific data is returned
    
        // Servo Information
        REQUEST_SERVO_POSITIONS,    // Master requests positions of all servos (excluding gripper)
        RESPOND_SERVO_POSITIONS,    // Arduino responds with [Base, Shoulder, Elbow, Wrist]
    
        // Motion Control
        SET_SERVO_POSITION,         // Master sets a single servo position
        SET_ALL_POSITIONS,          // Master sets all positions (excluding gripper)
        SET_SERVO_GOAL_VELOCITY,    // Master sets a single servo goal velocity
        SET_ALL_GOAL_VELOCITY,      // Master sets all servos max velocity
        STOP_MOVEMENT,              // Master sets all positions to the current position
    
        // Fault/Error Reporting
        REQUEST_ERROR_STATUS,   // Master requests current error statuses
        RESPOND_ERROR_STATUS,   // Arduino responds with the current error codes
    
        COMMUNICATION_ERROR     // Indicates a communication issue was detected
    };

    

    // System Status Codes
    // Arduino sends one of these status codes in response to REQUEST_STATUS.
    enum class StatusCode : uint8_t {
        INITIALIZING = 0x01,    // System is booting up, not ready yet
        IDLE,                   // Servos are idle
        MOVING,                 // Servos are moving
        FAULT,                  // System is in an error state and cannot recover (sends the number of errors)
    };

    
    // Error Codes
    // These errors are reported with ERROR_REPORT.
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
        BASE     = 0x01,  // Base rotation servo
        SHOULDER = 0x02,  // Shoulder joint servo
        ELBOW    = 0x03,  // Elbow joint servo
        WRIST    = 0x04,  // Wrist joint servo
        GRIPPER  = 0x05   // Gripper (NOT INCLUDED in SET_ALL_POSITIONS)
    };
}

#endif // COMMUNICATION_CODE_H
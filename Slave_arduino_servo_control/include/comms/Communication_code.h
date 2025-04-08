#ifndef COMMUNICATION_CODE_H
#define COMMUNICATION_CODE_H

#include <stdint.h>

namespace Com_code {
    // =====================
    // Main Command List
    // =====================
    // These commands are used for communication between the SBC (such as a Raspberry Pi) and the Arduino.
    // The SBC sends these commands to request information or control the servos.
    enum class MainCommand : uint8_t {
        // Status and Health
        REQUEST_STATUS              = 0x01,  // Master requests the system's current status (Arduino responds with RESPOND_STATUS)
        RESPOND_STATUS              = 0x02,  // Arduino responds with the system's current state
        ACKNOWLEDGE                 = 0x03,  // Generic ACK when no specific data is returned
        HEARTBEAT                   = 0x04,  // Master checks if Arduino is alive (Arduino responds with ACKNOWLEDGE)
    
        // Servo Information
        REQUEST_SERVO_POSITIONS     = 0x05,  // Master requests positions of all servos (excluding gripper)
        RESPOND_SERVO_POSITIONS     = 0x06,  // Arduino responds with [Base, Shoulder, Elbow, Wrist]
    
        REQUEST_CURRENT_SPEED       = 0x07,  // Master requests current speed of a servo (0x00 for slowest)
        RESPOND_CURRENT_SPEED       = 0x08,  // Arduino responds with current speed of requested servo
    
        // Motion Control
        SET_SERVO_POSITION          = 0x09,  // Master sets a single servo position
        SET_ALL_POSITIONS           = 0x0A,  // Master sets all positions (excluding gripper)
        SET_MAX_SPEED               = 0x0B,  // Master sets global max speed
        SYSTEM_CONTROL              = 0x0C,  // STOP, RESUME, RESTART (requires subcommand)
    
        // Fault/Error Reporting
        REQUEST_ERROR_STATUS        = 0x0D,  // Master requests current error status
        RESPOND_ERROR_STATUS        = 0x0E,  // Arduino responds with the current error code(s)
    
        COMMUNICATION_ERROR         = 0x0F   // Indicates a communication issue was detected
    };
    
    
    
    // =====================
    // Control Subcommands
    // =====================
    // These subcommands define actions that modify the system's operation.
    // Used with the SYSTEM_CONTROL command.
    enum class ControlSubCommand : uint8_t {
        STOP      = 0x01,  // Stop a specific servo (Requires ServoId). Servo ID 0x00 = all servos.
        RESUME    = 0x02,  // Resume operation after a stop (Requires ServoId). Servo ID 0x00 = all servos.
        RESTART   = 0x03   // Restart the system
    };
    
    // =====================
    // System Status Codes
    // =====================
    // Arduino sends one of these status codes in response to REQUEST_STATUS.
    enum class StatusCode : uint8_t {
        INITIALIZING        = 0x01,  // System is booting up, not ready yet
        IDLE                = 0x02,  // Servos is idle
        MOVING              = 0x03,  // Servos are moving
        DATA_READY          = 0x04,  // Data is ready to be sent
        WAITING             = 0x05,  // System is waiting to resume
        RECOVARABLE_FAULT   = 0x06,  // System is in an error state and is trying to recover (sends the number of errors)
        UNRECOVARABLE_FAULT = 0x07   // System is in an error state and cannot recover (sends the number of errors)
    };
    
    // =====================
    // Error Codes
    // =====================
    // These errors are reported with ERROR_REPORT.
    enum class ComErrorCode : uint8_t {    
        // SBC ↔ Arduino Communication Errors
        COMM_TIMEOUT        = 0x0B,  // Timeout waiting for full command
        CHECKSUM_ERROR      = 0x0C,  // Invalid checksum received
        UNKNOWN_COMMAND     = 0x0D,  // Sent an unrecognized command
        BUFFER_OVERFLOW     = 0x0E   // UART buffer overflow detected
    };

    // =====================
    // Servo Identifiers
    // =====================
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
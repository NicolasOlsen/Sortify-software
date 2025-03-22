#ifndef COMMUNICATION_CODE_H
#define COMMUNICATION_CODE_H

#include <stdint.h>

namespace Com_code {
    // =====================
    // Main Command List
    // =====================
    // These commands are used for communication between the SBC (such as a Raspberry Pi) and the Arduino.
    // The SBC sends these commands via serial to request information or control the servos.
    enum class MainCommand : uint8_t {
        REQUEST_STATUS              = 0x01,  // Master requests the system's current status (Arduino responds with RESPOND_STATUS and system state)
        RESPOND_STATUS              = 0x02,  // Arduino responds with the system's current status (IDLE, MOVING, FAULT, etc.)
        
        REQUEST_SERVO_POSITIONS     = 0x03,  // Master requests the current positions of all servos (except the gripper)
        RESPOND_SERVO_POSITIONS     = 0x04,  // Arduino responds with servo positions in order: [Base, Shoulder, Elbow, Wrist]
        
        SET_SERVO_POSITION          = 0x05,  // Master sets the position of a specific servo (Requires ServoId and position value)
        SET_ALL_POSITIONS           = 0x06,  // Master sets positions for all servos except the gripper [Base, Shoulder, Elbow, Wrist]
        
        SET_MAX_SPEED               = 0x07,  // Master sets a global speed limit for all servos (0 means dynamic speed based on slowest servo)
        REQUEST_CURRENT_SPEED       = 0x08,  // Master requests the current speed of a servo (Requires ServoId, 0x00 means the slowest moving servo speed)
        RESPOND_CURRENT_SPEED       = 0x08,  // Arduino responds with the current speed of a servo (Requires ServoId, 0x00 means the slowest moving servo speed)
        
        SYSTEM_CONTROL              = 0x08,  // Master sends a system-wide action command (STOP, RESUME, RESTART) using ControlSubCommand
        
        REQUEST_ERROR_REPORT        = 0x09,  // Master requests the latest error report (Arduino responds with RESPOND_ERROR_REPORT)
        RESPOND_ERROR_REPORT        = 0x0A,  // Arduino responds with the latest recorded error(s)
        COMMUNICATION_ERROR         = 0x0B   // One of the devices has had a communication error (will send with type of communication error), will make recieving device try again
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
    enum class ErrorCode : uint8_t {
        // Errors directly from Dynamixel status packet
        SERVO_VOLTAGE_ERROR     = 0x01,  // Voltage out of range (Dynamixel ERRBIT_VOLTAGE)
        SERVO_ANGLE_LIMIT_ERROR = 0x02,  // Commanded angle out of limit (Dynamixel ERRBIT_ANGLE)
        SERVO_OVERHEAT_ERROR    = 0x03,  // Overheating detected (Dynamixel ERRBIT_OVERHEATING)
        SERVO_RANGE_ERROR       = 0x04,  // Out of range (Dynamixel ERRBIT_RANGE)
        SERVO_CHECKSUM_ERROR    = 0x05,  // Checksum mismatch in response (Dynamixel ERRBIT_CHECKSUM)
        SERVO_OVERLOAD_ERROR    = 0x06,  // Overload detected (Dynamixel ERRBIT_OVERLOAD)
        SERVO_INSTRUCTION_ERROR = 0x07,  // Invalid instruction received by servo (Dynamixel ERRBIT_INSTRUCTION)
    
        // Errors inferred by Arduino
        SERVO_POSITION_ERROR    = 0x08,  // Servo didn't reach target position in time
        SERVO_RESPONSE_TIMEOUT  = 0x09,  // Timeout waiting for servo response
        SERVO_BAUDRATE_ERROR    = 0x0A,  // Baudrate mismatch detected
    
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
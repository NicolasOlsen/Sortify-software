#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <DynamixelShield.h>
#include <Adafruit_PWMServoDriver.h>

namespace ServoControl {

    extern DynamixelShield dxl;
    extern Adafruit_PWMServoDriver pwm;

    /**
     * @brief Pings the servo and returns if true succesful, will also save the lib error if it doesnt work
     * @param id ID of the servo to ping
     * @return True if the ping was succesfull
     */
    bool PingServo(uint8_t id);

    /**
     * @brief Sets the smartservo in the position that is stored in the goal position array, will also mark the id as changed
     * @param id ID of the servo to set position for
     */
    void SetSmartServoToGoalPosition(uint8_t id);

    /**
     * @brief Sets the analog servo in the position that is stored in the goal position array, will also mark the id as changed. Currently hardcoded for one servo at servo 0 on the servo driver
     * @param id ID of the servo to set position for
     */
    void SetAnalogServoToGoalPosition(uint8_t id);

    /**
     * @brief Stores the current smartservo position of the id, stores the error if failed
     * @param id ID of the servo to set position for
     */
    void StoreCurrentServoPosition(uint8_t id);

    /**
     * @brief Shortcut function for setting all the servos to the goal position stored in the shared array
     */
    void SetServosToGoalPosition();


    /**
     * @brief Shortcut function for storing the current servo positions in the shared array
     */
    void StoreCurrentServoPositions();
    
}

#endif
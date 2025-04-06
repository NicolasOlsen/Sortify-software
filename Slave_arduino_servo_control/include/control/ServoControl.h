#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <DynamixelShield.h>
#include <Adafruit_PWMServoDriver.h>

extern DynamixelShield dxl;
extern Adafruit_PWMServoDriver pwm;

/**
 * @brief Pings the servo and returns if succesful, will also save the lib error if it doesnt work
 * @param id ID of the servo to ping
 * @return Returns wether the ping was succesfull
 */
bool PingServo(uint8_t id);

/**
 * @brief Sets the smartservo in the position that is saved in the goal position array, will also mark the id as changed
 * @param id ID of the servo to set position for
 */
void SetSmartServoPosition(uint8_t id);

/**
 * @brief Sets the analog servo in the position that is saved in the goal position array, will also mark the id as changed. Currently hardcoded for one servo at servo 0 on the servo driver
 * @param id ID of the servo to set position for
 */
void SetAnalogServoPosition(uint8_t id);

/**
 * @brief Gets the smartservo position that is saved in the current position array
 * @param id ID of the servo to set position for
 */
void GetServoPosition(uint8_t id);

#endif
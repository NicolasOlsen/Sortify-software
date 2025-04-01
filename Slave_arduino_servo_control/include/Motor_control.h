#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/**
 * @brief Function for initializing the servos
 */
void Initialize_Servos();

/**
 * @brief Sets all the servos to the positions saved in the setPositions array 
 */
void Set_Positions();

/**
 * @brief Gets all the current servo positions and saves them in the currentPositions array
 */
void Get_positions();

#endif  // MOTOR_CONTROL_H
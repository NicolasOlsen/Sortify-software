#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

#include <stdint.h>
#include <Arduino_FreeRTOS.h>

/**
 * @brief Configuration for a FreeRTOS task.
 * 
 * Combines task period and priority into a single struct for easier management and cleaner code.
 */
struct TaskConfig {
    TickType_t period;           // How often the task should run (in ticks)    |   Be aware that pdMS is turned to tick
    UBaseType_t priority;        // FreeRTOS priority (higher = more important)
    uint16_t stackSize;          // Stack size in words (not bytes!)
};

// === Individual Task Configurations ===

/**
 * @brief Communication task
 * High priority.
 */
constexpr TaskConfig COMM_TASK = { pdMS_TO_TICKS(17), 4, 512 };

/**
 * @brief Think task
 * Medium-high priority.
 */
constexpr TaskConfig THINK_TASK = { pdMS_TO_TICKS(33), 3, 256 };
constexpr uint8_t MAX_ERRORS = 3;

/**
 * @brief Servo setter task
 * Medium-low priority.
 */
constexpr TaskConfig SET_TASK = { pdMS_TO_TICKS(33), 2, 512 };

/**
 * @brief Servo reader task
 * Lowest priority.
 */
constexpr TaskConfig READ_TASK = { pdMS_TO_TICKS(33), 1, 512 };

#endif // TASK_CONFIG_H

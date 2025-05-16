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
    TickType_t period;           // How often the task should run (in ticks)    |   Be aware that pdMS is turned to tick, the default is 1 tck is 15ms from the watchdog timer
    UBaseType_t priority;        // FreeRTOS priority (higher = more important)
    uint16_t stackSize;          ///< Stack size in words (not bytes!)
};

// === Individual Task Configurations ===

/**
 * @brief Communication task
 * High priority, frequent execution for critical I/O with external systems (SBC or PC).
 */
constexpr TaskConfig COMM_TASK = { pdMS_TO_TICKS(17), 4, 512 };

/**
 * @brief Think/task planner
 * Medium-high priority, background decision-making.
 */
constexpr TaskConfig THINK_TASK = { pdMS_TO_TICKS(33), 3, 256 };
constexpr uint8_t MAX_ERRORS = 3;

/**
 * @brief Servo setter task
 * Medium-low priority, updates target positions and speeds.
 */
constexpr TaskConfig SET_TASK = { pdMS_TO_TICKS(33), 2, 512 };

/**
 * @brief Servo reader task
 * Lowest priority, reads servo state at a moderate rate.
 */
constexpr TaskConfig READ_TASK = { pdMS_TO_TICKS(33), 1, 512 };

#endif // TASK_CONFIG_H

#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

#include <Arduino.h>
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
constexpr TaskConfig COMM_TASK = { pdMS_TO_TICKS(400), 3, 256 };

/**
 * @brief Servo reader task
 * Medium-high priority, reads servo state at a moderate rate.
 */
constexpr TaskConfig READ_TASK = { pdMS_TO_TICKS(600), 2, 256 };

/**
 * @brief Servo setter task
 * Medium-low priority, updates target positions and speeds.
 */
constexpr TaskConfig SET_TASK = { pdMS_TO_TICKS(1000), 1, 256 };

/**
 * @brief Think/task planner
 * Low priority, background decision-making.
 */
constexpr TaskConfig THINK_TASK = { pdMS_TO_TICKS(1200), 0, 256 };

#endif // TASK_CONFIG_H

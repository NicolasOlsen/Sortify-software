#ifndef TASK_TIMER_H
#define TASK_TIMER_H

#include <Arduino.h>
#include "config/project_config.h"

#ifdef TIMING_MODE

#ifndef TIMING_SERIAL
#define TIMING_SERIAL Serial
#endif

struct TaskTimingStats {
    uint32_t executionTime;
    uint32_t maxExecutionTime;
    uint32_t totalExecutionTime;
    uint32_t runCount;

    TaskTimingStats() 
        : executionTime(0), maxExecutionTime(0), totalExecutionTime(0), runCount(0) {}

    void update(uint32_t duration) {
        executionTime = duration;
        totalExecutionTime += duration;
        runCount++;
        if (duration > maxExecutionTime) {
            maxExecutionTime = duration;
        }
    }

    float averageTime() const {
        return runCount > 0 ? static_cast<float>(totalExecutionTime) / runCount : 0;
    }

    void printTimingStats(const String& task) const {
        TIMING_SERIAL.print(task + " Timing Avg: ");
        TIMING_SERIAL.print(averageTime());
        TIMING_SERIAL.print(" us, Max: ");
        TIMING_SERIAL.print(maxExecutionTime);
        TIMING_SERIAL.print(" us");
    }

    void reset() {
        executionTime = 0;
        maxExecutionTime = 0;
        totalExecutionTime = 0;
        runCount = 0;
    }
};

#endif // TIMING_MODE

#endif // TASK_TIMER_H

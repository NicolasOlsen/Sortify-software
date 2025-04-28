#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// === Modes (define ONE) ===
// #define DEBUG
// #define TIMING_MODE

#if defined(DEBUG) && defined(TIMING_MODE)
    #error "DEBUG and TIMING_MODE cannot be enabled at the same time"
#endif

// === Optional features ===
#define USE_COLOR false
#define USE_TIMESTAMPS true

// === Logging settings ===
#define DEBUG_LEVEL LOG_INFO
#define DEBUG_SERIAL Serial

// === Timing settings ===
#define TIMING_SERIAL Serial1
#define TIMING_SAMPLE_COUNT 1000

#define TIMING_DELAY_TASKS 1000

#endif // PROJECT_CONFIG_H

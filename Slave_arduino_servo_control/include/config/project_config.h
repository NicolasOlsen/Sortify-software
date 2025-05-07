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
#define TIMING_SERIAL Serial2
#define TIMING_BAUD 1000000
#define TIMING_SAMPLE_COUNT 1000

// Used for timing individuall tasks, comment to turn on/off
// #define INDIVIDUAL_TIMING_MODE
#define TIMING_DELAY_TASKS 1000000

#if defined(INDIVIDUAL_TIMING_MODE) && !defined(TIMING_MODE)
    #error "INDIVIDUAL_TIMING_MODE requires TIMING_MODE to be enabled"
#endif

#endif // PROJECT_CONFIG_H

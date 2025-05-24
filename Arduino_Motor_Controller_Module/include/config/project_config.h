#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// === Modes (define ONE) ===
// #define DEBUG
// #define TIMING_MODE

#if defined(DEBUG) && defined(TIMING_MODE)  // Debug logs take a considerable time, even at 1Mbps
    #error "DEBUG and TIMING_MODE cannot be enabled at the same time"
#endif

// === Optional features ===
#define USE_COLOR false             // Color for use in Serial monitors (needs to be supported, e.g. VS Code Serial monitor)
#define USE_TIMESTAMPS true         // Shows timestamps for logs in ms

// === Logging settings ===
#define DEBUG_LEVEL LOG_INFO        // Log level, less critical levels will be ignored
#define DEBUG_SERIAL Serial3        // Dont use Serial'0', it's used for Dxl bus
#define DEBUG_BAUD 1000000          // Baudrate of debug serial

// === Timing settings ===
#define TIMING_SERIAL Serial2       // Dont use Serial'0', it's used for Dxl bus
#define TIMING_BAUD 1000000         // Baudrate of timing serial
#define TIMING_SAMPLE_COUNT 1000    // Amount of samples before printing average and worst time

// Used for timing individuall tasks, comment to turn on/off
// #define INDIVIDUAL_TIMING_MODE
#define TIMING_DELAY_TASKS 1000000

#if defined(INDIVIDUAL_TIMING_MODE) && !defined(TIMING_MODE)
    #error "INDIVIDUAL_TIMING_MODE requires TIMING_MODE to be enabled"
#endif

#endif // PROJECT_CONFIG_H

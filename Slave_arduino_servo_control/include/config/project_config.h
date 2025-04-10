#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// === Debug Settings ===
// Global toggles for debugging output
constexpr bool DEBUG_MODE     = true;   // Enable or disable all debug output
constexpr bool USE_COLOR      = false;   // Use ANSI color codes in logs
constexpr bool USE_TIMESTAMPS = true;  // Prepend logs with milliseconds since boot

#define DEBUG_SERIAL Serial

#endif // PROJECT_CONFIG_H

#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <Arduino.h>
#include "config/project_config.h"

//  ANSI color codes 
#define COLOR_RED     "\033[31m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_RESET   "\033[0m"

enum LogLevel {
    LOG_NONE = 0,
    LOG_ERROR,
    LOG_WARN,
    LOG_INFO,
    LOG_VERBOSE
};

// Default if not overridden
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL LOG_INFO
#endif

#ifdef DEBUG

namespace Debug {

inline void init(uint32_t baudRate) {
    DEBUG_SERIAL.begin(baudRate);
}

inline void print(const String& msg) {
    DEBUG_SERIAL.print(msg);
}

inline void println(const String& msg) {
    DEBUG_SERIAL.println(msg);
}

inline void println() { DEBUG_SERIAL.println(); }

inline void printTimestamp() {
    if constexpr (USE_TIMESTAMPS) {
        DEBUG_SERIAL.print('(');
        DEBUG_SERIAL.print(millis());
        DEBUG_SERIAL.print(F("ms) "));
    }
}

inline void logPrefix(const char* label, const char* color = nullptr) {
    if constexpr (USE_COLOR) {
        if (color) DEBUG_SERIAL.print(color);
    }
    DEBUG_SERIAL.print(label);
    if constexpr (USE_COLOR) {
        if (color) DEBUG_SERIAL.print(COLOR_RESET);
    }
    printTimestamp();
}

//  Log functions by level 

inline void error(const String& msg) {
    if constexpr (DEBUG_LEVEL >= LOG_ERROR) {
        logPrefix("[ERROR] ", COLOR_RED);
        DEBUG_SERIAL.print(msg);
    }
}

inline void errorln(const String& msg) {
    if constexpr (DEBUG_LEVEL >= LOG_ERROR) {
        error(msg);
        DEBUG_SERIAL.println();
    }
}

inline void warn(const String& msg) {
    if constexpr (DEBUG_LEVEL >= LOG_WARN) {
        logPrefix("[WARN] ", COLOR_YELLOW);
        DEBUG_SERIAL.print(msg);
    }
}

inline void warnln(const String& msg) {
    if constexpr (DEBUG_LEVEL >= LOG_WARN) {
        warn(msg);
        DEBUG_SERIAL.println();
    }
}

inline void info(const String& msg) {
    if constexpr (DEBUG_LEVEL >= LOG_INFO) {
        logPrefix("[INFO] ", COLOR_GREEN);
        DEBUG_SERIAL.print(msg);
    }
}

inline void infoln(const String& msg) {
    if constexpr (DEBUG_LEVEL >= LOG_INFO) {
        info(msg);
        DEBUG_SERIAL.println();
    }
}

inline void verbose(const String& msg) {
    if constexpr (DEBUG_LEVEL >= LOG_VERBOSE) {
        logPrefix("[VERBO] ");
        DEBUG_SERIAL.print(msg);
    }
}

inline void verboseln(const String& msg) {
    if constexpr (DEBUG_LEVEL >= LOG_VERBOSE) {
        verbose(msg);
        DEBUG_SERIAL.println();
    }
}

//  Byte/Hex utils 

inline void printHex(uint8_t byte) {
    DEBUG_SERIAL.print(byte, HEX);
    DEBUG_SERIAL.print(' ');
}

inline void printHex(const uint8_t* bytes, uint8_t length) {
    for (size_t i = 0; i < length; i++) {
        DEBUG_SERIAL.print(bytes[i], HEX);
        DEBUG_SERIAL.print(' ');
    }
}

} // namespace Debug

#else // DEBUG not defined

namespace Debug {
    inline void init(uint32_t) {}
    inline void print(const String&) {}
    inline void println(const String&) {}

    inline void error(const String&) {}
    inline void errorln(const String&) {}
    inline void warn(const String&) {}
    inline void warnln(const String&) {}
    inline void info(const String&) {}
    inline void infoln(const String&) {}
    inline void verbose(const String&) {}
    inline void verboseln(const String&) {}

    inline void printHex(uint8_t) {}
    inline void printHex(const uint8_t*, uint8_t) {}
}

#endif // DEBUG

#endif // DEBUG_UTILS_H

// Included files
#include "utils/debug.h"
#include "config/project_config.h"

#define DEBUG_SERIAL Serial

void Debug::init(uint32_t baudRate) {
    if constexpr (DEBUG_MODE) {
        DEBUG_SERIAL.begin(baudRate);
    }
}

void Debug::print(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            DEBUG_SERIAL.print(msg);
        }
    }
}

void Debug::info(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            if constexpr (USE_COLOR) DEBUG_SERIAL.print(F("\033[32m")); // Green
            DEBUG_SERIAL.print(F("[INFO] "));
            if constexpr (USE_COLOR) DEBUG_SERIAL.print(F("\033[0m"));  // Reset
            if constexpr (USE_TIMESTAMPS) {
                DEBUG_SERIAL.print('(');
                DEBUG_SERIAL.print(millis());
                DEBUG_SERIAL.print(F("ms) "));
            }
            DEBUG_SERIAL.print(msg);
        }
    }
}

void Debug::infoln(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        info(msg, localDebug);
        if (localDebug) DEBUG_SERIAL.println();
    }
}

void Debug::warn(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            if constexpr (USE_COLOR) DEBUG_SERIAL.print(F("\033[33m")); // Yellow
            DEBUG_SERIAL.print(F("[WARN] "));
            if constexpr (USE_COLOR) DEBUG_SERIAL.print(F("\033[0m"));  // Reset
            if constexpr (USE_TIMESTAMPS) {
                DEBUG_SERIAL.print('(');
                DEBUG_SERIAL.print(millis());
                DEBUG_SERIAL.print(F("ms) "));
            }
            DEBUG_SERIAL.print(msg);
        }
    }
}

void Debug::warnln(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        warn(msg, localDebug);
        if (localDebug) DEBUG_SERIAL.println();
    }
}

void Debug::error(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            if constexpr (USE_COLOR) DEBUG_SERIAL.print(F("\033[31m")); // Red
            DEBUG_SERIAL.print(F("[ERROR] "));
            if constexpr (USE_COLOR) DEBUG_SERIAL.print(F("\033[0m"));  // Reset
            if constexpr (USE_TIMESTAMPS) {
                DEBUG_SERIAL.print('(');
                DEBUG_SERIAL.print(millis());
                DEBUG_SERIAL.print(F("ms) "));
            }
            DEBUG_SERIAL.print(msg);
        }
    }
}

void Debug::errorln(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        error(msg, localDebug);
        if (localDebug) DEBUG_SERIAL.println();
    }
}

void Debug::printHex(uint8_t byte, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            DEBUG_SERIAL.print(byte, HEX);
            DEBUG_SERIAL.print(' ');
        }
    }
}

void Debug::printHex(const uint8_t* bytes, uint8_t length, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            for (size_t i = 0; i < length; i++) {
                DEBUG_SERIAL.print(bytes[i], HEX);
                DEBUG_SERIAL.print(' ');
            }
        }
    }
}


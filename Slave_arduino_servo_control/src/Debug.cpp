#include "Debug.h"

void Debug::init(uint32_t baudRate) {
    if constexpr (DEBUG_MODE) {
        Serial.begin(baudRate);
    }
}

void Debug::print(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            Serial.print(msg);
        }
    }
}

void Debug::info(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            if constexpr (USE_COLOR) Serial.print(F("\033[32m")); // Green
            Serial.print(F("[INFO] "));
            if constexpr (USE_COLOR) Serial.print(F("\033[0m"));  // Reset
            if constexpr (USE_TIMESTAMPS) {
                Serial.print('(');
                Serial.print(millis());
                Serial.print(F("ms) "));
            }
            Serial.print(msg);
        }
    }
}

void Debug::infoln(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        info(msg, localDebug);
        if (localDebug) Serial.println();
    }
}

void Debug::warn(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            if constexpr (USE_COLOR) Serial.print(F("\033[33m")); // Yellow
            Serial.print(F("[WARN] "));
            if constexpr (USE_COLOR) Serial.print(F("\033[0m"));  // Reset
            if constexpr (USE_TIMESTAMPS) {
                Serial.print('(');
                Serial.print(millis());
                Serial.print(F("ms) "));
            }
            Serial.print(msg);
        }
    }
}

void Debug::warnln(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        warn(msg, localDebug);
        if (localDebug) Serial.println();
    }
}

void Debug::error(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            if constexpr (USE_COLOR) Serial.print(F("\033[31m")); // Red
            Serial.print(F("[ERROR] "));
            if constexpr (USE_COLOR) Serial.print(F("\033[0m"));  // Reset
            if constexpr (USE_TIMESTAMPS) {
                Serial.print('(');
                Serial.print(millis());
                Serial.print(F("ms) "));
            }
            Serial.print(msg);
        }
    }
}

void Debug::errorln(const String& msg, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        error(msg, localDebug);
        if (localDebug) Serial.println();
    }
}

void Debug::printHex(uint8_t byte, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            Serial.print(byte, HEX);
            Serial.print(' ');
        }
    }
}

void Debug::printHex(const uint8_t* bytes, uint8_t length, bool localDebug) {
    if constexpr (DEBUG_MODE) {
        if (localDebug) {
            for (size_t i = 0; i < length; i++) {
                Serial.print(bytes[i], HEX);
                Serial.print(' ');
            }
        }
    }
}


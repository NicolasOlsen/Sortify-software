#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Debug utility class for structured logging.
 * Supports logging levels (INFO, WARN, ERROR) with optional line endings and optional ANSI terminal colors.
 */
class Debug {
public:
    /**
     * @brief Global debug enable flag.
     * If set to false, all Debug calls are stripped at compile time.
     */
    static constexpr bool DEBUG_MODE = true;

    /**
     * @brief Enable ANSI color-coded log output (for terminals that support it).
     * If set to false, logs will be printed without color.
     */
    static constexpr bool USE_COLOR = true;

    /**
     * @brief Enable millisecond timestamps in log output.
     * Prepends time since boot in ms like "(12345ms)"
     */
    static constexpr bool USE_TIMESTAMPS = true;

    /**
     * @brief Initialize the debug system
     * @param baudRate The baud rate for Serial communication
     */
    static void init(uint32_t baudRate);

    /**
     * @brief Print a simple message, without level, timestamp or colour
     * @param msg The message to print
     * @param localDebug Optional local toggle, default is true
     */
    static void print(const String& msg, bool localDebug = true);

    /**
     * @brief Print an info message (no newline)
     * @param msg The message to print
     * @param localDebug Optional local toggle, default is true
     */
    static void info(const String& msg, bool localDebug = true);

    /**
     * @brief Print an info message with newline
     * @param msg The message to print
     * @param localDebug Optional local toggle, default is true
     */
    static void infoln(const String& msg, bool localDebug = true);

    /**
     * @brief Print a warning message (no newline)
     * @param msg The message to print
     * @param localDebug Optional local toggle, default is true
     */
    static void warn(const String& msg, bool localDebug = true);

    /**
     * @brief Print a warning message with newline
     * @param msg The message to print
     * @param localDebug Optional local toggle, default is true
     */
    static void warnln(const String& msg, bool localDebug = true);

    /**
     * @brief Print an error message (no newline)
     * @param msg The message to print
     * @param localDebug Optional local toggle, default is true
     */
    static void error(const String& msg, bool localDebug = true);

    /**
     * @brief Print an error message with newline
     * @param msg The message to print
     * @param localDebug Optional local toggle, default is true
     */
    static void errorln(const String& msg, bool localDebug = true);

    /**
     * @brief Print a byte as hexadecimal (no prefix)
     * @param byte The byte to print
     * @param localDebug Optional local toggle, default is true
     */
    static void printHex(uint8_t byte, bool localDebug = true);

    /**
     * @brief Print a byte array as hexadecimal (no prefix or spacing)
     * @param bytes Pointer to the byte array
     * @param length Number of bytes to print
     * @param localDebug Optional local toggle, default is true
     */
    static void printHex(const uint8_t* bytes, uint8_t length, bool localDebug = true);

};

#endif // DEBUG_H

#ifndef ANALOG_SERVO_CLASS_H
#define ANALOG_SERVO_CLASS_H

#include <stdint.h>

/**
 * @brief Represents a single analog servo controlled via PWM through the Adafruit PCA9685 servo driver.
 *        Requires the servo driver to be initialized once using initAnalogServoDriver().
 */
class AnalogServo
{
public:
    /**
     * @brief Construct a new AnalogServo object
     * 
     * @param channel PWM channel (0–15) used by the PCA9685
     * @param minPWM Minimum PWM pulse
     * @param maxPWM Maximum PWM pulse
     * @param maxDegree Physical max range of the servo (default 180°)
     * @param minAllowedDegree Optional minimum allowed position for this servo (default 0°)
     * @param maxAllowedDegree Optional maximum allowed position (default = maxDegree)
     */
    AnalogServo(uint8_t channel, uint16_t minPWM, uint16_t maxPWM,
                float maxDegree = 180.0f,
                float minAllowedDegree = 0.0f,
                float maxAllowedDegree = -1.0f);  // -1 means "use maxDegree"

    AnalogServo();

    /**
     * @brief Initializes the shared Adafruit PCA9685 PWM driver.
     *        Must be called once in setup() before using any AnalogServo objects.
     * 
     * @return True if initialization was successful
     */
    static bool initAnalogServoDriver();

    /**
     * @brief Sets the servo to the given angle (clamps to allowed limits).
     * 
     * @param position Angle in degrees
     * 
     * @return True if successful (driver acknowledged command)
     */
    bool setToPosition(float position);

    /**
     * @brief Attempts to set the position only if it's within allowed limits.
     * 
     * @param position Angle in degrees
     * 
     * @return True if within limits and command sent, false if out of range (nothing sent)
     */
    bool trySetToPosition(float position);

    /**
     * @brief Checks whether the given position is within the allowed range.
     * 
     * @param position Angle in degrees
     * 
     * @return True if within [_minAllowedDegree, _maxAllowedDegree]
     */
    bool checkPositionInAllowedRange(float position) const;

    /**
     * @brief Returns the PWM channel used for this servo.
     */
    uint8_t getChannel() const { return _channel; }

private:
    uint8_t _channel;
    uint16_t _minPWM;
    uint16_t _maxPWM;
    float _maxDegree;

    float _minAllowedDegree;
    float _maxAllowedDegree;

    uint16_t degreesToPwm(float degrees) const;
};

#endif // ANALOG_SERVO_CLASS_H

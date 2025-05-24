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
     * @param channel PWM channel (0–15 for one) used by the PCA9685
     * @param minPWM Minimum PWM pulse
     * @param maxPWM Maximum PWM pulse
     * @param maxDegree Physical max range of the servo
     * @param minAllowedDegree Minimum allowed position for this servo (for clamping and position limit validation)
     * @param maxAllowedDegree Maximum allowed position (for clamping and position limit validation)
     */
    AnalogServo(uint8_t channel, uint16_t minPWM, uint16_t maxPWM,
                float maxDegree = 180.0f,
                float minAllowedDegree = 0.0f,
                float maxAllowedDegree = 180.0f);

    /**
     * @brief A simple constructor only used to initaliy construct a AnalogServo object
     *          for the ServoManager. Should not be used otherwise.
     */
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
     * @return True if successful
     */
    bool setToPosition(float position);

    /**
     * @brief Checks whether the given position is within the allowed range.
     * 
     * @param position Angle in degrees
     * 
     * @return True if angle is within the allowed range
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

    /**
     * @brief Calculates the necessary PWM value for the given degree.
     * 
     * @param degrees Degree turn to PWM value 
     * 
     * @return The calculated PWM value in uint16_t
     */
    uint16_t degreesToPwm(float degrees) const;
};

/*
Note:
For future expansion, remember that multiple Adafruit PCA9685 servo drivers can be connected,
assuming they have different IDs. Choosen by the solder points
*/

#endif // ANALOG_SERVO_CLASS_H

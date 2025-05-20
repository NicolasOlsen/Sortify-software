#include "utils/math_utils.h"

namespace Utils
{
    float clamp(float value, float min, float max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
} // namespace Utils


#ifndef STATUS_H
#define STATUS_H

#include <Arduino.h>
#include <stdint.h>

#include "Communication_code.h"

using namespace Com_code;

namespace System_status
{
    extern StatusCode currentStatus;

    extern float setPositions[5];

    extern float currentPositions[5];
}



#endif // STATUS_H
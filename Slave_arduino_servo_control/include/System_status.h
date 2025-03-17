#ifndef STATUS_H
#define STATUS_H

#include <Arduino.h>
#include <stdint.h>

#include "UART_communication.h"

using namespace UART_communication;

namespace System_status
{
    extern StatusCode currentStatus;
}



#endif // STATUS_H
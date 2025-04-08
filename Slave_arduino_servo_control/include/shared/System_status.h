#ifndef STATUS_H
#define STATUS_H

#include <Arduino.h>
#include <stdint.h>

#include "comms/Communication_code.h"

using namespace Com_code;

namespace System_status
{
    /**
     * @brief Initiates the systemstatus mutex
     */
    void InitSystemStatusMutex();
    
    /**
     * @brief Safely gets the system state
     */
    StatusCode GetSystemState();

    /**
     * @brief Safely sets the system state
     */
    void SetSystemState(StatusCode state);
}



#endif // STATUS_H
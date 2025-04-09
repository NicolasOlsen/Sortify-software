#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

#include "comms/communication_code.h"

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
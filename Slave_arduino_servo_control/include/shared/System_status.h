#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

#include "comms/communication_code.h"
#include "shared_variable.h"

using namespace Com_code;

namespace System_status
{
    extern SharedVariable<StatusCode> systemState;
}



#endif // STATUS_H
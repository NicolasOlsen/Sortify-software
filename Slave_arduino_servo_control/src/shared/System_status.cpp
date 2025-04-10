#include "shared/System_status.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

using namespace Com_code;


namespace System_status
{
    SharedVariable<StatusCode> systemState(StatusCode::INITIALIZING);
}
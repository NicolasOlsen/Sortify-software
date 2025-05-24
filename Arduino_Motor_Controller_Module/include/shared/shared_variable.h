#ifndef SHARED_VARIABLE_H
#define SHARED_VARIABLE_H

#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include "utils/scoped_lock.h"

template <typename T>
class SharedVariable {
private:
    T var_m;
    SemaphoreHandle_t mutex;

public:
    SharedVariable(T var) : var_m{ var } {
        mutex = xSemaphoreCreateMutex();
    }

    ~SharedVariable() {
        if (mutex != nullptr) {
            vSemaphoreDelete(mutex);
        }
    }

    T Get() {
        T temp{};
        ScopedLock lock(mutex);
        if (lock.isLocked()) {
            temp = var_m;
        }
        return temp;
    }

    void Set(T var) {
        ScopedLock lock(mutex);
        if (lock.isLocked()) {
            var_m = var;
        }
    }
};

#endif  // SHARED_VARIABLE_H

#ifndef SCOPED_LOCK_H
#define SCOPED_LOCK_H

#include <Arduino_FreeRTOS.h>
#include <semphr.h>

/**
 * @brief RAII-style wrapper for FreeRTOS mutexes.
 * 
 * This class automatically takes a FreeRTOS mutex when constructed
 * and releases it when destroyed. It ensures that the mutex is
 * always released, even if a function returns early or throws.
 * 
 * Usage:
 * @code
 * ScopedLock lock(myMutex);
 * if (lock.isLocked()) {
 *     // Safe access to shared resources
 * }
 * @endcode
 */
class ScopedLock {
public:
    /**
     * @brief Constructs the lock and attempts to acquire the mutex.
     * 
     * @param mutex The FreeRTOS mutex to acquire. Can be NULL.
     */
    explicit ScopedLock(SemaphoreHandle_t mutex)
        : _mutex(mutex), _locked(false) {
        if (_mutex) {
            _locked = (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE);
        }
    }

    /**
     * @brief Destructor releases the mutex if it was acquired.
     */
    ~ScopedLock() {
        if (_mutex && _locked) {
            xSemaphoreGive(_mutex);
        }
    }

    /**
     * @brief Indicates whether the mutex was successfully acquired.
     * 
     * @return True if the lock was acquired and is currently held.
     * @return False if the mutex was null or acquisition failed.
     */
    bool isLocked() const { return _locked; }

private:
    SemaphoreHandle_t _mutex; // The mutex being managed
    bool _locked;             // True if the mutex was successfully taken
};

#endif // SCOPEDLOCK_H

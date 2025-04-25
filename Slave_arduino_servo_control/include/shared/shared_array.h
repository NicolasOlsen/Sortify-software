#ifndef SHARED_ARRAY_H
#define SHARED_ARRAY_H

#include <stdint.h>
#include <DynamixelShield.h>
#include <utils/scoped_lock.h>

#include "utils/debug_utils.h"

/**
 * @brief Shared container for thread-safe access to servo data.
 * 
 * @tparam T The type of data to store (float, enum, etc.)
 * @tparam size The number of elements in the array.
 */
template <typename T, uint8_t size>
class SharedArray
{
private:
    T arr_m[size] = {};
    SemaphoreHandle_t mutex;

public:
    /**
     * @brief Initializes the shared data container and creates its mutex.
     */
    SharedArray();

    /**
     * @brief Retrieves the data for a given servo with mutex protection.
     * 
     * @param id        The ID of the servo. If the ID is invalid (out of range), the returned value is default-initialized (e.g., 0).
     * 
     * @return The corresponding data value, or default-constructed T if the ID is invalid.
     */
    T Get(uint8_t id);

    /**
     * @brief Copies a segment of the stored servo data into the provided array with mutex protection.
     * 
     * @param arr         Pointer to the output array that will be filled with servo data.
     * @param size_       Number of elements to copy.
     * @param start_index Starting index in the internal data array (default: 0).
     */
    void Get(T* arr, uint8_t size_ = size, uint8_t start_index = 0);

    /**
     * @brief Sets the data for a given servo with mutex protection.
     * 
     * @param id         The ID of the servo. If the ID is invalid (out of range), the operation is ignored.
     * @param data       The value to set.
     */
    void Set(uint8_t id, T data);

    /**
     * @brief Sets a segment of the servo data array with mutex protection.
     * 
     * @param arr         Pointer to the input array containing the data to set.
     * @param size_       Number of elements to write.
     * @param start_index Starting index in the internal array to begin writing (default: 0).
     */
    void Set(const T* arr, uint8_t size_ = size, uint8_t start_index = 0);
};


template <typename T, uint8_t size>
SharedArray<T, size>::SharedArray() {
    mutex = xSemaphoreCreateMutex();
}


template <typename T, uint8_t size>
T SharedArray<T, size>::Get(uint8_t id) {
    T data = {};     // Safe default
    if (id >= size) return data;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        data = arr_m[id];

        Debug::infoln(String(id) + " got data " + String(data));
    }

    return data;
}

template <typename T, uint8_t size>
void SharedArray<T, size>::Get(T* arr, uint8_t size_, uint8_t start_index) {
    if (start_index + size_ > size) {
        Debug::errorln("Tried to go out of range in get sharedData");
        return;
    }
    
    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size_; i++) {
            arr[i] = arr_m[start_index + i];
        
            Debug::infoln(String(start_index + i) + " got data " + String(arr[i]));
        }
    }
}

template <typename T, uint8_t size>
void SharedArray<T, size>::Set(uint8_t id, T data) {
    if (id >= size) return;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        arr_m[id] = data;
        Debug::infoln(String(id) + " set data " + String(data));
    }
}

template <typename T, uint8_t size>
void SharedArray<T, size>::Set(const T* arr, uint8_t size_, uint8_t start_index) {
    if (start_index + size_ > size) {
        Debug::errorln("Tried to go out of range in set sharedData");
        return;
    }
    
    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size_; ++i) {
            arr_m[start_index + i] = arr[i];
            Debug::infoln(String(start_index + i) + " set data " + String(arr[i]));
        }        
    }
}

#endif

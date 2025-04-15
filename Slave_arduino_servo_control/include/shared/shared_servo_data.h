#ifndef SHARED_SERVO_DATA_H
#define SHARED_SERVO_DATA_H

#include <stdint.h>
#include <DynamixelShield.h>
#include <utils/scoped_lock.h>

#include "utils/Debug.h"

/**
 * @brief Shared container for thread-safe access to servo data.
 * 
 * @tparam T The type of data to store (float, enum, etc.)
 * @tparam size The number of elements in the array.
 */
template <typename T, uint8_t size>
class SharedServoData
{
private:
    T arr_m[size] = {};
    bool flags_m[size] = {false};
    SemaphoreHandle_t mutex;

public:
    /**
     * @brief Initializes the shared data container and creates its mutex.
     */
    SharedServoData();

    /**
     * @brief Retrieves the data for a given servo with mutex protection.
     * 
     * @param id        The ID of the servo. If the ID is invalid (out of range), the returned value is default-initialized (e.g., 0).
     * @param changeFlag If true, the updated flag will be cleared after the value is read.
     * @return The corresponding data value, or default-constructed T if the ID is invalid.
     */
    T Get(uint8_t id, bool changeFlag = false);

    /**
     * @brief Copies the current servo data into the provided array with mutex protection.
     * 
     * @param arr        The output array that will be filled with the current servo data.
     * @param changeFlag If true, the updated flags will be cleared after reading.
     */
    void Get(T* arr, uint8_t size_, uint8_t start_index = 0, bool changeFlag = false);


    /**
     * @brief Sets the data for a given servo with mutex protection.
     * 
     * @param id         The ID of the servo. If the ID is invalid (out of range), the operation is ignored.
     * @param data       The value to set.
     * @param changeFlag If true, the updated flag will be marked as true after setting.
     */
    void Set(uint8_t id, T data, bool changeFlag = true);

    /**
     * @brief Sets the data for all the servos with mutex protection.
     * 
     * @param arr        The array off the servo data to set, in order
     * @param changeFlag If true, the updated flag will be marked as true after setting.
     */
    void Set(const T* arr, uint8_t size_, uint8_t start_index = 0, bool changeFlag = true);

    /**
     * @brief Gets the updated flag for a given servo with mutex protection.
     * 
     * @param id The ID of the servo. If the ID is invalid (out of range), returns false.
     * @return True if the flag is set, false otherwise.
     */
    bool GetFlag(uint8_t id) const;

    /**
     * @brief Sets the updated flag for a given servo with mutex protection.
     * 
     * @param id   The ID of the servo. If the ID is invalid (out of range), the operation is ignored.
     * @param flag The flag value to set.
     */
    void SetFlag(uint8_t id, bool flag);

    /**
     * @brief Sets the all the flags the servos, with mutex protection.
     *
     * @param flag The flag value to set.
     */
    void SetAllFlags(bool flag);
};


template <typename T, uint8_t size>
SharedServoData<T, size>::SharedServoData() {
    mutex = xSemaphoreCreateMutex();
}


template <typename T, uint8_t size>
T SharedServoData<T, size>::Get(uint8_t id, bool changeFlag) {
    T data = {};     // Safe default
    if (id >= size) return data;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        data = arr_m[id];

        if (changeFlag) {           // Optional to change flag
            flags_m[id] = false;
        }

        Debug::infoln(String(id) + " got data " + String(data));
    }

    return data;
}

template <typename T, uint8_t size>
void SharedServoData<T, size>::Get(T* arr, uint8_t size_, uint8_t start_index, bool changeFlag) {
    if (start_index + size_ > size) {
        Debug::errorln("Tried to go out of range in get sharedData");
    }
    
    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size_; i++) {
            arr[i] = arr_m[start_index + i];
        
            Debug::infoln(String(start_index + i) + " got data " + String(arr[i]));
        
            if (changeFlag) {
                flags_m[start_index + i] = false;
            }
        }
    }
}

template <typename T, uint8_t size>
void SharedServoData<T, size>::Set(uint8_t id, T data, bool changeFlag) {
    if (id >= size) return;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        arr_m[id] = data;
        if (changeFlag) {
            flags_m[id] = true;
        }
        Debug::infoln(String(id) + " set data " + String(data));
    }
}

template <typename T, uint8_t size>
void SharedServoData<T, size>::Set(const T* arr, uint8_t size_, uint8_t start_index, bool changeFlag) {
    if (start_index + size_ > size) {
        Debug::errorln("Tried to go out of range in set sharedData");
        return;
    }
    
    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size_; ++i) {
            arr_m[start_index + i] = arr[i];
        
            if (changeFlag) {
                flags_m[start_index + i] = true;
            }
            Debug::infoln(String(start_index + i) + " set data " + String(arr[i]));
        }        
    }
}

template <typename T, uint8_t size>
bool SharedServoData<T, size>::GetFlag(uint8_t id) const{
    bool flag = false;
    if (id >= size) return flag;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        flag = flags_m[id];
        Debug::infoln(String(id) + " get flag " + String(flag));
    }

    return flag;
}

template <typename T, uint8_t size>
void SharedServoData<T, size>::SetFlag(uint8_t id, bool flag) {
    if (id >= size) return;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        flags_m[id] = flag;
        Debug::infoln(String(id) + " set flag " + String(flag));
    }
}

template <typename T, uint8_t size>
void SharedServoData<T, size>::SetAllFlags(bool flag) {
    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size; ++i) {
            flags_m[i] = flag;
        }
        Debug::infoln("All flags set to " + String(flag)); 
    }
}

#endif

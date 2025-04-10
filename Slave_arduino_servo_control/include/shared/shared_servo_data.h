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
    T arr_m[size] = {0};
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
     * @param id        The ID of the servo (1-based). If the ID is invalid (0 or out of range), the returned value is default-initialized (e.g., 0).
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
    void Get(T (&arr)[size], bool changeFlag = false);


    /**
     * @brief Sets the data for a given servo with mutex protection.
     * 
     * @param id         The ID of the servo (1-based). If the ID is invalid (0 or out of range), the operation is ignored.
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
    void Set(const T (&arr)[size], bool changeFlag = true);

    /**
     * @brief Gets the updated flag for a given servo with mutex protection.
     * 
     * @param id The ID of the servo (1-based). If the ID is invalid (0 or out of range), returns false.
     * @return True if the flag is set, false otherwise.
     */
    bool GetFlag(uint8_t id) const;

    /**
     * @brief Sets the updated flag for a given servo with mutex protection.
     * 
     * @param id   The ID of the servo (1-based). If the ID is invalid (0 or out of range), the operation is ignored.
     * @param flag The flag value to set.
     */
    void SetFlag(uint8_t id, bool flag);

private:
    /**
     * @brief Converts a 1-based servo ID into a 0-based array index.
     * 
     * @param id The servo ID (1-based).
     * @return The corresponding array index, or -1 if the ID is invalid.
     */
    int indexFromId(uint8_t id) const;
};


template <typename T, uint8_t size>
SharedServoData<T, size>::SharedServoData() {
    mutex = xSemaphoreCreateMutex();
}


template <typename T, uint8_t size>
int SharedServoData<T, size>::indexFromId(uint8_t id) const {
    if (id > size) return -1; // Invalid
    return id - 1;
}


template <typename T, uint8_t size>
T SharedServoData<T, size>::Get(uint8_t id, bool changeFlag) {
    T data = 0;     // Safe default
    int index = indexFromId(id);        // Get proper index, since ids is from 1
    if (index < 0) return data;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        data = arr_m[index];

        if (changeFlag) {           // Optional to change flag
            flags_m[index] = false;
        }

        Debug::infoln(String(id) + " got data " + String(data));
    }

    return data;
}

template <typename T, uint8_t size>
void SharedServoData<T, size>::Get(T (&arr)[size], bool changeFlag) {
    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size; i++) {
            arr[i] = arr_m[i];

            Debug::infoln(String(i + 1) + " got data " + String(arr[i]));

            if (changeFlag) {           // Optional to change flag
                flags_m[i] = false;
            }
        }
    }
}

template <typename T, uint8_t size>
void SharedServoData<T, size>::Set(uint8_t id, T data, bool changeFlag) {
    int index = indexFromId(id);
    if (index < 0) return;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        arr_m[index] = data;
        flags_m[index] = true; // optional: mark it as updated
        Debug::infoln(String(id) + " set data " + String(data));
    }
}

template <typename T, uint8_t size>
void SharedServoData<T, size>::Set(const T (&arr)[size], bool changeFlag) {
    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size; ++i) {
            arr_m[i] = arr[i];
            if (changeFlag) {
                flags_m[i] = true;
            }
            Debug::infoln(String(i + 1) + " set data " + String(arr[i]));
        }
    }
}

template <typename T, uint8_t size>
bool SharedServoData<T, size>::GetFlag(uint8_t id) const{
    int index = indexFromId(id);
    bool flag = false;
    if (index < 0) return flag;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        flag = flags_m[index];
        Debug::infoln(String(id) + " get flag " + String(flag));
    }

    return flag;
}

template <typename T, uint8_t size>
void SharedServoData<T, size>::SetFlag(uint8_t id, bool flag) {
    int index = indexFromId(id);
    if (index < 0) return;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        flags_m[index] = flag;
        Debug::infoln(String(id) + " set flag " + String(flag));
    }
}

#endif

#ifndef SHARED_ARRAY_WITH_FLAGS_H
#define SHARED_ARRAY_WITH_FLAGS_H

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
class SharedArrayWithFlags
{
private:
    T arr_m[size] = {};
    bool flags_m[size] = {false};
    SemaphoreHandle_t mutex;

public:
    /**
     * @brief Initializes the shared data container and creates its mutex.
     */
    SharedArrayWithFlags();

    /**
     * @brief Retrieves the data for a given servo with mutex protection.
     * 
     * @param id        The ID of the servo. If the ID is invalid (out of range), the returned value is default-initialized (e.g., 0).
     * @param changeFlag If true, the updated flag will be cleared after the value is read.
     * @return The corresponding data value, or default-constructed T if the ID is invalid.
     */
    T Get(uint8_t id, bool changeFlag = false);

    /**
     * @brief Copies a segment of the stored servo data into the provided array with mutex protection.
     * 
     * @param arr         Pointer to the output array that will be filled with servo data.
     * @param size_       Number of elements to copy.
     * @param start_index Starting index in the internal data array (default: 0).
     * @param changeFlag  If true, the update flags for the copied elements will be cleared.
     */
    void Get(T* arr, uint8_t size_ = size, uint8_t start_index = 0, bool changeFlag = false);

    /**
     * @brief Sets the data for a given servo with mutex protection.
     * 
     * @param id         The ID of the servo. If the ID is invalid (out of range), the operation is ignored.
     * @param data       The value to set.
     * @param changeFlag If true, the updated flag will be marked as true after setting.
     */
    void Set(uint8_t id, T data, bool changeFlag = true);

    /**
     * @brief Sets a segment of the servo data array with mutex protection.
     * 
     * @param arr         Pointer to the input array containing the data to set.
     * @param size_       Number of elements to write.
     * @param start_index Starting index in the internal array to begin writing (default: 0).
     * @param changeFlag  If true, the updated flags for the affected entries will be set.
     */
    void Set(const T* arr, uint8_t size_ = size, uint8_t start_index = 0, bool changeFlag = true);

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
     * @brief Gets a range of update flags with mutex protection.
     * 
     * @param out         The output array to fill with flags.
     * @param size_       Number of flags to retrieve.
     * @param start_index Index to start reading from.
     */
    void GetFlags(bool* out, uint8_t size_ = size, uint8_t start_index = 0) const;

    /**
     * @brief Sets a range of update flags for the servo data with mutex protection.
     * 
     * @param flags        Pointer to an array of boolean flag values to apply.
     * @param size_        Number of flag entries to write.
     * @param start_index  Starting index in the internal flag array (default: 0).
     */
    void SetFlags(const bool* flags, uint8_t size_ = size, uint8_t start_index = 0);

    /**
     * @brief Sets the all the flags the servos, with mutex protection.
     *
     * @param flag The flag value to set.
     */
    void SetAllFlags(bool flag);
};


template <typename T, uint8_t size>
SharedArrayWithFlags<T, size>::SharedArrayWithFlags() {
    mutex = xSemaphoreCreateMutex();
}


template <typename T, uint8_t size>
T SharedArrayWithFlags<T, size>::Get(uint8_t id, bool changeFlag) {
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
void SharedArrayWithFlags<T, size>::Get(T* arr, uint8_t size_, uint8_t start_index, bool changeFlag) {
    if (start_index + size_ > size) {
        Debug::errorln("Tried to go out of range in get sharedData");
        return;
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
void SharedArrayWithFlags<T, size>::Set(uint8_t id, T data, bool changeFlag) {
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
void SharedArrayWithFlags<T, size>::Set(const T* arr, uint8_t size_, uint8_t start_index, bool changeFlag) {
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
bool SharedArrayWithFlags<T, size>::GetFlag(uint8_t id) const{
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
void SharedArrayWithFlags<T, size>::GetFlags(bool* out, uint8_t size_, uint8_t start_index) const {
    if (start_index + size_ > size) {
        Debug::errorln("Tried to go out of range in GetFlags");
        return;
    }

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size_; ++i) {
            out[i] = flags_m[start_index + i];
        }
    }
}

template <typename T, uint8_t size>
void SharedArrayWithFlags<T, size>::SetFlag(uint8_t id, bool flag) {
    if (id >= size) return;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        flags_m[id] = flag;
        Debug::infoln(String(id) + " set flag " + String(flag));
    }
}

template <typename T, uint8_t size>
void SharedArrayWithFlags<T, size>::SetFlags(const bool* flags, uint8_t size_, uint8_t start_index) {
    if (start_index + size_ > size) {
        Debug::errorln("Tried to go out of range in setFlags sharedData");
        return;
    }

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size_; ++i) {
            flags_m[start_index + i] = flags[i];
            Debug::infoln("Set flag at index " + String(start_index + i) + " to " + String(flags[i]));
        }
    }
}

template <typename T, uint8_t size>
void SharedArrayWithFlags<T, size>::SetAllFlags(bool flag) {
    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        for (uint8_t i = 0; i < size; ++i) {
            flags_m[i] = flag;
        }
        Debug::infoln("All flags set to " + String(flag)); 
    }
}

#endif

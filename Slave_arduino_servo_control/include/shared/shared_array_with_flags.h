#ifndef SHARED_ARRAY_WITH_FLAGS_H
#define SHARED_ARRAY_WITH_FLAGS_H

#include <stdint.h>
#include <DynamixelShield.h>
#include <utils/scoped_lock.h>

#include "utils/debug_utils.h"

/**
 * @brief Shared container for thread-safe access to array data.
 * 
 * @tparam T The type of data to store
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
     * @brief Gets data, with mutex protection.
     * 
     * @param index         Index to get from
     * @param changeFlag    If true, the updated flag will be changed to False after the value is read
     * @return              The corresponding data value, or default-constructed T if the ID is invalid
     */
    T Get(uint8_t index, bool changeFlag = false);

    /**
     * @brief Copies a range of data into the provided array, with mutex protection.
     * 
     *        Will return without copying, if the the range is out of bounds.
     * 
     * @param arr           Pointer to the output array that will be filled with data
     * @param size_         Number of elements to copy
     * @param start_index   Starting index in the internal data array
     * @param changeFlag    If true, the update flags for the copied elements will be changed to False
     */
    void Get(T* arr, uint8_t size_ = size, uint8_t start_index = 0, bool changeFlag = false);

    /**
     * @brief Sets data, with mutex protection.
     * 
     * @param index         Index to set a value
     * @param data          The value to set
     * @param changeFlag    If true, the updated flag will be marked as true after setting
     */
    void Set(uint8_t index, T data, bool changeFlag = true);

    /**
     * @brief Sets a range of data into the internal array, with mutex protection.
     * 
     *        Will return without changing values, if the the range is out of bounds.
     * 
     * @param arr           Pointer to the input array containing the data to set
     * @param size_         Number of elements to set for
     * @param start_index   Starting index in the internal array to begin writing
     * @param changeFlag    If true, the updated flags for the affected entries will be set
     */
    void Set(const T* arr, uint8_t size_ = size, uint8_t start_index = 0, bool changeFlag = true);

    /**
     * @brief Gets the flag for a given index, with mutex protection.
     * 
     * @param index     The index of the flag. Return False if the index is out of range
     * @return          The flag value for the given index
     */
    bool GetFlag(uint8_t index) const;

    /**
     * @brief Sets the flag for a index, with mutex protection.
     * 
     * @param index     The index of the flag. Does nothing if the index is out of range
     * @param flag      The flag value to set
     */
    void SetFlag(uint8_t index, bool flag);

    /**
     * @brief Copies a range of flags, with mutex protection.
     * 
     * @param out         The output array to copy with flags
     * @param size_       Number of flags to retrieve
     * @param start_index Index to start reading from
     */
    void GetFlags(bool* out, uint8_t size_ = size, uint8_t start_index = 0) const;

    /**
     * @brief Sets a range values for the internal flags, with mutex protection.
     * 
     * @param flags        Pointer to an array of flag values to apply
     * @param size_        Number of flags to apply
     * @param start_index  Starting index in the internal flag array
     */
    void SetFlags(const bool* flags, uint8_t size_ = size, uint8_t start_index = 0);

    /**
     * @brief       Sets the value for all the flags, with mutex protection.
     *
     * @param flag  The common flag value to set
     */
    void SetAllFlags(bool flag);
};


template <typename T, uint8_t size>
SharedArrayWithFlags<T, size>::SharedArrayWithFlags() {
    mutex = xSemaphoreCreateMutex();
}


template <typename T, uint8_t size>
T SharedArrayWithFlags<T, size>::Get(uint8_t index, bool changeFlag) {
    T data = {};     // Safe default
    if (index >= size) return data;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        data = arr_m[index];

        if (changeFlag) {           // Optional to change flag
            flags_m[index] = false;
        }

        Debug::infoln(String(index) + " got data " + String(data));
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
void SharedArrayWithFlags<T, size>::Set(uint8_t index, T data, bool changeFlag) {
    if (index >= size) return;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        arr_m[index] = data;
        if (changeFlag) {
            flags_m[index] = true;
        }
        Debug::infoln(String(index) + " set data " + String(data));
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
bool SharedArrayWithFlags<T, size>::GetFlag(uint8_t index) const{
    bool flag = false;
    if (index >= size) return flag;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        flag = flags_m[index];
        Debug::infoln(String(index) + " get flag " + String(flag));
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
void SharedArrayWithFlags<T, size>::SetFlag(uint8_t index, bool flag) {
    if (index >= size) return;

    ScopedLock lock(mutex);
    if (lock.isLocked()) {
        flags_m[index] = flag;
        Debug::infoln(String(index) + " set flag " + String(flag));
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

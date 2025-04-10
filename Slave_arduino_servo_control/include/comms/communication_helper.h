#ifndef COMMUNICATION_HELPER_H
#define COMMUNICATION_HELPER_H

#include <stdint.h>

#include "shared/shared_servo_data.h"
#include "comms/Communication_code.h"
#include "config/communication_config.h"

namespace Com_helper {

using namespace Com_code;


/**
 * @brief This function converts the bytes stored in a communication packet to the proper type and adds it in a array
 * @tparam T The type to convert the bytes to
 * 
 * @param byteData The array to copy from
 * @param count The size of the output array
 * @param outputArray The array to save the byte data
 */
template<typename T>
void convertBytesToTypedArray(const uint8_t* byteData, size_t count, T* outputArray) {
    for (size_t i = 0; i < count; ++i) {    // Count is the amount of the data type
        memcpy(&outputArray[i], &byteData[i * sizeof(T)], sizeof(T));   // Copies from the bytedata, with the size of the data type at a time
    }
}

/**
 * @brief   Copies a single value from the packet to a sharedServoData object.
 *          It is hardcoded to fit with the current protocol.
 * 
 * @param packet The packet to copy from
 * @param dataStore The sharedServoData object to save to
 */
template<typename T, uint8_t size>
void handleSetSingleValue(const uint8_t* packet, SharedServoData<T, size>& dataStore) {
    uint8_t id = packet[ID_INDEX];  // Id place is hardcoded
    T temp[1];                      // Create a array of one to reuse the function
    convertBytesToTypedArray<T>(&packet[ID_PAYLOAD_INDEX], 1, temp);    // Use the function to add the byte to a type
    dataStore.Set(id, temp[0]);     // Store it dataStore
}

/**
 * @brief   Copies multiple value from the packet to a sharedServoData object.
 *          It is hardcoded to fit with the current protocol.
 * 
 * @param packet The packet to copy from
 * @param dataStore The sharedServoData object to save to
 */
template<typename T, uint8_t size>
void handleSetAllValues(const uint8_t* packet, SharedServoData<T, size>& dataStore) {
    T values[size];     // creates a array that has enough for the size of the internal array of dataStore
    convertBytesToTypedArray<T>(&packet[PAYLOAD_INDEX], size, values);  // Use the function to turn the bytes to the correct types
    dataStore.Set(values);  // Copy the array to dataStore
}


/**
 * @brief Checks if the system state is in fault mode
 */
bool CheckFault();

/**
 * @brief Makes the CRC and adds it to the packet to be sent
 * 
 * @param packet The packet to be send
 * @param size The size of the packet
 */
void makePacketCRC(uint8_t* packet, uint8_t packetSize);

/**
 * @brief Validates the CRC in the recieved packet
 * 
 * @param packet The packet to be validated
 * @param size The size of the packet
 */
bool validatePacketCRC(const uint8_t* packet, uint8_t packetSize);

/**
 * @brief Helper function for other CRC functions, to keep consistent
 * 
 * @param packet The packet to calculate CRC16
 * @param size The size of the packet
 */
uint16_t calculateCRC16(const uint8_t* packet, uint8_t packetSize);

}

#endif
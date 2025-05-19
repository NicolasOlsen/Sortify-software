#ifndef ARRAY_UTILS_H
#define ARRAY_UTILS_H

#include <stdint.h>

/*
These functions were generated with the help of ChatGPT (GPT-4o, OpenAI)
to address array slicing issues encountered during development.

The generation was based on a specific C++ usage context, and the resulting
templates have been reviewed, tested, and verified by the developer to ensure 
correctness and compatibility with the projects requirements.
*/


/**
 * @brief Slices the first N elements from a larger fixed-size array.
 * 
 * This avoids copying, works at compile-time, and preserves type safety.
 * 
 * @tparam N Number of elements to slice out
 * @tparam M Total size of the input array (deduced automatically)
 * 
 * @param arr Input array reference
 * 
 * @return Reference to a fixed-size subarray [0, N)
 */
template <typename T, uint8_t N, uint8_t M>
constexpr T (&sliceFirst(T (&arr)[M]))[N] {
    static_assert(N <= M, "sliceFirst: requested slice exceeds array bounds");
    return *reinterpret_cast<T (*)[N]>(&arr[0]);
}


/**
 * @brief Slices the first N elements from a larger fixed-size const array.
 * 
 * This avoids copying, works at compile-time, and preserves type safety.
 * 
 * @tparam N Number of elements to slice out
 * @tparam M Total size of the input array (deduced automatically)
 * 
 * @param arr Input array reference
 * 
 * @return Reference to a fixed-size subarray [0, N)
 */
template <typename T, uint8_t N, uint8_t M>
constexpr const T (&sliceFirst(const T (&arr)[M]))[N] {
    static_assert(N <= M, "Slice exceeds array bounds");
    return *reinterpret_cast<const T (*)[N]>(&arr[0]);
}

#endif // ARRAY_UTILS_H

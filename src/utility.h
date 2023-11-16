/*! @file settings.h*/
#ifndef SVEA_UTILITY
#define SVEA_UTILITY
#include <Arduino.h>

/*!
 * @brief Take the absolute difference between unsigned integeres a and b.
 *
 * @param a First integer.
 * @param b Second integer.
 */
inline uint32_t abs_difference(uint32_t a, uint32_t b) {
    return a > b ? a - b : b - a;
}

#endif /* SVEA_UTILITY */

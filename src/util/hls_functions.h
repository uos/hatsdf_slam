#pragma once

/**
 * @file hls_functions.h
 * @author Malte Hillmann
 */

// only defined during HLS synthesis
#ifdef __SYNTHESIS__
#include <hls_math.h>
#else
#include <math.h>
#endif

/**
 * @brief Takes the absolute value of x, one that is always positive regardless of the sign of x
 * 
 * @param x The input
 * @return A positive value
 */
template<typename T>
inline T hls_abs(T x)
{
    return x < 0 ? T(-x) : T(x);
}

/**
 * @brief Calculates the square root of x and rounds to nearest integer
 * 
 * @param x The input
 * @return int round(sqrt(x))
 */
inline int hls_sqrt_approx(int x)
{
#ifdef __SYNTHESIS__
    return hls::sqrt(x);
#else
    return std::round(std::sqrt(x));
#endif
}

/**
 * @brief Calculates the square root of x and rounds to nearest integer
 * 
 * @param x The input
 * @return long round(sqrt(x))
 */
inline long hls_sqrt_approx_arith(long x)
{
    return std::round(std::sqrt(x));
}

/**
 * @brief Calculates the square root of x
 * 
 * @param x The input
 * @return float sqrt(x)
 */
inline float hls_sqrt_float(float x)
{
#ifdef __SYNTHESIS__
    return hls::sqrt(x);
#else
    return std::sqrt(x);
#endif
}

/**
 * @brief Calculates both sin and cos of an angle
 * 
 * @param angle The angle in Radians
 * @param sin_out Pointer to the resulting sin value
 * @param cos_out Pointer to the resulting cos value
 */
inline void hls_sincos(float angle, float* sin_out, float* cos_out)
{
#ifdef __SYNTHESIS__
    return hls::sincos(angle, &sin_out, &cos_out);
#else
    sincosf(angle, sin_out, cos_out);
#endif
}

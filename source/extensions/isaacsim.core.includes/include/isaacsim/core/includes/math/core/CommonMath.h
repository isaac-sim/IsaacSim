// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "Core.h"
#include "Types.h"

#include <cassert>
#include <cmath>
#include <float.h>
#include <string.h>

#ifdef __CUDACC__
#    define CUDA_CALLABLE __host__ __device__
#else
#    define CUDA_CALLABLE
#endif

/**
 * @brief Mathematical constant π (pi) as a float
 * @details Represents the ratio of a circle's circumference to its diameter
 */
#define kPi 3.141592653589f

/**
 * @brief Mathematical constant 2π as a float
 * @details Double the value of π, useful for full circle calculations
 */
const float k2Pi = 2.0f * kPi;

/**
 * @brief Inverse of π (1/π) as a float
 * @details Reciprocal of π for efficient division by π operations
 */
const float kInvPi = 1.0f / kPi;

/**
 * @brief Inverse of 2π (1/(2π)) as a float
 * @details Reciprocal of 2π for efficient division by 2π operations
 */
const float kInv2Pi = 0.5f / kPi;

/**
 * @brief Conversion factor from degrees to radians
 * @details Multiply degrees by this constant to convert to radians (π/180)
 */
const float kDegToRad = kPi / 180.0f;

/**
 * @brief Conversion factor from radians to degrees
 * @details Multiply radians by this constant to convert to degrees (180/π)
 */
const float kRadToDeg = 180.0f / kPi;

/**
 * @brief Converts degrees to radians
 * @param[in] t Angle in degrees
 * @return Angle in radians
 */
CUDA_CALLABLE inline float DegToRad(float t)
{
    return t * kDegToRad;
}

/**
 * @brief Converts radians to degrees
 * @param[in] t Angle in radians
 * @return Angle in degrees
 */
CUDA_CALLABLE inline float RadToDeg(float t)
{
    return t * kRadToDeg;
}

/**
 * @brief Computes the sine of an angle
 * @param[in] theta Angle in radians
 * @return Sine of the angle
 */
CUDA_CALLABLE inline float Sin(float theta)
{
    return sinf(theta);
}

/**
 * @brief Computes the cosine of an angle
 * @param[in] theta Angle in radians
 * @return Cosine of the angle
 */
CUDA_CALLABLE inline float Cos(float theta)
{
    return cosf(theta);
}

/**
 * @brief Computes both sine and cosine of an angle simultaneously
 * @param[in] theta Angle in radians
 * @param[out] s Reference to store the sine value
 * @param[out] c Reference to store the cosine value
 * @details More efficient than calling Sin() and Cos() separately for some use cases
 */
CUDA_CALLABLE inline void SinCos(float theta, float& s, float& c)
{
    // no optimizations yet
    s = sinf(theta);
    c = cosf(theta);
}

/**
 * @brief Computes the tangent of an angle
 * @param[in] theta Angle in radians
 * @return Tangent of the angle
 */
CUDA_CALLABLE inline float Tan(float theta)
{
    return tanf(theta);
}

/**
 * @brief Computes the square root of a float value
 * @param[in] x Input value
 * @return Square root of x
 * @pre x >= 0
 */
CUDA_CALLABLE inline float Sqrt(float x)
{
    return sqrtf(x);
}

/**
 * @brief Computes the square root of a double value
 * @param[in] x Input value
 * @return Square root of x
 * @pre x >= 0
 */
CUDA_CALLABLE inline double Sqrt(double x)
{
    return sqrt(x);
}

/**
 * @brief Computes the arc sine (inverse sine) of a value
 * @param[in] theta Input value
 * @return Arc sine in radians
 * @pre -1.0 <= theta <= 1.0
 */
CUDA_CALLABLE inline float ASin(float theta)
{
    return asinf(theta);
}

/**
 * @brief Computes the arc cosine (inverse cosine) of a value
 * @param[in] theta Input value
 * @return Arc cosine in radians
 * @pre -1.0 <= theta <= 1.0
 */
CUDA_CALLABLE inline float ACos(float theta)
{
    return acosf(theta);
}

/**
 * @brief Computes the arc tangent (inverse tangent) of a value
 * @param[in] theta Input value
 * @return Arc tangent in radians
 */
CUDA_CALLABLE inline float ATan(float theta)
{
    return atanf(theta);
}

/**
 * @brief Computes the arc tangent of y/x using the signs to determine quadrant
 * @param[in] x X coordinate
 * @param[in] y Y coordinate
 * @return Arc tangent of y/x in radians, in the range [-π, π]
 * @details Returns the angle from the positive x-axis to the point (x, y)
 */
CUDA_CALLABLE inline float ATan2(float x, float y)
{
    return atan2f(x, y);
}

/**
 * @brief Computes the absolute value of a float
 * @param[in] x Input value
 * @return Absolute value of x
 */
CUDA_CALLABLE inline float Abs(float x)
{
    return fabsf(x);
}

/**
 * @brief Computes b raised to the power e
 * @param[in] b Base value
 * @param[in] e Exponent value
 * @return b^e
 */
CUDA_CALLABLE inline float Pow(float b, float e)
{
    return powf(b, e);
}

/**
 * @brief Computes the sign of a value
 * @param[in] x Input value
 * @return -1.0f if x < 0, 1.0f otherwise
 */
CUDA_CALLABLE inline float Sgn(float x)
{
    return (x < 0.0f ? -1.0f : 1.0f);
}

/**
 * @brief Computes the sign of a float value
 * @param[in] x Input value
 * @return -1.0f if x < 0, 1.0f otherwise
 */
CUDA_CALLABLE inline float Sign(float x)
{
    return x < 0.0f ? -1.0f : 1.0f;
}

/**
 * @brief Computes the sign of a double value
 * @param[in] x Input value
 * @return -1.0f if x < 0, 1.0f otherwise
 */
CUDA_CALLABLE inline double Sign(double x)
{
    return x < 0.0f ? -1.0f : 1.0f;
}

/**
 * @brief Computes the floating-point remainder of x/y
 * @param[in] x Dividend
 * @param[in] y Divisor
 * @return Floating-point remainder of x/y
 */
CUDA_CALLABLE inline float Mod(float x, float y)
{
    return fmod(x, y);
}

/**
 * @brief Returns the minimum of two values
 * @tparam T Type of the values to compare
 * @param[in] a First value
 * @param[in] b Second value
 * @return The smaller of a and b
 */
template <typename T>
CUDA_CALLABLE inline T Min(T a, T b)
{
    return a < b ? a : b;
}

/**
 * @brief Returns the maximum of two values
 * @tparam T Type of the values to compare
 * @param[in] a First value
 * @param[in] b Second value
 * @return The larger of a and b
 */
template <typename T>
CUDA_CALLABLE inline T Max(T a, T b)
{
    return a > b ? a : b;
}

/**
 * @brief Swaps two values
 * @tparam T Type of the values to swap
 * @param[in,out] a First value (will contain b after swap)
 * @param[in,out] b Second value (will contain a after swap)
 */
template <typename T>
CUDA_CALLABLE inline void Swap(T& a, T& b)
{
    T tmp = a;
    a = b;
    b = tmp;
}

/**
 * @brief Clamps a value between a minimum and maximum range
 * @tparam T Type of the values
 * @param[in] a Value to clamp
 * @param[in] low Minimum allowed value
 * @param[in] high Maximum allowed value
 * @return Clamped value in the range [low, high]
 * @details If low > high, the values are swapped before clamping
 */
template <typename T>
CUDA_CALLABLE inline T Clamp(T a, T low, T high)
{
    if (low > high)
        Swap(low, high);

    return Max(low, Min(a, high));
}

/**
 * @brief Performs linear interpolation between two values
 * @tparam V Type of the values to interpolate
 * @tparam T Type of the interpolation parameter
 * @param[in] start Starting value (when t = 0)
 * @param[in] end Ending value (when t = 1)
 * @param[in] t Interpolation parameter
 * @return Interpolated value: start + (end - start) * t
 */
template <typename V, typename T>
CUDA_CALLABLE inline V Lerp(const V& start, const V& end, const T& t)
{
    return start + (end - start) * t;
}

/**
 * @brief Computes the inverse square root of a value
 * @param[in] x Input value
 * @return 1.0f / sqrt(x)
 * @pre x > 0
 */
CUDA_CALLABLE inline float InvSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

/**
 * @brief Rounds a float value to the nearest integer
 * @param[in] f Input float value
 * @return Rounded integer value (rounds towards +infinity)
 */
CUDA_CALLABLE inline int Round(float f)
{
    return int(f + 0.5f);
}

/**
 * @brief Normalizes a vector to unit length
 * @tparam T Vector type that supports division by scalar and Length() function
 * @param[in] v Vector to normalize
 * @return Normalized vector with unit length
 * @pre Length(v) > 0
 */
template <typename T>
CUDA_CALLABLE T Normalize(const T& v)
{
    T a(v);
    a /= Length(v);
    return a;
}

/**
 * @brief Computes the squared length of a vector
 * @tparam T Vector type that supports Dot() function
 * @param[in] v Vector to compute squared length for
 * @return Squared length of the vector
 * @details More efficient than Length() when only comparing lengths
 */
template <typename T>
CUDA_CALLABLE inline typename T::value_type LengthSq(const T v)
{
    return Dot(v, v);
}

/**
 * @brief Computes the length (magnitude) of a vector
 * @tparam T Vector type that supports LengthSq() function
 * @param[in] v Vector to compute length for
 * @return Length of the vector
 */
template <typename T>
CUDA_CALLABLE inline typename T::value_type Length(const T& v)
{
    typename T::value_type lSq = LengthSq(v);
    if (lSq)
        return Sqrt(LengthSq(v));
    else
        return 0.0f;
}

/**
 * @brief Computes the distance between two vectors
 * @tparam T Vector type that supports subtraction and Length() function
 * @param[in] v1 First vector
 * @param[in] v2 Second vector
 * @return Distance between v1 and v2
 * @details Helper function mainly used by script interfaces
 */
template <typename T>
CUDA_CALLABLE inline typename T::value_type Distance(const T& v1, const T& v2)
{
    return Length(v1 - v2);
}

/**
 * @brief Safely normalizes a vector with fallback for zero-length vectors
 * @tparam T Vector type that supports LengthSq(), multiplication, and InvSqrt()
 * @param[in] v Vector to normalize
 * @param[in] fallback Fallback vector to return if v has zero length (default: default-constructed T)
 * @return Normalized vector if length > 0, fallback vector otherwise
 */
template <typename T>
CUDA_CALLABLE inline T SafeNormalize(const T& v, const T& fallback = T())
{
    float l = LengthSq(v);
    if (l > 0.0f)
    {
        return v * InvSqrt(l);
    }
    else
        return fallback;
}

/**
 * @brief Computes the square of a value
 * @tparam T Type of the value
 * @param[in] x Input value
 * @return x²
 */
template <typename T>
CUDA_CALLABLE inline T Sqr(T x)
{
    return x * x;
}

/**
 * @brief Computes the cube of a value
 * @tparam T Type of the value
 * @param[in] x Input value
 * @return x³
 */
template <typename T>
CUDA_CALLABLE inline T Cube(T x)
{
    return x * x * x;
}

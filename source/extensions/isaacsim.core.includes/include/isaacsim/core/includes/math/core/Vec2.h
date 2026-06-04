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

#include "CommonMath.h"

#if defined(_WIN32) && !defined(__CUDACC__)
#    if defined(_DEBUG)

#        define VEC2_VALIDATE()                                                                                        \
            {                                                                                                          \
                assert(_finite(x));                                                                                    \
                assert(!_isnan(x));                                                                                    \
                                                                                                                       \
                assert(_finite(y));                                                                                    \
                assert(!_isnan(y));                                                                                    \
            }
#    else

#        define VEC2_VALIDATE()                                                                                        \
            {                                                                                                          \
                assert(isfinite(x));                                                                                   \
                assert(isfinite(y));                                                                                   \
            }

#    endif // _WIN32

#else
#    define VEC2_VALIDATE()
#endif

/**
 * @brief Validates floating-point values for finiteness and non-NaN status.
 * @param[in] f The floating-point value to validate.
 * @note This macro is only active in debug builds.
 */
#ifdef _DEBUG
#    define FLOAT_VALIDATE(f)                                                                                          \
        {                                                                                                              \
            assert(_finite(f));                                                                                        \
            assert(!_isnan(f));                                                                                        \
        }
#else
#    define FLOAT_VALIDATE(f)
#endif

// vec2
/**
 * @class XVector2
 * @brief A template class representing a 2D vector with templated component type.
 * @tparam T The numeric type of the vector components (typically float or double).
 * @details XVector2 provides a complete set of vector operations including arithmetic,
 *          comparison, and conversion operators. The class includes debug validation
 *          for finite and non-NaN values when appropriate.
 */
template <typename T>
class XVector2
{
public:
    /** @brief Type definition for the component type. */
    typedef T value_type;

    /**
     * @brief Default constructor that initializes both components to zero.
     */
    CUDA_CALLABLE XVector2() : x(0.0f), y(0.0f)
    {
        VEC2_VALIDATE();
    }

    /**
     * @brief Constructor that initializes both components to the same value.
     * @param[in] _x The value to set for both x and y components.
     */
    CUDA_CALLABLE XVector2(T _x) : x(_x), y(_x)
    {
        VEC2_VALIDATE();
    }

    /**
     * @brief Constructor that initializes components from individual values.
     * @param[in] _x The x-component value.
     * @param[in] _y The y-component value.
     */
    CUDA_CALLABLE XVector2(T _x, T _y) : x(_x), y(_y)
    {
        VEC2_VALIDATE();
    }

    /**
     * @brief Constructor that initializes from an array.
     * @param[in] p Pointer to an array of at least 2 elements [x, y].
     * @warning The caller must ensure the array has at least 2 elements.
     */
    CUDA_CALLABLE XVector2(const T* p) : x(p[0]), y(p[1])
    {
    }

    /**
     * @brief Template copy constructor for type conversion.
     * @tparam U The source component type.
     * @param[in] v The vector to convert from.
     * @details Performs explicit type conversion between different numeric types.
     */
    template <typename U>
    CUDA_CALLABLE explicit XVector2(const XVector2<U>& v) : x(v.x), y(v.y)
    {
    }

    /**
     * @brief Conversion operator to a mutable pointer.
     * @return Pointer to the first component for array-style access.
     */
    CUDA_CALLABLE operator T*()
    {
        return &x;
    }

    /**
     * @brief Conversion operator to a const pointer.
     * @return Const pointer to the first component for array-style access.
     */
    CUDA_CALLABLE operator const T*() const
    {
        return &x;
    };

    /**
     * @brief Sets the vector components.
     * @param[in] x_ The new x-component value.
     * @param[in] y_ The new y-component value.
     */
    CUDA_CALLABLE void Set(T x_, T y_)
    {
        VEC2_VALIDATE();
        x = x_;
        y = y_;
    }

    /**
     * @brief Scales the vector by a scalar value.
     * @param[in] scale The scaling factor to apply to both components.
     * @return A new XVector2 representing the scaled vector.
     */
    CUDA_CALLABLE XVector2<T> operator*(T scale) const
    {
        XVector2<T> r(*this);
        r *= scale;
        VEC2_VALIDATE();
        return r;
    }

    /**
     * @brief Divides the vector by a scalar value.
     * @param[in] scale The divisor to apply to both components.
     * @return A new XVector2 representing the divided vector.
     * @warning Division by zero will result in undefined behavior.
     */
    CUDA_CALLABLE XVector2<T> operator/(T scale) const
    {
        XVector2<T> r(*this);
        r /= scale;
        VEC2_VALIDATE();
        return r;
    }

    /**
     * @brief Adds two vectors component-wise.
     * @param[in] v The vector to add.
     * @return A new XVector2 representing the sum.
     */
    CUDA_CALLABLE XVector2<T> operator+(const XVector2<T>& v) const
    {
        XVector2<T> r(*this);
        r += v;
        VEC2_VALIDATE();
        return r;
    }

    /**
     * @brief Subtracts two vectors component-wise.
     * @param[in] v The vector to subtract.
     * @return A new XVector2 representing the difference.
     */
    CUDA_CALLABLE XVector2<T> operator-(const XVector2<T>& v) const
    {
        XVector2<T> r(*this);
        r -= v;
        VEC2_VALIDATE();
        return r;
    }

    /**
     * @brief Scales this vector by a scalar value in-place.
     * @param[in] scale The scaling factor to apply to both components.
     * @return Reference to this vector after scaling.
     */
    CUDA_CALLABLE XVector2<T>& operator*=(T scale)
    {
        x *= scale;
        y *= scale;
        VEC2_VALIDATE();
        return *this;
    }

    /**
     * @brief Divides this vector by a scalar value in-place.
     * @param[in] scale The divisor to apply to both components.
     * @return Reference to this vector after division.
     * @warning Division by zero will result in undefined behavior.
     */
    CUDA_CALLABLE XVector2<T>& operator/=(T scale)
    {
        T s(1.0f / scale);
        x *= s;
        y *= s;
        VEC2_VALIDATE();
        return *this;
    }

    /**
     * @brief Adds another vector to this vector in-place.
     * @param[in] v The vector to add.
     * @return Reference to this vector after addition.
     */
    CUDA_CALLABLE XVector2<T>& operator+=(const XVector2<T>& v)
    {
        x += v.x;
        y += v.y;
        VEC2_VALIDATE();
        return *this;
    }

    /**
     * @brief Subtracts another vector from this vector in-place.
     * @param[in] v The vector to subtract.
     * @return Reference to this vector after subtraction.
     */
    CUDA_CALLABLE XVector2<T>& operator-=(const XVector2<T>& v)
    {
        x -= v.x;
        y -= v.y;
        VEC2_VALIDATE();
        return *this;
    }

    /**
     * @brief Multiplies this vector by another vector component-wise in-place.
     * @param[in] scale The vector to multiply by.
     * @return Reference to this vector after multiplication.
     */
    CUDA_CALLABLE XVector2<T>& operator*=(const XVector2<T>& scale)
    {
        x *= scale.x;
        y *= scale.y;
        VEC2_VALIDATE();
        return *this;
    }

    /**
     * @brief Returns the negation of this vector.
     * @return A new XVector2 with both components negated.
     */
    CUDA_CALLABLE XVector2<T> operator-() const
    {
        VEC2_VALIDATE();
        return XVector2<T>(-x, -y);
    }

    /** @brief The x-component of the vector. */
    T x;
    /** @brief The y-component of the vector. */
    T y;
};

/** @brief Type alias for XVector2<float> representing a 2D float vector. */
typedef XVector2<float> Vec2;
/** @brief Alternative type alias for XVector2<float>. */
typedef XVector2<float> Vector2;

/**
 * @brief Scales a vector by a scalar value (left-hand side scalar).
 * @tparam T The component type of the vector.
 * @param[in] lhs The scalar multiplier.
 * @param[in] rhs The vector to scale.
 * @return A new XVector2 representing the scaled vector.
 */
template <typename T>
CUDA_CALLABLE XVector2<T> operator*(T lhs, const XVector2<T>& rhs)
{
    XVector2<T> r(rhs);
    r *= lhs;
    return r;
}

/**
 * @brief Multiplies two vectors component-wise.
 * @tparam T The component type of the vectors.
 * @param[in] lhs The first vector.
 * @param[in] rhs The second vector.
 * @return A new XVector2 with components as the product of corresponding components.
 */
template <typename T>
CUDA_CALLABLE XVector2<T> operator*(const XVector2<T>& lhs, const XVector2<T>& rhs)
{
    XVector2<T> r(lhs);
    r *= rhs;
    return r;
}

/**
 * @brief Tests for equality between two vectors.
 * @tparam T The component type of the vectors.
 * @param[in] lhs The first vector to compare.
 * @param[in] rhs The second vector to compare.
 * @return True if all corresponding components are equal, false otherwise.
 */
template <typename T>
CUDA_CALLABLE bool operator==(const XVector2<T>& lhs, const XVector2<T>& rhs)
{
    return (lhs.x == rhs.x && lhs.y == rhs.y);
}

/**
 * @brief Computes the dot product of two vectors.
 * @tparam T The component type of the vectors.
 * @param[in] v1 The first vector.
 * @param[in] v2 The second vector.
 * @return The dot product as a scalar value.
 */
template <typename T>
CUDA_CALLABLE T Dot(const XVector2<T>& v1, const XVector2<T>& v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

/**
 * @brief Returns the counter-clockwise perpendicular vector.
 * @tparam T The component type of the vector.
 * @param[in] v The input vector.
 * @return A new XVector2 rotated 90 degrees counter-clockwise.
 * @details For a vector (x, y), returns (-y, x).
 */
template <typename T>
CUDA_CALLABLE XVector2<T> PerpCCW(const XVector2<T>& v)
{
    return XVector2<T>(-v.y, v.x);
}

/**
 * @brief Returns the clockwise perpendicular vector.
 * @tparam T The component type of the vector.
 * @param[in] v The input vector.
 * @return A new XVector2 rotated 90 degrees clockwise.
 * @details For a vector (x, y), returns (y, -x).
 */
template <typename T>
CUDA_CALLABLE XVector2<T> PerpCW(const XVector2<T>& v)
{
    return XVector2<T>(v.y, -v.x);
}

/**
 * @brief Computes the component-wise maximum of two vectors.
 * @tparam T The component type of the vectors.
 * @param[in] a The first vector.
 * @param[in] b The second vector.
 * @return A new XVector2 with each component being the maximum of the corresponding components.
 */
template <typename T>
CUDA_CALLABLE XVector2<T> Max(const XVector2<T>& a, const XVector2<T>& b)
{
    return XVector2<T>(Max(a.x, b.x), Max(a.y, b.y));
}

/**
 * @brief Computes the component-wise minimum of two vectors.
 * @tparam T The component type of the vectors.
 * @param[in] a The first vector.
 * @param[in] b The second vector.
 * @return A new XVector2 with each component being the minimum of the corresponding components.
 */
template <typename T>
CUDA_CALLABLE XVector2<T> Min(const XVector2<T>& a, const XVector2<T>& b)
{
    return XVector2<T>(Min(a.x, b.x), Min(a.y, b.y));
}

/**
 * @brief Computes the 2D cross product of two vectors.
 * @tparam T The component type of the vectors.
 * @param[in] a The first vector.
 * @param[in] b The second vector.
 * @return The magnitude of the z-component if the vectors were in the xy plane.
 * @details For vectors (a.x, a.y) and (b.x, b.y), returns (a.x * b.y - a.y * b.x).
 */
template <typename T>
CUDA_CALLABLE T Cross(const XVector2<T>& a, const XVector2<T>& b)
{
    return (a.x * b.y - a.y * b.x);
}

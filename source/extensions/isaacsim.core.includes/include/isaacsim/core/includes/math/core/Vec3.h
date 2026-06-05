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

#if 0 //_DEBUG
#    define VEC3_VALIDATE()                                                                                            \
        {                                                                                                              \
            assert(_finite(x));                                                                                        \
            assert(!_isnan(x));                                                                                        \
                                                                                                                       \
            assert(_finite(y));                                                                                        \
            assert(!_isnan(y));                                                                                        \
                                                                                                                       \
            assert(_finite(z));                                                                                        \
            assert(!_isnan(z));                                                                                        \
        }
#else
#    define VEC3_VALIDATE()
#endif

/**
 * @class XVector3
 * @brief Template class representing a 3-dimensional vector with x, y, and z components.
 * @details
 * A template-based 3D vector class that provides basic vector operations including
 * arithmetic operations, comparisons, and type conversions. This class is commonly
 * used for 3D coordinates, directions, and mathematical computations in graphics
 * and physics applications.
 *
 * @tparam T The numeric type for vector components (defaults to float)
 *
 * @note This class includes validation macros in debug builds to ensure finite values
 */
template <typename T = float>
class XVector3
{
public:
    /**
     * @brief Type alias for the template parameter T
     */
    typedef T value_type;

    /**
     * @brief Default constructor initializes all components to zero
     */
    CUDA_CALLABLE inline XVector3() : x(0.0f), y(0.0f), z(0.0f)
    {
    }

    /**
     * @brief Constructor that initializes all components to the same value
     * @param[in] a Value to assign to all components
     */
    CUDA_CALLABLE inline XVector3(T a) : x(a), y(a), z(a)
    {
    }

    /**
     * @brief Constructor that initializes components from an array
     * @param[in] p Pointer to array of at least 3 elements
     */
    CUDA_CALLABLE inline XVector3(const T* p) : x(p[0]), y(p[1]), z(p[2])
    {
    }

    /**
     * @brief Constructor that initializes components with specific values
     * @param[in] x_ X component value
     * @param[in] y_ Y component value
     * @param[in] z_ Z component value
     */
    CUDA_CALLABLE inline XVector3(T x_, T y_, T z_) : x(x_), y(y_), z(z_)
    {
        VEC3_VALIDATE();
    }

    /**
     * @brief Conversion operator to non-const pointer to components
     * @return Pointer to the first component (x)
     */
    CUDA_CALLABLE inline operator T*()
    {
        return &x;
    }

    /**
     * @brief Conversion operator to const pointer to components
     * @return Const pointer to the first component (x)
     */
    CUDA_CALLABLE inline operator const T*() const
    {
        return &x;
    };

    /**
     * @brief Sets all three components of the vector
     * @param[in] x_ New x component value
     * @param[in] y_ New y component value
     * @param[in] z_ New z component value
     */
    CUDA_CALLABLE inline void Set(T x_, T y_, T z_)
    {
        VEC3_VALIDATE();
        x = x_;
        y = y_;
        z = z_;
    }

    /**
     * @brief Scalar multiplication operator
     * @param[in] scale Scalar value to multiply with
     * @return New vector with each component multiplied by scale
     */
    CUDA_CALLABLE inline XVector3<T> operator*(T scale) const
    {
        XVector3<T> r(*this);
        r *= scale;
        return r;
        VEC3_VALIDATE();
    }

    /**
     * @brief Scalar division operator
     * @param[in] scale Scalar value to divide by
     * @return New vector with each component divided by scale
     */
    CUDA_CALLABLE inline XVector3<T> operator/(T scale) const
    {
        XVector3<T> r(*this);
        r /= scale;
        return r;
        VEC3_VALIDATE();
    }

    /**
     * @brief Vector addition operator
     * @param[in] v Vector to add
     * @return New vector that is the sum of this vector and v
     */
    CUDA_CALLABLE inline XVector3<T> operator+(const XVector3<T>& v) const
    {
        XVector3<T> r(*this);
        r += v;
        return r;
        VEC3_VALIDATE();
    }

    /**
     * @brief Vector subtraction operator
     * @param[in] v Vector to subtract
     * @return New vector that is the difference of this vector and v
     */
    CUDA_CALLABLE inline XVector3<T> operator-(const XVector3<T>& v) const
    {
        XVector3<T> r(*this);
        r -= v;
        return r;
        VEC3_VALIDATE();
    }

    /**
     * @brief Component-wise vector division operator
     * @param[in] v Vector to divide by component-wise
     * @return New vector with components divided element-wise
     */
    CUDA_CALLABLE inline XVector3<T> operator/(const XVector3<T>& v) const
    {
        XVector3<T> r(*this);
        r /= v;
        return r;
        VEC3_VALIDATE();
    }

    /**
     * @brief Component-wise vector multiplication operator
     * @param[in] v Vector to multiply component-wise
     * @return New vector with components multiplied element-wise
     */
    CUDA_CALLABLE inline XVector3<T> operator*(const XVector3<T>& v) const
    {
        XVector3<T> r(*this);
        r *= v;
        return r;
        VEC3_VALIDATE();
    }

    /**
     * @brief Scalar multiplication assignment operator
     * @param[in] scale Scalar value to multiply with
     * @return Reference to this vector after multiplication
     */
    CUDA_CALLABLE inline XVector3<T>& operator*=(T scale)
    {
        x *= scale;
        y *= scale;
        z *= scale;
        VEC3_VALIDATE();
        return *this;
    }

    /**
     * @brief Scalar division assignment operator
     * @param[in] scale Scalar value to divide by
     * @return Reference to this vector after division
     */
    CUDA_CALLABLE inline XVector3<T>& operator/=(T scale)
    {
        T s(1.0f / scale);
        x *= s;
        y *= s;
        z *= s;
        VEC3_VALIDATE();
        return *this;
    }

    /**
     * @brief Vector addition assignment operator
     * @param[in] v Vector to add
     * @return Reference to this vector after addition
     */
    CUDA_CALLABLE inline XVector3<T>& operator+=(const XVector3<T>& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        VEC3_VALIDATE();
        return *this;
    }

    /**
     * @brief Vector subtraction assignment operator
     * @param[in] v Vector to subtract
     * @return Reference to this vector after subtraction
     */
    CUDA_CALLABLE inline XVector3<T>& operator-=(const XVector3<T>& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        VEC3_VALIDATE();
        return *this;
    }

    /**
     * @brief Component-wise vector division assignment operator
     * @param[in] v Vector to divide by component-wise
     * @return Reference to this vector after component-wise division
     */
    CUDA_CALLABLE inline XVector3<T>& operator/=(const XVector3<T>& v)
    {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        VEC3_VALIDATE();
        return *this;
    }

    /**
     * @brief Component-wise vector multiplication assignment operator
     * @param[in] v Vector to multiply component-wise
     * @return Reference to this vector after component-wise multiplication
     */
    CUDA_CALLABLE inline XVector3<T>& operator*=(const XVector3<T>& v)
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        VEC3_VALIDATE();
        return *this;
    }

    /**
     * @brief Inequality comparison operator
     * @param[in] v Vector to compare with
     * @return True if any component differs, false if all components are equal
     */
    CUDA_CALLABLE inline bool operator!=(const XVector3<T>& v) const
    {
        return (x != v.x || y != v.y || z != v.z);
    }

    /**
     * @brief Unary negation operator
     * @return New vector with all components negated
     */
    CUDA_CALLABLE inline XVector3<T> operator-() const
    {
        VEC3_VALIDATE();
        return XVector3<T>(-x, -y, -z);
    }

    /**
     * @brief Validates vector components for finite values
     * @details Calls internal validation macro to check for NaN and infinite values
     */
    CUDA_CALLABLE void Validate()
    {
        VEC3_VALIDATE();
    }

    /**
     * @brief X component of the vector
     */
    T x;

    /**
     * @brief Y component of the vector
     */
    T y;

    /**
     * @brief Z component of the vector
     */
    T z;
};

typedef XVector3<float> Vec3;
typedef XVector3<float> Vector3;

/**
 * @brief Scalar multiplication operator (left-hand side)
 * @tparam T Numeric type of vector components
 * @param[in] lhs Scalar value on left side
 * @param[in] rhs Vector on right side
 * @return New vector with each component multiplied by scalar
 */
template <typename T>
CUDA_CALLABLE XVector3<T> operator*(T lhs, const XVector3<T>& rhs)
{
    XVector3<T> r(rhs);
    r *= lhs;
    return r;
}

/**
 * @brief Equality comparison operator for vectors
 * @tparam T Numeric type of vector components
 * @param[in] lhs First vector to compare
 * @param[in] rhs Second vector to compare
 * @return True if all components are equal, false otherwise
 */
template <typename T>
CUDA_CALLABLE bool operator==(const XVector3<T>& lhs, const XVector3<T>& rhs)
{
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

/**
 * @brief Computes the dot product of two vectors with value_type return
 * @tparam T Vector type that supports .x, .y, .z member access
 * @param[in] v1 First vector
 * @param[in] v2 Second vector
 * @return Dot product as the vector's value_type
 */
template <typename T>
CUDA_CALLABLE typename T::value_type Dot3(const T& v1, const T& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

/**
 * @brief Computes the dot product of two float arrays representing 3D vectors
 * @param[in] v1 Pointer to first vector array (must have at least 3 elements)
 * @param[in] v2 Pointer to second vector array (must have at least 3 elements)
 * @return Dot product as float
 */
CUDA_CALLABLE inline float Dot3(const float* v1, const float* v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/**
 * @brief Computes the dot product of two 3D vectors
 * @tparam T Numeric type of vector components
 * @param[in] v1 First vector
 * @param[in] v2 Second vector
 * @return Dot product as type T
 */
template <typename T>
CUDA_CALLABLE inline T Dot(const XVector3<T>& v1, const XVector3<T>& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

/**
 * @brief Computes the cross product of two 3D vectors
 * @param[in] b First vector
 * @param[in] c Second vector
 * @return Cross product vector (b × c)
 * @details Returns a vector perpendicular to both input vectors following right-hand rule
 */
CUDA_CALLABLE inline Vec3 Cross(const Vec3& b, const Vec3& c)
{
    return Vec3(b.y * c.z - b.z * c.y, b.z * c.x - b.x * c.z, b.x * c.y - b.y * c.x);
}

/**
 * @brief Component-wise maximum of two vectors
 * @tparam T Numeric type of vector components
 * @param[in] a First vector
 * @param[in] b Second vector
 * @return Vector with maximum components from both inputs
 */
template <typename T>
CUDA_CALLABLE inline XVector3<T> Max(const XVector3<T>& a, const XVector3<T>& b)
{
    return XVector3<T>(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z));
}

/**
 * @brief Component-wise minimum of two vectors
 * @tparam T Numeric type of vector components
 * @param[in] a First vector
 * @param[in] b Second vector
 * @return Vector with minimum components from both inputs
 */
template <typename T>
CUDA_CALLABLE inline XVector3<T> Min(const XVector3<T>& a, const XVector3<T>& b)
{
    return XVector3<T>(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z));
}

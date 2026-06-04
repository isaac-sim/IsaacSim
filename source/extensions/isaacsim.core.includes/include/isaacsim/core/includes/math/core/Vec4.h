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
#include "Vec3.h"

#include <cassert>

#if 0 // defined(_DEBUG) && defined(_WIN32)
#    define VEC4_VALIDATE()                                                                                            \
        {                                                                                                              \
            assert(_finite(x));                                                                                        \
            assert(!_isnan(x));                                                                                        \
                                                                                                                       \
            assert(_finite(y));                                                                                        \
            assert(!_isnan(y));                                                                                        \
                                                                                                                       \
            assert(_finite(z));                                                                                        \
            assert(!_isnan(z));                                                                                        \
                                                                                                                       \
            assert(_finite(w));                                                                                        \
            assert(!_isnan(w));                                                                                        \
        }
#else
#    define VEC4_VALIDATE()
#endif

/**
 * @class XVector4
 * @brief Template class representing a 4-dimensional vector with x, y, z, and w components.
 * @details
 * A template-based 4D vector class that provides basic vector operations including
 * arithmetic operations, comparisons, and type conversions. This class is commonly
 * used for homogeneous coordinates in 3D graphics and mathematical computations.
 *
 * @tparam T The numeric type for vector components (typically float or double)
 *
 * @note This class includes validation macros in debug builds to ensure finite values
 */
template <typename T>
class XVector4
{
public:
    /**
     * @brief Type alias for the template parameter T
     */
    typedef T value_type;

    /**
     * @brief Default constructor initializes all components to zero
     */
    CUDA_CALLABLE XVector4() : x(0), y(0), z(0), w(0)
    {
    }

    /**
     * @brief Constructor that initializes all components to the same value
     * @param[in] a Value to assign to all components
     */
    CUDA_CALLABLE XVector4(T a) : x(a), y(a), z(a), w(a)
    {
    }

    /**
     * @brief Constructor that initializes components from an array
     * @param[in] p Pointer to array of at least 4 elements
     */
    CUDA_CALLABLE XVector4(const T* p) : x(p[0]), y(p[1]), z(p[2]), w(p[3])
    {
    }

    /**
     * @brief Constructor that initializes components with specific values
     * @param[in] x_ X component value
     * @param[in] y_ Y component value
     * @param[in] z_ Z component value
     * @param[in] w_ W component value (defaults to 1.0f)
     */
    CUDA_CALLABLE XVector4(T x_, T y_, T z_, T w_ = 1.0f) : x(x_), y(y_), z(z_), w(w_)
    {
        VEC4_VALIDATE();
    }

    /**
     * @brief Constructor that creates a 4D vector from a 3D vector and w component
     * @param[in] v 3D vector for x, y, z components
     * @param[in] w W component value
     */
    CUDA_CALLABLE XVector4(const Vec3& v, float w) : x(v.x), y(v.y), z(v.z), w(w)
    {
    }

    /**
     * @brief Conversion operator to non-const pointer to components
     * @return Pointer to the first component (x)
     */
    CUDA_CALLABLE operator T*()
    {
        return &x;
    }

    /**
     * @brief Conversion operator to const pointer to components
     * @return Const pointer to the first component (x)
     */
    CUDA_CALLABLE operator const T*() const
    {
        return &x;
    };

    /**
     * @brief Sets all four components of the vector
     * @param[in] x_ New x component value
     * @param[in] y_ New y component value
     * @param[in] z_ New z component value
     * @param[in] w_ New w component value
     */
    CUDA_CALLABLE void Set(T x_, T y_, T z_, T w_)
    {
        VEC4_VALIDATE();
        x = x_;
        y = y_;
        z = z_;
        w = w_;
    }

    /**
     * @brief Scalar multiplication operator
     * @param[in] scale Scalar value to multiply with
     * @return New vector with each component multiplied by scale
     */
    CUDA_CALLABLE XVector4<T> operator*(T scale) const
    {
        XVector4<T> r(*this);
        r *= scale;
        VEC4_VALIDATE();
        return r;
    }

    /**
     * @brief Scalar division operator
     * @param[in] scale Scalar value to divide by
     * @return New vector with each component divided by scale
     */
    CUDA_CALLABLE XVector4<T> operator/(T scale) const
    {
        XVector4<T> r(*this);
        r /= scale;
        VEC4_VALIDATE();
        return r;
    }

    /**
     * @brief Vector addition operator
     * @param[in] v Vector to add
     * @return New vector that is the sum of this vector and v
     */
    CUDA_CALLABLE XVector4<T> operator+(const XVector4<T>& v) const
    {
        XVector4<T> r(*this);
        r += v;
        VEC4_VALIDATE();
        return r;
    }

    /**
     * @brief Vector subtraction operator
     * @param[in] v Vector to subtract
     * @return New vector that is the difference of this vector and v
     */
    CUDA_CALLABLE XVector4<T> operator-(const XVector4<T>& v) const
    {
        XVector4<T> r(*this);
        r -= v;
        VEC4_VALIDATE();
        return r;
    }

    /**
     * @brief Component-wise vector multiplication operator
     * @param[in] scale Vector to multiply component-wise
     * @return New vector with components multiplied element-wise
     */
    CUDA_CALLABLE XVector4<T> operator*(XVector4<T> scale) const
    {
        XVector4<T> r(*this);
        r *= scale;
        VEC4_VALIDATE();
        return r;
    }

    /**
     * @brief Scalar multiplication assignment operator
     * @param[in] scale Scalar value to multiply with
     * @return Reference to this vector after multiplication
     */
    CUDA_CALLABLE XVector4<T>& operator*=(T scale)
    {
        x *= scale;
        y *= scale;
        z *= scale;
        w *= scale;
        VEC4_VALIDATE();
        return *this;
    }

    /**
     * @brief Scalar division assignment operator
     * @param[in] scale Scalar value to divide by
     * @return Reference to this vector after division
     */
    CUDA_CALLABLE XVector4<T>& operator/=(T scale)
    {
        T s(1.0f / scale);
        x *= s;
        y *= s;
        z *= s;
        w *= s;
        VEC4_VALIDATE();
        return *this;
    }

    /**
     * @brief Vector addition assignment operator
     * @param[in] v Vector to add
     * @return Reference to this vector after addition
     */
    CUDA_CALLABLE XVector4<T>& operator+=(const XVector4<T>& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        w += v.w;
        VEC4_VALIDATE();
        return *this;
    }

    /**
     * @brief Vector subtraction assignment operator
     * @param[in] v Vector to subtract
     * @return Reference to this vector after subtraction
     */
    CUDA_CALLABLE XVector4<T>& operator-=(const XVector4<T>& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        w -= v.w;
        VEC4_VALIDATE();
        return *this;
    }

    /**
     * @brief Component-wise vector multiplication assignment operator
     * @param[in] v Vector to multiply component-wise
     * @return Reference to this vector after component-wise multiplication
     */
    CUDA_CALLABLE XVector4<T>& operator*=(const XVector4<T>& v)
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        w *= v.w;
        VEC4_VALIDATE();
        return *this;
    }

    /**
     * @brief Inequality comparison operator
     * @param[in] v Vector to compare with
     * @return True if any component differs, false if all components are equal
     */
    CUDA_CALLABLE bool operator!=(const XVector4<T>& v) const
    {
        return (x != v.x || y != v.y || z != v.z || w != v.w);
    }

    /**
     * @brief Unary negation operator
     * @return New vector with all components negated
     */
    CUDA_CALLABLE XVector4<T> operator-() const
    {
        VEC4_VALIDATE();
        return XVector4<T>(-x, -y, -z, -w);
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

    /**
     * @brief W component of the vector
     */
    T w;
};

typedef XVector4<float> Vector4;
typedef XVector4<float> Vec4;

// lhs scalar scale
template <typename T>
CUDA_CALLABLE XVector4<T> operator*(T lhs, const XVector4<T>& rhs)
{
    XVector4<T> r(rhs);
    r *= lhs;
    return r;
}

template <typename T>
CUDA_CALLABLE bool operator==(const XVector4<T>& lhs, const XVector4<T>& rhs)
{
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w);
}

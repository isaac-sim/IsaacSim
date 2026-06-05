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

#include "Vec4.h"

#include <ostream>

/**
 * @class Point3
 * @brief Represents a 3D point in space with floating-point coordinates.
 * @details The Point3 class provides a complete set of operations for working with 3D points,
 *          including arithmetic operations, conversions, and validation. Points are distinct from
 *          vectors in that they represent positions rather than directions or displacements.
 */
class Point3
{
public:
    /**
     * @brief Default constructor that initializes the point to the origin (0, 0, 0).
     */
    CUDA_CALLABLE Point3() : x(0), y(0), z(0)
    {
    }

    /**
     * @brief Constructor that initializes all components to the same value.
     * @param[in] a The value to set for all three components (x, y, z).
     */
    CUDA_CALLABLE Point3(float a) : x(a), y(a), z(a)
    {
    }

    /**
     * @brief Constructor that initializes from a float array.
     * @param[in] p Pointer to an array of at least 3 floats [x, y, z].
     * @warning The caller must ensure the array has at least 3 elements.
     */
    CUDA_CALLABLE Point3(const float* p) : x(p[0]), y(p[1]), z(p[2])
    {
    }

    /**
     * @brief Constructor that initializes from individual component values.
     * @param[in] x_ The x-coordinate of the point.
     * @param[in] y_ The y-coordinate of the point.
     * @param[in] z_ The z-coordinate of the point.
     */
    CUDA_CALLABLE Point3(float x_, float y_, float z_) : x(x_), y(y_), z(z_)
    {
        Validate();
    }

    /**
     * @brief Explicit constructor from a Vec3 vector.
     * @param[in] v The vector to convert to a point.
     * @details This conversion treats the vector components as position coordinates.
     */
    CUDA_CALLABLE explicit Point3(const Vec3& v) : x(v.x), y(v.y), z(v.z)
    {
    }

    /**
     * @brief Conversion operator to a mutable float pointer.
     * @return Pointer to the first component (x) for array-style access.
     */
    CUDA_CALLABLE operator float*()
    {
        return &x;
    }

    /**
     * @brief Conversion operator to a const float pointer.
     * @return Const pointer to the first component (x) for array-style access.
     */
    CUDA_CALLABLE operator const float*() const
    {
        return &x;
    };

    /**
     * @brief Conversion operator to a homogeneous Vec4.
     * @return Vec4 with (x, y, z, 1.0) representing the point in homogeneous coordinates.
     */
    CUDA_CALLABLE operator Vec4() const
    {
        return Vec4(x, y, z, 1.0f);
    }

    /**
     * @brief Sets the coordinates of the point.
     * @param[in] x_ The new x-coordinate.
     * @param[in] y_ The new y-coordinate.
     * @param[in] z_ The new z-coordinate.
     */
    CUDA_CALLABLE void Set(float x_, float y_, float z_)
    {
        Validate();
        x = x_;
        y = y_;
        z = z_;
    }

    /**
     * @brief Scales the point by a scalar value.
     * @param[in] scale The scaling factor to apply to all components.
     * @return A new Point3 representing the scaled point.
     */
    CUDA_CALLABLE Point3 operator*(float scale) const
    {
        Point3 r(*this);
        r *= scale;
        Validate();
        return r;
    }

    /**
     * @brief Divides the point by a scalar value.
     * @param[in] scale The divisor to apply to all components.
     * @return A new Point3 representing the divided point.
     * @warning Division by zero will result in undefined behavior.
     */
    CUDA_CALLABLE Point3 operator/(float scale) const
    {
        Point3 r(*this);
        r /= scale;
        Validate();
        return r;
    }

    /**
     * @brief Translates the point by a vector.
     * @param[in] v The translation vector to add.
     * @return A new Point3 representing the translated point.
     */
    CUDA_CALLABLE Point3 operator+(const Vec3& v) const
    {
        Point3 r(*this);
        r += v;
        Validate();
        return r;
    }

    /**
     * @brief Translates the point by the negative of a vector.
     * @param[in] v The translation vector to subtract.
     * @return A new Point3 representing the translated point.
     */
    CUDA_CALLABLE Point3 operator-(const Vec3& v) const
    {
        Point3 r(*this);
        r -= v;
        Validate();
        return r;
    }

    /**
     * @brief Scales this point by a scalar value in-place.
     * @param[in] scale The scaling factor to apply to all components.
     * @return Reference to this point after scaling.
     */
    CUDA_CALLABLE Point3& operator*=(float scale)
    {
        x *= scale;
        y *= scale;
        z *= scale;
        Validate();
        return *this;
    }

    /**
     * @brief Divides this point by a scalar value in-place.
     * @param[in] scale The divisor to apply to all components.
     * @return Reference to this point after division.
     * @warning Division by zero will result in undefined behavior.
     */
    CUDA_CALLABLE Point3& operator/=(float scale)
    {
        float s(1.0f / scale);
        x *= s;
        y *= s;
        z *= s;
        Validate();
        return *this;
    }

    /**
     * @brief Translates this point by a vector in-place.
     * @param[in] v The translation vector to add.
     * @return Reference to this point after translation.
     */
    CUDA_CALLABLE Point3& operator+=(const Vec3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        Validate();
        return *this;
    }

    /**
     * @brief Translates this point by the negative of a vector in-place.
     * @param[in] v The translation vector to subtract.
     * @return Reference to this point after translation.
     */
    CUDA_CALLABLE Point3& operator-=(const Vec3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        Validate();
        return *this;
    }

    /**
     * @brief Assigns a Vec3 to this point.
     * @param[in] v The vector whose components will be copied to this point.
     * @return Reference to this point after assignment.
     */
    CUDA_CALLABLE Point3& operator=(const Vec3& v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }

    /**
     * @brief Tests for inequality with another point.
     * @param[in] v The other point to compare against.
     * @return True if any component differs, false if all components are equal.
     */
    CUDA_CALLABLE bool operator!=(const Point3& v) const
    {
        return (x != v.x || y != v.y || z != v.z);
    }

    /**
     * @brief Returns the negation of this point.
     * @return A new Point3 with all components negated.
     */
    CUDA_CALLABLE Point3 operator-() const
    {
        Validate();
        return Point3(-x, -y, -z);
    }

    /** @brief The x-coordinate of the point. */
    float x;
    /** @brief The y-coordinate of the point. */
    float y;
    /** @brief The z-coordinate of the point. */
    float z;

    /**
     * @brief Validates the point's components for correctness.
     * @details Currently performs no validation but serves as a placeholder for future checks.
     */
    CUDA_CALLABLE void Validate() const
    {
    }
};

/**
 * @brief Scales a point by a scalar value (left-hand side scalar).
 * @param[in] lhs The scalar multiplier.
 * @param[in] rhs The point to scale.
 * @return A new Point3 representing the scaled point.
 */
CUDA_CALLABLE inline Point3 operator*(float lhs, const Point3& rhs)
{
    Point3 r(rhs);
    r *= lhs;
    return r;
}

/**
 * @brief Computes the vector from one point to another.
 * @param[in] lhs The starting point.
 * @param[in] rhs The ending point.
 * @return A Vec3 representing the displacement from lhs to rhs.
 */
CUDA_CALLABLE inline Vec3 operator-(const Point3& lhs, const Point3& rhs)
{
    return Vec3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

/**
 * @brief Adds two points component-wise.
 * @param[in] lhs The first point.
 * @param[in] rhs The second point.
 * @return A new Point3 with components as the sum of corresponding components.
 * @note This operation is mathematically unusual for points but provided for convenience.
 */
CUDA_CALLABLE inline Point3 operator+(const Point3& lhs, const Point3& rhs)
{
    return Point3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

/**
 * @brief Tests for equality between two points.
 * @param[in] lhs The first point to compare.
 * @param[in] rhs The second point to compare.
 * @return True if all corresponding components are equal, false otherwise.
 */
CUDA_CALLABLE inline bool operator==(const Point3& lhs, const Point3& rhs)
{
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

/**
 * @brief Computes the component-wise maximum of two points.
 * @param[in] a The first point.
 * @param[in] b The second point.
 * @return A new Point3 with each component being the maximum of the corresponding components.
 */
CUDA_CALLABLE inline Point3 Max(const Point3& a, const Point3& b)
{
    return Point3(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z));
}

/**
 * @brief Computes the component-wise minimum of two points.
 * @param[in] a The first point.
 * @param[in] b The second point.
 * @return A new Point3 with each component being the minimum of the corresponding components.
 */
CUDA_CALLABLE inline Point3 Min(const Point3& a, const Point3& b)
{
    return Point3(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z));
}

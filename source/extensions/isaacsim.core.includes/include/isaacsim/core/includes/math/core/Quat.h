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

#include "Vec3.h"

#include <cassert>

struct Matrix33;

/**
 * @class XQuat
 * @brief Template class representing a quaternion with x, y, z, and w components.
 * @details
 * A template-based quaternion class that provides quaternion operations including
 * arithmetic operations, rotations, and conversions. Quaternions are commonly used
 * for representing rotations in 3D space without gimbal lock issues.
 *
 * The quaternion is stored as (x, y, z, w) where (x, y, z) represents the vector
 * part and w represents the scalar part.
 *
 * @tparam T The numeric type for quaternion components (typically float or double)
 *
 * @note This class follows the Hamilton convention for quaternion multiplication
 */
template <typename T>
class XQuat
{
public:
    /**
     * @brief Type alias for the template parameter T
     */
    typedef T value_type;

    /**
     * @brief Default constructor creates identity quaternion (0, 0, 0, 1)
     */
    CUDA_CALLABLE XQuat() : x(0), y(0), z(0), w(1.0)
    {
    }

    /**
     * @brief Constructor that initializes components from an array
     * @param[in] p Pointer to array of at least 4 elements in order [x, y, z, w]
     */
    CUDA_CALLABLE XQuat(const T* p) : x(p[0]), y(p[1]), z(p[2]), w(p[3])
    {
    }

    /**
     * @brief Constructor that initializes components with specific values
     * @param[in] x_ X component (i part)
     * @param[in] y_ Y component (j part)
     * @param[in] z_ Z component (k part)
     * @param[in] w_ W component (scalar part)
     */
    CUDA_CALLABLE XQuat(T x_, T y_, T z_, T w_) : x(x_), y(y_), z(z_), w(w_)
    {
    }

    /**
     * @brief Constructor that creates quaternion from vector and scalar parts
     * @param[in] v Vector part containing x, y, z components
     * @param[in] w Scalar part (w component)
     */
    CUDA_CALLABLE XQuat(const Vec3& v, float w) : x(v.x), y(v.y), z(v.z), w(w)
    {
    }

    /**
     * @brief Constructor that creates quaternion from rotation matrix
     * @param[in] m 3x3 rotation matrix to convert
     * @details
     * Converts a 3x3 rotation matrix to a quaternion using Shepperd's method.
     * The algorithm selects the most numerically stable computation path based
     * on the trace and diagonal elements of the matrix.
     */
    CUDA_CALLABLE explicit XQuat(const Matrix33& m);

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
     * @brief Sets all four components of the quaternion
     * @param[in] x_ New x component value
     * @param[in] y_ New y component value
     * @param[in] z_ New z component value
     * @param[in] w_ New w component value
     */
    CUDA_CALLABLE void Set(T x_, T y_, T z_, T w_)
    {
        x = x_;
        y = y_;
        z = z_;
        w = w_;
    }

    /**
     * @brief Scalar multiplication operator
     * @param[in] scale Scalar value to multiply with
     * @return New quaternion with each component multiplied by scale
     */
    CUDA_CALLABLE XQuat<T> operator*(T scale) const
    {
        XQuat<T> r(*this);
        r *= scale;
        return r;
    }

    /**
     * @brief Scalar division operator
     * @param[in] scale Scalar value to divide by
     * @return New quaternion with each component divided by scale
     */
    CUDA_CALLABLE XQuat<T> operator/(T scale) const
    {
        XQuat<T> r(*this);
        r /= scale;
        return r;
    }

    /**
     * @brief Quaternion addition operator
     * @param[in] v Quaternion to add
     * @return New quaternion that is the sum of this quaternion and v
     */
    CUDA_CALLABLE XQuat<T> operator+(const XQuat<T>& v) const
    {
        XQuat<T> r(*this);
        r += v;
        return r;
    }

    /**
     * @brief Quaternion subtraction operator
     * @param[in] v Quaternion to subtract
     * @return New quaternion that is the difference of this quaternion and v
     */
    CUDA_CALLABLE XQuat<T> operator-(const XQuat<T>& v) const
    {
        XQuat<T> r(*this);
        r -= v;
        return r;
    }

    /**
     * @brief Quaternion multiplication operator (Hamilton product)
     * @param[in] q Quaternion to multiply with
     * @return New quaternion that is the Hamilton product of this quaternion and q
     * @details Performs quaternion multiplication following Hamilton convention
     */
    CUDA_CALLABLE XQuat<T> operator*(XQuat<T> q) const
    {
        // quaternion multiplication
        return XQuat<T>(w * q.x + q.w * x + y * q.z - q.y * z, w * q.y + q.w * y + z * q.x - q.z * x,
                        w * q.z + q.w * z + x * q.y - q.x * y, w * q.w - x * q.x - y * q.y - z * q.z);
    }

    /**
     * @brief Scalar multiplication assignment operator
     * @param[in] scale Scalar value to multiply with
     * @return Reference to this quaternion after multiplication
     */
    CUDA_CALLABLE XQuat<T>& operator*=(T scale)
    {
        x *= scale;
        y *= scale;
        z *= scale;
        w *= scale;
        return *this;
    }

    /**
     * @brief Scalar division assignment operator
     * @param[in] scale Scalar value to divide by
     * @return Reference to this quaternion after division
     */
    CUDA_CALLABLE XQuat<T>& operator/=(T scale)
    {
        T s(1.0f / scale);
        x *= s;
        y *= s;
        z *= s;
        w *= s;
        return *this;
    }

    /**
     * @brief Quaternion addition assignment operator
     * @param[in] v Quaternion to add
     * @return Reference to this quaternion after addition
     */
    CUDA_CALLABLE XQuat<T>& operator+=(const XQuat<T>& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        w += v.w;
        return *this;
    }

    /**
     * @brief Quaternion subtraction assignment operator
     * @param[in] v Quaternion to subtract
     * @return Reference to this quaternion after subtraction
     */
    CUDA_CALLABLE XQuat<T>& operator-=(const XQuat<T>& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        w -= v.w;
        return *this;
    }

    /**
     * @brief Inequality comparison operator
     * @param[in] v Quaternion to compare with
     * @return True if any component differs, false if all components are equal
     */
    CUDA_CALLABLE bool operator!=(const XQuat<T>& v) const
    {
        return (x != v.x || y != v.y || z != v.z || w != v.w);
    }

    /**
     * @brief Unary negation operator
     * @return New quaternion with all components negated
     */
    CUDA_CALLABLE XQuat<T> operator-() const
    {
        return XQuat<T>(-x, -y, -z, -w);
    }

    /**
     * @brief Gets the vector part of the quaternion
     * @return 3D vector containing the x, y, z components
     */
    CUDA_CALLABLE XVector3<T> GetAxis() const
    {
        return XVector3<T>(x, y, z);
    }

    /**
     * @brief X component of the quaternion (i part)
     */
    T x;

    /**
     * @brief Y component of the quaternion (j part)
     */
    T y;

    /**
     * @brief Z component of the quaternion (k part)
     */
    T z;

    /**
     * @brief W component of the quaternion (scalar part)
     */
    T w;
};

typedef XQuat<float> Quat;

// lhs scalar scale
template <typename T>
CUDA_CALLABLE XQuat<T> operator*(T lhs, const XQuat<T>& rhs)
{
    XQuat<T> r(rhs);
    r *= lhs;
    return r;
}

template <typename T>
CUDA_CALLABLE bool operator==(const XQuat<T>& lhs, const XQuat<T>& rhs)
{
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w);
}

/// @cond DOXYGEN_SHOULD_SKIP_THIS
template <typename T>
CUDA_CALLABLE inline XQuat<T> QuatFromAxisAngle(const Vec3& axis, float angle)
{
    Vec3 v = Normalize(axis);

    float half = angle * 0.5f;
    float w = cosf(half);

    const float sin_theta_over_two = sinf(half);
    v *= sin_theta_over_two;

    return XQuat<T>(v.x, v.y, v.z, w);
}
/// @endcond

CUDA_CALLABLE inline float Dot(const Quat& a, const Quat& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

CUDA_CALLABLE inline float Length(const Quat& a)
{
    return sqrtf(Dot(a, a));
}

CUDA_CALLABLE inline Quat QuatFromEulerZYX(float rotx, float roty, float rotz)
{
    Quat q;
    // Abbreviations for the various angular functions
    float cy = cos(rotz * 0.5f);
    float sy = sin(rotz * 0.5f);
    float cr = cos(rotx * 0.5f);
    float sr = sin(rotx * 0.5f);
    float cp = cos(roty * 0.5f);
    float sp = sin(roty * 0.5f);

    q.w = (float)(cy * cr * cp + sy * sr * sp);
    q.x = (float)(cy * sr * cp - sy * cr * sp);
    q.y = (float)(cy * cr * sp + sy * sr * cp);
    q.z = (float)(sy * cr * cp - cy * sr * sp);

    return q;
}

CUDA_CALLABLE inline void EulerFromQuatZYX(const Quat& q, float& rotx, float& roty, float& rotz)
{

    float x = q.x, y = q.y, z = q.z, w = q.w;
    float t0 = x * x - z * z;
    float t1 = w * w - y * y;
    float xx = 0.5f * (t0 + t1); // 1/2 x of x'
    float xy = x * y + w * z; // 1/2 y of x'
    float xz = w * y - x * z; // 1/2 z of x'
    float t = xx * xx + xy * xy; // cos(theta)^2
    float yz = 2.0f * (y * z + w * x); // z of y'

    rotz = atan2(xy, xx); // yaw   (psi)
    roty = atan(xz / sqrt(t)); // pitch (theta)

    // todo: doublecheck!
    if (fabsf(t) > 1e-6f)
    {
        rotx = (float)atan2(yz, t1 - t0);
    }
    else
    {
        rotx = (float)(2.0f * atan2(x, w) - copysignf(1.0f, xz) * rotz);
    }
}

// converts Euler angles to quaternion performing an intrinsic rotation in yaw
// then pitch then roll order i.e. first rotate by yaw around z, then rotate by
// pitch around the rotated y axis, then rotate around roll around the twice (by
// yaw and pitch) rotated x axis
CUDA_CALLABLE inline Quat rpy2quat(const float roll, const float pitch, const float yaw)
{
    Quat q;
    // Abbreviations for the various angular functions
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);

    q.w = (float)(cy * cr * cp + sy * sr * sp);
    q.x = (float)(cy * sr * cp - sy * cr * sp);
    q.y = (float)(cy * cr * sp + sy * sr * cp);
    q.z = (float)(sy * cr * cp - cy * sr * sp);

    return q;
}

// converts Euler angles to quaternion performing an intrinsic rotation in x
// then y then z order i.e. first rotate by x_rot around x, then rotate by y_rot
// around the rotated y axis, then rotate by z_rot around the twice (by roll and
// pitch) rotated z axis
CUDA_CALLABLE inline Quat euler_xyz2quat(const float x_rot, const float y_rot, const float z_rot)
{
    Quat q;
    // Abbreviations for the various angular functions
    float cy = std::cos(z_rot * 0.5f);
    float sy = std::sin(z_rot * 0.5f);
    float cr = std::cos(x_rot * 0.5f);
    float sr = std::sin(x_rot * 0.5f);
    float cp = std::cos(y_rot * 0.5f);
    float sp = std::sin(y_rot * 0.5f);

    q.w = (float)(cy * cr * cp - sy * sr * sp);
    q.x = (float)(cy * sr * cp + sy * cr * sp);
    q.y = (float)(cy * cr * sp - sy * sr * cp);
    q.z = (float)(sy * cr * cp + cy * sr * sp);

    return q;
}

// !!! preist@ This function produces euler angles according to this convention:
// https://www.euclideanspace.com/maths/standards/index.htm Heading = rotation
// about y axis Attitude = rotation about z axis Bank = rotation about x axis
// Order: Heading (y) -> Attitude (z) -> Bank (x), and applied intrinsically
CUDA_CALLABLE inline void quat2rpy(const Quat& q1, float& bank, float& attitude, float& heading)
{
    float sqw = q1.w * q1.w;
    float sqx = q1.x * q1.x;
    float sqy = q1.y * q1.y;
    float sqz = q1.z * q1.z;
    float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
    float test = q1.x * q1.y + q1.z * q1.w;

    if (test > 0.499f * unit)
    { // singularity at north pole
        heading = 2.f * atan2(q1.x, q1.w);
        attitude = kPi / 2.f;
        bank = 0.f;
        return;
    }

    if (test < -0.499f * unit)
    { // singularity at south pole
        heading = -2.f * atan2(q1.x, q1.w);
        attitude = -kPi / 2.f;
        bank = 0.f;
        return;
    }

    heading = atan2(2.f * q1.x * q1.y + 2.f * q1.w * q1.z, sqx - sqy - sqz + sqw);
    attitude = asin(-2.f * q1.x * q1.z + 2.f * q1.y * q1.w);
    bank = atan2(2.f * q1.y * q1.z + 2.f * q1.x * q1.w, -sqx - sqy + sqz + sqw);
}

// preist@:
// The Euler angles correspond to an extrinsic x-y-z i.e. intrinsic z-y-x
// rotation and this function does not guard against gimbal lock
CUDA_CALLABLE inline void zUpQuat2rpy(const Quat& q1, float& roll, float& pitch, float& yaw)
{
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q1.w * q1.x + q1.y * q1.z);
    float cosr_cosp = 1.0f - 2.0f * (q1.x * q1.x + q1.y * q1.y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q1.w * q1.y - q1.z * q1.x);
    if (fabs(sinp) > 0.999f)
        pitch = (float)copysign(kPi / 2.0f, sinp);
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q1.w * q1.z + q1.x * q1.y);
    float cosy_cosp = 1.0f - 2.0f * (q1.y * q1.y + q1.z * q1.z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

CUDA_CALLABLE inline void getEulerZYX(const Quat& q, float& yawZ, float& pitchY, float& rollX)
{
    float squ;
    float sqx;
    float sqy;
    float sqz;
    float sarg;
    sqx = q.x * q.x;
    sqy = q.y * q.y;
    sqz = q.z * q.z;
    squ = q.w * q.w;

    rollX = atan2(2 * (q.y * q.z + q.w * q.x), squ - sqx - sqy + sqz);
    sarg = (-2.0f) * (q.x * q.z - q.w * q.y);
    pitchY = sarg <= (-1.0f) ? (-0.5f) * kPi : (sarg >= (1.0f) ? (0.5f) * kPi : asinf(sarg));
    yawZ = atan2(2 * (q.x * q.y + q.w * q.z), squ + sqx - sqy - sqz);
}

// rotate vector by quaternion (q, w)
CUDA_CALLABLE inline Vec3 Rotate(const Quat& q, const Vec3& x)
{
    return x * (2.0f * q.w * q.w - 1.0f) + Cross(Vec3(q), x) * q.w * 2.0f + Vec3(q) * Dot(Vec3(q), x) * 2.0f;
}

CUDA_CALLABLE inline Vec3 operator*(const Quat& q, const Vec3& v)
{
    return Rotate(q, v);
}

CUDA_CALLABLE inline Vec3 GetBasisVector0(const Quat& q)
{
    return Rotate(q, Vec3(1.0f, 0.0f, 0.0f));
}
CUDA_CALLABLE inline Vec3 GetBasisVector1(const Quat& q)
{
    return Rotate(q, Vec3(0.0f, 1.0f, 0.0f));
}
CUDA_CALLABLE inline Vec3 GetBasisVector2(const Quat& q)
{
    return Rotate(q, Vec3(0.0f, 0.0f, 1.0f));
}

// rotate vector by inverse transform in (q, w)
CUDA_CALLABLE inline Vec3 RotateInv(const Quat& q, const Vec3& x)
{
    return x * (2.0f * q.w * q.w - 1.0f) - Cross(Vec3(q), x) * q.w * 2.0f + Vec3(q) * Dot(Vec3(q), x) * 2.0f;
}

CUDA_CALLABLE inline Quat Inverse(const Quat& q)
{
    return Quat(-q.x, -q.y, -q.z, q.w);
}

CUDA_CALLABLE inline Quat Normalize(const Quat& q)
{
    float lSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

    if (lSq > 0.0f)
    {
        float invL = 1.0f / sqrtf(lSq);

        return q * invL;
    }
    else
        return Quat();
}

//
// given two quaternions and a time-step returns the corresponding angular
// velocity vector
//
CUDA_CALLABLE inline Vec3 DifferentiateQuat(const Quat& q1, const Quat& q0, float invdt)
{
    Quat dq = q1 * Inverse(q0);

    float sinHalfTheta = Length(dq.GetAxis());
    float theta = asinf(sinHalfTheta) * 2.0f;

    if (fabsf(theta) < 0.001f)
    {
        // use linear approximation approx for small angles
        Quat dqdt = (q1 - q0) * invdt;
        Quat omega = dqdt * Inverse(q0);

        return Vec3(omega.x, omega.y, omega.z) * 2.0f;
    }
    else
    {
        // use inverse exponential map
        Vec3 axis = Normalize(dq.GetAxis());
        return axis * theta * invdt;
    }
}

CUDA_CALLABLE inline Quat IntegrateQuat(const Vec3& omega, const Quat& q0, float dt)
{
    Vec3 axis;
    float w = Length(omega);

    if (w * dt < 0.001f)
    {
        // sinc approx for small angles
        axis = omega * (0.5f * dt - (dt * dt * dt) / 48.0f * w * w);
    }
    else
    {
        axis = omega * (sinf(0.5f * w * dt) / w);
    }

    Quat dq;
    dq.x = axis.x;
    dq.y = axis.y;
    dq.z = axis.z;
    dq.w = cosf(w * dt * 0.5f);

    Quat q1 = dq * q0;

    // explicit re-normalization here otherwise we do some see energy drift
    return Normalize(q1);
}

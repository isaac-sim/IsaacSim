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
#include "Core.h"
#include "Mat22.h"
#include "Mat33.h"
#include "Mat44.h"
#include "Matnn.h"
#include "Point3.h"
#include "Quat.h"
#include "Types.h"
#include "Vec2.h"
#include "Vec3.h"
#include "Vec4.h"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <float.h>
#include <string.h>

/**
 * @struct Transform
 * @brief 3D transformation represented by position and rotation.
 * @details
 * This structure represents a 3D transformation consisting of a position (translation)
 * and rotation (quaternion). It provides composition operations through multiplication
 * and is commonly used for object positioning and coordinate space transformations.
 */
struct Transform
{
    /**
     * @brief Default constructor creating identity transform.
     * @details
     * Creates a transform with zero translation and identity rotation (no rotation).
     */
    CUDA_CALLABLE Transform() : p(0.0)
    {
    }
    /**
     * @brief Constructor with position and optional rotation.
     * @param[in] v Position vector
     * @param[in] q Rotation quaternion (default: identity rotation)
     */
    CUDA_CALLABLE Transform(const Vec3& v, const Quat& q = Quat()) : p(v), q(q)
    {
    }

    /**
     * @brief Transform composition operator.
     * @param[in] rhs The transform to compose with this transform
     * @return The composed transform
     * @details
     * Composes two transforms such that the result represents applying this transform
     * followed by the rhs transform.
     */
    CUDA_CALLABLE Transform operator*(const Transform& rhs) const
    {
        return Transform(Rotate(q, rhs.p) + p, q * rhs.q);
    }

    /**
     * @brief Position (translation) component.
     */
    Vec3 p;
    /**
     * @brief Rotation component as a quaternion.
     */
    Quat q;
};

CUDA_CALLABLE inline Transform Inverse(const Transform& transform)
{
    Transform t;
    t.q = Inverse(transform.q);
    t.p = -Rotate(t.q, transform.p);

    return t;
}

CUDA_CALLABLE inline Vec3 TransformVector(const Transform& t, const Vec3& v)
{
    return t.q * v;
}

CUDA_CALLABLE inline Vec3 TransformPoint(const Transform& t, const Vec3& v)
{
    return t.q * v + t.p;
}

CUDA_CALLABLE inline Vec3 InverseTransformVector(const Transform& t, const Vec3& v)
{
    return Inverse(t.q) * v;
}

CUDA_CALLABLE inline Vec3 InverseTransformPoint(const Transform& t, const Vec3& v)
{
    return Inverse(t.q) * (v - t.p);
}

// represents a plane in the form ax + by + cz - d = 0
/**
 * @class Plane
 * @brief 3D plane representation in the form ax + by + cz + d = 0.
 * @details
 * This class represents a 3D plane using the implicit equation ax + by + cz + d = 0,
 * where (a, b, c) is the plane normal and d is the distance from origin. It inherits
 * from Vec4 where x, y, z represent the normal components and w represents -d.
 */
class Plane : public Vec4
{
public:
    /**
     * @brief Default constructor creating an uninitialized plane.
     */
    CUDA_CALLABLE inline Plane()
    {
    }
    /**
     * @brief Constructor from plane equation coefficients.
     * @param[in] x Normal x-component (coefficient a)
     * @param[in] y Normal y-component (coefficient b)
     * @param[in] z Normal z-component (coefficient c)
     * @param[in] w Negative distance from origin (coefficient d)
     */
    CUDA_CALLABLE inline Plane(float x, float y, float z, float w) : Vec4(x, y, z, w)
    {
    }
    /**
     * @brief Constructor from point on plane and normal vector.
     * @param[in] p A point lying on the plane
     * @param[in] n Normal vector to the plane
     */
    CUDA_CALLABLE inline Plane(const Vec3& p, const Vector3& n)
    {
        x = n.x;
        y = n.y;
        z = n.z;
        w = -Dot3(p, n);
    }

    /**
     * @brief Gets the normal vector of the plane.
     * @return The plane's normal vector
     */
    CUDA_CALLABLE inline Vec3 GetNormal() const
    {
        return Vec3(x, y, z);
    }
    /**
     * @brief Gets a point on the plane closest to the origin.
     * @return A point on the plane
     */
    CUDA_CALLABLE inline Vec3 GetPoint() const
    {
        return Vec3(x * -w, y * -w, z * -w);
    }

    /**
     * @brief Constructor from Vec3 (assumes w=1).
     * @param[in] v Vector containing normal components
     */
    CUDA_CALLABLE inline Plane(const Vec3& v) : Vec4(v.x, v.y, v.z, 1.0f)
    {
    }
    /**
     * @brief Constructor from Vec4.
     * @param[in] v Vector containing plane equation coefficients
     */
    CUDA_CALLABLE inline Plane(const Vec4& v) : Vec4(v)
    {
    }
};

template <typename T>
CUDA_CALLABLE inline T Dot(const XVector4<T>& v1, const XVector4<T>& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
}

// helper function that assumes a w of 0
CUDA_CALLABLE inline float Dot(const Plane& p, const Vector3& v)
{
    return p.x * v.x + p.y * v.y + p.z * v.z;
}

CUDA_CALLABLE inline float Dot(const Vector3& v, const Plane& p)
{
    return Dot(p, v);
}

// helper function that assumes a w of 1
CUDA_CALLABLE inline float Dot(const Plane& p, const Point3& v)
{
    return p.x * v.x + p.y * v.y + p.z * v.z + p.w;
}

// ensures that the normal component of the plane is unit magnitude
CUDA_CALLABLE inline Vec4 NormalizePlane(const Vec4& p)
{
    float l = Length(Vec3(p));

    return (1.0f / l) * p;
}

//----------------------------------------------------------------------------
inline float RandomUnit()
{
    float r = (float)rand();
    r /= RAND_MAX;
    return r;
}

// Random number in range [-1,1]
inline float RandomSignedUnit()
{
    float r = (float)rand();
    r /= RAND_MAX;
    r = 2.0f * r - 1.0f;
    return r;
}

inline float Random(float lo, float hi)
{
    float r = (float)rand();
    r /= RAND_MAX;
    r = (hi - lo) * r + lo;
    return r;
}

inline void RandInit(uint32_t seed = 315645664)
{
    std::srand(static_cast<unsigned>(seed));
}

// random number generator
inline uint32_t Rand()
{
    return static_cast<uint32_t>(std::rand());
}

// returns a random number in the range [min, max)
inline uint32_t Rand(uint32_t min, uint32_t max)
{
    return min + Rand() % (max - min);
}

// returns random number between 0-1
inline float Randf()
{
    uint32_t value = Rand();
    uint32_t limit = 0xffffffff;

    return (float)value * (1.0f / (float)limit);
}

// returns random number between min and max
inline float Randf(float min, float max)
{
    //	return Lerp(min, max, ParticleRandf());
    float t = Randf();
    return (1.0f - t) * min + t * (max);
}

// returns random number between 0-max
inline float Randf(float max)
{
    return Randf() * max;
}

// returns a random unit vector (also can add an offset to generate around an
// off axis vector)
inline Vec3 RandomUnitVector()
{
    float phi = Randf(kPi * 2.0f);
    float theta = Randf(kPi * 2.0f);

    float cosTheta = Cos(theta);
    float sinTheta = Sin(theta);

    float cosPhi = Cos(phi);
    float sinPhi = Sin(phi);

    return Vec3(cosTheta * sinPhi, cosPhi, sinTheta * sinPhi);
}

inline Vec3 RandVec3()
{
    return Vec3(Randf(-1.0f, 1.0f), Randf(-1.0f, 1.0f), Randf(-1.0f, 1.0f));
}

// uniformly sample volume of a sphere using dart throwing
inline Vec3 UniformSampleSphereVolume()
{
    for (;;)
    {
        Vec3 v = RandVec3();
        if (Dot(v, v) < 1.0f)
            return v;
    }
}

inline Vec3 UniformSampleSphere()
{
    float u1 = Randf(0.0f, 1.0f);
    float u2 = Randf(0.0f, 1.0f);

    float z = 1.f - 2.f * u1;
    float r = sqrtf(Max(0.f, 1.f - z * z));
    float phi = 2.f * kPi * u2;
    float x = r * cosf(phi);
    float y = r * sinf(phi);

    return Vector3(x, y, z);
}

inline Vec3 UniformSampleHemisphere()
{
    // generate a random z value
    float z = Randf(0.0f, 1.0f);
    float w = Sqrt(1.0f - z * z);

    float phi = k2Pi * Randf(0.0f, 1.0f);
    float x = Cos(phi) * w;
    float y = Sin(phi) * w;

    return Vec3(x, y, z);
}

inline Vec2 UniformSampleDisc()
{
    float r = Sqrt(Randf(0.0f, 1.0f));
    float theta = k2Pi * Randf(0.0f, 1.0f);

    return Vec2(r * Cos(theta), r * Sin(theta));
}

inline void UniformSampleTriangle(float& u, float& v)
{
    float r = Sqrt(Randf());
    u = 1.0f - r;
    v = Randf() * r;
}

inline Vec3 CosineSampleHemisphere()
{
    Vec2 s = UniformSampleDisc();
    float z = Sqrt(Max(0.0f, 1.0f - s.x * s.x - s.y * s.y));

    return Vec3(s.x, s.y, z);
}

inline Vec3 SphericalToXYZ(float theta, float phi)
{
    float cosTheta = cos(theta);
    float sinTheta = sin(theta);

    return Vec3(sin(phi) * sinTheta, cosTheta, cos(phi) * sinTheta);
}

// returns random vector between -range and range
inline Vec4 Randf(const Vec4& range)
{
    return Vec4(Randf(-range.x, range.x), Randf(-range.y, range.y), Randf(-range.z, range.z), Randf(-range.w, range.w));
}

// generates a transform matrix with v as the z axis, taken from PBRT
CUDA_CALLABLE inline void BasisFromVector(const Vec3& w, Vec3* u, Vec3* v)
{
    if (fabsf(w.x) > fabsf(w.y))
    {
        float invLen = 1.0f / sqrtf(w.x * w.x + w.z * w.z);
        *u = Vec3(-w.z * invLen, 0.0f, w.x * invLen);
    }
    else
    {
        float invLen = 1.0f / sqrtf(w.y * w.y + w.z * w.z);
        *u = Vec3(0.0f, w.z * invLen, -w.y * invLen);
    }

    *v = Cross(w, *u);

    // assert(fabsf(Length(*u)-1.0f) < 0.01f);
    // assert(fabsf(Length(*v)-1.0f) < 0.01f);
}

// same as above but returns a matrix
inline Mat44 TransformFromVector(const Vec3& w, const Point3& t = Point3(0.0f, 0.0f, 0.0f))
{
    Mat44 m = Mat44::Identity();
    m.SetCol(2, Vec4(w.x, w.y, w.z, 0.0));
    m.SetCol(3, Vec4(t.x, t.y, t.z, 1.0f));

    BasisFromVector(w, reinterpret_cast<Vec3*>(m.columns[0]), reinterpret_cast<Vec3*>(m.columns[1]));

    return m;
}

// todo: sort out rotations
inline Mat44 ViewMatrix(const Point3& pos)
{

    float view[4][4] = { { 1.0f, 0.0f, 0.0f, 0.0f },
                         { 0.0f, 1.0f, 0.0f, 0.0f },
                         { 0.0f, 0.0f, 1.0f, 0.0f },
                         { -pos.x, -pos.y, -pos.z, 1.0f } };

    return Mat44(&view[0][0]);
}

inline Mat44 LookAtMatrix(const Point3& viewer, const Point3& target)
{
    // create a basis from viewer to target (OpenGL convention looking down -z)
    Vec3 forward = -Normalize(target - viewer);
    Vec3 up(0.0f, 1.0f, 0.0f);
    Vec3 left = Normalize(Cross(up, forward));
    up = Cross(forward, left);

    float xform[4][4] = { { left.x, left.y, left.z, 0.0f },
                          { up.x, up.y, up.z, 0.0f },
                          { forward.x, forward.y, forward.z, 0.0f },
                          { viewer.x, viewer.y, viewer.z, 1.0f } };

    return AffineInverse(Mat44(&xform[0][0]));
}

// generate a rotation matrix around an axis, from PBRT p74
inline Mat44 RotationMatrix(float angle, const Vec3& axis)
{
    Vec3 a = Normalize(axis);
    float s = sinf(angle);
    float c = cosf(angle);

    float m[4][4];

    m[0][0] = a.x * a.x + (1.0f - a.x * a.x) * c;
    m[0][1] = a.x * a.y * (1.0f - c) + a.z * s;
    m[0][2] = a.x * a.z * (1.0f - c) - a.y * s;
    m[0][3] = 0.0f;

    m[1][0] = a.x * a.y * (1.0f - c) - a.z * s;
    m[1][1] = a.y * a.y + (1.0f - a.y * a.y) * c;
    m[1][2] = a.y * a.z * (1.0f - c) + a.x * s;
    m[1][3] = 0.0f;

    m[2][0] = a.x * a.z * (1.0f - c) + a.y * s;
    m[2][1] = a.y * a.z * (1.0f - c) - a.x * s;
    m[2][2] = a.z * a.z + (1.0f - a.z * a.z) * c;
    m[2][3] = 0.0f;

    m[3][0] = 0.0f;
    m[3][1] = 0.0f;
    m[3][2] = 0.0f;
    m[3][3] = 1.0f;

    return Mat44(&m[0][0]);
}

inline Mat44 RotationMatrix(const Quat& q)
{
    Matrix33 rotation(q);

    Matrix44 m;
    m.SetAxis(0, rotation.cols[0]);
    m.SetAxis(1, rotation.cols[1]);
    m.SetAxis(2, rotation.cols[2]);
    m.SetTranslation(Point3(0.0f));

    return m;
}

inline Mat44 TranslationMatrix(const Point3& t)
{
    Mat44 m(Mat44::Identity());
    m.SetTranslation(t);
    return m;
}

inline Mat44 TransformMatrix(const Transform& t)
{
    return TranslationMatrix(Point3(t.p)) * RotationMatrix(t.q);
}

inline Mat44 OrthographicMatrix(float left, float right, float bottom, float top, float n, float f)
{

    float m[4][4] = { { 2.0f / (right - left), 0.0f, 0.0f, 0.0f },
                      { 0.0f, 2.0f / (top - bottom), 0.0f, 0.0f },
                      { 0.0f, 0.0f, -2.0f / (f - n), 0.0f },
                      { -(right + left) / (right - left), -(top + bottom) / (top - bottom), -(f + n) / (f - n), 1.0f } };

    return Mat44(&m[0][0]);
}

// this is designed as a drop in replacement for gluPerspective
inline Mat44 ProjectionMatrix(float fov, float aspect, float znear, float zfar)
{
    float f = 1.0f / tanf(DegToRad(fov * 0.5f));
    float zd = znear - zfar;

    float view[4][4] = { { f / aspect, 0.0f, 0.0f, 0.0f },
                         { 0.0f, f, 0.0f, 0.0f },
                         { 0.0f, 0.0f, (zfar + znear) / zd, -1.0f },
                         { 0.0f, 0.0f, (2.0f * znear * zfar) / zd, 0.0f } };

    return Mat44(&view[0][0]);
}

/**
 * @class Rotation
 * @brief Euler angle representation of 3D rotation.
 * @details
 * This class encapsulates a 3D orientation using Euler angles (yaw, pitch, roll).
 * While not as robust as quaternions for rotations, it provides an intuitive
 * interface for manipulating object orientations, especially from scripting contexts.
 *
 * All angles are stored in degrees for convenience in editing and visualization.
 *
 * @note Euler angles can suffer from gimbal lock and are order-dependent.
 * @warning Be careful with angle accumulation to avoid numerical drift.
 */
class Rotation
{
public:
    /**
     * @brief Default constructor creating zero rotation.
     * @details Creates a rotation with all angles set to zero (identity rotation).
     */
    Rotation() : yaw(0), pitch(0), roll(0)
    {
    }
    /**
     * @brief Constructor with explicit angle values.
     * @param[in] inYaw Yaw angle in degrees (rotation around Y-axis)
     * @param[in] inPitch Pitch angle in degrees (rotation around Z-axis)
     * @param[in] inRoll Roll angle in degrees (rotation around X-axis)
     */
    Rotation(float inYaw, float inPitch, float inRoll) : yaw(inYaw), pitch(inPitch), roll(inRoll)
    {
    }

    /**
     * @brief In-place addition operator.
     * @param[in] rhs Rotation to add
     * @return Reference to this rotation after addition
     */
    Rotation& operator+=(const Rotation& rhs)
    {
        yaw += rhs.yaw;
        pitch += rhs.pitch;
        roll += rhs.roll;
        return *this;
    }
    /**
     * @brief In-place subtraction operator.
     * @param[in] rhs Rotation to subtract
     * @return Reference to this rotation after subtraction
     */
    Rotation& operator-=(const Rotation& rhs)
    {
        yaw -= rhs.yaw;
        pitch -= rhs.pitch;
        roll -= rhs.roll;
        return *this;
    }

    /**
     * @brief Addition operator.
     * @param[in] rhs Rotation to add
     * @return New rotation with added angles
     */
    Rotation operator+(const Rotation& rhs) const
    {
        Rotation lhs(*this);
        lhs += rhs;
        return lhs;
    }
    /**
     * @brief Subtraction operator.
     * @param[in] rhs Rotation to subtract
     * @return New rotation with subtracted angles
     */
    Rotation operator-(const Rotation& rhs) const
    {
        Rotation lhs(*this);
        lhs -= rhs;
        return lhs;
    }

    /**
     * @brief Yaw angle in degrees (rotation around Y-axis).
     */
    float yaw;
    /**
     * @brief Pitch angle in degrees (rotation around Z-axis).
     */
    float pitch;
    /**
     * @brief Roll angle in degrees (rotation around X-axis).
     */
    float roll;
};

inline Mat44 ScaleMatrix(const Vector3& s)
{
    float m[4][4] = {
        { s.x, 0.0f, 0.0f, 0.0f }, { 0.0f, s.y, 0.0f, 0.0f }, { 0.0f, 0.0f, s.z, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f }
    };

    return Mat44(&m[0][0]);
}

// assumes yaw on y, then pitch on z, then roll on x
inline Mat44 TransformMatrix(const Rotation& r, const Point3& p)
{
    const float yaw = DegToRad(r.yaw);
    const float pitch = DegToRad(r.pitch);
    const float roll = DegToRad(r.roll);

    const float s1 = Sin(roll);
    const float c1 = Cos(roll);
    const float s2 = Sin(pitch);
    const float c2 = Cos(pitch);
    const float s3 = Sin(yaw);
    const float c3 = Cos(yaw);

    // interprets the angles as yaw around world-y, pitch around new z, roll
    // around new x
    float mr[4][4] = { { c2 * c3, s2, -c2 * s3, 0.0f },
                       { s1 * s3 - c1 * c3 * s2, c1 * c2, c3 * s1 + c1 * s2 * s3, 0.0f },
                       { c3 * s1 * s2 + c1 * s3, -c2 * s1, c1 * c3 - s1 * s2 * s3, 0.0f },
                       { p.x, p.y, p.z, 1.0f } };

    Mat44 m1(&mr[0][0]);

    return m1; // m2 * m1;
}

// aligns the z axis along the vector
inline Rotation AlignToVector(const Vec3& vector)
{
    // todo: fix, see spherical->cartesian coordinates wikipedia
    return Rotation(0.0f, RadToDeg(atan2(vector.y, vector.x)), 0.0f);
}

// creates a vector given an angle measured clockwise from horizontal (1,0)
inline Vec2 AngleToVector(float a)
{
    return Vec2(Cos(a), Sin(a));
}

inline float VectorToAngle(const Vec2& v)
{
    return atan2f(v.y, v.x);
}

CUDA_CALLABLE inline float SmoothStep(float a, float b, float t)
{
    t = Clamp(t - a / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// hermite spline interpolation
template <typename T>
T HermiteInterpolate(const T& a, const T& b, const T& t1, const T& t2, float t)
{
    // blending weights
    const float w1 = 1.0f - 3 * t * t + 2 * t * t * t;
    const float w2 = t * t * (3.0f - 2.0f * t);
    const float w3 = t * t * t - 2 * t * t + t;
    const float w4 = t * t * (t - 1.0f);

    // return weighted combination
    return a * w1 + b * w2 + t1 * w3 + t2 * w4;
}

template <typename T>
T HermiteTangent(const T& a, const T& b, const T& t1, const T& t2, float t)
{
    // first derivative blend weights
    const float w1 = 6.0f * t * t - 6 * t;
    const float w2 = -6.0f * t * t + 6 * t;
    const float w3 = 3 * t * t - 4 * t + 1;
    const float w4 = 3 * t * t - 2 * t;

    // weighted combination
    return a * w1 + b * w2 + t1 * w3 + t2 * w4;
}

template <typename T>
T HermiteSecondDerivative(const T& a, const T& b, const T& t1, const T& t2, float t)
{
    // first derivative blend weights
    const float w1 = 12 * t - 6.0f;
    const float w2 = -12.0f * t + 6;
    const float w3 = 6 * t - 4.0f;
    const float w4 = 6 * t - 2.0f;

    // weighted combination
    return a * w1 + b * w2 + t1 * w3 + t2 * w4;
}

inline float Log(float base, float x)
{
    // calculate the log of a value for an arbitary base, only use if you can't
    // use the standard bases (10, e)
    return logf(x) / logf(base);
}

inline int Log2(int x)
{
    int n = 0;
    while (x >= 2)
    {
        ++n;
        x /= 2;
    }

    return n;
}

// function which maps a value to a range
template <typename T>
T RangeMap(T value, T lower, T upper)
{
    assert(upper >= lower);
    return (value - lower) / (upper - lower);
}

/**
 * @class Colour
 * @brief RGBA color representation with floating point components.
 * @details
 * This class represents a color using red, green, blue, and alpha components as
 * floating point values. It provides various constructors for different color
 * representations and arithmetic operators for color manipulation.
 *
 * Color components are typically in the range [0.0, 1.0], though values outside
 * this range are supported for HDR operations.
 */
class Colour
{
public:
    /**
     * @brief Predefined color presets.
     * @details Enumeration of common colors for convenience.
     */
    enum Preset
    {
        /**
         * @brief Pure red color (1.0, 0.0, 0.0, 1.0).
         */
        kRed,
        /**
         * @brief Pure green color (0.0, 1.0, 0.0, 1.0).
         */
        kGreen,
        /**
         * @brief Pure blue color (0.0, 0.0, 1.0, 1.0).
         */
        kBlue,
        /**
         * @brief Pure white color (1.0, 1.0, 1.0, 1.0).
         */
        kWhite,
        /**
         * @brief Pure black color (0.0, 0.0, 0.0, 1.0).
         */
        kBlack
    };

    /**
     * @brief Default constructor with optional RGBA values.
     * @param[in] r_ Red component (default: 0.0)
     * @param[in] g_ Green component (default: 0.0)
     * @param[in] b_ Blue component (default: 0.0)
     * @param[in] a_ Alpha component (default: 1.0)
     */
    Colour(float r_ = 0.0f, float g_ = 0.0f, float b_ = 0.0f, float a_ = 1.0f) : r(r_), g(g_), b(b_), a(a_)
    {
    }
    /**
     * @brief Constructor from float array.
     * @param[in] p Array of 4 floats representing RGBA values
     */
    Colour(float* p) : r(p[0]), g(p[1]), b(p[2]), a(p[3])
    {
    }
    /**
     * @brief Constructor from packed 32-bit RGBA value.
     * @param[in] rgba Packed RGBA value in format 0xRRGGBBAA
     */
    Colour(uint32_t rgba)
    {
        a = ((rgba)&0xff) / 255.0f;
        r = ((rgba >> 24) & 0xff) / 255.0f;
        g = ((rgba >> 16) & 0xff) / 255.0f;
        b = ((rgba >> 8) & 0xff) / 255.0f;
    }
    /**
     * @brief Constructor from color preset.
     * @param[in] p Predefined color preset
     */
    Colour(Preset p)
    {
        switch (p)
        {
        case kRed:
            *this = Colour(1.0f, 0.0f, 0.0f);
            break;
        case kGreen:
            *this = Colour(0.0f, 1.0f, 0.0f);
            break;
        case kBlue:
            *this = Colour(0.0f, 0.0f, 1.0f);
            break;
        case kWhite:
            *this = Colour(1.0f, 1.0f, 1.0f);
            break;
        case kBlack:
            *this = Colour(0.0f, 0.0f, 0.0f);
            break;
        };
    }

    /**
     * @brief Conversion operator to const float pointer.
     * @return Pointer to the first color component
     */
    operator const float*() const
    {
        return &r;
    }
    /**
     * @brief Conversion operator to float pointer.
     * @return Pointer to the first color component
     */
    operator float*()
    {
        return &r;
    }

    /**
     * @brief Scalar multiplication operator.
     * @param[in] scale Scalar value to multiply all components by
     * @return New color with scaled components
     */
    Colour operator*(float scale) const
    {
        Colour r(*this);
        r *= scale;
        return r;
    }
    /**
     * @brief Scalar division operator.
     * @param[in] scale Scalar value to divide all components by
     * @return New color with divided components
     */
    Colour operator/(float scale) const
    {
        Colour r(*this);
        r /= scale;
        return r;
    }
    /**
     * @brief Color addition operator.
     * @param[in] v Color to add
     * @return New color with added components
     */
    Colour operator+(const Colour& v) const
    {
        Colour r(*this);
        r += v;
        return r;
    }
    /**
     * @brief Color subtraction operator.
     * @param[in] v Color to subtract
     * @return New color with subtracted components
     */
    Colour operator-(const Colour& v) const
    {
        Colour r(*this);
        r -= v;
        return r;
    }
    /**
     * @brief Component-wise color multiplication operator.
     * @param[in] scale Color to multiply with component-wise
     * @return New color with multiplied components
     */
    Colour operator*(const Colour& scale) const
    {
        Colour r(*this);
        r *= scale;
        return r;
    }

    /**
     * @brief In-place scalar multiplication operator.
     * @param[in] scale Scalar value to multiply all components by
     * @return Reference to this color after multiplication
     */
    Colour& operator*=(float scale)
    {
        r *= scale;
        g *= scale;
        b *= scale;
        a *= scale;
        return *this;
    }
    /**
     * @brief In-place scalar division operator.
     * @param[in] scale Scalar value to divide all components by
     * @return Reference to this color after division
     */
    Colour& operator/=(float scale)
    {
        float s(1.0f / scale);
        r *= s;
        g *= s;
        b *= s;
        a *= s;
        return *this;
    }
    /**
     * @brief In-place color addition operator.
     * @param[in] v Color to add
     * @return Reference to this color after addition
     */
    Colour& operator+=(const Colour& v)
    {
        r += v.r;
        g += v.g;
        b += v.b;
        a += v.a;
        return *this;
    }
    /**
     * @brief In-place color subtraction operator.
     * @param[in] v Color to subtract
     * @return Reference to this color after subtraction
     */
    Colour& operator-=(const Colour& v)
    {
        r -= v.r;
        g -= v.g;
        b -= v.b;
        a -= v.a;
        return *this;
    }
    /**
     * @brief In-place component-wise color multiplication operator.
     * @param[in] v Color to multiply with component-wise
     * @return Reference to this color after multiplication
     */
    Colour& operator*=(const Colour& v)
    {
        r *= v.r;
        g *= v.g;
        b *= v.b;
        a *= v.a;
        return *this;
    }

    /**
     * @brief Red color component.
     */
    float r;
    /**
     * @brief Green color component.
     */
    float g;
    /**
     * @brief Blue color component.
     */
    float b;
    /**
     * @brief Alpha (transparency) component.
     */
    float a;
};

inline bool operator==(const Colour& lhs, const Colour& rhs)
{
    return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b && lhs.a == rhs.a;
}

inline bool operator!=(const Colour& lhs, const Colour& rhs)
{
    return !(lhs == rhs);
}

inline Colour ToneMap(const Colour& s)
{
    // return Colour(s.r / (s.r+1.0f),	s.g / (s.g+1.0f), s.b /
    // (s.b+1.0f), 1.0f);
    float Y = 0.3333f * (s.r + s.g + s.b);
    return s / (1.0f + Y);
}

// lhs scalar scale
inline Colour operator*(float lhs, const Colour& rhs)
{
    Colour r(rhs);
    r *= lhs;
    return r;
}

inline Colour YxyToXYZ(float Y, float x, float y)
{
    float X = x * (Y / y);
    float Z = (1.0f - x - y) * Y / y;

    return Colour(X, Y, Z, 1.0f);
}

inline Colour HSVToRGB(float h, float s, float v)
{
    float r, g, b;

    int i;
    float f, p, q, t;
    if (s == 0)
    {
        // achromatic (grey)
        r = g = b = v;
    }
    else
    {
        h *= 6.0f; // sector 0 to 5
        i = int(floor(h));
        f = h - i; // factorial part of h
        p = v * (1 - s);
        q = v * (1 - s * f);
        t = v * (1 - s * (1 - f));
        switch (i)
        {
        case 0:
            r = v;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = v;
            b = p;
            break;
        case 2:
            r = p;
            g = v;
            b = t;
            break;
        case 3:
            r = p;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = p;
            b = v;
            break;
        default: // case 5:
            r = v;
            g = p;
            b = q;
            break;
        };
    }

    return Colour(r, g, b);
}

inline Colour XYZToLinear(float x, float y, float z)
{
    float c[4];
    c[0] = 3.240479f * x + -1.537150f * y + -0.498535f * z;
    c[1] = -0.969256f * x + 1.875991f * y + 0.041556f * z;
    c[2] = 0.055648f * x + -0.204043f * y + 1.057311f * z;
    c[3] = 1.0f;

    return Colour(c);
}

inline uint32_t ColourToRGBA8(const Colour& c)
{
    union SmallColor
    {
        uint8_t u8[4];
        uint32_t u32;
    };

    SmallColor s;
    s.u8[0] = (uint8_t)(Clamp(c.r, 0.0f, 1.0f) * 255);
    s.u8[1] = (uint8_t)(Clamp(c.g, 0.0f, 1.0f) * 255);
    s.u8[2] = (uint8_t)(Clamp(c.b, 0.0f, 1.0f) * 255);
    s.u8[3] = (uint8_t)(Clamp(c.a, 0.0f, 1.0f) * 255);

    return s.u32;
}

inline Colour LinearToSrgb(const Colour& c)
{
    const float kInvGamma = 1.0f / 2.2f;
    return Colour(powf(c.r, kInvGamma), powf(c.g, kInvGamma), powf(c.b, kInvGamma), c.a);
}

inline Colour SrgbToLinear(const Colour& c)
{
    const float kInvGamma = 2.2f;
    return Colour(powf(c.r, kInvGamma), powf(c.g, kInvGamma), powf(c.b, kInvGamma), c.a);
}

inline float SpecularRoughnessToExponent(float roughness, float maxExponent = 2048.0f)
{
    return powf(maxExponent, 1.0f - roughness);
}

inline float SpecularExponentToRoughness(float exponent, float maxExponent = 2048.0f)
{
    if (exponent <= 1.0f)
        return 1.0f;
    else
        return 1.0f - logf(exponent) / logf(maxExponent);
}

inline Colour JetColorMap(float low, float high, float x)
{
    float t = (x - low) / (high - low);

    return HSVToRGB(t, 1.0f, 1.0f);
}

inline Colour BourkeColorMap(float low, float high, float v)
{
    Colour c(1.0f, 1.0f, 1.0f); // white
    float dv;

    if (v < low)
        v = low;
    if (v > high)
        v = high;
    dv = high - low;

    if (v < (low + 0.25f * dv))
    {
        c.r = 0.f;
        c.g = 4.f * (v - low) / dv;
    }
    else if (v < (low + 0.5f * dv))
    {
        c.r = 0.f;
        c.b = 1.f + 4.f * (low + 0.25f * dv - v) / dv;
    }
    else if (v < (low + 0.75f * dv))
    {
        c.r = 4.f * (v - low - 0.5f * dv) / dv;
        c.b = 0.f;
    }
    else
    {
        c.g = 1.f + 4.f * (low + 0.75f * dv - v) / dv;
        c.b = 0.f;
    }

    return (c);
}

// intersection routines
inline bool IntersectRaySphere(const Point3& sphereOrigin,
                               float sphereRadius,
                               const Point3& rayOrigin,
                               const Vec3& rayDir,
                               float& t,
                               Vec3* hitNormal = nullptr)
{
    Vec3 d(sphereOrigin - rayOrigin);
    float deltaSq = LengthSq(d);
    float radiusSq = sphereRadius * sphereRadius;

    // if the origin is inside the sphere return no intersection
    if (deltaSq > radiusSq)
    {
        float dprojr = Dot(d, rayDir);

        // if ray pointing away from sphere no intersection
        if (dprojr < 0.0f)
            return false;

        // bit of Pythagoras to get closest point on ray
        float dSq = deltaSq - dprojr * dprojr;

        if (dSq > radiusSq)
            return false;
        else
        {
            // length of the half cord
            float thc = sqrt(radiusSq - dSq);

            // closest intersection
            t = dprojr - thc;

            // calculate normal if requested
            if (hitNormal)
                *hitNormal = Normalize((rayOrigin + rayDir * t) - sphereOrigin);

            return true;
        }
    }
    else
    {
        return false;
    }
}

template <typename T>
CUDA_CALLABLE inline bool SolveQuadratic(T a, T b, T c, T& minT, T& maxT)
{
    if (a == 0.0f && b == 0.0f)
    {
        minT = maxT = 0.0f;
        return true;
    }

    T discriminant = b * b - T(4.0) * a * c;

    if (discriminant < 0.0f)
    {
        return false;
    }

    // numerical receipes 5.6 (this method ensures numerical accuracy is
    // preserved)
    T t = T(-0.5) * (b + Sign(b) * Sqrt(discriminant));
    minT = t / a;
    maxT = c / t;

    if (minT > maxT)
    {
        Swap(minT, maxT);
    }

    return true;
}

// alternative ray sphere intersect, returns closest and furthest t values
inline bool IntersectRaySphere(const Point3& sphereOrigin,
                               float sphereRadius,
                               const Point3& rayOrigin,
                               const Vector3& rayDir,
                               float& minT,
                               float& maxT,
                               Vec3* hitNormal = nullptr)
{
    Vector3 q = rayOrigin - sphereOrigin;

    float a = 1.0f;
    float b = 2.0f * Dot(q, rayDir);
    float c = Dot(q, q) - (sphereRadius * sphereRadius);

    bool r = SolveQuadratic(a, b, c, minT, maxT);

    if (minT < 0.0)
        minT = 0.0f;

    // calculate the normal of the closest hit
    if (hitNormal && r)
    {
        *hitNormal = Normalize((rayOrigin + rayDir * minT) - sphereOrigin);
    }

    return r;
}

CUDA_CALLABLE inline bool IntersectRayPlane(const Point3& p, const Vector3& dir, const Plane& plane, float& t)
{
    float d = Dot(plane, dir);

    if (d == 0.0f)
    {
        return false;
    }
    else
    {
        t = -Dot(plane, p) / d;
    }

    return (t > 0.0f);
}

CUDA_CALLABLE inline bool IntersectLineSegmentPlane(const Vec3& start, const Vec3& end, const Plane& plane, Vec3& out)
{
    Vec3 u(end - start);

    float dist = -Dot(plane, start) / Dot(plane, u);

    if (dist > 0.0f && dist < 1.0f)
    {
        out = (start + u * dist);
        return true;
    }
    else
        return false;
}

// Moller and Trumbore's method
CUDA_CALLABLE inline bool IntersectRayTriTwoSided(const Vec3& p,
                                                  const Vec3& dir,
                                                  const Vec3& a,
                                                  const Vec3& b,
                                                  const Vec3& c,
                                                  float& t,
                                                  float& u,
                                                  float& v,
                                                  float& w,
                                                  float& sign,
                                                  Vec3* normal)
{
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 n = Cross(ab, ac);

    float d = Dot(-dir, n);
    float ood = 1.0f / d; // No need to check for division by zero here as
                          // infinity aritmetic will save us...
    Vector3 ap = p - a;

    t = Dot(ap, n) * ood;
    if (t < 0.0f)
        return false;

    Vector3 e = Cross(-dir, ap);
    v = Dot(ac, e) * ood;
    if (v < 0.0f || v > 1.0f) // ...here...
        return false;
    w = -Dot(ab, e) * ood;
    if (w < 0.0f || v + w > 1.0f) // ...and here
        return false;

    u = 1.0f - v - w;
    if (normal)
        *normal = n;

    sign = d;

    return true;
}

// mostly taken from Real Time Collision Detection - p192
inline bool IntersectRayTri(const Point3& p,
                            const Vec3& dir,
                            const Point3& a,
                            const Point3& b,
                            const Point3& c,
                            float& t,
                            float& u,
                            float& v,
                            float& w,
                            Vec3* normal)
{
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;

    // calculate normal
    Vec3 n = Cross(ab, ac);

    // need to solve a system of three equations to give t, u, v
    float d = Dot(-dir, n);

    // if dir is parallel to triangle plane or points away from triangle
    if (d <= 0.0f)
        return false;

    Vec3 ap = p - a;
    t = Dot(ap, n);

    // ignores tris behind
    if (t < 0.0f)
        return false;

    // compute barycentric coordinates
    Vec3 e = Cross(-dir, ap);
    v = Dot(ac, e);
    if (v < 0.0f || v > d)
        return false;

    w = -Dot(ab, e);
    if (w < 0.0f || v + w > d)
        return false;

    float ood = 1.0f / d;
    t *= ood;
    v *= ood;
    w *= ood;
    u = 1.0f - v - w;

    // optionally write out normal (todo: this branch is a performance concern,
    // should probably remove)
    if (normal)
        *normal = n;

    return true;
}

// mostly taken from Real Time Collision Detection - p192
CUDA_CALLABLE inline bool IntersectSegmentTri(const Vec3& p,
                                              const Vec3& q,
                                              const Vec3& a,
                                              const Vec3& b,
                                              const Vec3& c,
                                              float& t,
                                              float& u,
                                              float& v,
                                              float& w,
                                              Vec3* normal)
{
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 qp = p - q;

    // calculate normal
    Vec3 n = Cross(ab, ac);

    // need to solve a system of three equations to give t, u, v
    float d = Dot(qp, n);

    // if dir is parallel to triangle plane or points away from triangle
    // if (d <= 0.0f)
    //  return false;

    Vec3 ap = p - a;
    t = Dot(ap, n);

    // ignores tris behind
    if (t < 0.0f)
        return false;

    // ignores tris beyond segment
    if (t > d)
        return false;

    // compute barycentric coordinates
    Vec3 e = Cross(qp, ap);
    v = Dot(ac, e);
    if (v < 0.0f || v > d)
        return false;

    w = -Dot(ab, e);
    if (w < 0.0f || v + w > d)
        return false;

    float ood = 1.0f / d;
    t *= ood;
    v *= ood;
    w *= ood;
    u = 1.0f - v - w;

    // optionally write out normal (todo: this branch is a performance concern,
    // should probably remove)
    if (normal)
        *normal = n;

    return true;
}

CUDA_CALLABLE inline float ScalarTriple(const Vec3& a, const Vec3& b, const Vec3& c)
{
    return Dot(Cross(a, b), c);
}

// intersects a line (through points p and q, against a triangle a, b, c -
// mostly taken from Real Time Collision Detection - p186
CUDA_CALLABLE inline bool IntersectLineTri(const Vec3& p,
                                           const Vec3& q,
                                           const Vec3& a,
                                           const Vec3& b,
                                           const Vec3& c) //,  float& t, float& u, float& v, float&
                                                          // w, Vec3* normal, float expand)
{
    const Vec3 pq = q - p;
    const Vec3 pa = a - p;
    const Vec3 pb = b - p;
    const Vec3 pc = c - p;

    Vec3 m = Cross(pq, pc);
    float u = Dot(pb, m);
    if (u < 0.0f)
        return false;

    float v = -Dot(pa, m);
    if (v < 0.0f)
        return false;

    float w = ScalarTriple(pq, pb, pa);
    if (w < 0.0f)
        return false;

    return true;
}

CUDA_CALLABLE inline Vec3 ClosestPointToAABB(const Vec3& p, const Vec3& lower, const Vec3& upper)
{
    Vec3 c;

    for (int i = 0; i < 3; ++i)
    {
        float v = p[i];
        if (v < lower[i])
            v = lower[i];
        if (v > upper[i])
            v = upper[i];
        c[i] = v;
    }

    return c;
}

// RTCD 5.1.5, page 142
CUDA_CALLABLE inline Vec3 ClosestPointOnTriangle(
    const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p, float& v, float& w)
{
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ap = p - a;

    float d1 = Dot(ab, ap);
    float d2 = Dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        v = 0.0f;
        w = 0.0f;
        return a;
    }

    Vec3 bp = p - b;
    float d3 = Dot(ab, bp);
    float d4 = Dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
    {
        v = 1.0f;
        w = 0.0f;
        return b;
    }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        v = d1 / (d1 - d3);
        w = 0.0f;
        return a + v * ab;
    }

    Vec3 cp = p - c;
    float d5 = Dot(ab, cp);
    float d6 = Dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
    {
        v = 0.0f;
        w = 1.0f;
        return c;
    }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        v = 0.0f;
        w = d2 / (d2 - d6);
        return a + w * ac;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        v = 1.0f - w;
        return b + w * (c - b);
    }

    float denom = 1.0f / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    return a + ab * v + ac * w;
}

CUDA_CALLABLE inline Vec3 ClosestPointOnFatTriangle(
    const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p, const float thickness, float& v, float& w)
{
    const Vec3 x = ClosestPointOnTriangle(a, b, c, p, v, w);

    const Vec3 d = SafeNormalize(p - x);

    // apply thickness along delta dir
    return x + d * thickness;
}

// computes intersection between a ray and a triangle expanded by a constant
// thickness, also works for ray-sphere and ray-capsule this is an iterative
// method similar to sphere tracing but for convex objects, see
// http://dtecta.com/papers/jgt04raycast.pdf
CUDA_CALLABLE inline bool IntersectRayFatTriangle(const Vec3& p,
                                                  const Vec3& dir,
                                                  const Vec3& a,
                                                  const Vec3& b,
                                                  const Vec3& c,
                                                  float thickness,
                                                  float threshold,
                                                  float maxT,
                                                  float& t,
                                                  float& u,
                                                  float& v,
                                                  float& w,
                                                  Vec3* normal)
{
    t = 0.0f;
    Vec3 x = p;
    Vec3 n;
    float distSq;

    const float thresholdSq = threshold * threshold;
    const int maxIterations = 20;

    for (int i = 0; i < maxIterations; ++i)
    {
        const Vec3 closestPoint = ClosestPointOnFatTriangle(a, b, c, x, thickness, v, w);

        n = x - closestPoint;
        distSq = LengthSq(n);

        // early out
        if (distSq <= thresholdSq)
            break;

        float ndir = Dot(n, dir);

        // we've gone past the convex
        if (ndir >= 0.0f)
            return false;

        // we've exceeded max ray length
        if (t > maxT)
            return false;

        t = t - distSq / ndir;
        x = p + t * dir;
    }

    // calculate normal based on unexpanded geometry to avoid precision issues
    Vec3 cp = ClosestPointOnTriangle(a, b, c, x, v, w);
    n = x - cp;

    // if n faces away due to numerical issues flip it to face ray dir
    if (Dot(n, dir) > 0.0f)
        n *= -1.0f;

    u = 1.0f - v - w;
    *normal = SafeNormalize(n);

    return true;
}

CUDA_CALLABLE inline float SqDistPointSegment(Vec3 a, Vec3 b, Vec3 c)
{
    Vec3 ab = b - a, ac = c - a, bc = c - b;
    float e = Dot(ac, ab);

    if (e <= 0.0f)
        return Dot(ac, ac);
    float f = Dot(ab, ab);

    if (e >= f)
        return Dot(bc, bc);

    return Dot(ac, ac) - e * e / f;
}

CUDA_CALLABLE inline bool PointInTriangle(Vec3 a, Vec3 b, Vec3 c, Vec3 p)
{
    a -= p;
    b -= p;
    c -= p;

    /*
    float eps = 0.0f;

    float ab = Dot(a, b);
    float ac = Dot(a, c);
    float bc = Dot(b, c);
    float cc = Dot(c, c);

    if (bc *ac - cc * ab <= eps)
        return false;

    float bb = Dot(b, b);
    if (ab * bc - ac*bb <= eps)
        return false;

    return true;
    */

    Vec3 u = Cross(b, c);
    Vec3 v = Cross(c, a);

    if (Dot(u, v) <= 0.0f)
        return false;

    Vec3 w = Cross(a, b);

    if (Dot(u, w) <= 0.0f)
        return false;

    return true;
}

CUDA_CALLABLE inline void ClosestPointBetweenLineSegments(
    const Vec3& p, const Vec3& q, const Vec3& r, const Vec3& s, float& u, float& v)
{
    Vec3 d1 = q - p;
    Vec3 d2 = s - r;
    Vec3 rp = p - r;
    float a = Dot(d1, d1);
    float c = Dot(d1, rp);
    float e = Dot(d2, d2);
    float f = Dot(d2, rp);

    float b = Dot(d1, d2);
    float denom = a * e - b * b;
    if (denom != 0.0f)
        u = Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
    else
    {
        u = 0.0f;
    }

    v = (b * u + f) / e;

    if (v < 0.0f)
    {
        v = 0.0f;
        u = Clamp(-c / a, 0.0f, 1.0f);
    }
    else if (v > 1.0f)
    {
        v = 1.0f;
        u = Clamp((b - c) / a, 0.0f, 1.0f);
    }
}

CUDA_CALLABLE inline float ClosestPointBetweenLineSegmentAndTri(
    const Vec3& p, const Vec3& q, const Vec3& a, const Vec3& b, const Vec3& c, float& outT, float& outV, float& outW)
{
    float minDsq = FLT_MAX;
    float minT, minV, minW;

    float t, u, v, w, dSq;
    Vec3 r, s;

    // test if line segment intersects tri
    if (IntersectSegmentTri(p, q, a, b, c, t, u, v, w, nullptr))
    {
        outT = t;
        outV = v;
        outW = w;

        return 0.0f;
    }

    // edge ab
    ClosestPointBetweenLineSegments(p, q, a, b, t, v);
    r = p + (q - p) * t;
    s = a + (b - a) * v;
    dSq = LengthSq(r - s);

    if (dSq < minDsq)
    {
        minDsq = dSq;
        minT = u;

        // minU = 1.0f-v
        minV = v;
        minW = 0.0f;
    }

    // edge bc
    ClosestPointBetweenLineSegments(p, q, b, c, t, w);
    r = p + (q - p) * t;
    s = b + (c - b) * w;
    dSq = LengthSq(r - s);

    if (dSq < minDsq)
    {
        minDsq = dSq;
        minT = t;

        // minU = 0.0f;
        minV = 1.0f - w;
        minW = w;
    }

    // edge ca
    ClosestPointBetweenLineSegments(p, q, c, a, t, u);
    r = p + (q - p) * t;
    s = c + (a - c) * u;
    dSq = LengthSq(r - s);

    if (dSq < minDsq)
    {
        minDsq = dSq;
        minT = t;

        // minU = u;
        minV = 0.0f;
        minW = 1.0f - u;
    }

    // end point p
    ClosestPointOnTriangle(a, b, c, p, v, w);
    s = a * (1.0f - v - w) + b * v + c * w;
    dSq = LengthSq(s - p);

    if (dSq < minDsq)
    {
        minDsq = dSq;
        minT = 0.0f;
        minV = v;
        minW = w;
    }

    // end point q
    ClosestPointOnTriangle(a, b, c, q, v, w);
    s = a * (1.0f - v - w) + b * v + c * w;
    dSq = LengthSq(s - q);

    if (dSq < minDsq)
    {
        minDsq = dSq;
        minT = 1.0f;
        minV = v;
        minW = w;
    }

    // write mins
    outT = minT;
    outV = minV;
    outW = minW;

    return sqrtf(minDsq);
}

CUDA_CALLABLE inline float minf(const float a, const float b)
{
    return a < b ? a : b;
}
CUDA_CALLABLE inline float maxf(const float a, const float b)
{
    return a > b ? a : b;
}

CUDA_CALLABLE inline bool IntersectRayAABBFast(
    const Vec3& pos, const Vector3& rcp_dir, const Vector3& min, const Vector3& max, float& t)
{

    float l1 = (min.x - pos.x) * rcp_dir.x, l2 = (max.x - pos.x) * rcp_dir.x, lmin = minf(l1, l2), lmax = maxf(l1, l2);

    l1 = (min.y - pos.y) * rcp_dir.y;
    l2 = (max.y - pos.y) * rcp_dir.y;
    lmin = maxf(minf(l1, l2), lmin);
    lmax = minf(maxf(l1, l2), lmax);

    l1 = (min.z - pos.z) * rcp_dir.z;
    l2 = (max.z - pos.z) * rcp_dir.z;
    lmin = maxf(minf(l1, l2), lmin);
    lmax = minf(maxf(l1, l2), lmax);

    // return ((lmax > 0.f) & (lmax >= lmin));
    // return ((lmax > 0.f) & (lmax > lmin));
    bool hit = ((lmax >= 0.f) && (lmax >= lmin));
    if (hit)
        t = lmin;
    return hit;
}

CUDA_CALLABLE inline bool IntersectRayAABB(
    const Vec3& start, const Vector3& dir, const Vector3& min, const Vector3& max, float& t, Vector3* normal)
{
    //! calculate candidate plane on each axis
    float tx = -1.0f, ty = -1.0f, tz = -1.0f;
    bool inside = true;

    //! use unrolled loops

    //! x
    if (start.x < min.x)
    {
        if (dir.x != 0.0f)
            tx = (min.x - start.x) / dir.x;
        inside = false;
    }
    else if (start.x > max.x)
    {
        if (dir.x != 0.0f)
            tx = (max.x - start.x) / dir.x;
        inside = false;
    }

    //! y
    if (start.y < min.y)
    {
        if (dir.y != 0.0f)
            ty = (min.y - start.y) / dir.y;
        inside = false;
    }
    else if (start.y > max.y)
    {
        if (dir.y != 0.0f)
            ty = (max.y - start.y) / dir.y;
        inside = false;
    }

    //! z
    if (start.z < min.z)
    {
        if (dir.z != 0.0f)
            tz = (min.z - start.z) / dir.z;
        inside = false;
    }
    else if (start.z > max.z)
    {
        if (dir.z != 0.0f)
            tz = (max.z - start.z) / dir.z;
        inside = false;
    }

    //! if point inside all planes
    if (inside)
    {
        t = 0.0f;
        return true;
    }

    //! we now have t values for each of possible intersection planes
    //! find the maximum to get the intersection point
    float tmax = tx;
    int taxis = 0;

    if (ty > tmax)
    {
        tmax = ty;
        taxis = 1;
    }
    if (tz > tmax)
    {
        tmax = tz;
        taxis = 2;
    }

    if (tmax < 0.0f)
        return false;

    //! check that the intersection point lies on the plane we picked
    //! we don't test the axis of closest intersection for precision reasons

    //! no eps for now
    float eps = 0.0f;

    Vec3 hit = start + dir * tmax;

    if ((hit.x < min.x - eps || hit.x > max.x + eps) && taxis != 0)
        return false;
    if ((hit.y < min.y - eps || hit.y > max.y + eps) && taxis != 1)
        return false;
    if ((hit.z < min.z - eps || hit.z > max.z + eps) && taxis != 2)
        return false;

    //! output results
    t = tmax;

    return true;
}

// construct a plane equation such that ax + by + cz + dw = 0
CUDA_CALLABLE inline Vec4 PlaneFromPoints(const Vec3& p, const Vec3& q, const Vec3& r)
{
    Vec3 e0 = q - p;
    Vec3 e1 = r - p;

    Vec3 n = SafeNormalize(Cross(e0, e1));

    return Vec4(n.x, n.y, n.z, -Dot(p, n));
}

CUDA_CALLABLE inline bool IntersectPlaneAABB(const Vec4& plane, const Vec3& center, const Vec3& extents)
{
    float radius = Abs(extents.x * plane.x) + Abs(extents.y * plane.y) + Abs(extents.z * plane.z);
    float delta = Dot(center, Vec3(plane)) + plane.w;

    return Abs(delta) <= radius;
}

/**
 * @class Rect
 * @brief 2D rectangle representation using integer coordinates.
 * @details
 * This class represents a 2D rectangle defined by left, right, top, and bottom
 * boundaries using integer coordinates. It provides methods for computing dimensions,
 * expanding the rectangle, and testing point containment.
 *
 * The coordinate system assumes that left < right and top < bottom.
 */
class Rect
{
public:
    /**
     * @brief Default constructor creating a zero-sized rectangle.
     * @details Creates a rectangle with all boundaries set to zero.
     */
    Rect() : m_left(0), m_right(0), m_top(0), m_bottom(0)
    {
    }

    /**
     * @brief Constructor with explicit boundary values.
     * @param[in] left Left boundary coordinate
     * @param[in] right Right boundary coordinate
     * @param[in] top Top boundary coordinate
     * @param[in] bottom Bottom boundary coordinate
     * @pre left <= right and top <= bottom
     */
    Rect(uint32_t left, uint32_t right, uint32_t top, uint32_t bottom)
        : m_left(left), m_right(right), m_top(top), m_bottom(bottom)
    {
        assert(left <= right);
        assert(top <= bottom);
    }

    /**
     * @brief Computes the width of the rectangle.
     * @return Width as the difference between right and left boundaries
     */
    uint32_t Width() const
    {
        return m_right - m_left;
    }
    /**
     * @brief Computes the height of the rectangle.
     * @return Height as the difference between bottom and top boundaries
     */
    uint32_t Height() const
    {
        return m_bottom - m_top;
    }

    /**
     * @brief Expands the rectangle by a uniform amount in all directions.
     * @param[in] x Amount to expand by in each direction
     * @warning This operation may cause underflow if x is larger than the boundaries.
     */
    void Expand(uint32_t x)
    {
        m_left -= x;
        m_right += x;
        m_top -= x;
        m_bottom += x;
    }

    /**
     * @brief Gets the left boundary coordinate.
     * @return Left boundary value
     */
    uint32_t Left() const
    {
        return m_left;
    }
    /**
     * @brief Gets the right boundary coordinate.
     * @return Right boundary value
     */
    uint32_t Right() const
    {
        return m_right;
    }
    /**
     * @brief Gets the top boundary coordinate.
     * @return Top boundary value
     */
    uint32_t Top() const
    {
        return m_top;
    }
    /**
     * @brief Gets the bottom boundary coordinate.
     * @return Bottom boundary value
     */
    uint32_t Bottom() const
    {
        return m_bottom;
    }

    /**
     * @brief Tests if a point is contained within the rectangle.
     * @param[in] x X-coordinate of the point to test
     * @param[in] y Y-coordinate of the point to test
     * @return True if the point is inside or on the boundary of the rectangle, false otherwise
     */
    bool Contains(uint32_t x, uint32_t y) const
    {
        return (x >= m_left) && (x <= m_right) && (y >= m_top) && (y <= m_bottom);
    }

    /**
     * @brief Left boundary coordinate.
     */
    uint32_t m_left;
    /**
     * @brief Right boundary coordinate.
     */
    uint32_t m_right;
    /**
     * @brief Top boundary coordinate.
     */
    uint32_t m_top;
    /**
     * @brief Bottom boundary coordinate.
     */
    uint32_t m_bottom;
};

// doesn't really belong here but efficient (and I believe correct) in place
// random shuffle based on the Fisher-Yates / Knuth algorithm
template <typename T>
void RandomShuffle(T begin, T end)
{
    assert(end > begin);
    uint32_t n = distance(begin, end);

    for (uint32_t i = 0; i < n; ++i)
    {
        // pick a random number between 0 and n-1
        uint32_t r = Rand() % (n - i);

        // swap that location with the current randomly selected position
        swap(*(begin + i), *(begin + (i + r)));
    }
}

CUDA_CALLABLE inline Quat QuatFromAxisAngle(const Vec3& axis, float angle)
{
    Vec3 v = Normalize(axis);

    float half = angle * 0.5f;
    float w = cosf(half);

    const float sin_theta_over_two = sinf(half);
    v *= sin_theta_over_two;

    return Quat(v.x, v.y, v.z, w);
}

// rotate by quaternion (q, w)
CUDA_CALLABLE inline Vec3 rotate(const Vec3& q, float w, const Vec3& x)
{
    return 2.0f * (x * (w * w - 0.5f) + Cross(q, x) * w + q * Dot(q, x));
}

// rotate x by inverse transform in (q, w)
CUDA_CALLABLE inline Vec3 rotateInv(const Vec3& q, float w, const Vec3& x)
{
    return 2.0f * (x * (w * w - 0.5f) - Cross(q, x) * w + q * Dot(q, x));
}

// get rotation from u to v
CUDA_CALLABLE inline Quat GetRotationQuat(const Vec3& _u, const Vec3& _v)
{
    Vec3 u = Normalize(_u);
    Vec3 v = Normalize(_v);

    // check for aligned vectors
    float d = Dot(u, v);
    if (d > 1.0f - 1e-6f)
    {
        // vectors are colinear, return identity
        return Quat();
    }
    else if (d < 1e-6f - 1.0f)
    {
        // vectors are opposite, return a 180 degree rotation
        Vec3 axis = Cross({ 1.0f, 0.0f, 0.0f }, u);
        if (LengthSq(axis) < 1e-6f)
        {
            axis = Cross({ 0.0f, 1.0f, 0.0f }, u);
        }
        return QuatFromAxisAngle(Normalize(axis), kPi);
    }
    else
    {
        Vec3 c = Cross(u, v);
        float s = sqrtf((1.0f + d) * 2.0f);
        float invs = 1.0f / s;
        Quat q(invs * c.x, invs * c.y, invs * c.z, 0.5f * s);
        return Normalize(q);
    }
}

CUDA_CALLABLE inline void TransformBounds(const Quat& q, Vec3 extents, Vec3& newExtents)
{
    Matrix33 transform(q);

    transform.cols[0] *= extents.x;
    transform.cols[1] *= extents.y;
    transform.cols[2] *= extents.z;

    float ex = fabsf(transform.cols[0].x) + fabsf(transform.cols[1].x) + fabsf(transform.cols[2].x);
    float ey = fabsf(transform.cols[0].y) + fabsf(transform.cols[1].y) + fabsf(transform.cols[2].y);
    float ez = fabsf(transform.cols[0].z) + fabsf(transform.cols[1].z) + fabsf(transform.cols[2].z);

    newExtents = Vec3(ex, ey, ez);
}

CUDA_CALLABLE inline void TransformBounds(const Vec3& localLower,
                                          const Vec3& localUpper,
                                          const Vec3& translation,
                                          const Quat& rotation,
                                          float scale,
                                          Vec3& lower,
                                          Vec3& upper)
{
    Matrix33 transform(rotation);

    Vec3 extents = (localUpper - localLower) * scale;

    transform.cols[0] *= extents.x;
    transform.cols[1] *= extents.y;
    transform.cols[2] *= extents.z;

    float ex = fabsf(transform.cols[0].x) + fabsf(transform.cols[1].x) + fabsf(transform.cols[2].x);
    float ey = fabsf(transform.cols[0].y) + fabsf(transform.cols[1].y) + fabsf(transform.cols[2].y);
    float ez = fabsf(transform.cols[0].z) + fabsf(transform.cols[1].z) + fabsf(transform.cols[2].z);

    Vec3 center = (localUpper + localLower) * 0.5f * scale;

    lower = rotation * center + translation - Vec3(ex, ey, ez) * 0.5f;
    upper = rotation * center + translation + Vec3(ex, ey, ez) * 0.5f;
}

// Poisson sample the volume of a sphere with given separation
inline int PoissonSample3D(float radius, float separation, Vec3* points, int maxPoints, int maxAttempts)
{
    // naive O(n^2) dart throwing algorithm to generate a Poisson distribution
    int c = 0;
    while (c < maxPoints)
    {
        int a = 0;
        while (a < maxAttempts)
        {
            const Vec3 p = UniformSampleSphereVolume() * radius;

            // test against points already generated
            int i = 0;
            for (; i < c; ++i)
            {
                Vec3 d = p - points[i];

                // reject if closer than separation
                if (LengthSq(d) < separation * separation)
                    break;
            }

            // sample passed all tests, accept
            if (i == c)
            {
                points[c] = p;
                ++c;
                break;
            }

            ++a;
        }

        // exit if we reached the max attempts and didn't manage to add a point
        if (a == maxAttempts)
            break;
    }

    return c;
}

inline int PoissonSampleBox3D(Vec3 lower, Vec3 upper, float separation, Vec3* points, int maxPoints, int maxAttempts)
{
    // naive O(n^2) dart throwing algorithm to generate a Poisson distribution
    int c = 0;
    while (c < maxPoints)
    {
        int a = 0;
        while (a < maxAttempts)
        {
            const Vec3 p = Vec3(Randf(lower.x, upper.x), Randf(lower.y, upper.y), Randf(lower.z, upper.z));

            // test against points already generated
            int i = 0;
            for (; i < c; ++i)
            {
                Vec3 d = p - points[i];

                // reject if closer than separation
                if (LengthSq(d) < separation * separation)
                    break;
            }

            // sample passed all tests, accept
            if (i == c)
            {
                points[c] = p;
                ++c;
                break;
            }

            ++a;
        }

        // exit if we reached the max attempts and didn't manage to add a point
        if (a == maxAttempts)
            break;
    }

    return c;
}

// Generates an optimally dense sphere packing at the origin (implicit sphere at
// the origin)
inline int TightPack3D(float radius, float separation, Vec3* points, int maxPoints)
{
    int dim = int(ceilf(radius / separation));

    int c = 0;

    for (int z = -dim; z <= dim; ++z)
    {
        for (int y = -dim; y <= dim; ++y)
        {
            for (int x = -dim; x <= dim; ++x)
            {
                float xpos = x * separation + (((y + z) & 1) ? separation * 0.5f : 0.0f);
                float ypos = y * sqrtf(0.75f) * separation;
                float zpos = z * sqrtf(0.75f) * separation;

                Vec3 p(xpos, ypos, zpos);

                // skip center
                if (LengthSq(p) == 0.0f)
                    continue;

                if (c < maxPoints && Length(p) <= radius)
                {
                    points[c] = p;
                    ++c;
                }
            }
        }
    }

    return c;
}

/**
 * @struct Bounds
 * @brief Axis-aligned bounding box representation using lower and upper corner points.
 * @details
 * This structure represents a 3D axis-aligned bounding box (AABB) defined by two
 * corner points: lower and upper. It provides methods for computing geometric properties,
 * expanding the bounds, and testing for overlaps with points and other bounds.
 *
 * The bounds can be in an empty state where lower > upper, which is useful for
 * initialization and union operations.
 */
struct Bounds
{
    /**
     * @brief Default constructor creating empty bounds.
     * @details
     * Initializes the bounds to an empty state where lower is set to positive infinity
     * and upper is set to negative infinity. This allows subsequent union operations
     * to work correctly.
     */
    CUDA_CALLABLE inline Bounds() : lower(FLT_MAX), upper(-FLT_MAX)
    {
    }

    /**
     * @brief Constructor creating bounds from lower and upper corner points.
     * @param[in] lower The lower corner point of the bounds
     * @param[in] upper The upper corner point of the bounds
     */
    CUDA_CALLABLE inline Bounds(const Vec3& lower, const Vec3& upper) : lower(lower), upper(upper)
    {
    }

    /**
     * @brief Computes the center point of the bounds.
     * @return The center point as a Vec3
     */
    CUDA_CALLABLE inline Vec3 GetCenter() const
    {
        return 0.5f * (lower + upper);
    }
    /**
     * @brief Computes the dimensions (extents) of the bounds.
     * @return The dimensions as a Vec3 representing width, height, and depth
     */
    CUDA_CALLABLE inline Vec3 GetEdges() const
    {
        return upper - lower;
    }

    /**
     * @brief Expands the bounds by a uniform amount in all directions.
     * @param[in] r The amount to expand by in all directions
     */
    CUDA_CALLABLE inline void Expand(float r)
    {
        lower -= Vec3(r);
        upper += Vec3(r);
    }

    /**
     * @brief Expands the bounds by different amounts in each direction.
     * @param[in] r The amounts to expand by in each direction (x, y, z)
     */
    CUDA_CALLABLE inline void Expand(const Vec3& r)
    {
        lower -= r;
        upper += r;
    }

    /**
     * @brief Checks if the bounds are empty.
     * @return True if the bounds are empty (lower >= upper in any dimension), false otherwise
     */
    CUDA_CALLABLE inline bool Empty() const
    {
        return lower.x >= upper.x || lower.y >= upper.y || lower.z >= upper.z;
    }

    /**
     * @brief Tests if a point overlaps with the bounds.
     * @param[in] p The point to test
     * @return True if the point is inside or on the boundary of the bounds, false otherwise
     */
    CUDA_CALLABLE inline bool Overlaps(const Vec3& p) const
    {
        if (p.x < lower.x || p.y < lower.y || p.z < lower.z || p.x > upper.x || p.y > upper.y || p.z > upper.z)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * @brief Tests if another bounds overlaps with this bounds.
     * @param[in] b The other bounds to test against
     * @return True if the bounds overlap or touch, false otherwise
     */
    CUDA_CALLABLE inline bool Overlaps(const Bounds& b) const
    {
        if (lower.x > b.upper.x || lower.y > b.upper.y || lower.z > b.upper.z || upper.x < b.lower.x ||
            upper.y < b.lower.y || upper.z < b.lower.z)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * @brief The lower corner point of the bounds.
     */
    Vec3 lower;
    /**
     * @brief The upper corner point of the bounds.
     */
    Vec3 upper;
};

CUDA_CALLABLE inline Bounds Union(const Bounds& a, const Vec3& b)
{
    return Bounds(Min(a.lower, b), Max(a.upper, b));
}

CUDA_CALLABLE inline Bounds Union(const Bounds& a, const Bounds& b)
{
    return Bounds(Min(a.lower, b.lower), Max(a.upper, b.upper));
}

CUDA_CALLABLE inline Bounds Intersection(const Bounds& a, const Bounds& b)
{
    return Bounds(Max(a.lower, b.lower), Min(a.upper, b.upper));
}

CUDA_CALLABLE inline float SurfaceArea(const Bounds& b)
{
    Vec3 e = b.upper - b.lower;
    return 2.0f * (e.x * e.y + e.x * e.z + e.y * e.z);
}

inline void ExtractFrustumPlanes(const Matrix44& m, Plane* planes)
{
    // Based on Fast Extraction of Viewing Frustum Planes from the
    // WorldView-Projection Matrix, Gill Grib, Klaus Hartmann

    // Left clipping plane
    planes[0].x = m(3, 0) + m(0, 0);
    planes[0].y = m(3, 1) + m(0, 1);
    planes[0].z = m(3, 2) + m(0, 2);
    planes[0].w = m(3, 3) + m(0, 3);

    // Right clipping plane
    planes[1].x = m(3, 0) - m(0, 0);
    planes[1].y = m(3, 1) - m(0, 1);
    planes[1].z = m(3, 2) - m(0, 2);
    planes[1].w = m(3, 3) - m(0, 3);

    // Top clipping plane
    planes[2].x = m(3, 0) - m(1, 0);
    planes[2].y = m(3, 1) - m(1, 1);
    planes[2].z = m(3, 2) - m(1, 2);
    planes[2].w = m(3, 3) - m(1, 3);

    // Bottom clipping plane
    planes[3].x = m(3, 0) + m(1, 0);
    planes[3].y = m(3, 1) + m(1, 1);
    planes[3].z = m(3, 2) + m(1, 2);
    planes[3].w = m(3, 3) + m(1, 3);

    // Near clipping plane
    planes[4].x = m(3, 0) + m(2, 0);
    planes[4].y = m(3, 1) + m(2, 1);
    planes[4].z = m(3, 2) + m(2, 2);
    planes[4].w = m(3, 3) + m(2, 3);

    // Far clipping plane
    planes[5].x = m(3, 0) - m(2, 0);
    planes[5].y = m(3, 1) - m(2, 1);
    planes[5].z = m(3, 2) - m(2, 2);
    planes[5].w = m(3, 3) - m(2, 3);

    NormalizePlane(planes[0]);
    NormalizePlane(planes[1]);
    NormalizePlane(planes[2]);
    NormalizePlane(planes[3]);
    NormalizePlane(planes[4]);
    NormalizePlane(planes[5]);
}

inline bool TestSphereAgainstFrustum(const Plane* planes, Vec3 center, float radius)
{
    for (int i = 0; i < 6; ++i)
    {
        float d = -Dot(planes[i], Point3(center)) - radius;

        if (d > 0.0f)
            return false;
    }

    return true;
}

inline float sign(float x)
{
    return x < 0 ? -1.0f : 1.0f;
}

inline int getClosestAxis(const Vec3& vec)
{
    Vec3 normalized = Normalize(vec);
    Vec3 axes[3] = { Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1) };

    int maxIdx = 0;
    float maxDot = std::abs(Dot(normalized, axes[0]));

    for (int i = 1; i < 3; ++i)
    {
        float dot = std::abs(Dot(normalized, axes[i]));
        if (dot > maxDot)
        {
            maxDot = dot;
            maxIdx = i;
        }
    }

    return maxIdx;
}

inline Quat quaternionFromVectors(const Vec3& vec1, const Vec3& vec2)
{
    Vec3 v1 = Normalize(vec1);
    Vec3 v2 = Normalize(vec2);

    float dot = Dot(v1, v2);

    if (dot > 0.999999f)
    {
        return Quat(1, 0, 0, 0);
    }

    if (dot < -0.999999f)
    {
        return Quat(0, 1, 0, 0);
    }

    Vec3 cross = Cross(v1, v2);
    float s = std::sqrt((1.0f + dot) * 2.0f);
    float invs = 1.0f / s;

    return Quat(s * 0.5f, cross.x * invs, cross.y * invs, cross.z * invs);
}

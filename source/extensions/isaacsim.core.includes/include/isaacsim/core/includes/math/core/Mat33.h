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
#include "Quat.h"
#include "Vec3.h"

/**
 * @struct Matrix33
 * @brief 3x3 matrix representation with column-major storage.
 * @details
 * This structure represents a 3x3 matrix stored in column-major order.
 * It provides matrix operations including arithmetic, element access,
 * construction from quaternions, and static identity matrix creation.
 *
 * The matrix is commonly used for 3D rotations, scaling, and other
 * linear transformations in 3D space.
 */
struct Matrix33
{
    /**
     * @brief Default constructor creating an uninitialized matrix.
     */
    CUDA_CALLABLE Matrix33()
    {
    }
    /**
     * @brief Constructor from row-major float array.
     * @param[in] ptr Pointer to array of 9 floats in row-major order
     * @details
     * Creates a matrix from a float array where elements are stored as:
     * [0,1,2, 3,4,5, 6,7,8] representing rows [0], [1], [2]
     */
    CUDA_CALLABLE Matrix33(const float* ptr)
    {
        cols[0].x = ptr[0];
        cols[0].y = ptr[1];
        cols[0].z = ptr[2];

        cols[1].x = ptr[3];
        cols[1].y = ptr[4];
        cols[1].z = ptr[5];

        cols[2].x = ptr[6];
        cols[2].y = ptr[7];
        cols[2].z = ptr[8];
    }
    /**
     * @brief Constructor from three column vectors.
     * @param[in] c1 First column vector
     * @param[in] c2 Second column vector
     * @param[in] c3 Third column vector
     */
    CUDA_CALLABLE Matrix33(const Vec3& c1, const Vec3& c2, const Vec3& c3)
    {
        cols[0] = c1;
        cols[1] = c2;
        cols[2] = c3;
    }

    /**
     * @brief Constructor from quaternion rotation.
     * @param[in] q Quaternion representing the rotation
     * @details
     * Creates a rotation matrix from the given quaternion by rotating
     * the standard basis vectors (1,0,0), (0,1,0), (0,0,1).
     */
    CUDA_CALLABLE Matrix33(const Quat& q)
    {
        cols[0] = Rotate(q, Vec3(1.0f, 0.0f, 0.0f));
        cols[1] = Rotate(q, Vec3(0.0f, 1.0f, 0.0f));
        cols[2] = Rotate(q, Vec3(0.0f, 0.0f, 1.0f));
    }

    /**
     * @brief Const element access operator.
     * @param[in] i Row index (0, 1, or 2)
     * @param[in] j Column index (0, 1, or 2)
     * @return Const reference to the matrix element at (i,j)
     */
    CUDA_CALLABLE float operator()(int i, int j) const
    {
        return static_cast<const float*>(cols[j])[i];
    }
    /**
     * @brief Mutable element access operator.
     * @param[in] i Row index (0, 1, or 2)
     * @param[in] j Column index (0, 1, or 2)
     * @return Mutable reference to the matrix element at (i,j)
     */
    CUDA_CALLABLE float& operator()(int i, int j)
    {
        return static_cast<float*>(cols[j])[i];
    }

    /**
     * @brief Column vectors of the matrix.
     * @details
     * The matrix is stored as three column vectors in column-major order.
     * cols[0], cols[1], cols[2] represent the first, second, and third columns respectively.
     */
    Vec3 cols[3];

    /**
     * @brief Creates a 3x3 identity matrix.
     * @return Identity matrix with 1s on the diagonal and 0s elsewhere
     */
    CUDA_CALLABLE static inline Matrix33 Identity()
    {
        const Matrix33 sIdentity(Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f));
        return sIdentity;
    }
};

/**
 * @brief Computes the L2 magnitude (Frobenius norm) of a matrix
 * @param[in] matrix Matrix to compute magnitude for
 * @return L2 magnitude of the matrix
 * @details Computes the square root of the sum of squares of all matrix elements
 */
CUDA_CALLABLE inline float L2_magnitude(const Matrix33& matrix)
{
    float sum = 0.0;
    for (int i = 0; i < 3; i++)
    {
        sum += matrix.cols[i].x * matrix.cols[i].x;
        sum += matrix.cols[i].y * matrix.cols[i].y;
        sum += matrix.cols[i].z * matrix.cols[i].z;
    }
    return sqrtf(sum);
}

/**
 * @brief Creates a diagonal matrix from a 3D vector
 * @param[in] v Vector containing diagonal elements
 * @return Diagonal matrix with v.x, v.y, v.z on the diagonal
 * @details Off-diagonal elements are set to zero
 */
CUDA_CALLABLE inline Matrix33 Diagonalize(const Vec3& v)
{
    Matrix33 r;
    r.cols[0].x = v.x;
    r.cols[1].y = v.y;
    r.cols[2].z = v.z;
    return r;
}

/**
 * @brief Multiplies a matrix by a scalar
 * @param[in] s Scalar multiplier
 * @param[in] m Matrix to multiply
 * @return Matrix with all elements multiplied by scalar
 */
CUDA_CALLABLE inline Matrix33 Multiply(float s, const Matrix33& m)
{
    Matrix33 r = m;
    r.cols[0] *= s;
    r.cols[1] *= s;
    r.cols[2] *= s;
    return r;
}

/**
 * @brief Multiplies a matrix by a vector
 * @param[in] a Matrix to multiply
 * @param[in] x Vector to multiply
 * @return Resulting vector from matrix-vector multiplication
 */
CUDA_CALLABLE inline Vec3 Multiply(const Matrix33& a, const Vec3& x)
{
    return a.cols[0] * x.x + a.cols[1] * x.y + a.cols[2] * x.z;
}

/**
 * @brief Matrix-vector multiplication operator
 * @param[in] a Matrix on the left
 * @param[in] x Vector on the right
 * @return Resulting vector from multiplication
 */
CUDA_CALLABLE inline Vec3 operator*(const Matrix33& a, const Vec3& x)
{
    return Multiply(a, x);
}

/**
 * @brief Multiplies two matrices
 * @param[in] a First matrix
 * @param[in] b Second matrix
 * @return Product matrix a * b
 */
CUDA_CALLABLE inline Matrix33 Multiply(const Matrix33& a, const Matrix33& b)
{
    Matrix33 r;
    r.cols[0] = a * b.cols[0];
    r.cols[1] = a * b.cols[1];
    r.cols[2] = a * b.cols[2];
    return r;
}

/**
 * @brief Adds two matrices element-wise
 * @param[in] a First matrix
 * @param[in] b Second matrix
 * @return Sum of the two matrices
 */
CUDA_CALLABLE inline Matrix33 Add(const Matrix33& a, const Matrix33& b)
{
    return Matrix33(a.cols[0] + b.cols[0], a.cols[1] + b.cols[1], a.cols[2] + b.cols[2]);
}

/**
 * @brief Computes the determinant of a 3x3 matrix
 * @param[in] m Matrix to compute determinant for
 * @return Determinant value
 */
CUDA_CALLABLE inline float Determinant(const Matrix33& m)
{
    return Dot(m.cols[0], Cross(m.cols[1], m.cols[2]));
}

/**
 * @brief Computes the transpose of a matrix
 * @param[in] a Matrix to transpose
 * @return Transposed matrix
 * @details Exchanges rows and columns: result(i,j) = a(j,i)
 */
CUDA_CALLABLE inline Matrix33 Transpose(const Matrix33& a)
{
    Matrix33 r;
    for (uint32_t i = 0; i < 3; ++i)
        for (uint32_t j = 0; j < 3; ++j)
            r(i, j) = a(j, i);

    return r;
}

/**
 * @brief Computes the trace (sum of diagonal elements) of a matrix
 * @param[in] a Matrix to compute trace for
 * @return Sum of diagonal elements
 */
CUDA_CALLABLE inline float Trace(const Matrix33& a)
{
    return a(0, 0) + a(1, 1) + a(2, 2);
}

/**
 * @brief Computes the outer product of two vectors
 * @param[in] a First vector (column vector)
 * @param[in] b Second vector (treated as row vector)
 * @return Matrix where result(i,j) = a[i] * b[j]
 */
CUDA_CALLABLE inline Matrix33 Outer(const Vec3& a, const Vec3& b)
{
    return Matrix33(a * b.x, a * b.y, a * b.z);
}

/**
 * @brief Computes the inverse of a 3x3 matrix using single precision
 * @param[in] a Matrix to invert
 * @param[out] success Set to true if inversion succeeded, false if matrix is singular
 * @return Inverse matrix if successful, uninitialized matrix if failed
 * @details Uses cofactor method with single precision arithmetic
 */
CUDA_CALLABLE inline Matrix33 Inverse(const Matrix33& a, bool& success)
{
    float s = Determinant(a);

    const float eps = 0.0f;

    if (fabsf(s) > eps)
    {
        Matrix33 b;

        b(0, 0) = a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1);
        b(0, 1) = a(0, 2) * a(2, 1) - a(0, 1) * a(2, 2);
        b(0, 2) = a(0, 1) * a(1, 2) - a(0, 2) * a(1, 1);
        b(1, 0) = a(1, 2) * a(2, 0) - a(1, 0) * a(2, 2);
        b(1, 1) = a(0, 0) * a(2, 2) - a(0, 2) * a(2, 0);
        b(1, 2) = a(0, 2) * a(1, 0) - a(0, 0) * a(1, 2);
        b(2, 0) = a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0);
        b(2, 1) = a(0, 1) * a(2, 0) - a(0, 0) * a(2, 1);
        b(2, 2) = a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0);

        success = true;

        return Multiply(1.0f / s, b);
    }
    else
    {
        success = false;
        return Matrix33();
    }
}

/**
 * @brief Computes the inverse of a 3x3 matrix using double precision
 * @param[in] a Matrix to invert
 * @param[out] success Set to true if inversion succeeded, false if matrix is singular
 * @return Inverse matrix if successful, zero matrix if failed
 * @details Uses cofactor method with double precision arithmetic for higher accuracy
 */
CUDA_CALLABLE inline Matrix33 InverseDouble(const Matrix33& a, bool& success)
{

    double m[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m[i][j] = a(i, j);

    double det = m[0][0] * (m[2][2] * m[1][1] - m[2][1] * m[1][2]) - m[1][0] * (m[2][2] * m[0][1] - m[2][1] * m[0][2]) +
                 m[2][0] * (m[1][2] * m[0][1] - m[1][1] * m[0][2]);

    const double eps = 0.0f;

    if (fabs(det) > eps)
    {
        double b[3][3];

        b[0][0] = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        b[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
        b[0][2] = m[0][1] * m[1][2] - m[0][2] * m[1][1];
        b[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
        b[1][1] = m[0][0] * m[2][2] - m[0][2] * m[2][0];
        b[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
        b[2][0] = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        b[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
        b[2][2] = m[0][0] * m[1][1] - m[0][1] * m[1][0];

        success = true;

        double invDet = 1.0 / det;

        Matrix33 out;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                out(i, j) = (float)(b[i][j] * invDet);

        return out;
    }
    else
    {
        success = false;

        Matrix33 out;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                out(i, j) = 0.0f;

        return out;
    }
}

/**
 * @brief Scalar-matrix multiplication operator (scalar on left)
 * @param[in] s Scalar multiplier
 * @param[in] a Matrix to multiply
 * @return Matrix with all elements multiplied by scalar
 */
CUDA_CALLABLE inline Matrix33 operator*(float s, const Matrix33& a)
{
    return Multiply(s, a);
}

/**
 * @brief Matrix-scalar multiplication operator (scalar on right)
 * @param[in] a Matrix to multiply
 * @param[in] s Scalar multiplier
 * @return Matrix with all elements multiplied by scalar
 */
CUDA_CALLABLE inline Matrix33 operator*(const Matrix33& a, float s)
{
    return Multiply(s, a);
}

/**
 * @brief Matrix multiplication operator
 * @param[in] a First matrix
 * @param[in] b Second matrix
 * @return Product matrix a * b
 */
CUDA_CALLABLE inline Matrix33 operator*(const Matrix33& a, const Matrix33& b)
{
    return Multiply(a, b);
}

/**
 * @brief Matrix addition operator
 * @param[in] a First matrix
 * @param[in] b Second matrix
 * @return Sum of the two matrices
 */
CUDA_CALLABLE inline Matrix33 operator+(const Matrix33& a, const Matrix33& b)
{
    return Add(a, b);
}

/**
 * @brief Matrix subtraction operator
 * @param[in] a First matrix
 * @param[in] b Second matrix
 * @return Difference of the two matrices (a - b)
 */
CUDA_CALLABLE inline Matrix33 operator-(const Matrix33& a, const Matrix33& b)
{
    return Add(a, -1.0f * b);
}

/**
 * @brief Matrix addition assignment operator
 * @param[in,out] a Matrix to add to
 * @param[in] b Matrix to add
 * @return Reference to modified matrix a
 */
CUDA_CALLABLE inline Matrix33& operator+=(Matrix33& a, const Matrix33& b)
{
    a = a + b;
    return a;
}

/**
 * @brief Matrix subtraction assignment operator
 * @param[in,out] a Matrix to subtract from
 * @param[in] b Matrix to subtract
 * @return Reference to modified matrix a
 */
CUDA_CALLABLE inline Matrix33& operator-=(Matrix33& a, const Matrix33& b)
{
    a = a - b;
    return a;
}

/**
 * @brief Scalar multiplication assignment operator
 * @param[in,out] a Matrix to multiply
 * @param[in] s Scalar multiplier
 * @return Reference to modified matrix a
 */
CUDA_CALLABLE inline Matrix33& operator*=(Matrix33& a, float s)
{
    a.cols[0] *= s;
    a.cols[1] *= s;
    a.cols[2] *= s;
    return a;
}

/**
 * @brief Creates a skew-symmetric matrix from a vector
 * @param[in] v Input vector
 * @return Skew-symmetric matrix such that result * x = v × x for any vector x
 * @details Useful for representing cross products as matrix operations
 */
CUDA_CALLABLE inline Matrix33 Skew(const Vec3& v)
{
    return Matrix33(Vec3(0.0f, v.z, -v.y), Vec3(-v.z, 0.0f, v.x), Vec3(v.y, -v.x, 0.0f));
}

template <typename T>
CUDA_CALLABLE inline XQuat<T>::XQuat(const Matrix33& m)
{
    float tr = m(0, 0) + m(1, 1) + m(2, 2), h;
    if (tr >= 0)
    {
        h = sqrtf(tr + 1);
        w = 0.5f * h;
        h = 0.5f / h;

        x = (m(2, 1) - m(1, 2)) * h;
        y = (m(0, 2) - m(2, 0)) * h;
        z = (m(1, 0) - m(0, 1)) * h;
    }
    else
    {
        unsigned int i = 0;
        if (m(1, 1) > m(0, 0))
            i = 1;
        if (m(2, 2) > m(i, i))
            i = 2;
        switch (i)
        {
        case 0:
            h = sqrtf((m(0, 0) - (m(1, 1) + m(2, 2))) + 1);
            x = 0.5f * h;
            h = 0.5f / h;

            y = (m(0, 1) + m(1, 0)) * h;
            z = (m(2, 0) + m(0, 2)) * h;
            w = (m(2, 1) - m(1, 2)) * h;
            break;
        case 1:
            h = sqrtf((m(1, 1) - (m(2, 2) + m(0, 0))) + 1);
            y = 0.5f * h;
            h = 0.5f / h;

            z = (m(1, 2) + m(2, 1)) * h;
            x = (m(0, 1) + m(1, 0)) * h;
            w = (m(0, 2) - m(2, 0)) * h;
            break;
        case 2:
            h = sqrtf((m(2, 2) - (m(0, 0) + m(1, 1))) + 1);
            z = 0.5f * h;
            h = 0.5f / h;

            x = (m(2, 0) + m(0, 2)) * h;
            y = (m(1, 2) + m(2, 1)) * h;
            w = (m(1, 0) - m(0, 1)) * h;
            break;
        default: // Make compiler happy
            x = y = z = w = 0;
            break;
        }
    }

    *this = Normalize(*this);
}

/**
 * @brief Converts a quaternion to a 3x3 rotation matrix.
 * @param[in] q Quaternion to convert
 * @param[out] m Output rotation matrix
 * @details
 * Converts the input quaternion to its corresponding 3x3 rotation matrix
 * representation using the standard quaternion-to-matrix formula.
 */
CUDA_CALLABLE inline void quat2Mat(const XQuat<float>& q, Matrix33& m)
{
    float sqx = q.x * q.x;
    float sqy = q.y * q.y;
    float sqz = q.z * q.z;
    float squ = q.w * q.w;
    float s = 1.f / (sqx + sqy + sqz + squ);

    m(0, 0) = 1.f - 2.f * s * (sqy + sqz);
    m(0, 1) = 2.f * s * (q.x * q.y - q.z * q.w);
    m(0, 2) = 2.f * s * (q.x * q.z + q.y * q.w);
    m(1, 0) = 2.f * s * (q.x * q.y + q.z * q.w);
    m(1, 1) = 1.f - 2.f * s * (sqx + sqz);
    m(1, 2) = 2.f * s * (q.y * q.z - q.x * q.w);
    m(2, 0) = 2.f * s * (q.x * q.z - q.y * q.w);
    m(2, 1) = 2.f * s * (q.y * q.z + q.x * q.w);
    m(2, 2) = 1.f - 2.f * s * (sqx + sqy);
}

// rotation is using new rotation axis
// rotation: Rx(X)*Ry(Y)*Rz(Z)
/**
 * @brief Extracts Euler angles from a quaternion using XYZ rotation order.
 * @param[in] q Input quaternion representing the rotation
 * @param[out] X Output X-axis rotation angle in radians
 * @param[out] Y Output Y-axis rotation angle in radians
 * @param[out] Z Output Z-axis rotation angle in radians
 * @details
 * Converts a quaternion to Euler angles using the XYZ rotation order.
 * The rotation sequence is Rx(X) * Ry(Y) * Rz(Z) using intrinsic rotations.
 *
 * @note Handles gimbal lock cases when the Y rotation is near ±90 degrees
 */
CUDA_CALLABLE inline void getEulerXYZ(const XQuat<float>& q, float& X, float& Y, float& Z)
{
    Matrix33 rot;
    quat2Mat(q, rot);

    float cy = sqrtf(rot(2, 2) * rot(2, 2) + rot(1, 2) * rot(1, 2));
    if (cy > 1e-6f)
    {
        Z = -atan2(rot(0, 1), rot(0, 0));
        Y = -atan2(-rot(0, 2), cy);
        X = -atan2(rot(1, 2), rot(2, 2));
    }
    else
    {
        Z = -atan2(-rot(1, 0), rot(1, 1));
        Y = -atan2(-rot(0, 2), cy);
        X = 0.0f;
    }
}

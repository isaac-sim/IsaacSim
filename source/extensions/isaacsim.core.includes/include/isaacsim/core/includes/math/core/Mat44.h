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
#include "Point3.h"
#include "Vec4.h"

/**
 * @class XMatrix44
 * @brief Template class representing a 4x4 matrix stored in column-major order.
 * @details
 * A template-based 4x4 matrix class that provides matrix operations including
 * arithmetic operations, transformations, and conversions. This class stores
 * column vectors in column-major order, which is compatible with OpenGL and
 * many graphics APIs.
 *
 * The matrix is organized as follows:
 * - columns[0] = first column vector
 * - columns[1] = second column vector
 * - columns[2] = third column vector
 * - columns[3] = fourth column vector (often translation in transformation matrices)
 *
 * @tparam T The numeric type for matrix elements (typically float or double)
 *
 * @note Matrix operations follow column-major conventions common in graphics programming
 */
template <typename T>
class XMatrix44
{
public:
    /**
     * @brief Default constructor initializes matrix to zero
     */
    CUDA_CALLABLE XMatrix44()
    {
        memset(columns, 0, sizeof(columns));
    }

    /**
     * @brief Constructor that initializes matrix from array data
     * @param[in] d Pointer to array of 16 elements in column-major order
     */
    CUDA_CALLABLE XMatrix44(const T* d)
    {
        assert(d);
        memcpy(columns, d, sizeof(*this));
    }

    /**
     * @brief Constructor that initializes matrix with individual element values
     * @param[in] c11 Element at row 1, column 1
     * @param[in] c21 Element at row 2, column 1
     * @param[in] c31 Element at row 3, column 1
     * @param[in] c41 Element at row 4, column 1
     * @param[in] c12 Element at row 1, column 2
     * @param[in] c22 Element at row 2, column 2
     * @param[in] c32 Element at row 3, column 2
     * @param[in] c42 Element at row 4, column 2
     * @param[in] c13 Element at row 1, column 3
     * @param[in] c23 Element at row 2, column 3
     * @param[in] c33 Element at row 3, column 3
     * @param[in] c43 Element at row 4, column 3
     * @param[in] c14 Element at row 1, column 4
     * @param[in] c24 Element at row 2, column 4
     * @param[in] c34 Element at row 3, column 4
     * @param[in] c44 Element at row 4, column 4
     */
    CUDA_CALLABLE XMatrix44(
        T c11, T c21, T c31, T c41, T c12, T c22, T c32, T c42, T c13, T c23, T c33, T c43, T c14, T c24, T c34, T c44)
    {
        columns[0][0] = c11;
        columns[0][1] = c21;
        columns[0][2] = c31;
        columns[0][3] = c41;

        columns[1][0] = c12;
        columns[1][1] = c22;
        columns[1][2] = c32;
        columns[1][3] = c42;

        columns[2][0] = c13;
        columns[2][1] = c23;
        columns[2][2] = c33;
        columns[2][3] = c43;

        columns[3][0] = c14;
        columns[3][1] = c24;
        columns[3][2] = c34;
        columns[3][3] = c44;
    }

    /**
     * @brief Constructor that initializes matrix from four column vectors
     * @param[in] c1 First column vector
     * @param[in] c2 Second column vector
     * @param[in] c3 Third column vector
     * @param[in] c4 Fourth column vector
     */
    CUDA_CALLABLE XMatrix44(const Vec4& c1, const Vec4& c2, const Vec4& c3, const Vec4& c4)
    {
        columns[0][0] = c1.x;
        columns[0][1] = c1.y;
        columns[0][2] = c1.z;
        columns[0][3] = c1.w;

        columns[1][0] = c2.x;
        columns[1][1] = c2.y;
        columns[1][2] = c2.z;
        columns[1][3] = c2.w;

        columns[2][0] = c3.x;
        columns[2][1] = c3.y;
        columns[2][2] = c3.z;
        columns[2][3] = c3.w;

        columns[3][0] = c4.x;
        columns[3][1] = c4.y;
        columns[3][2] = c4.z;
        columns[3][3] = c4.w;
    }

    /**
     * @brief Conversion operator to non-const pointer to matrix data
     * @return Pointer to the first element in column-major order
     */
    CUDA_CALLABLE operator T*()
    {
        return &columns[0][0];
    }

    /**
     * @brief Conversion operator to const pointer to matrix data
     * @return Const pointer to the first element in column-major order
     */
    CUDA_CALLABLE operator const T*() const
    {
        return &columns[0][0];
    }

    /**
     * @brief Matrix multiplication operator (right multiply)
     * @param[in] rhs Matrix to multiply with (right operand)
     * @return New matrix that is the product of this matrix and rhs
     */
    CUDA_CALLABLE XMatrix44<T> operator*(const XMatrix44<T>& rhs) const
    {
        XMatrix44<T> r;
        MatrixMultiply(*this, rhs, r);
        return r;
    }

    /**
     * @brief Matrix multiplication assignment operator (right multiply)
     * @param[in] rhs Matrix to multiply with (right operand)
     * @return Reference to this matrix after multiplication
     */
    CUDA_CALLABLE XMatrix44<T>& operator*=(const XMatrix44<T>& rhs)
    {
        XMatrix44<T> r;
        MatrixMultiply(*this, rhs, r);
        *this = r;

        return *this;
    }

    /**
     * @brief Element access operator (const version)
     * @param[in] row Row index (0-3)
     * @param[in] col Column index (0-3)
     * @return Const reference to matrix element at specified row and column
     */
    CUDA_CALLABLE float operator()(int row, int col) const
    {
        return columns[col][row];
    }

    /**
     * @brief Element access operator (non-const version)
     * @param[in] row Row index (0-3)
     * @param[in] col Column index (0-3)
     * @return Reference to matrix element at specified row and column
     */
    CUDA_CALLABLE float& operator()(int row, int col)
    {
        return columns[col][row];
    }

    /**
     * @brief Scalar multiplication assignment operator
     * @param[in] s Scalar value to multiply all elements with
     * @return Reference to this matrix after scalar multiplication
     */
    CUDA_CALLABLE XMatrix44<T>& operator*=(const T& s)
    {
        for (int c = 0; c < 4; ++c)
        {
            for (int r = 0; r < 4; ++r)
            {
                columns[c][r] *= s;
            }
        }

        return *this;
    }

    /**
     * @brief Performs matrix multiplication of two 4x4 matrices
     * @param[in] lhs Left-hand side matrix (as array of 16 elements)
     * @param[in] rhs Right-hand side matrix (as array of 16 elements)
     * @param[out] result Output matrix (as array of 16 elements)
     * @details Computes result = lhs * rhs using column-major storage
     * @pre lhs, rhs, and result must point to different memory locations
     */
    CUDA_CALLABLE void MatrixMultiply(const T* __restrict lhs, const T* __restrict rhs, T* __restrict result) const
    {
        assert(lhs != rhs);
        assert(lhs != result);
        assert(rhs != result);

        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                result[j * 4 + i] = rhs[j * 4 + 0] * lhs[i + 0];
                result[j * 4 + i] += rhs[j * 4 + 1] * lhs[i + 4];
                result[j * 4 + i] += rhs[j * 4 + 2] * lhs[i + 8];
                result[j * 4 + i] += rhs[j * 4 + 3] * lhs[i + 12];
            }
        }
    }

    /**
     * @brief Sets a column of the matrix to the specified 4D vector
     * @param[in] index Column index (0-3)
     * @param[in] c 4D vector to set as the column
     */
    CUDA_CALLABLE void SetCol(int index, const Vec4& c)
    {
        columns[index][0] = c.x;
        columns[index][1] = c.y;
        columns[index][2] = c.z;
        columns[index][3] = c.w;
    }

    /**
     * @brief Sets a column of the matrix to the specified 3D vector (w=0)
     * @param[in] index Column index (0-3)
     * @param[in] a 3D vector to set as the column (w component set to 0)
     */
    CUDA_CALLABLE void SetAxis(uint32_t index, const XVector3<T>& a)
    {
        columns[index][0] = a.x;
        columns[index][1] = a.y;
        columns[index][2] = a.z;
        columns[index][3] = 0.0f;
    }

    /**
     * @brief Sets the translation column (fourth column) of the matrix
     * @param[in] p 3D point representing the translation
     */
    CUDA_CALLABLE void SetTranslation(const Point3& p)
    {
        columns[3][0] = p.x;
        columns[3][1] = p.y;
        columns[3][2] = p.z;
        columns[3][3] = 1.0f;
    }

    /**
     * @brief Gets a column as a 3D vector (ignoring w component)
     * @param[in] i Column index (0-3)
     * @return Const reference to 3D vector representing the column
     */
    CUDA_CALLABLE const Vec3& GetAxis(int i) const
    {
        return *reinterpret_cast<const Vec3*>(&columns[i]);
    }

    /**
     * @brief Gets a column as a 4D vector
     * @param[in] i Column index (0-3)
     * @return Const reference to 4D vector representing the column
     */
    CUDA_CALLABLE const Vec4& GetCol(int i) const
    {
        return *reinterpret_cast<const Vec4*>(&columns[i]);
    }

    /**
     * @brief Gets the translation from the matrix (fourth column as 3D point)
     * @return Const reference to 3D point representing the translation
     */
    CUDA_CALLABLE const Point3& GetTranslation() const
    {
        return *reinterpret_cast<const Point3*>(&columns[3]);
    }

    /**
     * @brief Gets a row of the matrix as a 4D vector
     * @param[in] i Row index (0-3)
     * @return 4D vector representing the row
     */
    CUDA_CALLABLE Vec4 GetRow(int i) const
    {
        return Vec4(columns[0][i], columns[1][i], columns[2][i], columns[3][i]);
    }

    /**
     * @brief Creates and returns an identity matrix
     * @return 4x4 identity matrix
     */
    CUDA_CALLABLE static inline XMatrix44 Identity()
    {
        const XMatrix44 sIdentity(Vec4(1.0f, 0.0f, 0.0f, 0.0f), Vec4(0.0f, 1.0f, 0.0f, 0.0f),
                                  Vec4(0.0f, 0.0f, 1.0f, 0.0f), Vec4(0.0f, 0.0f, 0.0f, 1.0f));
        return sIdentity;
    }

    /**
     * @brief Matrix data stored as column vectors in column-major order
     * @details columns[i][j] represents element at row j, column i
     */
    T columns[4][4];
};

// right multiply a point assumes w of 1
template <typename T>
CUDA_CALLABLE Point3 Multiply(const XMatrix44<T>& mat, const Point3& v)
{
    Point3 r;
    r.x = v.x * mat[0] + v.y * mat[4] + v.z * mat[8] + mat[12];
    r.y = v.x * mat[1] + v.y * mat[5] + v.z * mat[9] + mat[13];
    r.z = v.x * mat[2] + v.y * mat[6] + v.z * mat[10] + mat[14];

    return r;
}

// right multiply a vector3 assumes a w of 0
template <typename T>
CUDA_CALLABLE XVector3<T> Multiply(const XMatrix44<T>& mat, const XVector3<T>& v)
{
    XVector3<T> r;
    r.x = v.x * mat[0] + v.y * mat[4] + v.z * mat[8];
    r.y = v.x * mat[1] + v.y * mat[5] + v.z * mat[9];
    r.z = v.x * mat[2] + v.y * mat[6] + v.z * mat[10];

    return r;
}

// right multiply a vector4
template <typename T>
CUDA_CALLABLE XVector4<T> Multiply(const XMatrix44<T>& mat, const XVector4<T>& v)
{
    XVector4<T> r;
    r.x = v.x * mat[0] + v.y * mat[4] + v.z * mat[8] + v.w * mat[12];
    r.y = v.x * mat[1] + v.y * mat[5] + v.z * mat[9] + v.w * mat[13];
    r.z = v.x * mat[2] + v.y * mat[6] + v.z * mat[10] + v.w * mat[14];
    r.w = v.x * mat[3] + v.y * mat[7] + v.z * mat[11] + v.w * mat[15];

    return r;
}

template <typename T>
CUDA_CALLABLE Point3 operator*(const XMatrix44<T>& mat, const Point3& v)
{
    return Multiply(mat, v);
}

template <typename T>
CUDA_CALLABLE XVector4<T> operator*(const XMatrix44<T>& mat, const XVector4<T>& v)
{
    return Multiply(mat, v);
}

template <typename T>
CUDA_CALLABLE XVector3<T> operator*(const XMatrix44<T>& mat, const XVector3<T>& v)
{
    return Multiply(mat, v);
}

template <typename T>
CUDA_CALLABLE inline XMatrix44<T> Transpose(const XMatrix44<T>& m)
{
    XMatrix44<float> inv;

    // transpose
    for (uint32_t c = 0; c < 4; ++c)
    {
        for (uint32_t r = 0; r < 4; ++r)
        {
            inv.columns[c][r] = m.columns[r][c];
        }
    }

    return inv;
}

template <typename T>
CUDA_CALLABLE XMatrix44<T> AffineInverse(const XMatrix44<T>& m)
{
    XMatrix44<T> inv;

    // transpose upper 3x3
    for (int c = 0; c < 3; ++c)
    {
        for (int r = 0; r < 3; ++r)
        {
            inv.columns[c][r] = m.columns[r][c];
        }
    }

    // multiply -translation by upper 3x3 transpose
    inv.columns[3][0] = -Dot3(m.columns[3], m.columns[0]);
    inv.columns[3][1] = -Dot3(m.columns[3], m.columns[1]);
    inv.columns[3][2] = -Dot3(m.columns[3], m.columns[2]);
    inv.columns[3][3] = 1.0f;

    return inv;
}

CUDA_CALLABLE inline XMatrix44<float> Outer(const Vec4& a, const Vec4& b)
{
    return XMatrix44<float>(a * b.x, a * b.y, a * b.z, a * b.w);
}

// convenience
typedef XMatrix44<float> Mat44;
typedef XMatrix44<float> Matrix44;

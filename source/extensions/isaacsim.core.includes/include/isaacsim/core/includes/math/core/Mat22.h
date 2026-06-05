// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "Vec2.h"

/**
 * @struct Matrix22
 * @brief 2x2 matrix representation with column-major storage.
 * @details
 * This structure represents a 2x2 matrix stored in column-major order.
 * It provides basic matrix operations including arithmetic, element access,
 * and static identity matrix creation.
 *
 * The matrix is stored as two Vec2 columns, making it suitable for 2D
 * transformations and linear algebra operations.
 */
struct Matrix22
{
    /**
     * @brief Default constructor creating an uninitialized matrix.
     */
    CUDA_CALLABLE Matrix22()
    {
    }
    /**
     * @brief Constructor from individual matrix elements.
     * @param[in] a Element at position (0,0)
     * @param[in] b Element at position (0,1)
     * @param[in] c Element at position (1,0)
     * @param[in] d Element at position (1,1)
     * @details
     * Creates a matrix with the specified elements:
     * | a  b |
     * | c  d |
     */
    CUDA_CALLABLE Matrix22(float a, float b, float c, float d)
    {
        cols[0] = Vec2(a, c);
        cols[1] = Vec2(b, d);
    }

    /**
     * @brief Constructor from two column vectors.
     * @param[in] c1 First column vector
     * @param[in] c2 Second column vector
     */
    CUDA_CALLABLE Matrix22(const Vec2& c1, const Vec2& c2)
    {
        cols[0] = c1;
        cols[1] = c2;
    }

    /**
     * @brief Const element access operator.
     * @param[in] i Row index (0 or 1)
     * @param[in] j Column index (0 or 1)
     * @return Const reference to the matrix element at (i,j)
     */
    CUDA_CALLABLE float operator()(int i, int j) const
    {
        return static_cast<const float*>(cols[j])[i];
    }
    /**
     * @brief Mutable element access operator.
     * @param[in] i Row index (0 or 1)
     * @param[in] j Column index (0 or 1)
     * @return Mutable reference to the matrix element at (i,j)
     */
    CUDA_CALLABLE float& operator()(int i, int j)
    {
        return static_cast<float*>(cols[j])[i];
    }

    /**
     * @brief Column vectors of the matrix.
     * @details
     * The matrix is stored as two column vectors in column-major order.
     * cols[0] represents the first column, cols[1] represents the second column.
     */
    Vec2 cols[2];

    /**
     * @brief Creates a 2x2 identity matrix.
     * @return Identity matrix with 1s on the diagonal and 0s elsewhere
     */
    static inline Matrix22 Identity()
    {
        static const Matrix22 sIdentity(Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f));
        return sIdentity;
    }
};

/**
 * @brief Multiplies a 2x2 matrix by a scalar value.
 * @details
 * Performs element-wise multiplication of the matrix by the scalar.
 * Each element of the resulting matrix equals the corresponding element
 * of the input matrix multiplied by the scalar.
 *
 * @param[in] s Scalar value to multiply with
 * @param[in] m Matrix to be multiplied
 * @return New matrix with all elements multiplied by the scalar
 */
CUDA_CALLABLE inline Matrix22 Multiply(float s, const Matrix22& m)
{
    Matrix22 r = m;
    r.cols[0] *= s;
    r.cols[1] *= s;
    return r;
}

/**
 * @brief Multiplies two 2x2 matrices.
 * @details
 * Performs standard matrix multiplication where the result matrix element (i,j)
 * is the dot product of row i from matrix a and column j from matrix b.
 *
 * @param[in] a Left-hand side matrix
 * @param[in] b Right-hand side matrix
 * @return Product matrix a * b
 */
CUDA_CALLABLE inline Matrix22 Multiply(const Matrix22& a, const Matrix22& b)
{
    Matrix22 r;
    r.cols[0] = a.cols[0] * b.cols[0].x + a.cols[1] * b.cols[0].y;
    r.cols[1] = a.cols[0] * b.cols[1].x + a.cols[1] * b.cols[1].y;
    return r;
}

/**
 * @brief Adds two 2x2 matrices element-wise.
 * @details
 * Performs element-wise addition of two matrices. Each element of the
 * resulting matrix is the sum of the corresponding elements from the
 * input matrices.
 *
 * @param[in] a First matrix
 * @param[in] b Second matrix
 * @return Sum matrix a + b
 */
CUDA_CALLABLE inline Matrix22 Add(const Matrix22& a, const Matrix22& b)
{
    return Matrix22(a.cols[0] + b.cols[0], a.cols[1] + b.cols[1]);
}

/**
 * @brief Multiplies a 2x2 matrix by a 2D vector.
 * @details
 * Performs matrix-vector multiplication where the result is a linear
 * combination of the matrix columns weighted by the vector components.
 *
 * @param[in] a Matrix to multiply
 * @param[in] x Vector to multiply
 * @return Transformed vector a * x
 */
CUDA_CALLABLE inline Vec2 Multiply(const Matrix22& a, const Vec2& x)
{
    return a.cols[0] * x.x + a.cols[1] * x.y;
}

/**
 * @brief Scalar-matrix multiplication operator (scalar on left).
 * @param[in] s Scalar value
 * @param[in] a Matrix to multiply
 * @return Matrix with all elements multiplied by scalar
 */
CUDA_CALLABLE inline Matrix22 operator*(float s, const Matrix22& a)
{
    return Multiply(s, a);
}

/**
 * @brief Matrix-scalar multiplication operator (scalar on right).
 * @param[in] a Matrix to multiply
 * @param[in] s Scalar value
 * @return Matrix with all elements multiplied by scalar
 */
CUDA_CALLABLE inline Matrix22 operator*(const Matrix22& a, float s)
{
    return Multiply(s, a);
}

/**
 * @brief Matrix-matrix multiplication operator.
 * @param[in] a Left-hand side matrix
 * @param[in] b Right-hand side matrix
 * @return Product matrix a * b
 */
CUDA_CALLABLE inline Matrix22 operator*(const Matrix22& a, const Matrix22& b)
{
    return Multiply(a, b);
}

/**
 * @brief Matrix addition operator.
 * @param[in] a First matrix
 * @param[in] b Second matrix
 * @return Sum matrix a + b
 */
CUDA_CALLABLE inline Matrix22 operator+(const Matrix22& a, const Matrix22& b)
{
    return Add(a, b);
}

/**
 * @brief Matrix subtraction operator.
 * @param[in] a Minuend matrix
 * @param[in] b Subtrahend matrix
 * @return Difference matrix a - b
 */
CUDA_CALLABLE inline Matrix22 operator-(const Matrix22& a, const Matrix22& b)
{
    return Add(a, -1.0f * b);
}

/**
 * @brief Matrix addition assignment operator.
 * @param[in,out] a Matrix to add to (modified in place)
 * @param[in] b Matrix to add
 * @return Reference to the modified matrix a
 */
CUDA_CALLABLE inline Matrix22& operator+=(Matrix22& a, const Matrix22& b)
{
    a = a + b;
    return a;
}

/**
 * @brief Matrix subtraction assignment operator.
 * @param[in,out] a Matrix to subtract from (modified in place)
 * @param[in] b Matrix to subtract
 * @return Reference to the modified matrix a
 */
CUDA_CALLABLE inline Matrix22& operator-=(Matrix22& a, const Matrix22& b)
{
    a = a - b;
    return a;
}

/**
 * @brief Scalar multiplication assignment operator.
 * @param[in,out] a Matrix to multiply (modified in place)
 * @param[in] s Scalar value
 * @return Reference to the modified matrix a
 */
CUDA_CALLABLE inline Matrix22& operator*=(Matrix22& a, float s)
{
    a = a * s;
    return a;
}

/**
 * @brief Matrix-vector multiplication operator.
 * @param[in] a Matrix to multiply
 * @param[in] x Vector to multiply
 * @return Transformed vector a * x
 */
CUDA_CALLABLE inline Vec2 operator*(const Matrix22& a, const Vec2& x)
{
    return Multiply(a, x);
}

/**
 * @brief Calculates the determinant of a 2x2 matrix.
 * @details
 * Computes the determinant using the standard formula:
 * det(m) = m(0,0) * m(1,1) - m(1,0) * m(0,1)
 *
 * @param[in] m Matrix to compute determinant for
 * @return Determinant value of the matrix
 */
CUDA_CALLABLE inline float Determinant(const Matrix22& m)
{
    return m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1);
}

/**
 * @brief Computes the inverse of a 2x2 matrix.
 * @details
 * Calculates the matrix inverse using the analytical formula for 2x2 matrices.
 * If the determinant is close to zero (within FLT_EPSILON), the matrix is
 * considered singular and the original matrix is returned with determinant set to 0.
 *
 * @param[in] m Matrix to invert
 * @param[out] det Determinant of the input matrix
 * @return Inverse matrix if invertible, otherwise the original matrix
 *
 * @note If the matrix is singular (determinant near zero), the function returns
 *       the original matrix and sets det to 0.0f
 */
CUDA_CALLABLE inline Matrix22 Inverse(const Matrix22& m, float& det)
{
    det = Determinant(m);

    if (fabsf(det) > FLT_EPSILON)
    {
        Matrix22 inv;
        inv(0, 0) = m(1, 1);
        inv(1, 1) = m(0, 0);
        inv(0, 1) = -m(0, 1);
        inv(1, 0) = -m(1, 0);

        return Multiply(1.0f / det, inv);
    }
    else
    {
        det = 0.0f;
        return m;
    }
}

/**
 * @brief Computes the transpose of a 2x2 matrix.
 * @details
 * Creates a new matrix where rows and columns are swapped.
 * Element (i,j) in the transpose equals element (j,i) in the original matrix.
 *
 * @param[in] a Matrix to transpose
 * @return Transposed matrix
 */
CUDA_CALLABLE inline Matrix22 Transpose(const Matrix22& a)
{
    Matrix22 r;
    r(0, 0) = a(0, 0);
    r(0, 1) = a(1, 0);
    r(1, 0) = a(0, 1);
    r(1, 1) = a(1, 1);
    return r;
}

/**
 * @brief Calculates the trace of a 2x2 matrix.
 * @details
 * The trace is the sum of the diagonal elements: trace(a) = a(0,0) + a(1,1)
 *
 * @param[in] a Matrix to compute trace for
 * @return Trace (sum of diagonal elements) of the matrix
 */
CUDA_CALLABLE inline float Trace(const Matrix22& a)
{
    return a(0, 0) + a(1, 1);
}

/**
 * @brief Creates a 2D rotation matrix for the given angle.
 * @details
 * Constructs a rotation matrix that rotates points counter-clockwise
 * by the specified angle in radians. The matrix has the form:
 * | cos(θ)  -sin(θ) |
 * | sin(θ)   cos(θ) |
 *
 * @param[in] theta Rotation angle in radians
 * @return 2x2 rotation matrix
 */
CUDA_CALLABLE inline Matrix22 RotationMatrix(float theta)
{
    return Matrix22(Vec2(cosf(theta), sinf(theta)), Vec2(-sinf(theta), cosf(theta)));
}

/**
 * @brief Computes the outer product of two 2D vectors.
 * @details
 * Creates a 2x2 matrix from the outer product of vectors a and b,
 * where b is treated as a row vector. The resulting matrix has
 * elements: result(i,j) = a[i] * b[j]
 *
 * @param[in] a First vector (treated as column vector)
 * @param[in] b Second vector (treated as row vector)
 * @return 2x2 matrix representing the outer product a ⊗ b
 */
CUDA_CALLABLE inline Matrix22 Outer(const Vec2& a, const Vec2& b)
{
    return Matrix22(a * b.x, a * b.y);
}

/**
 * @brief Performs QR decomposition of a 2x2 matrix.
 * @details
 * Decomposes the matrix into an orthogonal matrix Q. This implementation
 * normalizes the first column and creates an orthogonal matrix using
 * the perpendicular vector.
 *
 * @param[in] m Matrix to decompose
 * @return Orthogonal matrix Q from the QR decomposition
 *
 * @note This function only returns the Q component of the QR decomposition
 */
CUDA_CALLABLE inline Matrix22 QRDecomposition(const Matrix22& m)
{
    Vec2 a = Normalize(m.cols[0]);
    Matrix22 q(a, PerpCCW(a));

    return q;
}

/**
 * @brief Performs polar decomposition of a 2x2 matrix.
 * @details
 * Decomposes the matrix into an orthogonal (rotation) matrix using
 * an optimized analytical method. The decomposition finds the closest
 * orthogonal matrix to the input matrix.
 *
 * @param[in] m Matrix to decompose
 * @return Orthogonal matrix from the polar decomposition
 *
 * @note This implementation uses an analytical approach rather than
 *       the iterative method shown in comments
 */
CUDA_CALLABLE inline Matrix22 PolarDecomposition(const Matrix22& m)
{
    /*
    //iterative method

    float det;
    Matrix22 q = m;

    for (int i=0; i < 4; ++i)
    {
        q = 0.5f*(q + Inverse(Transpose(q), det));
    }
    */

    Matrix22 q = m + Matrix22(m(1, 1), -m(1, 0), -m(0, 1), m(0, 0));

    float s = Length(q.cols[0]);
    q.cols[0] /= s;
    q.cols[1] /= s;

    return q;
}

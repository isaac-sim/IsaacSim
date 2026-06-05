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

/**
 * @class XMatrix
 * @brief Template class representing an m×n matrix with column-major storage.
 * @details
 * A template-based matrix class that provides basic matrix operations including
 * arithmetic operations, element access, and matrix transformations. The matrix
 * uses column-major storage order for compatibility with linear algebra libraries.
 *
 * @tparam m Number of rows in the matrix
 * @tparam n Number of columns in the matrix
 * @tparam T Numeric type for matrix elements (defaults to double)
 *
 * @note Matrix elements are stored in column-major order: data[column][row]
 */
template <int m, int n, typename T = double>
class XMatrix
{
public:
    /**
     * @brief Default constructor initializes all elements to zero
     */
    XMatrix()
    {
        memset(data, 0, sizeof(*this));
    }

    /**
     * @brief Copy constructor creates matrix from another matrix
     * @param[in] a Source matrix to copy from
     */
    XMatrix(const XMatrix<m, n>& a)
    {
        memcpy(data, a.data, sizeof(*this));
    }

    /**
     * @brief Constructor that initializes matrix from pointer to data
     * @param[in] ptr Pointer to array of m×n elements in row-major order
     * @tparam OtherT Type of source data (automatically converted to T)
     */
    template <typename OtherT>
    XMatrix(const OtherT* ptr)
    {
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < m; ++i)
                data[j][i] = *(ptr++);
    }

    /**
     * @brief Assignment operator copies data from another matrix
     * @param[in] a Source matrix to copy from
     * @return Reference to this matrix after assignment
     */
    const XMatrix<m, n>& operator=(const XMatrix<m, n>& a)
    {
        memcpy(data, a.data, sizeof(*this));
        return *this;
    }

    /**
     * @brief Sets a column of the matrix to the specified column vector
     * @param[in] j Column index (0 to n-1)
     * @param[in] c Column vector of size m×1 to set
     * @tparam OtherT Type of source column vector elements
     */
    template <typename OtherT>
    void SetCol(int j, const XMatrix<m, 1, OtherT>& c)
    {
        for (int i = 0; i < m; ++i)
            data[j][i] = c(i, 0);
    }

    /**
     * @brief Sets a row of the matrix to the specified row vector
     * @param[in] i Row index (0 to m-1)
     * @param[in] r Row vector of size 1×n to set
     * @tparam OtherT Type of source row vector elements
     */
    template <typename OtherT>
    void SetRow(int i, const XMatrix<1, n, OtherT>& r)
    {
        for (int j = 0; j < m; ++j)
            data[j][i] = r(0, j);
    }

    /**
     * @brief Element access operator (non-const version)
     * @param[in] row Row index (0 to m-1)
     * @param[in] col Column index (0 to n-1)
     * @return Reference to matrix element at specified position
     */
    T& operator()(int row, int col)
    {
        return data[col][row];
    }

    /**
     * @brief Element access operator (const version)
     * @param[in] row Row index (0 to m-1)
     * @param[in] col Column index (0 to n-1)
     * @return Const reference to matrix element at specified position
     */
    const T& operator()(int row, int col) const
    {
        return data[col][row];
    }

    /**
     * @brief Sets the matrix to identity matrix
     * @details Sets diagonal elements to 1.0 and off-diagonal elements to 0.0
     * @note Only valid for square matrices (m == n)
     */
    void SetIdentity()
    {
        for (int i = 0; i < m; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                if (i == j)
                    data[i][j] = 1.0;
                else
                    data[i][j] = 0.0;
            }
        }
    }

    /**
     * @brief Matrix data stored in column-major order
     * @details data[column][row] accesses element at specified row and column
     */
    T data[n][m];
};

template <int m, int n, typename T>
XMatrix<m, n, T> operator-(const XMatrix<m, n, T>& lhs, const XMatrix<m, n, T>& rhs)
{
    XMatrix<m, n> d;

    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; ++j)
            d(i, j) = lhs(i, j) - rhs(i, j);

    return d;
}

template <int m, int n, typename T>
XMatrix<m, n, T> operator+(const XMatrix<m, n, T>& lhs, const XMatrix<m, n, T>& rhs)
{
    XMatrix<m, n> d;

    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; ++j)
            d(i, j) = lhs(i, j) + rhs(i, j);

    return d;
}

template <int m, int n, int o, typename T>
XMatrix<m, o> Multiply(const XMatrix<m, n, T>& lhs, const XMatrix<n, o, T>& rhs)
{
    XMatrix<m, o> ret;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < o; ++j)
        {
            T sum = 0.0f;

            for (int k = 0; k < n; ++k)
            {
                sum += lhs(i, k) * rhs(k, j);
            }

            ret(i, j) = sum;
        }
    }

    return ret;
}

template <int m, int n>
XMatrix<n, m> Transpose(const XMatrix<m, n>& a)
{
    XMatrix<n, m> ret;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            ret(j, i) = a(i, j);
        }
    }

    return ret;
}

// matrix to swap row i and j when multiplied on the right
template <int n>
XMatrix<n, n> Permutation(int i, int j)
{
    XMatrix<n, n> m;
    m.SetIdentity();

    m(i, i) = 0.0;
    m(i, j) = 1.0;

    m(j, j) = 0.0;
    m(j, i) = 1.0;

    return m;
}

template <int m, int n>
void PrintMatrix(const char* name, XMatrix<m, n> a)
{
    printf("%s = [\n", name);

    for (int i = 0; i < m; ++i)
    {
        printf("[ ");

        for (int j = 0; j < n; ++j)
        {
            printf("% .4f", float(a(i, j)));

            if (j < n - 1)
                printf(" ");
        }

        printf(" ]\n");
    }

    printf("]\n");
}

template <int n, typename T>
XMatrix<n, n, T> LU(const XMatrix<n, n, T>& m, XMatrix<n, n, T>& L)
{
    XMatrix<n, n> U = m;
    L.SetIdentity();

    // for each row
    for (int j = 0; j < n; ++j)
    {
        XMatrix<n, n> Li, LiInv;
        Li.SetIdentity();
        LiInv.SetIdentity();

        T pivot = U(j, j);

        if (pivot == 0.0)
            return U;

        assert(pivot != 0.0);

        // zero our all entries below pivot
        for (int i = j + 1; i < n; ++i)
        {
            T l = -U(i, j) / pivot;

            Li(i, j) = l;

            // create inverse of L1..Ln as we go (this is L)
            L(i, j) = -l;
        }

        U = Multiply(Li, U);
    }

    return U;
}

template <int m, typename T>
XMatrix<m, 1, T> Solve(const XMatrix<m, m, T>& L, const XMatrix<m, m, T>& U, const XMatrix<m, 1, T>& b)
{
    XMatrix<m, 1> y;
    XMatrix<m, 1> x;

    // Ly = b (forward substitution)
    for (int i = 0; i < m; ++i)
    {
        T sum = 0.0;

        for (int j = 0; j < i; ++j)
        {
            sum += y(j, 0) * L(i, j);
        }

        assert(L(i, i) != 0.0);

        y(i, 0) = (b(i, 0) - sum) / L(i, i);
    }

    // Ux = y (back substitution)
    for (int i = m - 1; i >= 0; --i)
    {
        T sum = 0.0;

        for (int j = i + 1; j < m; ++j)
        {
            sum += x(j, 0) * U(i, j);
        }

        assert(U(i, i) != 0.0);

        x(i, 0) = (y(i, 0) - sum) / U(i, i);
    }

    return x;
}

template <int n, typename T>
T Determinant(const XMatrix<n, n, T>& A, XMatrix<n, n, T>& L, XMatrix<n, n, T>& U)
{
    U = LU(A, L);

    // determinant is the product of diagonal entries of U (assume L has 1s on
    // diagonal)
    T det = 1.0;
    for (int i = 0; i < n; ++i)
        det *= U(i, i);

    return det;
}

template <int n, typename T>
XMatrix<n, n, T> Inverse(const XMatrix<n, n, T>& A, T& det)
{
    XMatrix<n, n> L, U;
    det = Determinant(A, L, U);

    XMatrix<n, n> Inv;

    if (det != 0.0f)
    {
        for (int i = 0; i < n; ++i)
        {
            // solve for each column of the identity matrix
            XMatrix<n, 1> I;
            I(i, 0) = 1.0;

            XMatrix<n, 1> x = Solve(L, U, I);

            Inv.SetCol(i, x);
        }
    }

    return Inv;
}

template <int m, int n, typename T>
T FrobeniusNorm(const XMatrix<m, n, T>& A)
{
    T sum = 0.0;
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; ++j)
            sum += A(i, j) * A(i, j);

    return sqrt(sum);
}

// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// Minimal compatibility header providing warp types used by Newton tensors.
// Matches the memory layout of warp/native types (vec_t, quat_t, mat_t,
// transform_t, spatial_vector_t, array_t) so data produced by Python warp
// arrays can be reinterpreted in C++/CUDA without pulling in the full
// warp header tree.

#pragma once

#include <cassert>
#include <cstddef>

#ifdef __CUDACC__
#    define WP_CUDA_CALLABLE __host__ __device__
#else
#    define WP_CUDA_CALLABLE
#endif

namespace wp
{

/// Maximum rank of a Warp array.
const int ARRAY_MAX_DIMS = 4;

/// Dimension descriptor for a Warp array.
///
/// Stores up to :data:`ARRAY_MAX_DIMS` element counts. Trailing unused slots are
/// left zero. Layout matches ``wp::shape_t`` in the native Warp runtime so a
/// ``wp.array`` struct can be ``reinterpret_cast``'d directly.
struct shape_t
{
    int dims[ARRAY_MAX_DIMS]; ///< Per-dimension element counts; trailing unused slots are zero.

    WP_CUDA_CALLABLE shape_t() : dims()
    {
    }

    /**
     * @brief Returns the element count for the given dimension.
     * @param[in] i Zero-based dimension index, must be less than ARRAY_MAX_DIMS.
     * @return The element count of dimension i.
     */
    WP_CUDA_CALLABLE int operator[](int i) const
    {
        assert(i < ARRAY_MAX_DIMS);
        return dims[i];
    }

    /**
     * @brief Returns a mutable reference to the element count for the given dimension.
     * @param[in] i Zero-based dimension index, must be less than ARRAY_MAX_DIMS.
     * @return Reference to the element count of dimension i.
     */
    WP_CUDA_CALLABLE int& operator[](int i)
    {
        assert(i < ARRAY_MAX_DIMS);
        return dims[i];
    }
};

/// Binary-compatible view of a Python ``wp.array`` of element type ``T``.
///
/// Layout matches ``wp::array_t<T>`` in the native Warp runtime: ``T* data``,
/// ``T* grad``, :class:`shape_t` shape, ``int strides[4]``, ``int ndim``.
/// Strides are byte strides. ``grad`` is null when the array has no gradient.
template <typename T>
struct array_t
{
    WP_CUDA_CALLABLE array_t() : data(nullptr), grad(nullptr), shape(), strides(), ndim(0)
    {
    }

    /**
     * @brief Constructs a one-dimensional array view over an existing buffer.
     * @param[in] data Pointer to the first element of the buffer.
     * @param[in] size Number of elements in the buffer.
     * @param[in] grad Optional pointer to the gradient buffer, or nullptr if none.
     */
    WP_CUDA_CALLABLE array_t(T* data, int size, T* grad = nullptr) : data(data), grad(grad), ndim(1)
    {
        shape.dims[0] = size;
        shape.dims[1] = 0;
        shape.dims[2] = 0;
        shape.dims[3] = 0;
        strides[0] = sizeof(T);
        strides[1] = 0;
        strides[2] = 0;
        strides[3] = 0;
    }

    /**
     * @brief Returns whether the array has no backing data.
     * @return True if the data pointer is null; false otherwise.
     */
    WP_CUDA_CALLABLE bool empty() const
    {
        return !data;
    }

    T* data; ///< Pointer to the first element of the array data.
    T* grad; ///< Pointer to the gradient buffer, or null when the array has no gradient.
    shape_t shape; ///< Per-dimension element counts of the array.
    int strides[ARRAY_MAX_DIMS]; ///< Per-dimension byte strides between consecutive elements.
    int ndim; ///< Number of valid dimensions in the array.

    /**
     * @brief Implicitly converts the array view to a pointer to its underlying data.
     * @return Pointer to the first element of the array data.
     */
    WP_CUDA_CALLABLE operator T*() const
    {
        return data;
    }
};

/// 3-element float vector, binary-compatible with ``wp::vec_t<3, float>``.
struct vec3
{
    float c[3]; ///< The three vector components.

    WP_CUDA_CALLABLE vec3() : c()
    {
    }

    /**
     * @brief Returns the vector component at the given index.
     * @param[in] i Zero-based component index in the range [0, 2].
     * @return The component value at index i.
     */
    WP_CUDA_CALLABLE float operator[](int i) const
    {
        return c[i];
    }
    /**
     * @brief Returns a mutable reference to the vector component at the given index.
     * @param[in] i Zero-based component index in the range [0, 2].
     * @return Reference to the component at index i.
     */
    WP_CUDA_CALLABLE float& operator[](int i)
    {
        return c[i];
    }
};

/// Quaternion laid out as ``{x, y, z, w}``, binary-compatible with ``wp::quat_t<float>``.
struct quat
{
    float c[4]; ///< The quaternion components ordered as {x, y, z, w}.

    WP_CUDA_CALLABLE quat() : c()
    {
    }

    /**
     * @brief Returns the quaternion component at the given index.
     * @param[in] i Zero-based component index in the range [0, 3] (x, y, z, w).
     * @return The component value at index i.
     */
    WP_CUDA_CALLABLE float operator[](int i) const
    {
        return c[i];
    }
    /**
     * @brief Returns a mutable reference to the quaternion component at the given index.
     * @param[in] i Zero-based component index in the range [0, 3] (x, y, z, w).
     * @return Reference to the component at index i.
     */
    WP_CUDA_CALLABLE float& operator[](int i)
    {
        return c[i];
    }
};

/// 3x3 row-major float matrix, binary-compatible with ``wp::mat_t<3, 3, float>``.
struct mat33
{
    float data[3][3]; ///< Row-major 3x3 matrix elements.

    WP_CUDA_CALLABLE mat33() : data()
    {
    }
};

/// Rigid-body transform laid out as seven contiguous floats ``{p.x, p.y, p.z, q.x, q.y, q.z, q.w}``,
/// binary-compatible with ``wp::transform_t<float>``.
struct transform
{
    vec3 p; ///< Translation component of the transform.
    quat q; ///< Rotation component of the transform, as a quaternion {x, y, z, w}.

    WP_CUDA_CALLABLE transform()
    {
    }
};

/// 6-element spatial vector laid out as ``{w[0..2], v[0..2]}`` (angular then linear),
/// binary-compatible with ``wp::vec_t<6, float>`` (``wp::spatial_vector_t<float>``).
struct spatial_vector
{
    float w[3]; ///< Angular (rotational) part of the spatial vector.
    float v[3]; ///< Linear (translational) part of the spatial vector.

    WP_CUDA_CALLABLE spatial_vector() : w(), v()
    {
    }
};

} // namespace wp

// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaacsim/common/array/Export.h"

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace array
{

namespace details
{

/**
 * @brief Variant type representing all accepted forms of a shape specification.
 * @details
 * Scalar integer variants produce a 1-D shape whose single dimension equals the
 * given value. Vector variants are interpreted as an ordered list of dimension
 * sizes, one element per axis.
 */
using SupportedShapeSpecification = std::variant<int8_t,
                                                 int16_t,
                                                 int32_t,
                                                 int64_t,
                                                 uint8_t,
                                                 uint16_t,
                                                 uint32_t,
                                                 uint64_t,
                                                 std::vector<int8_t>,
                                                 std::vector<int16_t>,
                                                 std::vector<int32_t>,
                                                 std::vector<int64_t>,
                                                 std::vector<uint8_t>,
                                                 std::vector<uint16_t>,
                                                 std::vector<uint32_t>,
                                                 std::vector<uint64_t>>;

} // namespace details

/**
 * @class Shape
 * @brief Represents the multi-dimensional shape of an array.
 * @details
 * A Shape holds an ordered sequence of dimension sizes as signed integers,
 * one per axis. A default-constructed Shape is empty (zero dimensions). Dimensions
 * are accessed by index via @ref operator[](), with negative indices counting from
 * the last axis.
 *
 * Shape supports NumPy-style broadcasting semantics via @ref canBroadcastTo() and
 * @ref resolve().
 */
class ISAACSIM_COMMON_ARRAY_API Shape
{
public:
    /**
     * @brief Constructs a Shape from a supported shape specification.
     * @details
     * A scalar variant produces a 1-D shape; a vector variant produces an N-D shape
     * with one dimension per element.
     *
     * @param[in] shape Integer scalar or vector of dimension sizes.
     */
    Shape(const details::SupportedShapeSpecification& shape);

    /**
     * @brief Constructs a Shape from a brace-enclosed list of integral dimension sizes.
     * @tparam T An integral type used to express the dimension sizes.
     * @param[in] dimensions Ordered list of dimension sizes, one per axis.
     */
    template <typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
    Shape(std::initializer_list<T> dimensions)
    {
        m_shape.reserve(dimensions.size());
        for (const auto& dimension : dimensions)
        {
            m_shape.push_back(static_cast<int64_t>(dimension));
        }
    }

    /** @brief Constructs an empty, zero-dimensional shape. */
    Shape() = default;
    /** @brief Copy-constructs a shape. */
    Shape(const Shape& other) = default;
    /** @brief Move-constructs a shape. */
    Shape(Shape&& other) = default;
    ~Shape() = default;

    /** @brief Copy-assigns a shape. */
    Shape& operator=(const Shape& other) = default;
    /** @brief Move-assigns a shape. */
    Shape& operator=(Shape&& other) = default;

    /**
     * @brief Returns the size of the dimension at the given index.
     * @details
     * Negative indices are supported and count from the last axis:
     * index -1 refers to the last dimension.
     *
     * @param[in] index Axis index, in the range `[-ndim(), ndim())`.
     * @return Size of the specified dimension.
     */
    int64_t operator[](int64_t index) const;

    /**
     * @brief Checks whether two shapes are identical.
     * @param[in] other The Shape to compare against.
     * @return `true` if both shapes have the same number of dimensions and equal sizes.
     */
    bool operator==(const Shape& other) const;

    /**
     * @brief Checks whether two shapes differ.
     * @param[in] other The Shape to compare against.
     * @return `true` if the shapes differ in rank or in any dimension size.
     */
    bool operator!=(const Shape& other) const;

    /**
     * @brief Returns the dimension sizes as a vector of signed integers.
     * @return A vector of length @ref ndim() containing the size of each axis.
     */
    std::vector<int64_t> shape() const;

    /**
     * @brief Returns a read-only pointer to the stored dimension sizes.
     * @details The pointer remains valid until this Shape is assigned or destroyed.
     * @return Pointer to the first dimension, or an unspecified pointer for a zero-dimensional shape.
     */
    const int64_t* data() const;

    /**
     * @brief Returns the number of dimensions (rank) of this shape.
     * @return Number of axes.
     */
    size_t ndim() const;

    /**
     * @brief Returns the total number of elements described by this shape.
     * @return Product of all dimension sizes, or 1 for a zero-dimensional shape.
     * @throws std::invalid_argument if any dimension is negative.
     */
    size_t size() const;

    /**
     * @brief Returns whether this shape can be broadcast to @p other.
     * @details
     * Follows NumPy broadcasting rules: dimensions are compared from the trailing
     * axis, and each dimension of this shape must either equal the corresponding
     * dimension of @p other or be 1.
     *
     * @param[in] other The target shape to broadcast to.
     * @return `true` if this shape is broadcast-compatible with @p other.
     */
    bool canBroadcastTo(const Shape& other) const;

    /**
     * @brief Reshape helper: resolves a target shape against this shape's total element count.
     * @details
     * Returns a fully specified shape with the same total number of elements as this shape,
     * suitable for use as a reshape target. At most one dimension in @p other may be -1,
     * in which case its size is inferred so that the total number of elements equals
     * @ref size(). All other dimensions in @p other must be non-negative. This shape must
     * contain no negative dimensions.
     *
     * @param[in] other Target shape, optionally containing one -1 dimension to infer.
     * @return A fully specified Shape with the same total element count as this shape.
     * @throws std::invalid_argument if this shape contains negative dimensions, if
     *         @p other contains more than one -1 dimension, if @p other contains
     *         negative dimensions other than -1, or if the total element count is
     *         incompatible with @p other.
     */
    Shape resolve(const Shape& other) const;

    /**
     * @brief Returns a human-readable string representation of this shape.
     * @return A string such as `"(3, 4, 5)"`, or `"()"` for a scalar shape.
     */
    std::string toString() const;

protected:
    /**
     * @brief Ordered list of dimension sizes, one entry per axis.
     */
    std::vector<int64_t> m_shape;
};

} // namespace array
} // namespace common
} // namespace isaacsim

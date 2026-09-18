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

#include "isaacsim/common/array/Array.hpp"
#include "isaacsim/common/array/Export.h"

#include <cstdint>
#include <optional>
#include <vector>

/**
 * @file Functions.hpp
 * @brief Free functions that create @ref isaacsim::common::array::Array instances, rearrange their
 *        elements, and apply element-wise arithmetic, logical, and reduction operations.
 * @details
 * Every function here is named after, and behaves like, its NumPy counterpart, including
 * broadcasting, type promotion, and the treatment of empty and 0-D operands. Rather than repeat
 * that on each declaration, the documentation below mentions NumPy only where this library
 * deliberately departs from it, or where the corresponding NumPy spelling helps to locate the
 * equivalent call.
 *
 * One departure applies throughout: operations that NumPy can satisfy with a view always return a
 * newly allocated array here. An @ref isaacsim::common::array::Array is densely packed and stores
 * no per-axis strides, so a strided view of another array's buffer is not representable.
 */

namespace isaacsim
{
namespace common
{
namespace array
{

/**
 * @brief Creates an array of the given shape without initializing its elements.
 * @details
 * The returned array owns a freshly allocated buffer whose contents are unspecified: read an
 * element before writing it and you get whatever the allocator handed back. Use it only when every
 * element is about to be overwritten; reach for `zeros()` or `ones()` otherwise.
 *
 * A zero-element shape still allocates, so `Array::data()` is a valid, distinct pointer rather than null.
 *
 * @param[in] shape  Shape of the new array. An empty shape produces a 0-D array holding one element.
 * @param[in] dtype  Element type. Defaults to `float64`.
 * @param[in] device Device on which to allocate. Defaults to the CPU.
 * @return A new Array with uninitialized elements.
 */
ISAACSIM_COMMON_ARRAY_API Array empty(const Shape& shape,
                                      Dtype dtype = Dtype::Float64(),
                                      const Device& device = Device::Cpu());

/**
 * @brief Creates an array of the given shape with every element set to zero.
 * @details
 * Zero is written as a zeroed byte pattern, which is the representation of `0` for every supported
 * dtype: `false` for `bool`, `0` for the integers, and `+0.0` for `float32` and `float64`.
 *
 * @param[in] shape  Shape of the new array. An empty shape produces a 0-D array holding one element.
 * @param[in] dtype  Element type. Defaults to `float64`.
 * @param[in] device Device on which to allocate. Defaults to the CPU.
 * @return A new Array whose elements are all zero.
 */
ISAACSIM_COMMON_ARRAY_API Array zeros(const Shape& shape,
                                      Dtype dtype = Dtype::Float64(),
                                      const Device& device = Device::Cpu());

/**
 * @brief Creates an array of the given shape with every element set to one.
 * @details
 * The value is cast to `dtype`, so the elements are `true` for `bool`, `1` for the integers, and
 * `1.0` for `float32` and `float64`.
 *
 * @param[in] shape  Shape of the new array. An empty shape produces a 0-D array holding one element.
 * @param[in] dtype  Element type. Defaults to `float64`.
 * @param[in] device Device on which to allocate. Defaults to the CPU.
 * @return A new Array whose elements are all one.
 */
ISAACSIM_COMMON_ARRAY_API Array ones(const Shape& shape,
                                     Dtype dtype = Dtype::Float64(),
                                     const Device& device = Device::Cpu());

/**
 * @brief Adds two arrays element-wise.
 * @details
 * The operands are broadcast against each other and cast to their common promoted dtype
 * (see `subtract()` for a description of the shared broadcasting and promotion rules).
 * Boolean operands are added as a logical OR.
 *
 * @param[in] a Left-hand operand.
 * @param[in] b Right-hand operand.
 * @return A new Array holding the element-wise sum.
 * @throws std::invalid_argument if the operands reside on different devices or their shapes are
 *         not broadcast-compatible.
 */
ISAACSIM_COMMON_ARRAY_API Array add(const Array& a, const Array& b);

/**
 * @brief Overload of `add(const Array&, const Array&)` accepting a scalar, `std::vector`, or
 *        `std::vector<std::vector>` operand, converted to an Array with its dtype inferred from
 *        the value and
 * matching the other operand's device.
 */
ISAACSIM_COMMON_ARRAY_API Array add(const Array& a, const details::SupportedInputSpecification& b);

/** @copydoc add(const Array&, const details::SupportedInputSpecification&) */
ISAACSIM_COMMON_ARRAY_API Array add(const details::SupportedInputSpecification& a, const Array& b);

/**
 * @brief Subtracts two arrays element-wise.
 * @details
 * The operands are broadcast against each other: dimensions are compared
 * from the trailing axis, and each must either be equal or be 1. Both operands are then cast to
 * their common promoted dtype, which is also the dtype of the result. Promotion follows the same
 * rules, so mixing a signed and an unsigned integer widens to a signed type large enough to hold
 * both (falling back to `float64` when no such integer type exists), and mixing an integer with
 * `float32` widens to `float64` when the integer is wider than `float32`'s mantissa.
 *
 * Subtracting boolean arrays is not supported.
 *
 * @param[in] a Left-hand operand.
 * @param[in] b Right-hand operand.
 * @return A new Array holding the element-wise difference.
 * @throws std::invalid_argument if the operands reside on different devices, their shapes are not
 *         broadcast-compatible, or both operands are boolean.
 */
ISAACSIM_COMMON_ARRAY_API Array subtract(const Array& a, const Array& b);

/**
 * @brief Overload of `subtract(const Array&, const Array&)` accepting a scalar, `std::vector`,
 *        or `std::vector<std::vector>` operand, converted to an Array with its dtype inferred
 *        from the value
 * and matching the other operand's device.
 */
ISAACSIM_COMMON_ARRAY_API Array subtract(const Array& a, const details::SupportedInputSpecification& b);

/** @copydoc subtract(const Array&, const details::SupportedInputSpecification&) */
ISAACSIM_COMMON_ARRAY_API Array subtract(const details::SupportedInputSpecification& a, const Array& b);

/**
 * @brief Multiplies two arrays element-wise.
 * @details
 * The operands are broadcast against each other and cast to their common promoted dtype
 * (see `subtract()` for a description of the shared broadcasting and promotion rules).
 * Boolean operands are multiplied as a logical AND.
 *
 * @param[in] a Left-hand operand.
 * @param[in] b Right-hand operand.
 * @return A new Array holding the element-wise product.
 * @throws std::invalid_argument if the operands reside on different devices or their shapes are
 *         not broadcast-compatible.
 */
ISAACSIM_COMMON_ARRAY_API Array multiply(const Array& a, const Array& b);

/**
 * @brief Overload of `multiply(const Array&, const Array&)` accepting a scalar, `std::vector`,
 *        or `std::vector<std::vector>` operand, converted to an Array with its dtype inferred
 *        from the value
 * and matching the other operand's device.
 */
ISAACSIM_COMMON_ARRAY_API Array multiply(const Array& a, const details::SupportedInputSpecification& b);

/** @copydoc multiply(const Array&, const details::SupportedInputSpecification&) */
ISAACSIM_COMMON_ARRAY_API Array multiply(const details::SupportedInputSpecification& a, const Array& b);

/**
 * @brief Divides two arrays element-wise using true division.
 * @details
 * The operands are broadcast against each other (see `subtract()` for a description of the
 * shared broadcasting and promotion rules). The result is always
 * floating-point: it is
 * `float32` only when the promoted dtype of the operands is already
 * `float32`, and `float64` otherwise, so boolean
 * and integer operands produce a `float64` result.
 * Division by zero therefore yields infinity or NaN rather than
 * being undefined.
 *
 * @param[in] a Left-hand operand.
 * @param[in] b Right-hand operand.
 * @return A new Array holding the element-wise quotient.
 * @throws std::invalid_argument if the operands reside on different devices or their shapes are
 *         not broadcast-compatible.
 */
ISAACSIM_COMMON_ARRAY_API Array divide(const Array& a, const Array& b);

/**
 * @brief Overload of `divide(const Array&, const Array&)` accepting a scalar, `std::vector`,
 *        or `std::vector<std::vector>` operand, converted to an Array with its dtype inferred
 *        from the value
 * and matching the other operand's device.
 */
ISAACSIM_COMMON_ARRAY_API Array divide(const Array& a, const details::SupportedInputSpecification& b);

/** @copydoc divide(const Array&, const details::SupportedInputSpecification&) */
ISAACSIM_COMMON_ARRAY_API Array divide(const details::SupportedInputSpecification& a, const Array& b);

/**
 * @brief Computes the element-wise truth value of `NOT x`.
 * @details
 * Accepts operands of any dtype: an element is treated as `true` when it is non-zero (so `NaN` is
 * `true`), and the result is always a boolean Array with the operand's shape and
 * device. Use it to invert a mask without first casting it to `bool`.
 *
 * @param[in] a Operand.
 * @return A new boolean Array holding the element-wise logical negation.
 */
ISAACSIM_COMMON_ARRAY_API Array logicalNot(const Array& a);

/**
 * @brief Negates an array element-wise.
 * @details
 * The result keeps the operand's dtype, shape, and device. Integer negation wraps around on
 * overflow, so negating the minimum value of a signed type reproduces that value and
 * negating an unsigned value wraps modulo the type's range.
 *
 * Negating a boolean array is not supported; use `logicalNot()` instead.
 *
 * @param[in] a Operand.
 * @return A new Array holding the element-wise negation.
 * @throws std::invalid_argument if the operand is boolean.
 */
ISAACSIM_COMMON_ARRAY_API Array negative(const Array& a);

/**
 * @brief Computes the element-wise absolute value.
 * @details
 * The result keeps the operand's dtype, shape, and device. The operation is the identity for
 * unsigned and boolean operands. The absolute value of the minimum value of a
 * signed integer type overflows and reproduces that value.
 *
 * @param[in] a Operand.
 * @return A new Array holding the element-wise absolute value.
 */
ISAACSIM_COMMON_ARRAY_API Array absolute(const Array& a);

/**
 * @brief Tests whether all array elements evaluate to `true`.
 * @details
 * Accepts operands of any dtype: an element is treated as `true` when it is non-zero (so `NaN` is
 * `true`). The whole array is reduced, regardless of its rank, and the result is a
 * 0-D boolean Array on the operand's device, so a CUDA result stays on the device; read it with
 * `item<bool>()`. An empty array reduces to `true`.
 *
 * @param[in] a Operand.
 * @return A new 0-D boolean Array holding the reduction.
 */
ISAACSIM_COMMON_ARRAY_API Array all(const Array& a);

/**
 * @brief Tests whether any array element evaluates to `true`.
 * @details
 * Accepts operands of any dtype: an element is treated as `true` when it is non-zero (so `NaN` is
 * `true`). The whole array is reduced, regardless of its rank, and the result is a
 * 0-D boolean Array on the operand's device, so a CUDA result stays on the device; read it with
 * `item<bool>()`. An empty array reduces to `false`.
 *
 * @param[in] a Operand.
 * @return A new 0-D boolean Array holding the reduction.
 */
ISAACSIM_COMMON_ARRAY_API Array any(const Array& a);

/**
 * @brief Sums all array elements.
 * @details
 * The whole array is reduced, regardless of its rank, to a 0-D Array on the operand's device, so a
 * CUDA result stays on the device; read it with `item<T>()`. The sum accumulates in a
 * wider dtype than the operand so that narrow integers do not overflow: boolean and signed integer
 * operands accumulate in `int64`, unsigned integer operands in `uint64`, and floating-point
 * operands keep their own dtype. An empty array sums to 0.
 *
 * @note The summation order is unspecified and differs between devices: the CPU accumulates
 *       sequentially while the CUDA path folds partial sums in a tree. Integer results are
 *       identical either way, but a floating-point sum may differ in its last bits between the two,
 *       and neither reproduces NumPy's pairwise summation exactly.
 *
 * @param[in] a Operand.
 * @return A new 0-D Array holding the sum.
 */
ISAACSIM_COMMON_ARRAY_API Array sum(const Array& a);

/**
 * @brief Multiplies all array elements.
 * @details
 * The whole array is reduced, regardless of its rank, to a 0-D Array on the operand's device, so a
 * CUDA result stays on the device; read it with `item<T>()`. The product accumulates in the same
 * widened dtype as `sum()`. An empty array has a product of 1.
 *
 * @param[in] a Operand.
 * @return A new 0-D Array holding the product.
 */
ISAACSIM_COMMON_ARRAY_API Array prod(const Array& a);

/**
 * @brief Returns the smallest array element.
 * @details
 * The whole array is reduced, regardless of its rank, to a 0-D Array that keeps the operand's dtype
 * and device, so a CUDA result stays on the device; read it with `item<T>()`. `NaN` propagates: a
 * floating-point array containing one reduces to `NaN`.
 *
 * @note Named after NumPy's `numpy.amin` alias rather than `min` because `<windows.h>` defines
 *       `min` and `max` as function-like macros. The preprocessor runs before name lookup, so the
 *       namespace would not protect the call: `array::min(a)` expands the macro and fails to
 *       compile in any translation unit that includes `<windows.h>` without `NOMINMAX`.
 *
 * @param[in] a Operand.
 * @return A new 0-D Array holding the smallest element.
 * @throws std::invalid_argument if the operand is empty, which has no minimum.
 */
ISAACSIM_COMMON_ARRAY_API Array amin(const Array& a);

/**
 * @brief Returns the largest array element.
 * @details
 * The whole array is reduced, regardless of its rank, to a 0-D Array that keeps the operand's dtype
 * and device, so a CUDA result stays on the device; read it with `item<T>()`. `NaN` propagates: a
 * floating-point array containing one reduces to `NaN`.
 *
 * @note Named after NumPy's `numpy.amax` alias rather than `max`, for the reason given in
 *       `amin()`.
 *
 * @param[in] a Operand.
 * @return A new 0-D Array holding the largest element.
 * @throws std::invalid_argument if the operand is empty, which has no maximum.
 */
ISAACSIM_COMMON_ARRAY_API Array amax(const Array& a);

/**
 * @brief Permutes an array's axes.
 * @details
 * Axis `i` of the result is axis `axes[i]` of the operand, so the result has shape
 * `{a.shape()[axes[0]], a.shape()[axes[1]], ...}`. @p axes must be a permutation of
 * `[0, a.ndim())`; negative entries count from the last axis, as elsewhere in this library. Omitting
 * @p axes reverses the axis order, so a 0-D or 1-D operand comes back unchanged. The result keeps
 * the operand's dtype and device, and holds a copy rather than a view of the elements.
 *
 * @param[in] a    Operand.
 * @param[in] axes Permutation of the operand's axes, one entry per dimension, or `std::nullopt` to
 *                 reverse the axis order.
 * @return A new Array holding the permuted elements.
 * @throws std::invalid_argument if @p axes does not hold exactly one entry per dimension of @p a,
 *         if any entry is out of range, or if any axis is repeated.
 */
ISAACSIM_COMMON_ARRAY_API Array transpose(const Array& a, const std::optional<std::vector<int64_t>>& axes = std::nullopt);

/**
 * @brief Gathers elements along a single axis by index.
 * @details
 * Selects the positions listed in @p indices along @p axis, so the result has the operand's shape
 * with that axis resized to `indices.size()`. When @p axis is omitted the operand is flattened
 * first and the result is 1-D. Negative entries in @p indices count from the end of the axis, and a
 * negative @p axis counts from the last axis. Indices may repeat, which duplicates elements, and
 * may be empty, which produces a zero-sized axis. A 0-D operand is treated as 1-D of size 1 whether
 * or not an axis is given. The result keeps the operand's dtype and device, and holds a copy rather
 * than a view of the elements.
 *
 * @param[in] a       Operand.
 * @param[in] indices Positions to gather along @p axis.
 * @param[in] axis    Axis to gather along, or `std::nullopt` to gather from the flattened operand.
 * @return A new Array holding the gathered elements.
 * @throws std::invalid_argument if @p axis is out of range for the operand.
 * @throws std::out_of_range if any entry of @p indices is out of range for that axis. The two are
 *         kept distinct as NumPy does, which reports an out-of-range axis as an `AxisError` (a
 *         `ValueError`) but an out-of-range index as an `IndexError`.
 */
ISAACSIM_COMMON_ARRAY_API Array take(const Array& a,
                                     const std::vector<int64_t>& indices,
                                     const std::optional<int64_t>& axis = std::nullopt);

/**
 * @brief Operator form of `add()`.
 * @param[in] a Left-hand operand.
 * @param[in] b Right-hand operand.
 * @return A new Array holding the element-wise sum.
 */
ISAACSIM_COMMON_ARRAY_API Array operator+(const Array& a, const Array& b);

/** @copydoc operator+(const Array&, const Array&) */
ISAACSIM_COMMON_ARRAY_API Array operator+(const Array& a, const details::SupportedInputSpecification& b);

/** @copydoc operator+(const Array&, const Array&) */
ISAACSIM_COMMON_ARRAY_API Array operator+(const details::SupportedInputSpecification& a, const Array& b);

/**
 * @brief Operator form of `subtract()`.
 * @param[in] a Left-hand operand.
 * @param[in] b Right-hand operand.
 * @return A new Array holding the element-wise difference.
 */
ISAACSIM_COMMON_ARRAY_API Array operator-(const Array& a, const Array& b);

/** @copydoc operator-(const Array&, const Array&) */
ISAACSIM_COMMON_ARRAY_API Array operator-(const Array& a, const details::SupportedInputSpecification& b);

/** @copydoc operator-(const Array&, const Array&) */
ISAACSIM_COMMON_ARRAY_API Array operator-(const details::SupportedInputSpecification& a, const Array& b);

/**
 * @brief Operator form of `multiply()`.
 * @param[in] a Left-hand operand.
 * @param[in] b Right-hand operand.
 * @return A new Array holding the element-wise product.
 */
ISAACSIM_COMMON_ARRAY_API Array operator*(const Array& a, const Array& b);

/** @copydoc operator*(const Array&, const Array&) */
ISAACSIM_COMMON_ARRAY_API Array operator*(const Array& a, const details::SupportedInputSpecification& b);

/** @copydoc operator*(const Array&, const Array&) */
ISAACSIM_COMMON_ARRAY_API Array operator*(const details::SupportedInputSpecification& a, const Array& b);

/**
 * @brief Operator form of `divide()`.
 * @param[in] a Left-hand operand.
 * @param[in] b Right-hand operand.
 * @return A new Array holding the element-wise quotient.
 */
ISAACSIM_COMMON_ARRAY_API Array operator/(const Array& a, const Array& b);

/** @copydoc operator/(const Array&, const Array&) */
ISAACSIM_COMMON_ARRAY_API Array operator/(const Array& a, const details::SupportedInputSpecification& b);

/** @copydoc operator/(const Array&, const Array&) */
ISAACSIM_COMMON_ARRAY_API Array operator/(const details::SupportedInputSpecification& a, const Array& b);

/**
 * @brief Operator form of `negative()`.
 * @param[in] a Operand.
 * @return A new Array holding the element-wise negation.
 */
ISAACSIM_COMMON_ARRAY_API Array operator-(const Array& a);

/**
 * @brief Operator form of `logicalNot()`.
 * @param[in] a Operand.
 * @return A new boolean Array holding the element-wise logical negation.
 */
ISAACSIM_COMMON_ARRAY_API Array operator!(const Array& a);

} // namespace array
} // namespace common
} // namespace isaacsim

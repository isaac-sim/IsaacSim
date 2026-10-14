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
#include "isaacsim/common/math/Export.h"

#include <string>

/**
 * @file Rotation.hpp
 * @brief Rotation operations over @ref isaacsim::common::array::Array batches.
 * @details
 * Quaternions are the only rotation representation today; functions are named for the
 * representation they operate on (`normalizeQuaternion()`, `multiplyQuaternions()`, ...) so this
 * file can grow other representations later without renaming what is already here.
 *
 * A quaternion is four contiguous elements. Every function accepts them in either component order,
 * `xyzw` (scalar last, the order Warp, the physics engine tensor buffers, and the USD foundation
 * APIs use) or `wxyz` (scalar first), selected with the `rotationFormat` argument; the default,
 * `xyzw`, needs no reordering at any of those boundaries.
 *
 * Every function takes and returns an @ref isaacsim::common::array::Array shaped `(4,)` for a
 * single quaternion or `(N, 4)` for a batch, and correspondingly `(3,)` or `(N, 3)` for the vectors
 * a rotation applies to. A single quaternion broadcasts against a batch of vectors and a single
 * vector against a batch of quaternions; any other mismatch is an error, as is mixing devices
 * between two operands. Results carry the operands' device, and are `float64` regardless of the
 * operand's dtype.
 *
 * Element-wise arithmetic on quaternion batches -- adding two of them, negating one -- is already
 * `add()` and `negative()` in `isaacsim/common/array/Functions.hpp`, so it is not repeated here.
 * This header holds the operations whose meaning depends on the four elements being one rotation.
 */

namespace isaacsim
{
namespace common
{
namespace math
{

/**
 * @brief Scales each quaternion to unit length.
 * @details
 * A quaternion of zero length has no direction to preserve, and normalizing it returns the identity
 * rather than a division by zero. Callers rely on that: averaging a window of quaternions that
 * cancel exactly is expected to yield the identity, not NaNs.
 *
 * @param[in] quaternions Quaternion or batch of quaternions, ordered per @p rotationFormat.
 * @param[in] rotationFormat Component order of @p quaternions and the result: @c "xyzw" or @c "wxyz".
 * @return A new Array of unit quaternions with the operand's shape, ordered per @p rotationFormat.
 * @throws std::invalid_argument if the last axis is not of size four, or if @p rotationFormat is
 *         neither @c "xyzw" nor @c "wxyz".
 */
ISAACSIM_COMMON_MATH_API isaacsim::common::array::Array normalizeQuaternion(
    const isaacsim::common::array::Array& quaternions, const std::string& rotationFormat = "xyzw");

/**
 * @brief Conjugates each quaternion, negating its vector part.
 * @details
 * For a unit quaternion the conjugate is the inverse rotation, which is what
 * @ref rotateByQuaternionInverse() applies. @ref inverseQuaternion() covers the general case.
 *
 * @param[in] quaternions Quaternion or batch of quaternions, ordered per @p rotationFormat.
 * @param[in] rotationFormat Component order of @p quaternions and the result: @c "xyzw" or @c "wxyz".
 * @return A new Array of conjugates with the operand's shape, ordered per @p rotationFormat.
 * @throws std::invalid_argument if the last axis is not of size four, or if @p rotationFormat is
 *         neither @c "xyzw" nor @c "wxyz".
 */
ISAACSIM_COMMON_MATH_API isaacsim::common::array::Array conjugateQuaternion(
    const isaacsim::common::array::Array& quaternions, const std::string& rotationFormat = "xyzw");

/**
 * @brief Inverts each quaternion.
 * @details
 * The inverse is the conjugate divided by the squared norm, so it undoes a rotation whether or not
 * the quaternion is normalized. A quaternion of zero length inverts to the identity, for the reason
 * @ref normalizeQuaternion() returns it.
 *
 * @param[in] quaternions Quaternion or batch of quaternions, ordered per @p rotationFormat.
 * @param[in] rotationFormat Component order of @p quaternions and the result: @c "xyzw" or @c "wxyz".
 * @return A new Array of inverses with the operand's shape, ordered per @p rotationFormat.
 * @throws std::invalid_argument if the last axis is not of size four, or if @p rotationFormat is
 *         neither @c "xyzw" nor @c "wxyz".
 */
ISAACSIM_COMMON_MATH_API isaacsim::common::array::Array inverseQuaternion(const isaacsim::common::array::Array& quaternions,
                                                                          const std::string& rotationFormat = "xyzw");

/**
 * @brief Composes two rotations with the Hamilton product.
 * @details
 * The right operand applies first: `multiplyQuaternions(body, mount)` is the mount offset followed
 * by the body rotation, which is how a sensor frame mounted on a body composes. The product is not
 * commutative.
 *
 * @param[in] a Left-hand quaternion or batch, ordered per @p rotationFormat.
 * @param[in] b Right-hand quaternion or batch, ordered per @p rotationFormat, applied first.
 * @param[in] rotationFormat Component order of @p a, @p b, and the result: @c "xyzw" or @c "wxyz".
 * @return A new Array of products, batched to whichever operand is batched, ordered per
 *         @p rotationFormat.
 * @throws std::invalid_argument if a last axis is not of size four, if the batch sizes neither
 *         match nor broadcast, if the operands are on different devices, or if @p rotationFormat is
 *         neither @c "xyzw" nor @c "wxyz".
 */
ISAACSIM_COMMON_MATH_API isaacsim::common::array::Array multiplyQuaternions(const isaacsim::common::array::Array& a,
                                                                            const isaacsim::common::array::Array& b,
                                                                            const std::string& rotationFormat = "xyzw");

/**
 * @brief Computes the four-element dot product of two quaternions.
 * @details
 * The sign of the result tells whether two quaternions describe the same rotation the same way
 * round: a quaternion and its negation are one rotation, so a negative dot product means the two
 * are on opposite hemispheres. @ref averageQuaternions() uses that to align a window before
 * averaging it.
 *
 * @param[in] a Left-hand quaternion or batch, ordered per @p rotationFormat.
 * @param[in] b Right-hand quaternion or batch, ordered per @p rotationFormat.
 * @param[in] rotationFormat Component order of @p a and @p b: @c "xyzw" or @c "wxyz". The result is
 *            a plain sum of products, so it does not depend on which format was used, as long as
 *            both operands agree with it.
 * @return A new Array of dot products, shaped `(N,)` for batched operands and 0-D for two single
 *         quaternions.
 * @throws std::invalid_argument if a last axis is not of size four, if the batch sizes neither
 *         match nor broadcast, if the operands are on different devices, or if @p rotationFormat is
 *         neither @c "xyzw" nor @c "wxyz".
 */
ISAACSIM_COMMON_MATH_API isaacsim::common::array::Array dotQuaternions(const isaacsim::common::array::Array& a,
                                                                       const isaacsim::common::array::Array& b,
                                                                       const std::string& rotationFormat = "xyzw");

/**
 * @brief Rotates vectors by quaternions.
 * @details
 * Applies `q v q^-1` without forming a rotation matrix. The quaternions are normalized first, so a
 * caller need not do it.
 *
 * @param[in] quaternions Quaternion or batch of quaternions, ordered per @p rotationFormat.
 * @param[in] vectors Vector or batch of vectors, ordered `xyz`.
 * @param[in] rotationFormat Component order of @p quaternions: @c "xyzw" or @c "wxyz".
 * @return A new Array of rotated vectors, batched to whichever operand is batched.
 * @throws std::invalid_argument if the quaternion last axis is not four, the vector last axis is
 *         not three, the batch sizes neither match nor broadcast, the operands are on different
 *         devices, or @p rotationFormat is neither @c "xyzw" nor @c "wxyz".
 */
ISAACSIM_COMMON_MATH_API isaacsim::common::array::Array rotateByQuaternion(const isaacsim::common::array::Array& quaternions,
                                                                           const isaacsim::common::array::Array& vectors,
                                                                           const std::string& rotationFormat = "xyzw");

/**
 * @brief Rotates vectors by the inverse of quaternions.
 * @details
 * The counterpart of @ref rotateByQuaternion(), taking a world-frame quantity into the frame the
 * quaternion describes. Rotating a velocity into a sensor's own frame is this call.
 *
 * @param[in] quaternions Quaternion or batch of quaternions, ordered per @p rotationFormat.
 * @param[in] vectors Vector or batch of vectors, ordered `xyz`.
 * @param[in] rotationFormat Component order of @p quaternions: @c "xyzw" or @c "wxyz".
 * @return A new Array of rotated vectors, batched to whichever operand is batched.
 * @throws std::invalid_argument if the quaternion last axis is not four, the vector last axis is
 *         not three, the batch sizes neither match nor broadcast, the operands are on different
 *         devices, or @p rotationFormat is neither @c "xyzw" nor @c "wxyz".
 */
ISAACSIM_COMMON_MATH_API isaacsim::common::array::Array rotateByQuaternionInverse(
    const isaacsim::common::array::Array& quaternions,
    const isaacsim::common::array::Array& vectors,
    const std::string& rotationFormat = "xyzw");

/**
 * @brief Averages a batch of quaternions into one mean rotation.
 * @details
 * Signs are aligned against the first quaternion before summing, because a quaternion and its
 * negation are the same rotation and an unaligned window averages toward zero. The sum is then
 * normalized, so a window that cancels exactly yields the identity.
 *
 * Each element is scaled to unit length before it is summed, so a longer quaternion does not weigh
 * more heavily than a shorter one describing the same rotation.
 *
 * This is the linear mean, not the eigenvector of the outer-product matrix. Over the small,
 * closely-spaced windows a sensor filter averages, the two agree to well within sensor noise.
 *
 * @param[in] quaternions Batch of quaternions, ordered per @p rotationFormat.
 * @param[in] rotationFormat Component order of @p quaternions and the result: @c "xyzw" or @c "wxyz".
 * @return A new Array shaped `(4,)` holding the mean rotation, ordered per @p rotationFormat, or the
 *         identity for an empty batch.
 * @throws std::invalid_argument if the last axis is not of size four, or if @p rotationFormat is
 *         neither @c "xyzw" nor @c "wxyz".
 */
ISAACSIM_COMMON_MATH_API isaacsim::common::array::Array averageQuaternions(const isaacsim::common::array::Array& quaternions,
                                                                           const std::string& rotationFormat = "xyzw");

} // namespace math
} // namespace common
} // namespace isaacsim

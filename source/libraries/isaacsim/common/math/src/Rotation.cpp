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

#include "isaacsim/common/math/Rotation.hpp"

#include "isaacsim/common/array/Dtype.hpp"
#include "isaacsim/common/array/Functions.hpp"
#include "isaacsim/common/array/Shape.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace math
{
namespace
{

using isaacsim::common::array::Array;
using isaacsim::common::array::Device;
using isaacsim::common::array::Dtype;
using isaacsim::common::array::Shape;

//! One quaternion, held as the four `xyzw` components the array carries.
struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};

//! One vector, held as the three `xyz` components the array carries.
struct Vector
{
    double x;
    double y;
    double z;
};

/**
 * @brief Rejects a @c rotationFormat other than the two this library understands.
 * @param[in] rotationFormat Format to check.
 * @throws std::invalid_argument if @p rotationFormat is neither `xyzw` nor `wxyz`.
 */
void validateRotationFormat(const std::string& rotationFormat)
{
    if (rotationFormat != "xyzw" && rotationFormat != "wxyz")
    {
        throw std::invalid_argument("Invalid rotation format: '" + rotationFormat +
                                    "'. Supported formats are 'xyzw' and 'wxyz'");
    }
}

//! Where x, y, z, and w sit within one quaternion row of a given @c rotationFormat.
struct QuaternionLayout
{
    size_t x;
    size_t y;
    size_t z;
    size_t w;
};

/**
 * @brief Resolves the component offsets for a validated @c rotationFormat.
 * @param[in] rotationFormat Either `xyzw` or `wxyz`.
 * @return The offset of each component within one row of that format.
 */
QuaternionLayout resolveLayout(const std::string& rotationFormat)
{
    return rotationFormat == "wxyz" ? QuaternionLayout{ 1, 2, 3, 0 } : QuaternionLayout{ 0, 1, 2, 3 };
}

/**
 * @brief Reads an operand into contiguous host doubles, checking its trailing dimension.
 * @details
 * The elements are always read on the CPU as `float64`: quaternion arithmetic mixes components
 * across a small, fixed-size row (Hamilton products, cross products, per-row norms), which is not
 * expressible as the element-wise and broadcast operations `isaacsim/common/array/Functions.hpp`
 * provides, so it cannot be composed from device-side array primitives. A CUDA operand therefore
 * round-trips through the host, and the caller gets the result back on its own device.
 *
 * @param[in] array Operand to read.
 * @param[in] components Required size of the last axis, four for a quaternion and three for a vector.
 * @param[in] name Operand name, for the error message.
 * @return Its elements, batch-major, one row per quaternion or vector.
 * @throws std::invalid_argument if the last axis is not @p components long.
 */
std::vector<double> readComponents(const Array& array, size_t components, const char* name)
{
    const Shape& shape = array.shape();
    const size_t trailing = array.ndim() == 0 ? 0 : shape[array.ndim() - 1];
    if (array.ndim() == 0 || array.ndim() > 2 || trailing != components)
    {
        throw std::invalid_argument(std::string(name) + " must be shaped (" + std::to_string(components) +
                                    ",) or (N, " + std::to_string(components) + "), got " + shape.toString());
    }
    const Array host = array.toDtype(Dtype::Float64()).toDevice(Device::Cpu());
    const double* elements = static_cast<const double*>(host.data());
    return std::vector<double>(elements, elements + host.size());
}

/**
 * @brief Rejects operands that live on different devices.
 * @details
 * `isaacsim::common::array` throws for a mixed-device binary operation rather than guessing which
 * device the caller wanted, and these functions follow it.
 *
 * @param[in] a First operand.
 * @param[in] b Second operand.
 * @param[in] callerName Calling function's name, for the error message.
 * @throws std::invalid_argument if the operands are on different devices.
 */
void checkSameDevice(const Array& a, const Array& b, const char* callerName)
{
    if (a.device() != b.device())
    {
        throw std::invalid_argument(std::string(callerName) + "(): operands are on different devices, " +
                                    a.device().toString() + " and " + b.device().toString());
    }
}

/**
 * @brief Resolves the batch size two operands share.
 * @details
 * Equal batches pair up element-wise, and a single element broadcasts against a batch. Anything
 * else has no meaning: there is no rule that would pair three quaternions with five vectors.
 *
 * @param[in] left Number of rows in the first operand.
 * @param[in] right Number of rows in the second operand.
 * @return The resulting batch size.
 * @throws std::invalid_argument if the batches neither match nor broadcast.
 */
size_t resolveBatch(size_t left, size_t right)
{
    if (left == right || right == 1)
    {
        return left;
    }
    if (left == 1)
    {
        return right;
    }
    throw std::invalid_argument("operands hold " + std::to_string(left) + " and " + std::to_string(right) +
                                " elements, which neither match nor broadcast");
}

/**
 * @brief Reads one quaternion out of a component block, in the internal `xyzw` order.
 * @param[in] elements Components of every quaternion, batch-major, in @p layout order.
 * @param[in] index Row to read, ignored for a single-element operand, which broadcasts.
 * @param[in] layout Component offsets of @p elements' rotation format.
 * @return The quaternion at that row.
 */
Quaternion readQuaternion(const std::vector<double>& elements, size_t index, const QuaternionLayout& layout)
{
    const size_t offset = (elements.size() == 4 ? 0 : index) * 4;
    return { elements[offset + layout.x], elements[offset + layout.y], elements[offset + layout.z],
             elements[offset + layout.w] };
}

/**
 * @brief Writes one quaternion into a result block, converting from the internal `xyzw` order.
 * @param[out] result Components of every result quaternion, batch-major, in @p layout order.
 * @param[in] index Row to write.
 * @param[in] q Quaternion to write, in the internal `xyzw` order.
 * @param[in] layout Component offsets of the result's rotation format.
 */
void writeQuaternion(std::vector<double>& result, size_t index, const Quaternion& q, const QuaternionLayout& layout)
{
    const size_t offset = index * 4;
    result[offset + layout.x] = q.x;
    result[offset + layout.y] = q.y;
    result[offset + layout.z] = q.z;
    result[offset + layout.w] = q.w;
}

/**
 * @brief Reads one vector out of a component block.
 * @param[in] elements Components of every vector, batch-major.
 * @param[in] index Row to read, ignored for a single-element operand, which broadcasts.
 * @return The vector at that row.
 */
Vector readVector(const std::vector<double>& elements, size_t index)
{
    const size_t offset = (elements.size() == 3 ? 0 : index) * 3;
    return { elements[offset], elements[offset + 1], elements[offset + 2] };
}

/**
 * @brief Scales a quaternion to unit length.
 * @param[in] q Quaternion to scale.
 * @return The unit quaternion, or the identity when @p q has no length to preserve.
 */
Quaternion normalizeQuaternion(const Quaternion& q)
{
    const double length = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (length == 0.0)
    {
        return { 0.0, 0.0, 0.0, 1.0 };
    }
    return { q.x / length, q.y / length, q.z / length, q.w / length };
}

/**
 * @brief Composes two rotations with the Hamilton product.
 * @param[in] a Left-hand quaternion.
 * @param[in] b Right-hand quaternion, applied first.
 * @return Their product.
 */
Quaternion composeQuaternions(const Quaternion& a, const Quaternion& b)
{
    return { a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
             a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
}

/**
 * @brief Rotates a vector by a unit quaternion, in the cross-product form rather than as a matrix.
 * @param[in] q Unit quaternion to rotate by.
 * @param[in] v Vector to rotate.
 * @return The rotated vector.
 */
Vector rotateVector(const Quaternion& q, const Vector& v)
{
    const Vector u{ q.x, q.y, q.z };
    const Vector uv{ u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x };
    const Vector uuv{ u.y * uv.z - u.z * uv.y, u.z * uv.x - u.x * uv.z, u.x * uv.y - u.y * uv.x };
    return { v.x + 2.0 * (q.w * uv.x + uuv.x), v.y + 2.0 * (q.w * uv.y + uuv.y), v.z + 2.0 * (q.w * uv.z + uuv.z) };
}

/**
 * @brief Builds the result array, matching the operands' batching and device.
 * @param[in] values Result elements, batch-major.
 * @param[in] components Elements per row.
 * @param[in] batched Whether the result keeps a leading batch axis.
 * @param[in] device Device to place the result on.
 * @return The result array, `float64`.
 */
Array buildResult(const std::vector<double>& values, size_t components, bool batched, const Device& device)
{
    Array result(values, Dtype::Float64(), Device::Cpu());
    if (batched)
    {
        result =
            result.reshape(Shape({ static_cast<int64_t>(values.size() / components), static_cast<int64_t>(components) }));
    }
    return result.toDevice(device);
}

} // namespace

Array normalizeQuaternion(const Array& quaternions, const std::string& rotationFormat)
{
    validateRotationFormat(rotationFormat);
    const QuaternionLayout layout = resolveLayout(rotationFormat);
    const std::vector<double> elements = readComponents(quaternions, 4, "quaternions");
    std::vector<double> result(elements.size());
    for (size_t index = 0; index < elements.size() / 4; ++index)
    {
        writeQuaternion(result, index, normalizeQuaternion(readQuaternion(elements, index, layout)), layout);
    }
    return buildResult(result, 4, quaternions.ndim() == 2, quaternions.device());
}

Array conjugateQuaternion(const Array& quaternions, const std::string& rotationFormat)
{
    validateRotationFormat(rotationFormat);
    const QuaternionLayout layout = resolveLayout(rotationFormat);
    const std::vector<double> elements = readComponents(quaternions, 4, "quaternions");
    std::vector<double> result(elements.size());
    for (size_t index = 0; index < elements.size() / 4; ++index)
    {
        const Quaternion q = readQuaternion(elements, index, layout);
        writeQuaternion(result, index, { -q.x, -q.y, -q.z, q.w }, layout);
    }
    return buildResult(result, 4, quaternions.ndim() == 2, quaternions.device());
}

Array inverseQuaternion(const Array& quaternions, const std::string& rotationFormat)
{
    validateRotationFormat(rotationFormat);
    const QuaternionLayout layout = resolveLayout(rotationFormat);
    const std::vector<double> elements = readComponents(quaternions, 4, "quaternions");
    std::vector<double> result(elements.size());
    for (size_t index = 0; index < elements.size() / 4; ++index)
    {
        const Quaternion q = readQuaternion(elements, index, layout);
        const double squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        // A quaternion of zero length has no inverse rotation, so report the identity, as
        // normalizeQuaternion() does, rather than dividing by zero.
        const double scale = squared == 0.0 ? 0.0 : 1.0 / squared;
        writeQuaternion(
            result, index, { -q.x * scale, -q.y * scale, -q.z * scale, squared == 0.0 ? 1.0 : q.w * scale }, layout);
    }
    return buildResult(result, 4, quaternions.ndim() == 2, quaternions.device());
}

Array multiplyQuaternions(const Array& a, const Array& b, const std::string& rotationFormat)
{
    validateRotationFormat(rotationFormat);
    checkSameDevice(a, b, "multiplyQuaternions");
    const QuaternionLayout layout = resolveLayout(rotationFormat);
    const std::vector<double> left = readComponents(a, 4, "a");
    const std::vector<double> right = readComponents(b, 4, "b");
    const size_t batch = resolveBatch(left.size() / 4, right.size() / 4);
    std::vector<double> result(batch * 4);
    for (size_t index = 0; index < batch; ++index)
    {
        const Quaternion composed =
            composeQuaternions(readQuaternion(left, index, layout), readQuaternion(right, index, layout));
        writeQuaternion(result, index, composed, layout);
    }
    return buildResult(result, 4, a.ndim() == 2 || b.ndim() == 2, a.device());
}

Array dotQuaternions(const Array& a, const Array& b, const std::string& rotationFormat)
{
    validateRotationFormat(rotationFormat);
    checkSameDevice(a, b, "dotQuaternions");
    const QuaternionLayout layout = resolveLayout(rotationFormat);
    const std::vector<double> left = readComponents(a, 4, "a");
    const std::vector<double> right = readComponents(b, 4, "b");
    const size_t batch = resolveBatch(left.size() / 4, right.size() / 4);
    std::vector<double> result(batch);
    for (size_t index = 0; index < batch; ++index)
    {
        const Quaternion p = readQuaternion(left, index, layout);
        const Quaternion q = readQuaternion(right, index, layout);
        result[index] = p.x * q.x + p.y * q.y + p.z * q.z + p.w * q.w;
    }
    if (a.ndim() == 1 && b.ndim() == 1)
    {
        return Array(result[0], Dtype::Float64(), Device::Cpu()).toDevice(a.device());
    }
    return Array(result, Dtype::Float64(), Device::Cpu()).toDevice(a.device());
}

Array rotateByQuaternion(const Array& quaternions, const Array& vectors, const std::string& rotationFormat)
{
    validateRotationFormat(rotationFormat);
    checkSameDevice(quaternions, vectors, "rotateByQuaternion");
    const QuaternionLayout layout = resolveLayout(rotationFormat);
    const std::vector<double> rotations = readComponents(quaternions, 4, "quaternions");
    const std::vector<double> points = readComponents(vectors, 3, "vectors");
    const size_t batch = resolveBatch(rotations.size() / 4, points.size() / 3);
    std::vector<double> result(batch * 3);
    for (size_t index = 0; index < batch; ++index)
    {
        const Vector turned =
            rotateVector(normalizeQuaternion(readQuaternion(rotations, index, layout)), readVector(points, index));
        result[index * 3] = turned.x;
        result[index * 3 + 1] = turned.y;
        result[index * 3 + 2] = turned.z;
    }
    return buildResult(result, 3, quaternions.ndim() == 2 || vectors.ndim() == 2, quaternions.device());
}

Array rotateByQuaternionInverse(const Array& quaternions, const Array& vectors, const std::string& rotationFormat)
{
    validateRotationFormat(rotationFormat);
    checkSameDevice(quaternions, vectors, "rotateByQuaternionInverse");
    const QuaternionLayout layout = resolveLayout(rotationFormat);
    const std::vector<double> rotations = readComponents(quaternions, 4, "quaternions");
    const std::vector<double> points = readComponents(vectors, 3, "vectors");
    const size_t batch = resolveBatch(rotations.size() / 4, points.size() / 3);
    std::vector<double> result(batch * 3);
    for (size_t index = 0; index < batch; ++index)
    {
        // The conjugate of a unit quaternion is its inverse, so normalize first and conjugate after.
        const Quaternion unit = normalizeQuaternion(readQuaternion(rotations, index, layout));
        const Vector turned = rotateVector({ -unit.x, -unit.y, -unit.z, unit.w }, readVector(points, index));
        result[index * 3] = turned.x;
        result[index * 3 + 1] = turned.y;
        result[index * 3 + 2] = turned.z;
    }
    return buildResult(result, 3, quaternions.ndim() == 2 || vectors.ndim() == 2, quaternions.device());
}

Array averageQuaternions(const Array& quaternions, const std::string& rotationFormat)
{
    validateRotationFormat(rotationFormat);
    const QuaternionLayout layout = resolveLayout(rotationFormat);
    const std::vector<double> elements = readComponents(quaternions, 4, "quaternions");
    const size_t count = elements.size() / 4;
    // An empty batch has nothing to average, and reading a reference from it would run off the end.
    if (count == 0)
    {
        std::vector<double> identity(4);
        writeQuaternion(identity, 0, { 0.0, 0.0, 0.0, 1.0 }, layout);
        return Array(identity, Dtype::Float64(), Device::Cpu()).toDevice(quaternions.device());
    }

    // Each element is scaled to unit length before it is summed, so a longer quaternion does not
    // pull the mean toward itself. Every other entry point here treats a quaternion as a rotation
    // rather than a magnitude, and this keeps that true. It also gives the sign alignment below a
    // non-zero reference to work against, which a zero first element would not.
    Quaternion total{ 0.0, 0.0, 0.0, 0.0 };
    const Quaternion reference = normalizeQuaternion(readQuaternion(elements, 0, layout));
    for (size_t index = 0; index < count; ++index)
    {
        const Quaternion q = normalizeQuaternion(readQuaternion(elements, index, layout));
        // A quaternion and its negation are one rotation, so align signs against the first or an
        // alternating window averages to zero.
        const double alignment = q.x * reference.x + q.y * reference.y + q.z * reference.z + q.w * reference.w;
        const double sign = alignment >= 0.0 ? 1.0 : -1.0;
        total.x += sign * q.x;
        total.y += sign * q.y;
        total.z += sign * q.z;
        total.w += sign * q.w;
    }
    const Quaternion mean = normalizeQuaternion(total);
    std::vector<double> result(4);
    writeQuaternion(result, 0, mean, layout);
    return Array(result, Dtype::Float64(), Device::Cpu()).toDevice(quaternions.device());
}

} // namespace math
} // namespace common
} // namespace isaacsim

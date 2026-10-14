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

#include "ArrayTensorSpecification.hpp"
#include "TensorDescription.hpp"
#include "TensorSpecification.hpp"

#include <isaacsim/common/array/Array.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace isaacsim
{
namespace physics
{
namespace tensors
{
/** @brief Convert a concrete descriptor dtype to an Array dtype. */
inline array::Dtype toArrayDtype(DType dtype)
{
    switch (dtype)
    {
    case DType::eFloat32:
        return array::Dtype::Float32();
    case DType::eFloat64:
        return array::Dtype::Float64();
    case DType::eInt8:
        return array::Dtype::Int8();
    case DType::eInt16:
        return array::Dtype::Int16();
    case DType::eInt32:
        return array::Dtype::Int32();
    case DType::eInt64:
        return array::Dtype::Int64();
    case DType::eUInt8:
        return array::Dtype::UInt8();
    case DType::eUInt16:
        return array::Dtype::UInt16();
    case DType::eUInt32:
        return array::Dtype::UInt32();
    case DType::eUInt64:
        return array::Dtype::UInt64();
    case DType::eBool:
        return array::Dtype::Bool();
    default:
        throw std::invalid_argument("An unspecified tensor dtype cannot describe Array storage");
    }
}
/** @brief Convert an Array dtype to the descriptor vocabulary. */
inline DType toTensorDtype(array::Dtype dtype)
{
    if (dtype == array::Dtype::Float32())
        return DType::eFloat32;
    if (dtype == array::Dtype::Float64())
        return DType::eFloat64;
    if (dtype == array::Dtype::Int8())
        return DType::eInt8;
    if (dtype == array::Dtype::Int16())
        return DType::eInt16;
    if (dtype == array::Dtype::Int32())
        return DType::eInt32;
    if (dtype == array::Dtype::Int64())
        return DType::eInt64;
    if (dtype == array::Dtype::UInt8())
        return DType::eUInt8;
    if (dtype == array::Dtype::UInt16())
        return DType::eUInt16;
    if (dtype == array::Dtype::UInt32())
        return DType::eUInt32;
    if (dtype == array::Dtype::UInt64())
        return DType::eUInt64;
    if (dtype == array::Dtype::Bool())
        return DType::eBool;
    throw std::invalid_argument("Unsupported tensor dtype");
}
/** @brief Borrow descriptor storage as an Array, retaining its optional owner.
 * @throws std::invalid_argument for layouts the Array transport cannot represent.
 * Negative strides remain supported by descriptor callbacks without this conversion.
 */
inline std::optional<array::Array> toArray(const TensorDescription& descriptor)
{
    // A shaped zero-element tensor is present even when its storage pointer is null.
    if (descriptor.data == nullptr && descriptor.shape.empty())
        return std::nullopt;
    const auto dtype = toArrayDtype(descriptor.dtype);
    if (!descriptor.strides.empty() && descriptor.strides.size() != descriptor.shape.size())
        throw std::invalid_argument("Tensor stride rank differs from shape rank");
    const bool empty = std::find(descriptor.shape.begin(), descriptor.shape.end(), 0) != descriptor.shape.end();
    auto strides = descriptor.strides;
    size_t span = 1;
    size_t contiguous = 1;
    for (size_t reverse = descriptor.shape.size(); reverse > 0; --reverse)
    {
        const size_t axis = reverse - 1;
        const int64_t dimension = descriptor.shape[axis];
        if (dimension < 0)
            throw std::invalid_argument("Negative tensor dimension");
        if (!strides.empty() && strides[axis] < 0)
        {
            if (!empty && dimension != 1)
                throw std::invalid_argument("Array providers cannot represent negative descriptor strides");
            strides[axis] = 1;
        }
        const size_t stride = strides.empty() ? contiguous : static_cast<size_t>(strides[axis]);
        if (dimension > 0)
        {
            const size_t extent = static_cast<size_t>(dimension - 1);
            if (extent && stride > (std::numeric_limits<size_t>::max() - span) / extent)
                throw std::overflow_error("Tensor storage span overflow");
            span += extent * stride;
        }
        if (dimension && contiguous > std::numeric_limits<size_t>::max() / static_cast<size_t>(dimension))
            throw std::overflow_error("Tensor shape overflow");
        contiguous *= static_cast<size_t>(dimension);
    }
    if (empty)
        span = 0;
    if (span > std::numeric_limits<size_t>::max() / dtype.size())
        throw std::overflow_error("Tensor byte span overflow");
    if (descriptor.device == DeviceKind::eEngineDefault)
        throw std::invalid_argument("A tensor buffer requires a concrete device");
    const auto device =
        descriptor.device == DeviceKind::eCpu ? array::Device::Cpu() : array::Device::Cuda(descriptor.deviceOrdinal);
    std::shared_ptr<std::byte[]> owner(descriptor.keepAlive, static_cast<std::byte*>(descriptor.data));
    return array::Array::fromBuffer(
        std::move(owner), span * dtype.size(), array::Shape(descriptor.shape), dtype, device, 0, strides);
}
/** @brief Retain Array storage in a descriptor without copying its elements. */
inline TensorDescription toTensorDescription(const array::Array& value)
{
    auto owner = std::make_shared<array::Array>(value);
    TensorDescription descriptor;
    descriptor.data = const_cast<void*>(static_cast<const array::Array&>(*owner).data());
    descriptor.dtype = toTensorDtype(value.dtype());
    descriptor.shape = value.shape().shape();
    descriptor.strides = value.strides();
    descriptor.device = value.device().isCpu() ? DeviceKind::eCpu : DeviceKind::eGpu;
    descriptor.deviceOrdinal = value.device().ordinal();
    descriptor.keepAlive = std::move(owner);
    return descriptor;
}
/** @brief Describe an optional Array using the unspecified descriptor sentinel. */
inline TensorDescription toTensorDescription(const std::optional<array::Array>& value)
{
    return value ? toTensorDescription(*value) : TensorDescription{};
}
/** @brief Convert declared operation capabilities to Array transport metadata. */
inline ArrayTensorSpecification toArraySpecification(const TensorSpecification& value)
{
    ArrayTensorSpecification result;
    result.dtype = toArrayDtype(value.dtype);
    if (value.deviceKind != DeviceKind::eEngineDefault)
        result.device = value.deviceKind == DeviceKind::eCpu ? array::Device::Cpu() : array::Device::Cuda();
    result.shapeHint = value.shapeHint;
    result.supports = value.supports;
    result.supportsIndexedRead = value.supportsIndexedRead;
    result.supportsIndexedWrite = value.supportsIndexedWrite;
    result.supportsMaskedWrite = value.supportsMaskedWrite;
    result.requiresHostData = value.requiresHostData;
    return result;
}
/** @brief Convert Array transport metadata to descriptor capability vocabulary. */
inline TensorSpecification toTensorSpecification(const ArrayTensorSpecification& value)
{
    TensorSpecification result;
    result.dtype = toTensorDtype(value.dtype);
    result.deviceKind = !value.device         ? DeviceKind::eEngineDefault :
                        value.device->isCpu() ? DeviceKind::eCpu :
                                                DeviceKind::eGpu;
    result.shapeHint = value.shapeHint;
    result.supports = value.supports;
    result.supportsIndexedRead = value.supportsIndexedRead;
    result.supportsIndexedWrite = value.supportsIndexedWrite;
    result.supportsMaskedWrite = value.supportsMaskedWrite;
    result.requiresHostData = value.requiresHostData;
    return result;
}
} // namespace tensors
} // namespace physics
} // namespace isaacsim

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

#include "isaacsim/common/array/DLPack.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace array
{

namespace
{

Dtype fromDLPackDtype(const DLDataType& dtype)
{
    if (dtype.lanes != 1)
    {
        throw std::invalid_argument("fromDLPack(): vectorized dtypes (lanes != 1) are not supported");
    }

    switch (static_cast<DLDataTypeCode>(dtype.code))
    {
    case kDLBool:
        if (dtype.bits == 8)
        {
            return Dtype::Bool();
        }
        break;
    case kDLInt:
        switch (dtype.bits)
        {
        case 8:
            return Dtype::Int8();
        case 16:
            return Dtype::Int16();
        case 32:
            return Dtype::Int32();
        case 64:
            return Dtype::Int64();
        }
        break;
    case kDLUInt:
        switch (dtype.bits)
        {
        case 8:
            return Dtype::UInt8();
        case 16:
            return Dtype::UInt16();
        case 32:
            return Dtype::UInt32();
        case 64:
            return Dtype::UInt64();
        }
        break;
    case kDLFloat:
        switch (dtype.bits)
        {
        case 32:
            return Dtype::Float32();
        case 64:
            return Dtype::Float64();
        }
        break;
    default:
        break;
    }

    throw std::invalid_argument("fromDLPack(): unsupported dtype (code=" + std::to_string(dtype.code) +
                                ", bits=" + std::to_string(dtype.bits) + ")");
}

DLDataType toDLPackDtype(const Dtype& dtype)
{
    switch (dtype.kind())
    {
    case Dtype::Kind::eBool:
        return { static_cast<uint8_t>(kDLBool), 8, 1 };
    case Dtype::Kind::eInt8:
        return { static_cast<uint8_t>(kDLInt), 8, 1 };
    case Dtype::Kind::eInt16:
        return { static_cast<uint8_t>(kDLInt), 16, 1 };
    case Dtype::Kind::eInt32:
        return { static_cast<uint8_t>(kDLInt), 32, 1 };
    case Dtype::Kind::eInt64:
        return { static_cast<uint8_t>(kDLInt), 64, 1 };
    case Dtype::Kind::eUInt8:
        return { static_cast<uint8_t>(kDLUInt), 8, 1 };
    case Dtype::Kind::eUInt16:
        return { static_cast<uint8_t>(kDLUInt), 16, 1 };
    case Dtype::Kind::eUInt32:
        return { static_cast<uint8_t>(kDLUInt), 32, 1 };
    case Dtype::Kind::eUInt64:
        return { static_cast<uint8_t>(kDLUInt), 64, 1 };
    case Dtype::Kind::eFloat32:
        return { static_cast<uint8_t>(kDLFloat), 32, 1 };
    case Dtype::Kind::eFloat64:
        return { static_cast<uint8_t>(kDLFloat), 64, 1 };
    }

    throw std::invalid_argument("toDLPack(): unsupported dtype");
}

Device fromDLPackDevice(const DLDevice& device)
{
    switch (device.device_type)
    {
    case kDLCPU:
        return Device::Cpu();
    case kDLCUDA:
        if (device.device_id < 0)
        {
            throw std::invalid_argument("fromDLPack(): CUDA device ID cannot be negative");
        }
        return Device::Cuda(device.device_id);
    default:
        throw std::invalid_argument("fromDLPack(): unsupported device type " +
                                    std::to_string(static_cast<int32_t>(device.device_type)));
    }
}

size_t getCheckedElementCount(const std::vector<int64_t>& dimensions, const char* functionName)
{
    size_t count = 1;
    for (const int64_t dimension : dimensions)
    {
        if (dimension < 0)
        {
            throw std::invalid_argument(std::string(functionName) + ": negative dimensions are not supported");
        }
        const size_t unsignedDimension = static_cast<size_t>(dimension);
        if (unsignedDimension != 0 && count > std::numeric_limits<size_t>::max() / unsignedDimension)
        {
            throw std::overflow_error(std::string(functionName) + ": tensor element count overflows size_t");
        }
        count *= unsignedDimension;
    }
    return count;
}

void validateContiguous(const DLTensor& tensor, const std::vector<int64_t>& dimensions, size_t elementCount)
{
    if (tensor.strides == nullptr || elementCount == 0)
    {
        return;
    }

    int64_t expectedStride = 1;
    for (size_t axis = dimensions.size(); axis-- > 0;)
    {
        const int64_t dimension = dimensions[axis];
        if (dimension > 1 && tensor.strides[axis] != expectedStride)
        {
            throw std::invalid_argument("fromDLPack(): only C-contiguous tensors are supported");
        }
        if (dimension != 0 && expectedStride > std::numeric_limits<int64_t>::max() / dimension)
        {
            throw std::overflow_error("fromDLPack(): tensor strides overflow int64_t");
        }
        expectedStride *= dimension;
    }
}

} // namespace

Array fromDLPack(const DLTensor& tensor)
{
    if (tensor.ndim < 0)
    {
        throw std::invalid_argument("fromDLPack(): tensor rank cannot be negative");
    }
    if (tensor.ndim > 0 && tensor.shape == nullptr)
    {
        throw std::invalid_argument("fromDLPack(): shape cannot be null for a tensor with non-zero rank");
    }

    const Dtype dtype = fromDLPackDtype(tensor.dtype);
    const Device device = fromDLPackDevice(tensor.device);
    std::vector<int64_t> dimensions;
    if (tensor.ndim > 0)
    {
        dimensions.assign(tensor.shape, tensor.shape + tensor.ndim);
    }
    const size_t elementCount = getCheckedElementCount(dimensions, "fromDLPack()");
    if (elementCount > std::numeric_limits<size_t>::max() / dtype.size())
    {
        throw std::overflow_error("fromDLPack(): tensor byte count overflows size_t");
    }
    const size_t byteCount = elementCount * dtype.size();
    if (elementCount != 0 && tensor.data == nullptr)
    {
        throw std::invalid_argument("fromDLPack(): data cannot be null for a non-empty tensor");
    }
    if (tensor.byte_offset > std::numeric_limits<size_t>::max())
    {
        throw std::overflow_error("fromDLPack(): byte offset overflows size_t");
    }
    const size_t byteOffset = static_cast<size_t>(tensor.byte_offset);
    if (byteOffset > std::numeric_limits<size_t>::max() - byteCount)
    {
        throw std::overflow_error("fromDLPack(): byte offset and tensor size overflow size_t");
    }

    validateContiguous(tensor, dimensions, elementCount);

    auto noOpDeleter = [](std::byte*) noexcept {};
    std::shared_ptr<std::byte[]> borrowedData(static_cast<std::byte*>(tensor.data), noOpDeleter);
    return Array::fromBuffer(std::move(borrowedData), Shape(dimensions), dtype, device, byteOffset);
}

DLTensor toDLPack(const Array& array)
{
    const std::vector<int64_t> dimensions = array.shape().shape();
    const Device device = array.device();
    if (dimensions.size() > static_cast<size_t>(std::numeric_limits<int32_t>::max()))
    {
        throw std::overflow_error("toDLPack(): array rank exceeds the DLPack int32 range");
    }
    const size_t elementCount = getCheckedElementCount(dimensions, "toDLPack()");
    if (elementCount > std::numeric_limits<size_t>::max() / array.dtype().size())
    {
        throw std::overflow_error("toDLPack(): array byte count overflows size_t");
    }

    DLTensor tensor{};
    tensor.data = elementCount ? const_cast<void*>(array.data()) : nullptr;
    tensor.device = { device.isCpu() ? kDLCPU : kDLCUDA, device.isCpu() ? 0 : device.ordinal() };
    tensor.ndim = static_cast<int32_t>(dimensions.size());
    tensor.dtype = toDLPackDtype(array.dtype());
    tensor.shape = dimensions.empty() ? nullptr : const_cast<int64_t*>(array.shape().data());
    tensor.strides = nullptr;
    tensor.byte_offset = 0;
    return tensor;
}

} // namespace array
} // namespace common
} // namespace isaacsim

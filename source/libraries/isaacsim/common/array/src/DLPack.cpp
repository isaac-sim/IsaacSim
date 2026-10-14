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

#include "details/ArrayUtils.hpp"

#include <algorithm>
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
        throw std::invalid_argument("DLPack import: vectorized dtypes (lanes != 1) are not supported");
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

    throw std::invalid_argument("DLPack import: unsupported dtype (code=" + std::to_string(dtype.code) +
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

    throw std::invalid_argument("DLPack export: unsupported dtype");
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
            throw std::invalid_argument("DLPack import: CUDA device ID cannot be negative");
        }
        return Device::Cuda(device.device_id);
    default:
        throw std::invalid_argument("DLPack import: unsupported device type " +
                                    std::to_string(static_cast<int32_t>(device.device_type)));
    }
}

size_t getCheckedElementCount(const std::vector<int64_t>& dimensions, const char* functionName)
{
    if (std::any_of(dimensions.begin(), dimensions.end(), [](int64_t dimension) { return dimension < 0; }))
    {
        throw std::invalid_argument(std::string(functionName) + ": negative dimensions are not supported");
    }
    if (std::find(dimensions.begin(), dimensions.end(), 0) != dimensions.end())
    {
        return 0;
    }
    size_t count = 1;
    for (const int64_t dimension : dimensions)
    {
        const size_t unsignedDimension = static_cast<size_t>(dimension);
        if (unsignedDimension != 0 && count > std::numeric_limits<size_t>::max() / unsignedDimension)
        {
            throw std::overflow_error(std::string(functionName) + ": tensor element count overflows size_t");
        }
        count *= unsignedDimension;
    }
    return count;
}

} // namespace

namespace
{

Array fromDLPackImpl(const DLTensor& tensor, std::shared_ptr<std::byte[]> data, bool writeable = true)
{
    if (tensor.ndim < 0)
    {
        throw std::invalid_argument("DLPack import: tensor rank cannot be negative");
    }
    if (tensor.ndim > 0 && tensor.shape == nullptr)
    {
        throw std::invalid_argument("DLPack import: shape cannot be null for a tensor with non-zero rank");
    }

    const Dtype dtype = fromDLPackDtype(tensor.dtype);
    const Device device = fromDLPackDevice(tensor.device);
    std::vector<int64_t> dimensions;
    if (tensor.ndim > 0)
    {
        dimensions.assign(tensor.shape, tensor.shape + tensor.ndim);
    }
    const size_t elementCount = getCheckedElementCount(dimensions, "DLPack import");
    if (elementCount > std::numeric_limits<size_t>::max() / dtype.size())
    {
        throw std::overflow_error("DLPack import: tensor byte count overflows size_t");
    }
    if (elementCount != 0 && tensor.data == nullptr)
    {
        throw std::invalid_argument("DLPack import: data cannot be null for a non-empty tensor");
    }
    if (tensor.byte_offset > std::numeric_limits<size_t>::max())
    {
        throw std::overflow_error("DLPack import: byte offset overflows size_t");
    }
    const size_t byteOffset = static_cast<size_t>(tensor.byte_offset);
    std::vector<int64_t> strides;
    if (tensor.strides != nullptr)
    {
        strides.assign(tensor.strides, tensor.strides + tensor.ndim);
    }
    const Shape shape(dimensions);
    const std::vector<int64_t> resolvedStrides = strides.empty() ? details::computeContiguousStrides(shape) : strides;
    const size_t storageSpan = details::computeStorageSpanNbytes(shape, resolvedStrides, dtype);
    if (byteOffset > std::numeric_limits<size_t>::max() - storageSpan)
    {
        throw std::overflow_error("DLPack import: byte offset and tensor span overflow size_t");
    }

    return details::ArrayStorageFactory::fromBuffer(
        std::move(data), byteOffset + storageSpan, shape, dtype, device, byteOffset, resolvedStrides, writeable);
}

// Fills the DLPack view metadata for `source`. `data` must already address element 0: DLPack permits
// a non-zero byte_offset, but warp, NumPy and PyTorch all read straight from `data` and ignore the
// field, so exporting an offset view as base pointer + byte_offset hands them the wrong element.
// Imports still honor a producer's byte_offset -- only what this library emits is normalized.
//
// `shape` and `strides` must outlive `tensor`; a contiguous layout publishes null strides.
void fillDLTensor(
    DLTensor& tensor, const Array& source, const void* data, const int64_t* shape, size_t ndim, const int64_t* strides)
{
    tensor.data = source.size() ? const_cast<void*>(data) : nullptr;
    tensor.device = { source.device().isCpu() ? kDLCPU : kDLCUDA,
                      source.device().isCpu() ? 0 : source.device().ordinal() };
    tensor.ndim = static_cast<int32_t>(ndim);
    tensor.dtype = toDLPackDtype(source.dtype());
    tensor.shape = ndim == 0 ? nullptr : const_cast<int64_t*>(shape);
    // isContiguous() is true for a 0-D array, so that case also takes the null-strides path.
    tensor.strides = source.isContiguous() ? nullptr : const_cast<int64_t*>(strides);
    tensor.byte_offset = 0;
}

struct ManagedTensorContext
{
    explicit ManagedTensorContext(const Array& source, std::uintptr_t consumerStream)
        : access(acquireRead(source, consumerStream)), shape(source.shape().shape()), strides(source.strides())
    {
        fillDLTensor(tensor.dl_tensor, source, access.data(), shape.data(), shape.size(), strides.data());
        tensor.manager_ctx = this;
        tensor.deleter = [](DLManagedTensor* managed) noexcept
        { delete static_cast<ManagedTensorContext*>(managed->manager_ctx); };
    }

    ArrayReadAccess access;
    std::vector<int64_t> shape;
    std::vector<int64_t> strides;
    DLManagedTensor tensor{};
};

struct VersionedManagedTensorContext
{
    explicit VersionedManagedTensorContext(const Array& source, std::uintptr_t consumerStream)
        : access(acquireRead(source, consumerStream)), shape(source.shape().shape()), strides(source.strides())
    {
        tensor.version = { DLPACK_MAJOR_VERSION, DLPACK_MINOR_VERSION };
        tensor.manager_ctx = this;
        tensor.deleter = [](DLManagedTensorVersioned* managed) noexcept
        { delete static_cast<VersionedManagedTensorContext*>(managed->manager_ctx); };
        tensor.flags = DLPACK_FLAG_BITMASK_READ_ONLY;
        fillDLTensor(tensor.dl_tensor, source, access.data(), shape.data(), shape.size(), strides.data());
    }

    ArrayReadAccess access;
    std::vector<int64_t> shape;
    std::vector<int64_t> strides;
    DLManagedTensorVersioned tensor{};
};

} // namespace

Array fromDLPack(const DLTensor& tensor)
{
    DLTensor contiguous = tensor;
    contiguous.strides = nullptr;
    Array result = fromDLPackBorrowed(contiguous);
    if (tensor.strides != nullptr && result.size() != 0)
    {
        const auto& strides = result.strides();
        for (int32_t axis = 0; axis < tensor.ndim; ++axis)
        {
            if (tensor.shape[axis] > 1 && tensor.strides[axis] != strides[axis])
            {
                throw std::invalid_argument("fromDLPack(): tensor must be C-contiguous");
            }
        }
    }
    return result;
}

DLTensor toDLPack(const Array& array)
{
    return toDLPackBorrowed(array);
}

Array fromDLPackBorrowed(const DLTensor& tensor)
{
    auto noOpDeleter = [](std::byte*) noexcept {};
    return fromDLPackImpl(
        tensor, std::shared_ptr<std::byte[]>(static_cast<std::byte*>(tensor.data), std::move(noOpDeleter)));
}

Array fromDLPackManaged(DLManagedTensor* tensor, std::uintptr_t producerStream)
{
    if (tensor == nullptr || tensor->deleter == nullptr)
    {
        throw std::invalid_argument("fromDLPackManaged(): managed tensor and its deleter must not be null");
    }
    auto owner = std::shared_ptr<std::byte[]>(
        static_cast<std::byte*>(tensor->dl_tensor.data), [tensor](std::byte*) noexcept { tensor->deleter(tensor); });
    Array result = fromDLPackImpl(tensor->dl_tensor, std::move(owner));
    if (result.device().isCpu())
    {
        if (producerStream != 0)
        {
            throw std::invalid_argument("fromDLPackManaged(): CPU tensors require stream 0");
        }
    }
    else
    {
        recordCudaWrite(result, producerStream);
    }
    return result;
}

Array fromDLPackVersioned(DLManagedTensorVersioned* tensor, std::uintptr_t producerStream)
{
    if (tensor == nullptr || tensor->deleter == nullptr)
    {
        throw std::invalid_argument("fromDLPackVersioned(): versioned managed tensor and its deleter must not be null");
    }
    if (tensor->version.major != DLPACK_MAJOR_VERSION)
    {
        tensor->deleter(tensor);
        throw std::invalid_argument("fromDLPackVersioned(): incompatible DLPack major version");
    }

    auto owner = std::shared_ptr<std::byte[]>(
        static_cast<std::byte*>(tensor->dl_tensor.data), [tensor](std::byte*) noexcept { tensor->deleter(tensor); });
    const bool writeable = (tensor->flags & DLPACK_FLAG_BITMASK_READ_ONLY) == 0;
    Array result = fromDLPackImpl(tensor->dl_tensor, std::move(owner), writeable);
    if (result.device().isCpu())
    {
        if (producerStream != 0)
        {
            throw std::invalid_argument("fromDLPackVersioned(): CPU tensors require stream 0");
        }
    }
    else
    {
        recordCudaWrite(result, producerStream);
    }
    return result;
}

DLTensor toDLPackBorrowed(const Array& array)
{
    const std::vector<int64_t> dimensions = array.shape().shape();
    if (dimensions.size() > static_cast<size_t>(std::numeric_limits<int32_t>::max()))
    {
        throw std::overflow_error("toDLPackBorrowed(): array rank exceeds the DLPack int32 range");
    }
    const size_t elementCount = getCheckedElementCount(dimensions, "toDLPackBorrowed()");
    if (elementCount > std::numeric_limits<size_t>::max() / array.dtype().size())
    {
        throw std::overflow_error("toDLPackBorrowed(): array byte count overflows size_t");
    }

    DLTensor tensor{};
    fillDLTensor(tensor, array, array.data(), array.shape().data(), dimensions.size(), array.strides().data());
    return tensor;
}

DLManagedTensor* toDLPackManaged(const Array& array, std::uintptr_t consumerStream)
{
    if (array.shape().shape().size() > static_cast<size_t>(std::numeric_limits<int32_t>::max()))
    {
        throw std::overflow_error("toDLPackManaged(): array rank exceeds the DLPack int32 range");
    }
    auto context = std::make_unique<ManagedTensorContext>(array, consumerStream);
    DLManagedTensor* tensor = &context->tensor;
    context.release();
    return tensor;
}

DLManagedTensorVersioned* toDLPackVersioned(const Array& array, std::uintptr_t consumerStream)
{
    if (array.shape().shape().size() > static_cast<size_t>(std::numeric_limits<int32_t>::max()))
    {
        throw std::overflow_error("toDLPackVersioned(): array rank exceeds the DLPack int32 range");
    }
    auto context = std::make_unique<VersionedManagedTensorContext>(array, consumerStream);
    DLManagedTensorVersioned* tensor = &context->tensor;
    context.release();
    return tensor;
}

} // namespace array
} // namespace common
} // namespace isaacsim

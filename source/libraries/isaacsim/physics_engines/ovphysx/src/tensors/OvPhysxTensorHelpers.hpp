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

// Pure, dependency-light tensor helpers shared by OvPhysxEntityViews.cpp and its
// unit tests. Kept free of ovphysx / USD headers so the in-process doctest suite
// can compile them directly without linking the backend .so.

#include <dlpack/dlpack.h>
#include <isaacsim/physics/registration/tensors/TensorDescription.hpp>

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{

inline size_t checkedSizeProduct(size_t left, size_t right, const char* label)
{
    if (right != 0 && left > std::numeric_limits<size_t>::max() / right)
    {
        throw std::overflow_error(std::string(label) + ": element count overflow");
    }
    return left * right;
}

template <typename DimensionRange>
inline size_t checkedElementCountFromRange(const DimensionRange& dimensions, const char* label)
{
    size_t elementCount = 1;
    for (const int64_t dimension : dimensions)
    {
        if (dimension < 0)
        {
            throw std::runtime_error(std::string(label) + ": negative tensor dimension");
        }

        const uint64_t unsignedDimension = static_cast<uint64_t>(dimension);
        if (unsignedDimension > std::numeric_limits<size_t>::max())
        {
            throw std::overflow_error(std::string(label) + ": tensor dimension exceeds size_t");
        }
        elementCount = checkedSizeProduct(elementCount, static_cast<size_t>(unsignedDimension), label);
    }
    return elementCount;
}

inline size_t checkedElementCount(const std::vector<int64_t>& dimensions, const char* label)
{
    return checkedElementCountFromRange(dimensions, label);
}

inline size_t checkedElementCount(std::initializer_list<int64_t> dimensions, const char* label)
{
    return checkedElementCountFromRange(dimensions, label);
}

inline DLDataType convertToDLDataType(isaacsim::physics::tensors::DType dtype)
{
    using isaacsim::physics::tensors::DType;
    switch (dtype)
    {
    case DType::eFloat32:
        return { static_cast<uint8_t>(kDLFloat), 32, 1 };
    case DType::eFloat64:
        return { static_cast<uint8_t>(kDLFloat), 64, 1 };
    case DType::eInt8:
        return { static_cast<uint8_t>(kDLInt), 8, 1 };
    case DType::eInt16:
        return { static_cast<uint8_t>(kDLInt), 16, 1 };
    case DType::eInt32:
        return { static_cast<uint8_t>(kDLInt), 32, 1 };
    case DType::eInt64:
        return { static_cast<uint8_t>(kDLInt), 64, 1 };
    case DType::eUInt8:
        return { static_cast<uint8_t>(kDLUInt), 8, 1 };
    case DType::eUInt16:
        return { static_cast<uint8_t>(kDLUInt), 16, 1 };
    case DType::eUInt32:
        return { static_cast<uint8_t>(kDLUInt), 32, 1 };
    case DType::eUInt64:
        return { static_cast<uint8_t>(kDLUInt), 64, 1 };
    case DType::eBool:
        return { static_cast<uint8_t>(kDLUInt), 8, 1 }; // bool as uint8
    default:
        return { static_cast<uint8_t>(kDLFloat), 32, 1 };
    }
}

// Convert a manager TensorDescription to a stack-allocated DLTensor. The DLTensor borrows
// descriptor.data and descriptor.shape -- the caller must keep `descriptor` alive for the engine call. A GPU
// tensor must name its device; C-contiguous strides normalize to nullptr (the DLPack
// convention), while genuine strides pass through so ovphysx rejects them loudly.
inline DLTensor convertToDLTensor(const isaacsim::physics::tensors::TensorDescription& descriptor)
{
    using isaacsim::physics::tensors::DeviceKind;
    DLTensor tensor{};
    tensor.data = descriptor.data;
    tensor.device.device_type = (descriptor.device == DeviceKind::eGpu) ? kDLCUDA : kDLCPU;
    // A GPU tensor must name its device; a negative ordinal would fall through to
    // device 0 and silently read/write the wrong GPU. CPU ignores device_id.
    if (descriptor.device == DeviceKind::eGpu && descriptor.deviceOrdinal < 0)
    {
        throw std::runtime_error("GPU tensor has no device ordinal");
    }
    tensor.device.device_id = (descriptor.deviceOrdinal >= 0) ? descriptor.deviceOrdinal : 0;
    tensor.dtype = convertToDLDataType(descriptor.dtype);
    if (descriptor.shape.size() > static_cast<size_t>(std::numeric_limits<int32_t>::max()))
    {
        throw std::overflow_error("tensor rank exceeds the DLPack int32 range");
    }
    checkedElementCount(descriptor.shape, "DLTensor");
    tensor.ndim = static_cast<int32_t>(descriptor.shape.size());
    tensor.shape = const_cast<int64_t*>(descriptor.shape.data());
    // Normalize C-contiguous strides to nullptr (the DLPack convention for
    // C-order tensors). For non-contiguous tensors, pass the actual strides
    // so ovphysx rejects them loudly rather than misreading strided data
    // as if it were contiguous.
    {
        bool isCContiguous = true;
        if (!descriptor.strides.empty())
        {
            if (descriptor.strides.size() != descriptor.shape.size())
            {
                throw std::runtime_error("DLTensor strides must match the tensor rank");
            }
            int64_t expected = 1;
            for (int32_t i = static_cast<int32_t>(descriptor.shape.size()) - 1; isCContiguous && i >= 0; --i)
            {
                const int64_t dimension = descriptor.shape[static_cast<size_t>(i)];
                if (dimension < 0 || (dimension != 0 && expected > std::numeric_limits<int64_t>::max() / dimension))
                {
                    isCContiguous = false;
                    break;
                }
                if (descriptor.strides[static_cast<size_t>(i)] != expected)
                {
                    isCContiguous = false;
                    break;
                }
                expected *= dimension;
            }
        }
        tensor.strides =
            (descriptor.strides.empty() || isCContiguous) ? nullptr : const_cast<int64_t*>(descriptor.strides.data());
    }
    tensor.byte_offset = 0;
    return tensor;
}

// True if `descriptor` is C-contiguous (row-major): no explicit strides, or strides equal to the
// row-major strides for its shape. The ndarray-to-descriptor conversion always records element-strides, so an empty
// strides vector also counts as contiguous. A unit dim's stride is irrelevant to contiguity.
inline bool isContiguousRowMajor(const isaacsim::physics::tensors::TensorDescription& descriptor)
{
    if (descriptor.strides.empty())
    {
        return true;
    }
    if (descriptor.strides.size() != descriptor.shape.size())
    {
        return false;
    }
    int64_t expected = 1;
    for (size_t i = descriptor.shape.size(); i-- > 0;)
    {
        const int64_t dimension = descriptor.shape[i];
        if (dimension < 0)
        {
            return false;
        }
        if (dimension != 1 && descriptor.strides[i] != expected)
        {
            return false;
        }
        if (dimension != 0 && expected > std::numeric_limits<int64_t>::max() / dimension)
        {
            return false;
        }
        expected *= dimension;
    }
    return true;
}

// Indexed get/set requires a 1-D, C-contiguous int32 index buffer: the loops that
// dereference `indices.data` treat it as a flat int32[K]. Reject other dtypes, a multi-dim
// shape, or a strided view loudly rather than misread the buffer -- an int64 [0,1] read as
// int32 misreads as [0,0], a 2-D input is only partially consumed, and a strided view is
// read as adjacent values. Skip only an omitted index (empty shape); a supplied zero-size
// tensor -- (0,), (0,1), (1,0), which Warp gives a null pointer -- is still validated, so its
// dtype/rank cannot slip past isEmpty() (which is also true for null data).
inline void requireInt32Indices(const isaacsim::physics::tensors::TensorDescription& indices, const char* label)
{
    using isaacsim::physics::tensors::DType;
    if (indices.shape.empty())
    {
        return;
    }
    if (indices.dtype != DType::eInt32)
    {
        throw std::runtime_error(std::string(label) + ": indices must use the manager's int32 convention");
    }
    if (indices.shape.size() != 1)
    {
        throw std::runtime_error(std::string(label) + ": indices must be 1-D");
    }
    const size_t indexCount = checkedElementCount(indices.shape, label);
    if (indexCount > 0 && indices.data == nullptr)
    {
        throw std::runtime_error(std::string(label) + ": non-empty indices have no data");
    }
    if (!isContiguousRowMajor(indices))
    {
        throw std::runtime_error(std::string(label) + ": indices must be C-contiguous");
    }
}

// A caller-supplied GET `out` is read into in place by the engine, so it must be able to
// hold the binding's full read (`shape`/`dtype`, the engine writes exactly that many
// contiguous elements) before the read. A wrong dtype, a buffer that can't hold the read, a
// non-contiguous buffer, or a GPU buffer with no ordinal would otherwise overrun, mis-type,
// or scatter the full [N, ...] read -- host heap corruption or a CUDA illegal access. The
// capacity check is on element count, not the shape vector: the binding reports some data 1-D
// (e.g. masses [N]) while callers legitimately allocate the matching 2-D buffer ([N, 1]) --
// same capacity, different layout.
inline void requireMatchingOutput(const isaacsim::physics::tensors::TensorDescription& output,
                                  const std::vector<int64_t>& shape,
                                  isaacsim::physics::tensors::DType dtype,
                                  const char* label)
{
    using isaacsim::physics::tensors::DeviceKind;
    if (output.dtype != dtype)
    {
        throw std::runtime_error(std::string(label) + ": out buffer dtype mismatch");
    }

    const size_t outputElementCount = checkedElementCount(output.shape, label);
    const size_t bindingElementCount = checkedElementCount(shape, label);
    if (outputElementCount != bindingElementCount)
    {
        throw std::runtime_error(std::string(label) + ": out buffer element count mismatch");
    }

    // The read passes strides=nullptr (C-contiguous); a strided out would be filled as if
    // contiguous, so require row-major layout.
    if (!isContiguousRowMajor(output))
    {
        throw std::runtime_error(std::string(label) + ": out buffer must be C-contiguous");
    }
    // A GPU out must name a device; -1 would build DLDevice{ kDLCUDA, -1 }.
    if (output.device == DeviceKind::eGpu && output.deviceOrdinal < 0)
    {
        throw std::runtime_error(std::string(label) + ": GPU out buffer has no device ordinal");
    }
}

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim

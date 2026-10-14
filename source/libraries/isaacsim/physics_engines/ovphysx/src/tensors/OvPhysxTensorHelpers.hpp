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
#include <isaacsim/common/array/Array.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{

// Delegates to array::toDLPack for everything but the boolean dtype: ovphysx reads a boolean tensor as
// uint8, whereas DLPack's own spelling for it is kDLBool. The DLTensor borrows the array's data and shape
// metadata, so the array must outlive it.
inline DLTensor convertToDLTensor(const isaacsim::common::array::Array& source)
{
    namespace array = isaacsim::common::array;
    DLTensor tensor = array::toDLPackBorrowed(source);
    if (source.dtype() == array::Dtype::Bool())
    {
        tensor.dtype = { static_cast<uint8_t>(kDLUInt), 8, 1 };
    }
    return tensor;
}

inline DLTensor convertToDLTensor(const isaacsim::common::array::Array& source,
                                  const isaacsim::common::array::ArrayReadAccess& access)
{
    DLTensor tensor = convertToDLTensor(source);
    tensor.data = source.size() ? const_cast<void*>(access.data()) : nullptr;
    tensor.byte_offset = 0;
    return tensor;
}

inline DLTensor convertToDLTensor(const isaacsim::common::array::Array& source,
                                  const isaacsim::common::array::ArrayWriteAccess& access)
{
    DLTensor tensor = convertToDLTensor(source);
    tensor.data = source.size() ? access.data() : nullptr;
    tensor.byte_offset = 0;
    return tensor;
}

// Wraps storage the impl owns as a byte buffer that shares its lifetime while pointing somewhere inside it,
// so a result handed to a caller cannot outlive the allocation behind it. This does not make the result a
// private copy: a view-lifetime buffer is still reused by the next read, so the values are valid only until
// then and callers that need to retain them must copy.
template <typename OwnerType>
inline std::shared_ptr<std::byte[]> shareBytes(const std::shared_ptr<OwnerType>& owner, void* data)
{
    return std::shared_ptr<std::byte[]>(owner, static_cast<std::byte*>(data));
}

// The index values are read as a dense int32[K] block, both by ovphysx and by the host-side gather, so a
// strided or broadcast selection has to be rejected rather than walked as if it were packed.
inline void requireInt32Indices(const isaacsim::common::array::Array& indices, const char* label)
{
    namespace array = isaacsim::common::array;
    if (indices.dtype() != array::Dtype::Int32())
    {
        throw std::runtime_error(std::string(label) + ": indices must use the manager's int32 convention");
    }
    if (indices.ndim() != 1)
    {
        throw std::runtime_error(std::string(label) + ": indices must be 1-D");
    }
    if (!indices.isContiguous())
    {
        throw std::runtime_error(std::string(label) + ": indices must be C-contiguous");
    }
    if (indices.size() > 0 && indices.data() == nullptr)
    {
        throw std::runtime_error(std::string(label) + ": non-empty indices have no data");
    }
}

// An Array's Device always names a valid ordinal, so the dtype, the capacity and the layout still need
// checking. The capacity check is on element count rather than the shape itself because the binding
// reports some data 1-D (e.g. masses [N]) while callers legitimately allocate the matching 2-D buffer
// ([N, 1]). Shape::size() rejects a negative dimension and an element count that overflows size_t, so a
// malformed shape fails here rather than sizing an undersized allocation.
inline void requireMatchingOutput(const isaacsim::common::array::Array& output,
                                  const isaacsim::common::array::Shape& shape,
                                  isaacsim::common::array::Dtype dtype,
                                  const char* label)
{
    if (output.dtype() != dtype)
    {
        throw std::runtime_error(std::string(label) + ": out buffer dtype mismatch");
    }
    if (output.size() != shape.size())
    {
        throw std::runtime_error(std::string(label) + ": out buffer element count mismatch");
    }
    // Callers view the buffer under `shape` with reshape(), which silently detaches a non-contiguous
    // array into a copy the caller never sees written.
    if (!output.isContiguous())
    {
        throw std::runtime_error(std::string(label) + ": out buffer must be C-contiguous");
    }
    // A broadcast buffer would otherwise fault on the mutable data() deep inside the write.
    if (!output.isWriteable())
    {
        throw std::runtime_error(std::string(label) + ": out buffer must be writeable");
    }
}

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim

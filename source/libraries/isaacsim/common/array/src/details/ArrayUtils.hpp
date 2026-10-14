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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace array
{

class Array;
class Device;
class Dtype;
class Shape;

namespace details
{

class Storage;

std::vector<int64_t> computeContiguousStrides(const Shape& shape);
size_t computeStorageSpanNbytes(const Shape& shape, const std::vector<int64_t>& strides, const Dtype& dtype);

/**
 * @brief Returns the internal CUDA stream on which this library queues transfers and kernels.
 * @details Lazily created once per device. The result is a native `cudaStream_t` reinterpreted as an
 *          integer, matching the convention of @ref acquireRead().
 */
std::uintptr_t operationStream(const Device& device);

/**
 * @brief Library-internal gateway to the private state of @ref Array.
 * @details Array grants friendship to this one type rather than to each internal caller, so the
 *          public header does not have to name them. `ManagedAccess.cpp` reaches the shared
 *          allocation and the read-only check through it.
 */
class ArrayStorageFactory
{
public:
    static Array fromBuffer(std::shared_ptr<std::byte[]> data,
                            size_t capacity,
                            const Shape& shape,
                            Dtype dtype,
                            const Device& device,
                            size_t offset,
                            const std::vector<int64_t>& strides,
                            bool writeable);

    /** @brief Returns the reference-counted allocation shared by every view over @p array. */
    static const std::shared_ptr<Storage>& storageOf(const Array& array);

    /**
     * @brief Throws unless @p array permits writes, naming @p callerName in the message.
     * @throws std::runtime_error if @p array is read-only.
     */
    static void requireWriteable(const Array& array, const std::string& callerName);
};

} // namespace details
} // namespace array
} // namespace common
} // namespace isaacsim

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

#include "TensorTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace tensors
{

/**
 * @brief Describes the storage, layout, and memory location of a tensor.
 *
 * The @ref data pointer does not own its backing memory. When @ref keepAlive is set, it retains an external owner for
 * that memory. A descriptor with a null @ref data pointer or an empty @ref shape represents an unspecified tensor.
 */
class TensorDescription
{
public:
    /** @brief A non-owning pointer to the first tensor element. */
    void* data{ nullptr };

    /** @brief The tensor element data type. */
    DType dtype{ DType::eUnknown };

    /** @brief The size of each tensor dimension. */
    std::vector<int64_t> shape;

    /** @brief The stride of each dimension, in elements. An empty vector denotes C-contiguous storage. */
    std::vector<int64_t> strides;

    /** @brief The memory location containing @ref data. */
    DeviceKind device{ DeviceKind::eCpu };

    /**
     * @brief The device index when @ref device is @c DeviceKind::eGpu.
     *
     * The value is ignored for CPU memory. CPU descriptors may use the default value of @c -1 or the device identifier
     * supplied by an external tensor protocol.
     */
    int32_t deviceOrdinal{ -1 };

    /**
     * @brief Optional shared owner that keeps @ref data valid.
     *
     * This member is empty when the descriptor relies on the caller to maintain the backing memory's lifetime.
     */
    std::shared_ptr<void> keepAlive;

    /**
     * @brief Reports whether this descriptor represents an unspecified tensor.
     *
     * @return @c true if @ref data is null or @ref shape is empty; otherwise, @c false.
     */
    bool isEmpty() const noexcept
    {
        return data == nullptr || shape.empty();
    }

    /**
     * @brief Computes the number of elements described by @ref shape.
     *
     * @return The product of all dimensions, or zero when @ref shape is empty.
     * @pre Every dimension in @ref shape is nonnegative.
     * @pre The product of the dimensions is representable by @c int64_t.
     */
    int64_t computeElementCount() const noexcept
    {
        if (shape.empty())
        {
            return 0;
        }
        int64_t elementCount = 1;
        for (const int64_t dimension : shape)
        {
            elementCount *= dimension;
        }
        return elementCount;
    }
};

} // namespace tensors
} // namespace physics
} // namespace isaacsim

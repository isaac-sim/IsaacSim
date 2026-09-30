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

#include <cstdint>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace tensors
{

/**
 * @brief Describes the expected layout and capabilities of a tensor operation.
 *
 * A shape dimension of @c -1 indicates that the engine determines that dimension when the operation executes.
 */
struct TensorSpecification
{
    /** @brief The operation's tensor element data type. */
    DType dtype{ DType::eFloat32 };

    /** @brief The expected tensor shape, with @c -1 for dimensions resolved at execution. */
    std::vector<int64_t> shapeHint;

    /** @brief The preferred tensor memory location. */
    DeviceKind deviceKind{ DeviceKind::eEngineDefault };

    /** @brief Whether the registered operation is supported by the current engine configuration. */
    bool supports{ true };

    /**
     * @brief Whether read operations accept a nonempty operation-defined selector or query descriptor.
     */
    bool supportsIndexedRead{ false };

    /**
     * @brief Whether write operations accept a nonempty operation-defined selector descriptor.
     */
    bool supportsIndexedWrite{ false };

    /**
     * @brief Whether the provider advertises support for Boolean-mask write selectors.
     *
     * This member is provider metadata. @c EntityView does not enforce it independently: every nonempty write selector,
     * including a Boolean mask, is gated by @ref supportsIndexedWrite. The provider is responsible for interpreting and
     * validating an accepted mask.
     */
    bool supportsMaskedWrite{ false };

    /**
     * @brief Whether the operation requires host-accessible input and output buffers.
     *
     * When this member is @c true, the frontend must stage tensor data, operation-specific selector or query inputs,
     * and any supplied output buffers in host memory before invoking the operation.
     */
    bool requiresHostData{ false };
};

} // namespace tensors
} // namespace physics
} // namespace isaacsim

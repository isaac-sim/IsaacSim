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

#include <cstdint>

namespace isaacsim
{
namespace physics
{
namespace tensors
{

/**
 * @brief Element data types supported by tensor descriptors.
 */
enum class DType : int32_t
{
    /** @brief An unknown or unspecified element type. */
    eUnknown = 0,

    /** @brief A 32-bit IEEE 754 floating-point value. */
    eFloat32,

    /** @brief A 64-bit IEEE 754 floating-point value. */
    eFloat64,

    /** @brief An 8-bit signed integer. */
    eInt8,

    /** @brief A 16-bit signed integer. */
    eInt16,

    /** @brief A 32-bit signed integer. */
    eInt32,

    /** @brief A 64-bit signed integer. */
    eInt64,

    /** @brief An 8-bit unsigned integer. */
    eUInt8,

    /** @brief A 16-bit unsigned integer. */
    eUInt16,

    /** @brief A 32-bit unsigned integer. */
    eUInt32,

    /** @brief A 64-bit unsigned integer. */
    eUInt64,

    /** @brief A Boolean value. */
    eBool,
};

/**
 * @brief Memory locations supported by tensor descriptors.
 */
enum class DeviceKind : int32_t
{
    /** @brief Host-accessible CPU memory. */
    eCpu = 0,

    /** @brief Device-accessible GPU memory. */
    eGpu,

    /** @brief The physics engine's preferred memory location. */
    eEngineDefault,
};

/**
 * @brief Operation directions used to classify tensor implementations.
 */
enum class ImplementationKind : int32_t
{
    /** @brief A read operation that retrieves tensor data. */
    eGet = 0,

    /** @brief A write operation that updates tensor data. */
    eSet,
};

} // namespace tensors
} // namespace physics
} // namespace isaacsim

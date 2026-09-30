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

#include "isaacsim/common/array/Export.h"

#include <cstdint>
#include <string>
#include <variant>

namespace isaacsim
{
namespace common
{
namespace array
{

namespace details
{

/**
 * @brief Variant type representing all accepted forms of a device specification.
 * @details
 * Non-negative integer variants are interpreted as CUDA device indices. A string variant
 * is parsed by @ref Device::fromString to resolve named devices such as `"cpu"`
 * or `"cuda:0"`.
 */
using SupportedDeviceSpecification =
    std::variant<int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t, std::string>;

} // namespace details

/**
 * @class Device
 * @brief Represents a compute device, either CPU or a CUDA-capable GPU.
 * @details
 * A Device encodes a compute target as a signed device index (ordinal). The CPU is
 * represented by ordinal -1; CUDA devices are represented by their non-negative
 * device index.
 *
 * Instances can be constructed from an ordinal, a string descriptor
 * (e.g. `"cpu"`, `"cuda"`, `"cuda:0"`), or via the named factory methods
 * @ref Cpu() and @ref Cuda().
 */
class ISAACSIM_COMMON_ARRAY_API Device
{
public:
    /**
     * @brief Constructs a Device from a supported device specification.
     * @param[in] input Integer ordinal or string descriptor identifying the device.
     */
    Device(details::SupportedDeviceSpecification input);

    Device() = delete;
    /** @brief Copy-constructs a device descriptor. */
    Device(const Device& other) = default;
    /** @brief Move-constructs a device descriptor. */
    Device(Device&& other) = default;
    ~Device() = default;

    /** @brief Copy-assigns a device descriptor. */
    Device& operator=(const Device& other) = default;
    /** @brief Move-assigns a device descriptor. */
    Device& operator=(Device&& other) = default;

    /**
     * @brief Checks whether two devices refer to the same compute target.
     * @param[in] other The Device to compare against.
     * @return `true` if both devices have equal ordinals.
     */
    bool operator==(const Device& other) const;

    /**
     * @brief Checks whether two devices refer to different compute targets.
     * @param[in] other The Device to compare against.
     * @return `true` if the devices have different ordinals.
     */
    bool operator!=(const Device& other) const;

    /**
     * @brief Returns the device ordinal.
     * @return -1 for the CPU, or a non-negative CUDA device index.
     */
    int32_t ordinal() const;

    /**
     * @brief Returns whether this device is the CPU.
     * @return `true` if the ordinal is -1.
     */
    bool isCpu() const;

    /**
     * @brief Returns whether this device is a CUDA GPU.
     * @return `true` if the ordinal is non-negative.
     */
    bool isCuda() const;

    /**
     * @brief Returns whether this device is present and usable on the current system.
     * @return `true` if the device is accessible at runtime.
     */
    bool isAvailable() const;

    /**
     * @brief Returns a human-readable string descriptor for this device.
     * @return `"cpu"` for the CPU, or `"cuda:<ordinal>"` for a CUDA device.
     */
    std::string toString() const;

    /**
     * @brief Constructs a Device from a string descriptor.
     * @details
     * Accepted formats: `"cpu"`, `"cuda"` (equivalent to `"cuda:0"`), and
     * `"cuda:<N>"` where `N` is a non-negative integer ordinal.
     *
     * @param[in] name String descriptor of the device.
     * @return The corresponding Device.
     * @throws std::invalid_argument if @p name is not a recognized descriptor.
     */
    static Device fromString(const std::string& name);

    /**
     * @brief Constructs a Device representing the CPU.
     * @return A Device with ordinal -1.
     */
    static Device Cpu();

    /**
     * @brief Constructs a Device representing a CUDA GPU.
     * @param[in] ordinal Non-negative CUDA device index. Defaults to 0.
     * @return A Device with the specified CUDA ordinal.
     */
    static Device Cuda(int32_t ordinal = 0);

private:
    /**
     * @brief Signed device ordinal; -1 for CPU, ≥ 0 for CUDA devices.
     */
    int32_t m_ordinal;
};


/**
 * @class DeviceGuard
 * @brief RAII guard that temporarily sets the active CUDA device and restores it on destruction.
 * @details
 * On construction, sets the current CUDA device to the specified ordinal. On
 * destruction, restores the device that was active before the guard was created.
 *
 * `DeviceGuard` is non-copyable and non-movable to prevent accidental reuse or
 * duplication of the saved device state.
 *
 * @note Has no effect when the target device is the CPU (ordinal -1).
 */
class ISAACSIM_COMMON_ARRAY_API DeviceGuard
{
public:
    /**
     * @brief Sets the active CUDA device to the device represented by @p device.
     * @param[in] device The Device to activate.
     */
    explicit DeviceGuard(const Device& device);

    /**
     * @brief Sets the active CUDA device to the given ordinal.
     * @param[in] ordinal Non-negative CUDA device index, or -1 to target the CPU (no-op).
     */
    explicit DeviceGuard(int32_t ordinal);

    /**
     * @brief Restores the previously active CUDA device.
     */
    ~DeviceGuard();

    DeviceGuard(const DeviceGuard&) = delete;
    DeviceGuard& operator=(const DeviceGuard&) = delete;
    DeviceGuard(DeviceGuard&&) = delete;
    DeviceGuard& operator=(DeviceGuard&&) = delete;

private:
    /**
     * @brief Ordinal of the CUDA device that was active before this guard was constructed.
     */
    int32_t m_previousOrdinal{ -1 };

    /**
     * @brief Whether to restore the previous device on destruction.
     */
    bool m_restore{ false };
};

} // namespace array
} // namespace common
} // namespace isaacsim

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

#include "isaacsim/common/array/Device.hpp"

#include "details/CudaRuntime.hpp"

#include <stdexcept>

namespace isaacsim
{
namespace common
{
namespace array
{

Device::Device(details::SupportedDeviceSpecification input)
    : m_ordinal(std::visit(
          [](auto&& arg) -> int32_t
          {
              using T = std::decay_t<decltype(arg)>;
              if constexpr (std::is_same_v<T, std::string>)
              {
                  return fromString(arg).m_ordinal;
              }
              else
              {
                  return static_cast<int32_t>(arg);
              }
          },
          input))
{
}

bool Device::operator==(const Device& other) const
{
    return m_ordinal == other.m_ordinal;
}

bool Device::operator!=(const Device& other) const
{
    return !(*this == other);
}

int32_t Device::ordinal() const
{
    return m_ordinal;
}

bool Device::isCpu() const
{
    return m_ordinal < 0;
}

bool Device::isCuda() const
{
    return m_ordinal >= 0;
}

bool Device::isAvailable() const
{
    if (isCpu())
    {
        return true;
    }
    return static_cast<int32_t>(CudaRuntime::getInstance(/*throwIfInvalid=*/false).cudaGetDeviceCount()) > m_ordinal;
}

std::string Device::toString() const
{
    return isCpu() ? "cpu" : "cuda:" + std::to_string(m_ordinal);
}

Device Device::fromString(const std::string& name)
{
    if (name.find("cuda:") == 0)
    {
        try
        {
            int32_t ordinal = std::stoi(name.substr(5));
            if (ordinal >= 0)
            {
                return Device(ordinal);
            }
        }
        catch (const std::exception&)
        {
        }
    }
    else if (name == "cuda")
    {
        return Device(0);
    }
    else if (name == "cpu")
    {
        return Device(-1);
    }
    throw std::invalid_argument("Invalid device specification: '" + name + "'");
}

Device Device::Cpu()
{
    return Device(-1);
}

Device Device::Cuda(int32_t ordinal)
{
    if (ordinal < 0)
    {
        throw std::invalid_argument("CUDA device ordinal must be non-negative");
    }
    return Device(ordinal);
}

DeviceGuard::DeviceGuard(const Device& device)
{
    if (device.isCuda())
    {
        auto& cudaRuntime = CudaRuntime::getInstance();
        cudaRuntime.cudaGetDevice(&m_previousOrdinal);
        if (m_previousOrdinal == device.ordinal())
        {
            return;
        }
        cudaRuntime.cudaSetDevice(device.ordinal());
        m_restore = true;
    }
}

DeviceGuard::DeviceGuard(int32_t ordinal) : DeviceGuard(Device(ordinal))
{
}

DeviceGuard::~DeviceGuard()
{
    if (m_restore)
    {
        CudaRuntime::getInstance().cudaSetDevice(m_previousOrdinal);
    }
}

} // namespace array
} // namespace common
} // namespace isaacsim

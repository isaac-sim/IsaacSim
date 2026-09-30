// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <functional>
#include <string>
#include <variant>

namespace ovsim
{
namespace interfaces
{
namespace control
{
namespace simulation
{

/** @brief Values accepted by provider parameter setters. */
using InputParameterType = std::variant<bool, std::uintptr_t, std::int64_t, double, std::string>;
/** @brief Values returned by provider parameter getters. */
using OutputParameterType = std::variant<bool, std::uintptr_t, std::int64_t, double, std::string>;

/** @brief Callable signature for starting or resuming simulation. */
using PlayFunction = std::function<void()>;
/** @brief Callable signature for pausing simulation. */
using PauseFunction = std::function<void()>;
/** @brief Callable signature for stopping simulation. */
using StopFunction = std::function<void()>;

/** @brief Callable signature for initializing simulation resources. */
using InitializeFunction = std::function<void()>;
/** @brief Callable signature for invalidating simulation resources. */
using InvalidateFunction = std::function<void()>;
/** @brief Callable signature for advancing simulation by one step. */
using StepFunction = std::function<void()>;

/** @brief Callable signature for setting a named provider parameter. */
using SetParameterFunction = std::function<void(
    const std::string& /*provider*/, const std::string& /*parameterName*/, const InputParameterType& /*value*/)>;
/** @brief Callable signature for getting a named provider parameter. */
using GetParameterFunction =
    std::function<OutputParameterType(const std::string& /*provider*/, const std::string& /*parameterName*/)>;

} // namespace simulation
} // namespace control
} // namespace interfaces
} // namespace ovsim

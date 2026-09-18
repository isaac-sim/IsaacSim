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

#include <isaacsim/common/array/Array.hpp>

#include <functional>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace ovsim
{
namespace interfaces
{
namespace data
{

namespace array = isaacsim::common::array;

/** @brief One prim path or a collection of prim paths. */
using PathType = std::variant<std::string, std::vector<std::string>>;
/** @brief Attribute values accepted by data providers. */
using InputValueType =
    std::variant<std::string, std::vector<std::string>, std::vector<std::vector<std::string>>, array::Array>;
/** @brief Attribute values returned by data providers. */
using OutputValueType = std::variant<std::vector<std::string>, std::vector<std::vector<std::string>>, array::Array>;

/** @brief Callable signature for reading a prim attribute. */
using ReadFunction = std::function<OutputValueType(
    const PathType& /*paths*/, const std::string& /*attributeName*/, std::optional<double> /*timestamp*/)>;
/** @brief Callable signature for writing a prim attribute. */
using WriteFunction = std::function<void(const PathType& /*paths*/,
                                         const std::string& /*attributeName*/,
                                         const InputValueType& /*values*/,
                                         std::optional<double> /*timestamp*/)>;

} // namespace data
} // namespace interfaces
} // namespace ovsim

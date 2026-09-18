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
#include <ovsim/interfaces/data/Data.hpp>

#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace isaacsim
{
namespace foundation
{
namespace ovsim
{
namespace details
{

namespace array = isaacsim::common::array;

using ::ovsim::interfaces::data::InputValueType;
using ::ovsim::interfaces::data::OutputValueType;

void clearRegistry();

std::vector<std::string> processPaths(const std::variant<std::string, std::vector<std::string>>& paths);

OutputValueType getAttributeValues(const std::vector<std::string>& paths,
                                   const std::string& attributeName,
                                   const std::optional<array::Array>& indices = std::nullopt);

void setAttributeValues(const std::vector<std::string>& paths,
                        const std::string& attributeName,
                        const InputValueType& values,
                        const std::optional<array::Array>& indices = std::nullopt);

} // namespace details
} // namespace ovsim
} // namespace foundation
} // namespace isaacsim

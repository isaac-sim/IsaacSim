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

#include <isaacsim/foundation/ovsim/Export.h>
#include <ovsim/interfaces/data/Data.hpp>

#include <optional>
#include <string>

namespace isaacsim
{
namespace foundation
{
namespace ovsim
{
namespace data
{

using ::ovsim::interfaces::data::InputValueType;
using ::ovsim::interfaces::data::OutputValueType;
using ::ovsim::interfaces::data::PathType;

/**
 * @brief Read one attribute from one or more prims on the active stage.
 * @param[in] paths Absolute prim path or paths to read.
 * @param[in] attributeName Name of the attribute or registered facade property to read.
 * @param[in] timestamp Optional time code in stage time units. The current implementation reads default-time values.
 * @return Values read for the selected prim paths.
 */
ISAACSIM_FOUNDATION_OVSIM_API OutputValueType read(const PathType& paths,
                                                   const std::string& attributeName,
                                                   std::optional<double> timestamp = std::nullopt);

/**
 * @brief Write one attribute on one or more prims on the active stage.
 * @param[in] paths Absolute prim path or paths to modify.
 * @param[in] attributeName Name of the attribute or registered facade property to write.
 * @param[in] values Values to write to the selected prim paths.
 * @param[in] timestamp Optional time code in stage time units. The current implementation writes default-time values.
 */
ISAACSIM_FOUNDATION_OVSIM_API void write(const PathType& paths,
                                         const std::string& attributeName,
                                         const InputValueType& values,
                                         std::optional<double> timestamp = std::nullopt);

} // namespace data
} // namespace ovsim
} // namespace foundation
} // namespace isaacsim

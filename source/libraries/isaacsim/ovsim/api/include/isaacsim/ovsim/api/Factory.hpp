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

#include <isaacsim/ovsim/api/Export.h>
#include <isaacsim/ovsim/api/Types.hpp>

#include <optional>
#include <string>
#include <unordered_map>

namespace isaacsim
{
namespace ovsim
{
namespace api
{

/** @brief Create an OV SIM client implementation.
 * @param[in] name Client implementation name. Use `in-process` or `local` for the in-process client, or `grpc` for a
 * remote session.
 * @param[in] configuration Optional implementation-specific configuration values.
 * @return Control and data interfaces for the selected client implementation.
 * @throws std::runtime_error If `name` does not identify a supported client implementation.
 * @throws std::invalid_argument If the selected implementation rejects its configuration.
 */
ISAACSIM_OVSIM_API_API types::Implementation makeClient(
    const std::string& name,
    const std::optional<std::unordered_map<std::string, std::string>>& configuration = std::nullopt);

} // namespace api
} // namespace ovsim
} // namespace isaacsim

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

#include <isaacsim/ovsim/clients/local/Export.h>
#include <ovsim/interfaces/data/Data.hpp>

namespace isaacsim
{
namespace ovsim
{
namespace clients
{
namespace local
{
namespace data
{

using ::ovsim::interfaces::data::InputValueType;
using ::ovsim::interfaces::data::OutputValueType;
using ::ovsim::interfaces::data::PathType;

/** @copydoc isaacsim::physics::ovsim::data::read */
ISAACSIM_OVSIM_CLIENTS_LOCAL_API OutputValueType read(const PathType& paths,
                                                      const std::string& attributeName,
                                                      std::optional<double> timestamp = std::nullopt);

/** @copydoc isaacsim::physics::ovsim::data::write */
ISAACSIM_OVSIM_CLIENTS_LOCAL_API void write(const PathType& paths,
                                            const std::string& attributeName,
                                            const InputValueType& values,
                                            std::optional<double> timestamp = std::nullopt);

} // namespace data
} // namespace local
} // namespace clients
} // namespace ovsim
} // namespace isaacsim

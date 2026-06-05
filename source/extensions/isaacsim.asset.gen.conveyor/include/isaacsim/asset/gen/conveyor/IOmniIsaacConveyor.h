// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/Interface.h>

namespace isaacsim
{
namespace asset
{
namespace gen
{
namespace conveyor
{

/**
 * @brief Interface for the OmniIsaacConveyor plugin.
 *
 * @details This interface provides the core functionality for the conveyor belt plugin.
 * It enables loading the plugin, triggering carbOnPluginStartup() and carbOnPluginShutdown() methods,
 * and allows usage of other Carbonite plugins. This serves as a foundational interface for Kit extensions.
 */
struct IOmniIsaacConveyor
{
    CARB_PLUGIN_INTERFACE("isaacsim::asset::gen::conveyor", 1, 0);
};

} // namespace conveyor
} // namespace gen
} // namespace asset
} // namespace isaacsim

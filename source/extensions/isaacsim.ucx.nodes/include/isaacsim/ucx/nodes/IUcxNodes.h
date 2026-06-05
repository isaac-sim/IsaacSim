// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

namespace isaacsim::ucx::nodes
{

/**
 * @brief Minimal Carbonite interface for Isaac UCX Nodes extension.
 *
 * This interface doesn't have any functions, but implementing it and acquiring will:
 * - Load the plugin
 * - Trigger carbOnPluginStartup() and carbOnPluginShutdown()
 * - Allow usage of other Carbonite plugins
 * - Register OmniGraph nodes
 */
struct IUcxNodes
{
    CARB_PLUGIN_INTERFACE("isaacsim::ucx::nodes", 1, 0);
};

} // namespace isaacsim::ucx::nodes

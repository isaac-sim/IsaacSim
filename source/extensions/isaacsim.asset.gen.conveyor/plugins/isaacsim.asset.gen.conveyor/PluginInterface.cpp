// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#define CARB_EXPORTS


#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/asset/gen/conveyor/IOmniIsaacConveyor.h>
#include <omni/fabric/IToken.h>
#include <omni/graph/core/OgnHelpers.h>
#include <omni/graph/core/iComputeGraph.h>
#include <omni/graph/core/ogn/Registration.h>

#include <algorithm>

/**
 * @brief Plugin descriptor for the conveyor belt plugin
 */
const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.asset.gen.conveyor.plugin",
                                                    "OmniGraph Isaac Conveyor Node plugin.", "NVIDIA",
                                                    carb::PluginHotReload::eEnabled, "dev" };

CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::asset::gen::conveyor::IOmniIsaacConveyor)
CARB_PLUGIN_IMPL_DEPS(omni::graph::core::IGraphRegistry, omni::fabric::IToken, carb::settings::ISettings)
DECLARE_OGN_NODES()

/**
 * @brief Fills the interface implementation
 * @param iface Interface to initialize
 */
void fillInterface(isaacsim::asset::gen::conveyor::IOmniIsaacConveyor& iface)
{
    iface = {};
}

/**
 * @brief Plugin startup handler
 */
CARB_EXPORT void carbOnPluginStartup(){ INITIALIZE_OGN_NODES() }

/**
 * @brief Plugin shutdown handler
 */
CARB_EXPORT void carbOnPluginShutdown()
{
    RELEASE_OGN_NODES()
}

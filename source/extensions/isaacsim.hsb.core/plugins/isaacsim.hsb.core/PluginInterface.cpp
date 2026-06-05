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

#define CARB_EXPORTS

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/hsb/core/IHsbCore.h>

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.hsb.core.plugin", "Isaac HSB Core Plugin", "NVIDIA",
                                                    carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::hsb::core::IHsbCore)
CARB_PLUGIN_IMPL_DEPS(carb::settings::ISettings)

void fillInterface(isaacsim::hsb::core::IHsbCore& iface)
{
    iface = {};
}

CARB_EXPORT void carbOnPluginStartup()
{
    CARB_LOG_INFO("HSB Core plugin startup");
}

CARB_EXPORT void carbOnPluginShutdown()
{
    CARB_LOG_INFO("HSB Core plugin shutdown");
}

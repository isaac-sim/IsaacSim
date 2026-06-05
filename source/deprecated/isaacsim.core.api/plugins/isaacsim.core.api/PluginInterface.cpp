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
#include <carb/PluginUtils.h>

#include <isaacSim/Version.h>
#include <omni/ext/IExt.h>

namespace isaacsim::core::api
{
class CoreExt : public omni::ext::IExt
{
public:
    void onStartup(const char* extId) override{};
    void onShutdown() override{};
};
}

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.core.api.plugin", "Core interface to Isaac sim", "NVIDIA",
                                                    carb::PluginHotReload::eEnabled, "dev" };
CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::core::api::CoreExt)

void addCrashreporterMetadata()
{
    carb::crashreporter::addCrashMetadata("lib_isaacSim_buildVersion", ISAACSIM_BUILD_VERSION);
    carb::crashreporter::addCrashMetadata("lib_isaacSim_buildRepo", ISAACSIM_BUILD_REPO);
    carb::crashreporter::addCrashMetadata("lib_isaacSim_buildHash", ISAACSIM_BUILD_SHA);
    carb::crashreporter::addCrashMetadata("lib_isaacSim_buildBranch", ISAACSIM_BUILD_BRANCH);
    carb::crashreporter::addCrashMetadata("lib_isaacSim_buildDate", ISAACSIM_BUILD_DATE);
}

CARB_EXPORT void carbOnPluginStartup()
{
    addCrashreporterMetadata();
}

CARB_EXPORT void carbOnPluginShutdown()
{
}

void fillInterface(isaacsim::core::api::CoreExt& iface)
{
    iface = {};
}

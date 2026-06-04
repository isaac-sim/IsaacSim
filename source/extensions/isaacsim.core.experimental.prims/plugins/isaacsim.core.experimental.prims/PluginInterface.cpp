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

#include <carb/PluginUtils.h>

#include <omni/ext/IExt.h>

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.core.experimental.prims.plugin",
                                                    "Core prims extension plugin (interface host)", "NVIDIA",
                                                    carb::PluginHotReload::eEnabled, "dev" };

namespace isaacsim
{
namespace core
{
namespace experimental
{
namespace prims
{

class Extension : public omni::ext::IExt
{
public:
    void onStartup(const char* extId) override
    {
        (void)extId;
    }

    void onShutdown() override
    {
    }
};

} // namespace prims
} // namespace experimental
} // namespace core
} // namespace isaacsim

CARB_EXPORT void carbOnPluginStartup()
{
}

CARB_EXPORT void carbOnPluginShutdown()
{
}

CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::core::experimental::prims::Extension)

void fillInterface(isaacsim::core::experimental::prims::Extension& iface)
{
    (void)iface;
}

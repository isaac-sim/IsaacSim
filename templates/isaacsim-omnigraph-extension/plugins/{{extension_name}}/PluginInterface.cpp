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

#include <{{python_module_path}}/IExampleNodes.h>
#include <omni/graph/core/ogn/Registration.h>

const struct carb::PluginImplDesc g_kPluginDesc = { "{{extension_name}}.plugin",
                                                    "{{title}} OmniGraph Plugin", "NVIDIA",
                                                    carb::PluginHotReload::eEnabled, "dev" };

CARB_PLUGIN_IMPL(g_kPluginDesc, {{ extension_name.replace(".", "::") }}::IExampleNodes)

DECLARE_OGN_NODES()

void fillInterface({{ extension_name.replace(".", "::") }}::IExampleNodes& iface)
{
    iface = {};
}

CARB_EXPORT void carbOnPluginStartup()
{
    INITIALIZE_OGN_NODES()
}

CARB_EXPORT void carbOnPluginShutdown()
{
    RELEASE_OGN_NODES()
}

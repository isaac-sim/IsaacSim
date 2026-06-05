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

#include "SimulationBackend.h"

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>

#include <omni/ext/IExt.h>
#include <omni/physics/tensors/TensorApi.h>

#include <memory>

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.physics.newton.tensors.plugin",
                                                    "Newton Tensor Backend for Physics", "NVIDIA",
                                                    carb::PluginHotReload::eDisabled, "dev" };

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

class Extension : public omni::ext::IExt
{
public:
    void onStartup(const char* extId) override;
    void onShutdown() override;
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim

using namespace omni::physics::tensors;

namespace
{
std::unique_ptr<::isaacsim::physics::newton::tensors::SimulationBackend> g_newtonBackend;
BackendRegistry* g_backendRegistry = nullptr;
constexpr const char* g_simBackendName = "newton";
bool g_tensorsStarted = false;
}

// Shared init/shutdown pattern following omni.physx.tensors design
void tensorsInit()
{
    if (g_tensorsStarted)
    {
        CARB_LOG_VERBOSE("isaacsim.physics.newton.tensors tensorsInit() already ran, skipping duplicate");
        return;
    }

    CARB_LOG_INFO("isaacsim.physics.newton.tensors tensorsInit() starting...");

    carb::Framework* framework = carb::getFramework();
    if (!framework)
    {
        CARB_LOG_ERROR("Failed to get Carbonite framework");
        return;
    }

    // Acquire backend registry
    g_backendRegistry = framework->tryAcquireInterface<BackendRegistry>();
    if (!g_backendRegistry)
    {
        CARB_LOG_ERROR("Failed to acquire simulation backend registry interface");
        return;
    }

    // Create simulation backend
    if (!g_newtonBackend)
    {
        g_newtonBackend = std::make_unique<::isaacsim::physics::newton::tensors::SimulationBackend>();
    }

    // Register Newton backend
    if (!g_backendRegistry->registerBackend(g_simBackendName, g_newtonBackend.get()))
    {
        CARB_LOG_ERROR("Failed to register simulation backend '%s'", g_simBackendName);
    }
    else
    {
        CARB_LOG_INFO("Registered simulation backend '%s'", g_simBackendName);
    }

    g_tensorsStarted = true;
}

void tensorsShutdown()
{
    if (!g_tensorsStarted)
        return;
    g_tensorsStarted = false;

    if (g_backendRegistry)
    {
        g_backendRegistry->unregisterBackend(g_simBackendName);
        g_backendRegistry = nullptr;
    }

    if (g_newtonBackend)
    {
        g_newtonBackend.reset();
    }

    CARB_LOG_INFO("isaacsim.physics.newton.tensors shutdown complete");
}

void isaacsim::physics::newton::tensors::Extension::onStartup(const char* extId)
{
    tensorsInit();
}

void isaacsim::physics::newton::tensors::Extension::onShutdown()
{
    tensorsShutdown();
}

CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::physics::newton::tensors::Extension)
CARB_PLUGIN_IMPL_DEPS(BackendRegistry)

void fillInterface(isaacsim::physics::newton::tensors::Extension& iface)
{
}

CARB_EXPORT void carbOnPluginStartup()
{
    tensorsInit();
}

CARB_EXPORT void carbOnPluginShutdown()
{
    tensorsShutdown();
}

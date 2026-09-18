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

// clang-format off
#include "isaacsim/physics_engines/ovphysx/Backend.hpp"
// clang-format on

#include "OvPhysxAdapter.hpp"
#include "tensors/OvPhysxEntityFactories.hpp"

#include <isaacsim/common/logging/Logging.hpp>
#include <ovphysx/ovphysx.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{
namespace
{

const isaacsim::common::logging::Logger g_kLogger("isaacsim.physics_engines.ovphysx");
std::mutex g_pluginMutex;
std::shared_ptr<OvPhysxAdapter> g_adapter;
bool g_started = false;

// Declared by the caller that configured the tensor device; ovphysx offers no query for it. Read on every
// entity-view access, so it may be declared after the views exist. Atomic because views read it from whichever
// thread drives the data plane.
std::atomic<int> g_tensorDeviceOrdinal{ -1 };

} // namespace

void setTensorDeviceOrdinal(int ordinal)
{
    g_tensorDeviceOrdinal.store(ordinal, std::memory_order_relaxed);
}

int getTensorDeviceOrdinal()
{
    return g_tensorDeviceOrdinal.load(std::memory_order_relaxed);
}

bool activate()
{
    // Never let a C++ exception cross into the nanobind/C caller frames; report
    // failure as a bool the Python facade can turn into a clean error instead.
    try
    {
        std::lock_guard<std::mutex> lock(g_pluginMutex);
        if (g_started)
        {
            return true;
        }

        OvPhysxAdapter::Configuration configuration;
        g_adapter = OvPhysxAdapter::create(configuration);
        size_t simulationId = g_adapter->registerWithManager("ovphysx");
        // Register the adapter in the global map keyed by simulationId so the
        // entity factories (create_entity(engine, entity, paths)) can find the live handle.
        storeAdapter(static_cast<int64_t>(simulationId), g_adapter);
        registerOvPhysxEntityFactories();
        g_started = true;
        return true;
    }
    catch (const std::exception& error)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "ovphysx activate failed: {}", error.what());
        return false;
    }
    catch (...)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "ovphysx activate failed: unknown error");
        return false;
    }
}

void setSuppressReadback(bool enable)
{
    // Host opt-in to PhysX DirectGPU (PxSceneFlag::eENABLE_DIRECT_GPU_API). This
    // writes a PROCESS-GLOBAL carb setting (/physics/suppressReadback) shared by
    // every ovphysx instance/scene in the process; ovphysx latches it when it
    // (re)creates a scene, so set it BEFORE that scene's first step. On a
    // GPU-dynamics scene it makes tensors GPU-resident.
    ovphysx_config_entry_t configurationEntry{};
    configurationEntry.key_type = OVPHYSX_CONFIG_KEY_TYPE_CARBONITE;
    configurationEntry.key.carbonite_key = OVPHYSX_LITERAL("/physics/suppressReadback");
    configurationEntry.value.string_value = enable ? OVPHYSX_LITERAL("true") : OVPHYSX_LITERAL("false");
    ovphysx_set_global_config(configurationEntry);
}

void shutdown()
{
    std::lock_guard<std::mutex> lock(g_pluginMutex);
    if (!g_started)
    {
        return;
    }

    unregisterOvPhysxEntityFactories();
    if (g_adapter)
    {
        size_t simulationId = g_adapter->getSimulationId();
        removeAdapterHandle(static_cast<int64_t>(simulationId));
        g_adapter->unregisterFromManager();
        g_adapter.reset();
    }
    g_started = false;
}

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim

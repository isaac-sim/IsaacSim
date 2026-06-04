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

// Shared utility functions and common data fields for all experimental
// physics sensor implementations (Contact, Effort, IMU).
//
// Each sensor impl has its own ImplData struct, but the common fields
// and initialization patterns are documented here. Sensor-specific
// fields are added in each impl's ImplData.
//
// Common ImplData fields (present in all sensors):
//   long stageId = 0;
//   float lastDt = 0.0f;
//   int stepCount = 0;
//   ISimulationManager* simManager = nullptr;
//   pxr::UsdStageRefPtr usdStage;
//   omni::physics::IPhysicsSimulation* physicsSimulation = nullptr;
//   omni::physics::SubscriptionId physicsStepSub = omni::physics::kInvalidSubscriptionId;
//   carb::events::ISubscriptionPtr physicsEventSub;
//   std::unordered_map<std::string, SensorData> sensors;  // keyed by prim path
//
// Common lifecycle:
//   1. _subscribeToPhysicsEvents() - subscribe to eStopped/eResumed
//   2. On eResumed: _initializeFromContext() -> _initializeStage() -> [_discoverSensorsFromStage()]
//   3. On eStopped: _unsubscribeFromPhysicsStepEvents() + _clearSensors()
//   4. On each step: _stepSensors(dt) -> per-sensor _processSensor()
//   5. shutdown(): _unsubscribeFromPhysicsStepEvents() + reset event sub + _clearSensors()

#include <omni/usd/UsdContext.h>
#include <pxr/usd/usdUtils/stageCache.h>

namespace isaacsim
{
namespace sensors
{
namespace experimental
{
namespace physics
{
namespace utils
{

/// Get the current USD stage and its cache ID.
/// Returns true if a valid stage is available, false otherwise.
inline bool getStageAndId(pxr::UsdStageRefPtr& outStage, long& outStageId)
{
    auto* usdContext = omni::usd::UsdContext::getContext();
    if (!usdContext)
        return false;

    outStage = usdContext->getStage();
    if (!outStage)
        return false;

    pxr::UsdStageCache& cache = pxr::UsdUtilsStageCache::Get();
    outStageId = cache.GetId(outStage).ToLongInt();
    return outStageId != 0;
}

/// Resolve a cached stage from a stage ID.
inline pxr::UsdStageRefPtr resolveStageFromId(long stageId)
{
    pxr::UsdStageCache& cache = pxr::UsdUtilsStageCache::Get();
    return cache.Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
}

/// Read the default physics engine type from settings.
/// Returns the engine string, or empty string if not set.
inline std::string getEngineTypeFromSettings()
{
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
    if (settings)
    {
        const char* engineSetting = settings->getStringBuffer("/exts/isaacsim.core.simulation_manager/default_engine");
        if (engineSetting && engineSetting[0] != '\0')
            return std::string(engineSetting);
    }
    return {};
}

} // namespace utils
} // namespace physics
} // namespace experimental
} // namespace sensors
} // namespace isaacsim

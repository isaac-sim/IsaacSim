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

#include <isaacsim/sensors/experimental/physics/IRaycastSensor.h>

#include <memory>
#include <string>

namespace isaacsim
{
namespace sensors
{
namespace experimental
{
namespace physics
{

/**
 * @brief Carbonite plugin implementation of IRaycastSensor.
 * @details Self-driving raycast sensor manager that subscribes to PhysX
 * simulation events and physics step callbacks. Uses PxSceneQueryExt
 * for per-ray raycasting each substep.
 */
class RaycastSensorImpl : public IRaycastSensor
{
public:
    RaycastSensorImpl();
    ~RaycastSensorImpl();

    void shutdown() override;
    bool createSensor(const char* primPath) override;
    void removeSensor(const char* primPath) override;
    RaycastSensorReading getSensorReading(const char* primPath) override;

private:
    struct ImplData; ///< @brief Forward-declared implementation data (pimpl).
    std::unique_ptr<ImplData> m_impl;

    void _initializeFromContext();
    void _initializeStage(long stageId);
    void _discoverSensorsFromStage();
    void _clearSensors();
    void _subscribeToPhysicsEvents();
    void _subscribeToPhysicsStepEvents();
    void _unsubscribeFromPhysicsStepEvents();
    void _stepSensors(float dt);
    void _processSensor(ImplData& impl, const std::string& primPath, float dt, double simTime);
};

} // namespace physics
} // namespace experimental
} // namespace sensors
} // namespace isaacsim

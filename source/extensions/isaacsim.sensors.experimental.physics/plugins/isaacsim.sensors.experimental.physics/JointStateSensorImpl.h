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

#pragma once

#include <isaacsim/sensors/experimental/physics/IJointStateSensor.h>

#include <memory>
#include <string>

namespace isaacsim::sensors::experimental::physics
{

/**
 * Implementation of IJointStateSensor: manages per-articulation joint state sensors that report
 * DOF positions, velocities, and efforts. Uses IArticulationDataView for engine-agnostic access.
 * Subscribes to simulation events (eResumed / eStopped) and physics step events to drive updates.
 */
class JointStateSensorImpl : public IJointStateSensor
{
public:
    JointStateSensorImpl();
    ~JointStateSensorImpl();

    JointStateSensorImpl(const JointStateSensorImpl&) = delete;
    JointStateSensorImpl& operator=(const JointStateSensorImpl&) = delete;

    void shutdown() override;
    bool createSensor(const char* articulationRootPath) override;
    void removeSensor(const char* articulationRootPath) override;
    JointStateSensorReading getSensorReading(const char* articulationRootPath) override;

private:
    struct ImplData;
    std::unique_ptr<ImplData> m_impl;

    /// Resolve stage from USD context and initialize stage/reader; called on eResumed.
    void _initializeFromContext();
    /// Bind to stage, (re)init reader and step subscription; may clear sensors on stage change.
    void _initializeStage(long stageId);
    /// Remove all articulation views and clear the sensor map.
    void _clearSensors();
    /// Subscribe to simulation stop/resume to clear state or reinit.
    void _subscribeToPhysicsEvents();
    /// Subscribe to physics step to update sensor readings each step.
    void _subscribeToPhysicsStepEvents();
    /// Unsubscribe from physics step events.
    void _unsubscribeFromPhysicsStepEvents();
    /// Called each physics step; updates all sensors from articulation views.
    void _stepSensors(float dt);
    /// Recreate articulation views after reader generation change.
    void _recreateSensorViews();
    /// Update one sensor's positions/velocities/efforts from its articulation view.
    static void _processSensor(ImplData& impl, const std::string& articulationRootPath, double simTime);
};

} // namespace isaacsim::sensors::experimental::physics

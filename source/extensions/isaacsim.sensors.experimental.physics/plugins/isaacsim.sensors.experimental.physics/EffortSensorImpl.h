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

#include <isaacsim/sensors/experimental/physics/IEffortSensor.h>

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
 * @brief Implementation of the IEffortSensor interface for physics-based effort sensing.
 */
class EffortSensorImpl : public IEffortSensor
{
public:
    EffortSensorImpl();
    ~EffortSensorImpl();

    void shutdown() override;
    bool createSensor(const char* jointPrimPath) override;
    void removeSensor(const char* jointPrimPath) override;
    EffortSensorReading getSensorReading(const char* jointPrimPath) override;

private:
    struct ImplData;
    std::unique_ptr<ImplData> m_impl;

    void _initializeFromContext();
    void _initializeStage(long stageId);
    void _clearSensors();
    void _subscribeToPhysicsEvents();
    void _subscribeToPhysicsStepEvents();
    void _unsubscribeFromPhysicsStepEvents();
    void _stepSensors(float dt);
    void _recreateSensorViews();
    static void _processSensor(ImplData& impl, const std::string& jointPrimPath, float dt, double simTime);
};

} // namespace physics
} // namespace experimental
} // namespace sensors
} // namespace isaacsim

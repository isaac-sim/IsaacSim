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

#pragma once

#include <omni/physics/tensors/ISimulationBackend.h>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using omni::physics::tensors::ISimulationBackend;
using omni::physics::tensors::ISimulationView;

/// Newton tensor backend entry point.
///
/// Registered as an omni.physics.tensors plugin. Creates either a CpuSimulationView or
/// GpuSimulationView depending on the Newton model's simulation device.
class SimulationBackend : public ISimulationBackend
{
public:
    SimulationBackend();
    ~SimulationBackend() override;

    /// Initializes Newton via pybind11, detects the simulation device, and returns
    /// a CpuSimulationView (device ordinal -1) or GpuSimulationView (ordinal >= 0).
    ISimulationView* createSimulationView(long stageId = -1) override;

    /// Releases any cached simulation view state so a fresh backend can be created.
    void reset() override;
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim

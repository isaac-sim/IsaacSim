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

#pragma once

#include <carb/InterfaceUtils.h>

#include <omni/physics/simulation/IPhysics.h>

#include <cstddef>
#include <vector>

namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @brief Return the name of the first active physics simulation, or nullptr if none is active.
 * @details Queries `omni::physics::IPhysics` for all registered simulations
 * and returns the name of the first one that is currently active. The returned
 * pointer is owned by the physics interface and remains valid until the
 * simulation is unregistered.
 *
 * Typical use: pass the result to `TensorApi::createSimulationView` as the
 * `backendName` parameter so the correct tensor backend is selected
 * automatically. Passing nullptr (no active simulation) falls back to PhysX.
 */
inline const char* getActivePhysicsEngineName()
{
    auto* physics = carb::getCachedInterface<omni::physics::IPhysics>();
    if (!physics)
        return nullptr;

    size_t numSims = physics->getNumSimulations();
    std::vector<omni::physics::SimulationId> ids(numSims);
    physics->getSimulationIds(ids.data(), numSims);
    for (const auto& id : ids)
    {
        if (physics->isSimulationActive(id))
            return physics->getSimulationName(id);
    }
    return nullptr;
}

} // namespace includes
} // namespace core
} // namespace isaacsim

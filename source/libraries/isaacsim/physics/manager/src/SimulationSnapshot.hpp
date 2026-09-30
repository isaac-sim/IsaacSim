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

#include <isaacsim/physics/registration/Physics.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace manager
{
namespace details
{

/// Value snapshot of a registered simulation's state.
struct SimulationSnapshotRecord
{
    registration::Simulation simulation;
    std::string simulationName;
    bool isActive;
};

using SimulationSnapshots =
    std::unordered_map<registration::SimulationId, SimulationSnapshotRecord, registration::SimulationIdHash>;

/// Copy the currently registered simulations through the public registration API.
///
/// The registry can change between the ID and value queries. Entries removed during
/// collection are skipped, and entries added during collection are observed on the
/// next manager operation.
inline SimulationSnapshots getSimulationSnapshots()
{
    std::vector<registration::SimulationId> ids(registration::getSimulationCount());
    ids.resize(registration::getSimulationIds(ids.data(), ids.size()));

    SimulationSnapshots snapshots;
    snapshots.reserve(ids.size());
    for (const registration::SimulationId& id : ids)
    {
        const registration::Simulation* simulation = registration::getSimulation(id);
        if (simulation == nullptr)
        {
            continue;
        }
        snapshots.emplace(id, SimulationSnapshotRecord{ *simulation, registration::getSimulationName(id),
                                                        registration::isSimulationActive(id) });
    }
    return snapshots;
}

} // namespace details
} // namespace manager
} // namespace physics
} // namespace isaacsim

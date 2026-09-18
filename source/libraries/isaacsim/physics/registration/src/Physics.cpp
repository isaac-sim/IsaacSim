// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaacsim/physics/registration/Physics.hpp"

#include "SimulationRegistry.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace isaacsim
{
namespace physics
{
namespace registration
{

SimulationId registerSimulation(const Simulation& simulation, const std::string& simulationName)
{
    return getSimulationRegistry().registerSimulation(simulation, simulationName);
}

void unregisterSimulation(const SimulationId& simulationId)
{
    getSimulationRegistry().unregisterSimulation(simulationId);
}

const Simulation* getSimulation(const SimulationId& simulationId)
{
    return getSimulationRegistry().getSimulation(simulationId);
}

std::string getSimulationName(const SimulationId& simulationId)
{
    return getSimulationRegistry().getSimulationName(simulationId);
}

size_t getSimulationCount()
{
    return getSimulationRegistry().getSimulationCount();
}

size_t getSimulationIds(SimulationId* simulationIds, size_t bufferSize)
{
    const SimulationMap& simulations = getSimulationRegistry().getSimulations();
    const size_t simulationCount = std::min(simulations.size(), bufferSize);
    if (simulationCount == 0 || simulationIds == nullptr)
    {
        return 0;
    }

    size_t simulationIndex = 0;
    for (const SimulationMap::value_type& simulation : simulations)
    {
        simulationIds[simulationIndex++] = simulation.first;
        if (simulationIndex == simulationCount)
        {
            break;
        }
    }
    return simulationCount;
}

void activateSimulation(const SimulationId& simulationId)
{
    getSimulationRegistry().activateSimulation(simulationId);
}

void deactivateSimulation(const SimulationId& simulationId)
{
    getSimulationRegistry().deactivateSimulation(simulationId);
}

bool isSimulationActive(const SimulationId& simulationId)
{
    return getSimulationRegistry().isSimulationActive(simulationId);
}

SimulationId getActiveSimulationId(const std::string& simulationName)
{
    const SimulationMap& simulations = getSimulationRegistry().getSimulations();
    SimulationId matchedSimulationId = g_kInvalidSimulationId;
    size_t matchCount = 0;
    // Every match is counted rather than returning the first: activation is not exclusive, so several
    // simulations can carry one name, and the registry is unordered. Returning whichever the map happened to
    // yield first would bind callers to a different simulation between runs.
    for (const SimulationMap::value_type& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulationName == simulationName)
        {
            matchedSimulationId = simulation.first;
            ++matchCount;
        }
    }
    if (matchCount > 1)
    {
        throw std::runtime_error("getActiveSimulationId: " + std::to_string(matchCount) + " active simulations named '" +
                                 simulationName + "'; the caller must identify one rather than this choosing");
    }
    return matchedSimulationId;
}

SubscriptionId subscribeSimulationRegistryEvents(OnSimulationRegistryEventFunction onEvent, void* userData)
{
    return getSimulationRegistry().subscribeSimulationRegistryEvents(onEvent, userData);
}

void unsubscribeSimulationRegistryEvents(SubscriptionId subscriptionId)
{
    getSimulationRegistry().unsubscribeSimulationRegistryEvents(subscriptionId);
}

} // namespace registration
} // namespace physics
} // namespace isaacsim

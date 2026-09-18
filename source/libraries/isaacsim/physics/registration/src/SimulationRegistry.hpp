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

#include <isaacsim/physics/registration/Physics.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace registration
{

struct SimulationRecord
{
    Simulation simulation;
    std::string simulationName;
    bool isActive;
};

using SimulationMap = std::unordered_map<SimulationId, SimulationRecord, SimulationIdHash>;

using SimulationRegistryEventSubscribersMap =
    std::unordered_map<SubscriptionId, std::pair<OnSimulationRegistryEventFunction, void*>, SubscriptionIdHash>;

/// Simulation registry
/// This class is used to register and unregister simulations
class SimulationRegistry
{
public:
    /// Constructor
    SimulationRegistry();

    /// Destructor
    ~SimulationRegistry() = default;

    /// Register a simulation
    /// \param[in] simulation The simulation to register
    /// \return The id of the simulation
    SimulationId registerSimulation(const Simulation& simulation, const std::string& simulationName);

    /// Unregister a simulation
    /// \param[in] simulationId The identifier of the simulation to unregister.
    void unregisterSimulation(const SimulationId& simulationId);

    /// Get a simulation
    /// \param[in] simulationId The identifier of the simulation to get.
    /// \return The simulation.
    const Simulation* getSimulation(const SimulationId& simulationId) const;

    /// Get a simulation name
    /// \param[in] simulationId The identifier of the simulation to get.
    /// \return The simulation name, or an empty string if the simulation id is not registered.
    std::string getSimulationName(const SimulationId& simulationId) const;

    /// Get the number of registered simulations.
    /// \return Number of registered simulations.
    size_t getSimulationCount() const
    {
        return m_simulations.size();
    }

    /// Get all simulations
    /// \return All simulations
    const SimulationMap& getSimulations() const
    {
        return m_simulations;
    }
    SimulationMap& getSimulations()
    {
        return m_simulations;
    }

    /// Activate a simulation
    /// \param[in] simulationId The identifier of the simulation to activate.
    void activateSimulation(const SimulationId& simulationId);

    /// Deactivate a simulation
    /// \param[in] simulationId The identifier of the simulation to deactivate.
    void deactivateSimulation(const SimulationId& simulationId);

    /// Check if a simulation is active
    /// \param[in] simulationId The identifier of the simulation to check.
    /// \return True if the simulation is active, false otherwise.
    bool isSimulationActive(const SimulationId& simulationId) const;

    /// Subscribe to simulation registry events
    /// \param[in] onEvent The callback function to be called on simulation registry events
    /// \param[in] userData Pointer to user data to be passed to the callback function. Default is nullptr.
    /// \return The subscription id used for unsubscribing.
    SubscriptionId subscribeSimulationRegistryEvents(OnSimulationRegistryEventFunction onEvent, void* userData = nullptr);

    /// Unsubscribe from simulation registry events
    /// \param[in] subscriptionId The subscription id returned when subscribing.
    void unsubscribeSimulationRegistryEvents(SubscriptionId subscriptionId);

private:
    /// The number of simulations
    size_t m_nextSimulationId;

    /// All simulations
    SimulationMap m_simulations;

    SimulationRegistryEventSubscribersMap m_simulationRegistryEventCallbacks;
    size_t m_nextSubscriptionId;

    void _notifyEvent(SimulationRegistryEventType eventType,
                      const SimulationId& simulationId,
                      const std::string& simulationName);
};

/// Return the process-owned simulation registry.
SimulationRegistry& getSimulationRegistry();

} // namespace registration
} // namespace physics
} // namespace isaacsim

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

#include "SimulationRegistry.hpp"

#include <isaacsim/common/logging/Logging.hpp>

#include <exception>

namespace
{
const isaacsim::common::logging::Logger g_kLogger("isaacsim.physics.registration");
}

namespace isaacsim
{
namespace physics
{
namespace registration
{

SimulationRegistry& getSimulationRegistry()
{
    static SimulationRegistry s_registry;
    return s_registry;
}

SimulationRegistry::SimulationRegistry() : m_nextSimulationId(0), m_nextSubscriptionId(0)
{
}

void SimulationRegistry::_notifyEvent(SimulationRegistryEventType eventType,
                                      const SimulationId& simulationId,
                                      const std::string& simulationName)
{
    // Snapshot subscriptions so callbacks can safely add or remove subscriptions.
    // Subscribers present at notification start receive the current event once.
    const SimulationRegistryEventSubscribersMap subscriptions = m_simulationRegistryEventCallbacks;
    for (const auto& subscription : subscriptions)
    {
        try
        {
            // Call the callback function for each subscriber, passing the stored user data pointer.
            subscription.second.first(eventType, simulationId, simulationName, subscription.second.second);
        }
        catch (const std::exception& error)
        {
            ISAACSIM_LOG_ERROR(g_kLogger, "Simulation registry event callback threw: {}", error.what());
        }
        catch (...)
        {
            ISAACSIM_LOG_ERROR(g_kLogger, "Simulation registry event callback threw an unknown exception");
        }
    }
}

SimulationId SimulationRegistry::registerSimulation(const Simulation& simulation, const std::string& simulationName)
{
    const SimulationId simulationId = m_nextSimulationId++;
    m_simulations[simulationId] = SimulationRecord{ simulation, simulationName, true };
    _notifyEvent(SimulationRegistryEventType::eSimulationRegistered, simulationId, simulationName);
    return simulationId;
}

void SimulationRegistry::unregisterSimulation(const SimulationId& simulationId)
{
    const SimulationMap::const_iterator simulationIterator = m_simulations.find(simulationId);
    if (simulationIterator == m_simulations.end())
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "SimulationRegistry::unregisterSimulation: simulation not found");
        return;
    }

    // Copy to temporary string to be able to send the notice after the erasure.
    const std::string simulationName(simulationIterator->second.simulationName);

    m_simulations.erase(simulationIterator);
    _notifyEvent(SimulationRegistryEventType::eSimulationUnregistered, simulationId, simulationName);
}

const Simulation* SimulationRegistry::getSimulation(const SimulationId& simulationId) const
{
    const SimulationMap::const_iterator simulationIterator = m_simulations.find(simulationId);
    if (simulationIterator == m_simulations.end())
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "SimulationRegistry::getSimulation: simulation not found");
        return nullptr;
    }
    return &simulationIterator->second.simulation;
}

std::string SimulationRegistry::getSimulationName(const SimulationId& simulationId) const
{
    const SimulationMap::const_iterator simulationIterator = m_simulations.find(simulationId);
    if (simulationIterator == m_simulations.end())
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "SimulationRegistry::getSimulation: simulation not found");
        return {};
    }
    return simulationIterator->second.simulationName;
}

void SimulationRegistry::activateSimulation(const SimulationId& simulationId)
{
    const SimulationMap::iterator simulationIterator = m_simulations.find(simulationId);
    if (simulationIterator == m_simulations.end())
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "SimulationRegistry::activateSimulation: simulation not found");
        return;
    }
    simulationIterator->second.isActive = true;
    _notifyEvent(SimulationRegistryEventType::eSimulationActivated, simulationId, getSimulationName(simulationId));
}

void SimulationRegistry::deactivateSimulation(const SimulationId& simulationId)
{
    const SimulationMap::iterator simulationIterator = m_simulations.find(simulationId);
    if (simulationIterator == m_simulations.end())
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "SimulationRegistry::deactivateSimulation: simulation not found");
        return;
    }
    simulationIterator->second.isActive = false;
    _notifyEvent(SimulationRegistryEventType::eSimulationDeactivated, simulationId, getSimulationName(simulationId));
}

bool SimulationRegistry::isSimulationActive(const SimulationId& simulationId) const
{
    const SimulationMap::const_iterator simulationIterator = m_simulations.find(simulationId);
    if (simulationIterator == m_simulations.end())
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "SimulationRegistry::isSimulationActive: simulation not found");
        return false;
    }
    return simulationIterator->second.isActive;
}

SubscriptionId SimulationRegistry::subscribeSimulationRegistryEvents(OnSimulationRegistryEventFunction onEvent,
                                                                     void* userData)
{
    if (!onEvent)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "SimulationRegistry::subscribeSimulationRegistryEvents: onEvent is empty");
        return g_kInvalidSubscriptionId;
    }
    const SubscriptionId subscriptionId = SubscriptionId(m_nextSubscriptionId++);
    m_simulationRegistryEventCallbacks.emplace(subscriptionId, std::make_pair(onEvent, userData));
    return subscriptionId;
}

void SimulationRegistry::unsubscribeSimulationRegistryEvents(SubscriptionId subscriptionId)
{
    if (subscriptionId == g_kInvalidSubscriptionId)
    {
        ISAACSIM_LOG_ERROR(
            g_kLogger, "SimulationRegistry::unsubscribeSimulationRegistryEvents: subscriptionId is invalid");
        return;
    }
    m_simulationRegistryEventCallbacks.erase(subscriptionId);
}

} // namespace registration
} // namespace physics
} // namespace isaacsim

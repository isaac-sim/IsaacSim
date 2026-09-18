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

#include <isaacsim/physics/manager/PhysicsManager.hpp>
#include <isaacsim/physics/manager/PhysicsSimulation.hpp>
#include <isaacsim/physics/registration/Physics.hpp>
#include <isaacsim/physics/registration/tensors/TensorRegistry.hpp>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace manager
{

namespace registration = isaacsim::physics::registration;

namespace
{

std::string convertToLowercase(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

} // namespace

PhysicsManager& PhysicsManager::getInstance()
{
    static PhysicsManager s_instance;
    return s_instance;
}

bool PhysicsManager::isInitialized() const
{
    return m_initialized;
}

void PhysicsManager::configure(float deltaTime)
{
    m_deltaTime = deltaTime;
    m_simulatedTime = 0.0f;
    m_simulatedPhysicsStepCount = 0;
}

bool PhysicsManager::initialize(void* ovstageInstance, int64_t usdStageId)
{
    // Check if there are any simulations registered
    const size_t simulationCount = registration::getSimulationCount();
    if (!simulationCount)
    {
        return false;
    }
    // Check if there are any active simulations
    std::vector<registration::SimulationId> simulationIds(simulationCount);
    simulationIds.resize(registration::getSimulationIds(simulationIds.data(), simulationIds.size()));
    const bool anyActiveSimulation =
        std::any_of(simulationIds.begin(), simulationIds.end(),
                    [](const registration::SimulationId& id) { return registration::isSimulationActive(id); });
    if (!anyActiveSimulation)
    {
        return false;
    }

    const std::string usdIdentifier = std::to_string(usdStageId);
    const InitializeResult result = isaacsim::physics::manager::initialize(ovstageInstance, usdIdentifier.c_str());
    if (result == InitializeResult::eOk)
    {
        m_initialized = true;
        m_simulatedTime = 0.0f;
        m_simulatedPhysicsStepCount = 0;
        return true;
    }
    return false;
}

bool PhysicsManager::invalidate()
{
    if (!isaacsim::physics::manager::close())
    {
        return false;
    }
    m_initialized = false;
    m_simulatedTime = 0.0f;
    m_simulatedPhysicsStepCount = 0;
    return true;
}

int PhysicsManager::step(int stepCount, std::function<bool(int, int)> callback)
{
    if (!m_initialized)
    {
        throw std::runtime_error("PhysicsManager::step called before initialize()");
    }
    // Step the physics simulation
    int currentStep = 0;
    for (; currentStep < stepCount; ++currentStep)
    {
        isaacsim::physics::manager::simulate(m_deltaTime, m_simulatedTime);
        m_simulatedTime += m_deltaTime;
        ++m_simulatedPhysicsStepCount;

        if (callback && !callback(currentStep + 1, stepCount))
        {
            return currentStep + 1;
        }
    }
    return currentStep;
}

bool PhysicsManager::publishTransformsToStage()
{
    if (!m_initialized)
    {
        throw std::runtime_error("PhysicsManager::publishTransformsToStage called before initialize()");
    }
    return isaacsim::physics::manager::publishTransformsToStage();
}

float PhysicsManager::getSimulatedTime() const
{
    return m_simulatedTime;
}

int PhysicsManager::getSimulatedPhysicsStepCount() const
{
    return m_simulatedPhysicsStepCount;
}

size_t PhysicsManager::registerCallback(SimulationEventFunction callback, PhysicsEvent event, int order)
{
    const size_t callbackId = ++m_nextCallbackId;

    switch (event)
    {
    case PhysicsEvent::ePhysicsPreStep:
    case PhysicsEvent::ePhysicsPostStep:
    {
        auto onStep = [callback = std::move(callback)](float elapsedTime, const registration::PhysicsStepContext& context)
        {
            SimulationEventPayload payload;
            payload.emplace("elapsed_time", elapsedTime);
            payload.emplace("scene_path", static_cast<size_t>(context.scenePath.path));
            payload.emplace("simulation_id", context.simulationId.id);
            callback(std::move(payload));
        };
        registration::SubscriptionId subscriptionId = isaacsim::physics::manager::subscribePhysicsOnStepEvents(
            event == PhysicsEvent::ePhysicsPreStep, order, std::move(onStep));
        if (subscriptionId != registration::g_kInvalidSubscriptionId)
        {
            m_callbacks[callbackId] = subscriptionId;
            return callbackId;
        }
        break;
    }
    default:
        break;
    }

    return 0;
}

bool PhysicsManager::deregisterCallback(size_t callbackId) noexcept
{
    auto iterator = m_callbacks.find(callbackId);
    if (iterator == m_callbacks.end())
    {
        return false;
    }
    try
    {
        isaacsim::physics::manager::unsubscribePhysicsOnStepEvents(iterator->second);
    }
    catch (...)
    {
        // Backend threw during unsubscription; remove the tracked entry so this
        // callbackId is not retried, but report failure to the caller.
        m_callbacks.erase(iterator);
        return false;
    }
    m_callbacks.erase(iterator);
    return true;
}

void PhysicsManager::deregisterAllCallbacks() noexcept
{
    std::vector<size_t> callbackIds;
    callbackIds.reserve(m_callbacks.size());
    for (const auto& [callbackId, _] : m_callbacks)
    {
        callbackIds.push_back(callbackId);
    }
    for (size_t callbackId : callbackIds)
    {
        this->deregisterCallback(callbackId);
    }
    m_callbacks.clear();
}

std::vector<std::pair<std::string, bool>> PhysicsManager::getRegisteredPhysicsEngines() const
{
    std::vector<std::pair<std::string, bool>> engines;

    const size_t simulationCount = registration::getSimulationCount();
    if (!simulationCount)
    {
        return engines;
    }

    std::vector<registration::SimulationId> simulationIds(simulationCount);
    const size_t copiedCount = registration::getSimulationIds(simulationIds.data(), simulationIds.size());
    simulationIds.resize(copiedCount);
    if (simulationIds.empty())
    {
        return engines;
    }

    engines.reserve(simulationIds.size());
    for (const registration::SimulationId simulationId : simulationIds)
    {
        std::string simulationName = registration::getSimulationName(simulationId);
        if (simulationName.empty())
        {
            continue;
        }
        const bool isActive = registration::isSimulationActive(simulationId);
        engines.emplace_back(convertToLowercase(simulationName), isActive);
    }
    return engines;
}

bool PhysicsManager::switchPhysicsEngine(const std::string& engine)
{

    const size_t simulationCount = registration::getSimulationCount();
    if (!simulationCount)
    {
        return false;
    }

    std::vector<registration::SimulationId> simulationIds(simulationCount);
    const size_t copiedCount = registration::getSimulationIds(simulationIds.data(), simulationIds.size());
    simulationIds.resize(copiedCount);
    if (simulationIds.empty())
    {
        return false;
    }

    const std::string targetEngine = convertToLowercase(engine);
    registration::SimulationId targetSimulationId = registration::g_kInvalidSimulationId;
    for (const registration::SimulationId simulationId : simulationIds)
    {
        std::string simulationName = registration::getSimulationName(simulationId);
        if (simulationName.empty())
        {
            continue;
        }
        if (convertToLowercase(simulationName) == targetEngine)
        {
            targetSimulationId = simulationId;
            break;
        }
    }

    if (targetSimulationId == registration::g_kInvalidSimulationId)
    {
        return false;
    }

    // deactivate all other engines for mutual exclusivity
    for (const registration::SimulationId simulationId : simulationIds)
    {
        if (simulationId == targetSimulationId)
        {
            continue;
        }
        if (registration::isSimulationActive(simulationId))
        {
            registration::deactivateSimulation(simulationId);
        }
    }

    // activate the target engine
    const bool isActive = registration::isSimulationActive(targetSimulationId);
    if (!isActive)
    {
        registration::activateSimulation(targetSimulationId);
    }
    return true;
}

std::shared_ptr<isaacsim::physics::tensors::IEntityView> PhysicsManager::createEntity(
    const std::string& engine,
    const std::string& entityType,
    const std::variant<std::string, std::vector<std::string>>& primPathPatterns,
    const std::optional<isaacsim::physics::tensors::EntityOptions>& options) const
{
    const std::vector<std::string> primPaths = std::holds_alternative<std::string>(primPathPatterns) ?
                                                   std::vector<std::string>{ std::get<std::string>(primPathPatterns) } :
                                                   std::get<std::vector<std::string>>(primPathPatterns);
    return isaacsim::physics::tensors::TensorRegistry::getInstance().createEntity(engine, entityType, primPaths, options);
}

} // namespace manager
} // namespace physics
} // namespace isaacsim

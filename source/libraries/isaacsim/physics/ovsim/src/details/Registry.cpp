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

#include "Registry.hpp"

#include <isaacsim/common/exceptions/Exceptions.hpp>
#include <isaacsim/common/string/String.hpp>
#include <isaacsim/physics/entities/ArticulationEntity.hpp>
#include <isaacsim/physics/entities/PhysicsEntity.hpp>
#include <isaacsim/physics/entities/RigidBodyEntity.hpp>
#include <isaacsim/physics/manager/PhysicsManager.hpp>
#include <ovsim/interfaces/details/Exception.hpp>

#include <algorithm>
#include <memory>
#include <unordered_map>

namespace isaacsim
{
namespace physics
{
namespace ovsim
{
namespace details
{

namespace entities = isaacsim::physics::entities;

namespace
{

enum class InstanceType
{
    eArticulationEntity,
    eRigidBodyEntity,
};

enum class InstanceMethodType
{
    eArticulationEntityDofLowerLimits,
    eArticulationEntityDofUpperLimits,
    eArticulationEntityRootPositions,
    eArticulationEntityRootOrientations,
    eArticulationEntityRootLinearVelocities,
    eArticulationEntityRootAngularVelocities,
    eRigidBodyEntityPositions,
    eRigidBodyEntityOrientations,
    eRigidBodyEntityLinearVelocities,
    eRigidBodyEntityAngularVelocities,
};

std::unordered_map<std::vector<std::string>,
                   std::pair<InstanceType, std::shared_ptr<entities::PhysicsEntity>>,
                   isaacsim::common::string::VectorStringHasher>
    g_instanceMap;
const std::unordered_map<InstanceType, std::unordered_map<std::string, std::variant<std::string, InstanceMethodType>>>
    g_kInstanceMethodMap = {
        { InstanceType::eArticulationEntity,
          { { "dof-efforts", "dof-actuation-forces" },
            { "dof-lower-limits", InstanceMethodType::eArticulationEntityDofLowerLimits },
            { "dof-upper-limits", InstanceMethodType::eArticulationEntityDofUpperLimits },
            { "root-position", InstanceMethodType::eArticulationEntityRootPositions },
            { "root-orientation", InstanceMethodType::eArticulationEntityRootOrientations },
            { "root-linear-velocity", InstanceMethodType::eArticulationEntityRootLinearVelocities },
            { "root-angular-velocity", InstanceMethodType::eArticulationEntityRootAngularVelocities } } },
        { InstanceType::eRigidBodyEntity,
          { { "mass", "masses" },
            { "inverse-mass", "inv-masses" },
            { "inertia", "inertias" },
            { "inverse-inertia", "inv-inertias" },
            { "position", InstanceMethodType::eRigidBodyEntityPositions },
            { "orientation", InstanceMethodType::eRigidBodyEntityOrientations },
            { "linear-velocity", InstanceMethodType::eRigidBodyEntityLinearVelocities },
            { "angular-velocity", InstanceMethodType::eRigidBodyEntityAngularVelocities } } }
    };

std::pair<InstanceType, std::shared_ptr<entities::PhysicsEntity>> getOrCreateInstance(const std::vector<std::string>& paths,
                                                                                      const std::string& attributeName)
{
    auto it = g_instanceMap.find(paths);
    if (it != g_instanceMap.end())
    {
        return it->second;
    }

    InstanceType instanceType;
    std::shared_ptr<entities::PhysicsEntity> instance;
    std::string activeEngine = getActivePhysicsEngine();

    // TODO: Provide using the umbrella/tensor API a list of supported implementations for each entity type.
    static const std::vector<std::string> s_kArticulationImplementationNames = {
        "dof-armatures",
        "dof-positions",
        "dof-position-targets",
        "dof-velocities",
        "dof-velocity-targets",
        "dof-actuation-forces",
        "dof-efforts",
        "dof-stiffnesses",
        "dof-dampings",
        "dof-lower-limits",
        "dof-upper-limits",
        "root-position",
        "root-orientation",
        "root-linear-velocity",
        "root-angular-velocity",
        "jacobians",
        "generalized-mass-matrices",
    };
    static const std::vector<std::string> s_kRigidBodyImplementationNames = {
        "mass",     "inverse-mass", "inertia",         "inverse-inertia",
        "position", "orientation",  "linear-velocity", "angular-velocity"
    };

    auto inList = [](const std::string& name, const std::vector<std::string>& list)
    { return std::find(list.begin(), list.end(), name) != list.end(); };

    if (inList(attributeName, s_kArticulationImplementationNames))
    {
        instanceType = InstanceType::eArticulationEntity;
        instance = std::make_shared<entities::ArticulationEntity>(activeEngine, paths);
    }
    else if (inList(attributeName, s_kRigidBodyImplementationNames))
    {
        instanceType = InstanceType::eRigidBodyEntity;
        instance = std::make_shared<entities::RigidBodyEntity>(activeEngine, paths);
    }
    else
    {
        throw ::ovsim::interfaces::details::AttributeError(attributeName);
    }

    g_instanceMap.emplace(paths, std::make_pair(instanceType, instance));
    return { instanceType, instance };
}

} // namespace

void clearRegistry()
{
    g_instanceMap.clear();
}

std::string getActivePhysicsEngine()
{
    auto& physicsManager = manager::PhysicsManager::getInstance();
    if (!physicsManager.isInitialized())
    {
        throw ::ovsim::interfaces::details::InitializationError("physics");
    }
    std::vector<std::pair<std::string, bool>> engines = physicsManager.getRegisteredPhysicsEngines();
    for (const auto& engine : engines)
    {
        if (engine.second)
        {
            return engine.first;
        }
    }
    throw std::runtime_error("No active physics engine found");
}

std::vector<std::string> processPaths(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return std::holds_alternative<std::string>(paths) ? std::vector<std::string>{ std::get<std::string>(paths) } :
                                                        std::get<std::vector<std::string>>(paths);
}

OutputValueType getAttributeValues(const std::vector<std::string>& paths,
                                   const std::string& attributeName,
                                   const std::optional<array::Array>& indices)
{
    auto [instanceType, instance] = getOrCreateInstance(paths, attributeName);
    // Look up for instance-specific implementations.
    auto instanceTypeIterator = g_kInstanceMethodMap.find(instanceType);
    if (instanceTypeIterator != g_kInstanceMethodMap.end())
    {
        // Look up for attribute-specific implementations.
        auto methodIterator = instanceTypeIterator->second.find(attributeName);
        if (methodIterator != instanceTypeIterator->second.end())
        {
            // Attribute mapping to another attribute name
            if (std::holds_alternative<std::string>(methodIterator->second))
            {
                return instance->getData(std::get<std::string>(methodIterator->second), indices);
            }
            // Attribute mapping to an instance method
            switch (std::get<InstanceMethodType>(methodIterator->second))
            {
            case InstanceMethodType::eArticulationEntityDofLowerLimits:
                return std::get<0>(
                    std::static_pointer_cast<entities::ArticulationEntity>(instance)->getDofLimits(indices));
            case InstanceMethodType::eArticulationEntityDofUpperLimits:
                return std::get<1>(
                    std::static_pointer_cast<entities::ArticulationEntity>(instance)->getDofLimits(indices));
            case InstanceMethodType::eArticulationEntityRootPositions:
                return std::get<0>(
                    std::static_pointer_cast<entities::ArticulationEntity>(instance)->getWorldPoses(indices));
            case InstanceMethodType::eArticulationEntityRootOrientations:
                return std::get<1>(
                    std::static_pointer_cast<entities::ArticulationEntity>(instance)->getWorldPoses(indices));
            case InstanceMethodType::eArticulationEntityRootLinearVelocities:
                return std::get<0>(
                    std::static_pointer_cast<entities::ArticulationEntity>(instance)->getVelocities(indices));
            case InstanceMethodType::eArticulationEntityRootAngularVelocities:
                return std::get<1>(
                    std::static_pointer_cast<entities::ArticulationEntity>(instance)->getVelocities(indices));
            case InstanceMethodType::eRigidBodyEntityPositions:
                return std::get<0>(std::static_pointer_cast<entities::RigidBodyEntity>(instance)->getWorldPoses(indices));
            case InstanceMethodType::eRigidBodyEntityOrientations:
                return std::get<1>(std::static_pointer_cast<entities::RigidBodyEntity>(instance)->getWorldPoses(indices));
            case InstanceMethodType::eRigidBodyEntityLinearVelocities:
                return std::get<0>(std::static_pointer_cast<entities::RigidBodyEntity>(instance)->getVelocities(indices));
            case InstanceMethodType::eRigidBodyEntityAngularVelocities:
                return std::get<1>(std::static_pointer_cast<entities::RigidBodyEntity>(instance)->getVelocities(indices));
            default:
                throw std::runtime_error("Non-implemented instance method mapping for attribute '" + attributeName + "'");
            }
        }
    }
    // Default implementation: PhysicsEntity::getData.
    try
    {
        return instance->getData(attributeName, indices);
    }
    catch (const std::out_of_range&)
    {
        throw ::ovsim::interfaces::details::AttributeError(attributeName);
    }
}

void setAttributeValues(const std::vector<std::string>& paths,
                        const std::string& attributeName,
                        const InputValueType& values,
                        const std::optional<array::Array>& indices)
{
    if (!std::holds_alternative<array::Array>(values))
    {
        throw ::ovsim::interfaces::details::AttributeError("Unsupported input value type");
    }
    auto arrayValues = std::get<array::Array>(values);

    auto [instanceType, instance] = getOrCreateInstance(paths, attributeName);
    // Look up for instance-specific implementations.
    auto instanceTypeIterator = g_kInstanceMethodMap.find(instanceType);
    if (instanceTypeIterator != g_kInstanceMethodMap.end())
    {
        // Look up for attribute-specific implementations.
        auto methodIterator = instanceTypeIterator->second.find(attributeName);
        if (methodIterator != instanceTypeIterator->second.end())
        {
            // Attribute mapping to another attribute name
            if (std::holds_alternative<std::string>(methodIterator->second))
            {
                instance->setData(std::get<std::string>(methodIterator->second), arrayValues, indices);
                return;
            }
            // Attribute mapping to an instance method
            switch (std::get<InstanceMethodType>(methodIterator->second))
            {
            case InstanceMethodType::eArticulationEntityDofLowerLimits:
                std::static_pointer_cast<entities::ArticulationEntity>(instance)->setDofLimits(
                    arrayValues, std::nullopt, indices);
                return;
            case InstanceMethodType::eArticulationEntityDofUpperLimits:
                std::static_pointer_cast<entities::ArticulationEntity>(instance)->setDofLimits(
                    std::nullopt, arrayValues, indices);
                return;
            case InstanceMethodType::eArticulationEntityRootPositions:
                std::static_pointer_cast<entities::ArticulationEntity>(instance)->setWorldPoses(
                    arrayValues, std::nullopt, indices);
                return;
            case InstanceMethodType::eArticulationEntityRootOrientations:
                std::static_pointer_cast<entities::ArticulationEntity>(instance)->setWorldPoses(
                    std::nullopt, arrayValues, indices);
                return;
            case InstanceMethodType::eArticulationEntityRootLinearVelocities:
                std::static_pointer_cast<entities::ArticulationEntity>(instance)->setVelocities(
                    arrayValues, std::nullopt, indices);
                return;
            case InstanceMethodType::eArticulationEntityRootAngularVelocities:
                std::static_pointer_cast<entities::ArticulationEntity>(instance)->setVelocities(
                    std::nullopt, arrayValues, indices);
                return;
            case InstanceMethodType::eRigidBodyEntityPositions:
                std::static_pointer_cast<entities::RigidBodyEntity>(instance)->setWorldPoses(
                    arrayValues, std::nullopt, indices);
                return;
            case InstanceMethodType::eRigidBodyEntityOrientations:
                std::static_pointer_cast<entities::RigidBodyEntity>(instance)->setWorldPoses(
                    std::nullopt, arrayValues, indices);
                return;
            case InstanceMethodType::eRigidBodyEntityLinearVelocities:
                std::static_pointer_cast<entities::RigidBodyEntity>(instance)->setVelocities(
                    arrayValues, std::nullopt, indices);
                return;
            case InstanceMethodType::eRigidBodyEntityAngularVelocities:
                std::static_pointer_cast<entities::RigidBodyEntity>(instance)->setVelocities(
                    std::nullopt, arrayValues, indices);
                return;
            default:
                throw std::runtime_error("Non-implemented instance method mapping for attribute '" + attributeName + "'");
            }
        }
    }
    // Default implementation: PhysicsEntity::setData.
    try
    {
        instance->setData(attributeName, arrayValues, indices);
    }
    catch (const std::out_of_range&)
    {
        throw ::ovsim::interfaces::details::AttributeError(attributeName);
    }
}

} // namespace details
} // namespace ovsim
} // namespace physics
} // namespace isaacsim

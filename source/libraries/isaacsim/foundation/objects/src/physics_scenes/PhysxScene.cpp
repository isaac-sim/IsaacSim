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

#include "PhysicsSceneUtils.hpp"

#include <isaacsim/foundation/objects/physics_scenes/PhysxScene.hpp>

#include <array>
#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace physics_scenes
{

namespace
{

/** @brief Pairs a GPU configuration field with the PhysX attribute backing it. */
struct GpuConfigurationField
{
    std::optional<array::Array> PhysxGpuConfiguration::*member;
    const char* attributeName;
};

/** @brief Mapping between the GPU configuration fields and their PhysX attributes. */
constexpr std::array<GpuConfigurationField, 12> g_kGpuConfigurationFields = {
    { { &PhysxGpuConfiguration::gpuCollisionStackSize, "physxScene:gpuCollisionStackSize" },
      { &PhysxGpuConfiguration::gpuFoundLostAggregatePairsCapacity, "physxScene:gpuFoundLostAggregatePairsCapacity" },
      { &PhysxGpuConfiguration::gpuFoundLostPairsCapacity, "physxScene:gpuFoundLostPairsCapacity" },
      { &PhysxGpuConfiguration::gpuHeapCapacity, "physxScene:gpuHeapCapacity" },
      { &PhysxGpuConfiguration::gpuMaximumDeformableSurfaceContacts, "physxScene:gpuMaxDeformableSurfaceContacts" },
      { &PhysxGpuConfiguration::gpuMaximumDeformableVolumeContacts, "physxScene:gpuMaxDeformableVolumeContacts" },
      { &PhysxGpuConfiguration::gpuMaximumPartitionCount, "physxScene:gpuMaxNumPartitions" },
      { &PhysxGpuConfiguration::gpuMaximumParticleContacts, "physxScene:gpuMaxParticleContacts" },
      { &PhysxGpuConfiguration::gpuMaximumRigidContactCount, "physxScene:gpuMaxRigidContactCount" },
      { &PhysxGpuConfiguration::gpuMaximumRigidPatchCount, "physxScene:gpuMaxRigidPatchCount" },
      { &PhysxGpuConfiguration::gpuTemporaryBufferCapacity, "physxScene:gpuTempBufferCapacity" },
      { &PhysxGpuConfiguration::gpuTotalAggregatePairsCapacity, "physxScene:gpuTotalAggregatePairsCapacity" } }
};

} // namespace

PhysxScene::PhysxScene(const std::variant<std::string, std::vector<std::string>>& paths) : PhysicsScene(paths)
{
    this->applyApi("PhysxSceneAPI");
}

void PhysxScene::setDeltaTimes(const array::Array& deltaTimes, const std::optional<array::Array>& indices)
{
    this->setTimeStepsPerSecond(details::deltaTimesToTimeStepsPerSecond(deltaTimes), indices);
}

array::Array PhysxScene::getDeltaTimes(const std::optional<array::Array>& indices)
{
    return details::timeStepsPerSecondToDeltaTimes(this->getTimeStepsPerSecond(indices));
}

void PhysxScene::setTimeStepsPerSecond(const array::Array& timeStepsPerSecond, const std::optional<array::Array>& indices)
{
    // Keep the solver-agnostic step frequency in sync.
    PhysicsScene::setTimeStepsPerSecond(timeStepsPerSecond, indices);
    this->setAttributeValues("physxScene:timeStepsPerSecond", timeStepsPerSecond, indices);
}

array::Array PhysxScene::getTimeStepsPerSecond(const std::optional<array::Array>& indices)
{
    // The constructor applies the PhysX schema to every wrapped prim, so this only falls back if it
    // was removed afterwards. Reading the attribute of a prim that lacks the schema would throw.
    if (!array::all(this->hasApi("PhysxSceneAPI", std::nullopt, indices)).item<bool>())
    {
        return PhysicsScene::getTimeStepsPerSecond(indices);
    }
    return std::get<array::Array>(this->getAttributeValues("physxScene:timeStepsPerSecond", indices));
}

void PhysxScene::setSolverTypes(const std::variant<std::string, std::vector<std::string>>& solverTypes,
                                const std::optional<array::Array>& indices)
{
    const std::vector<std::string> tokens = _resolveStringList(solverTypes, indices);
    details::validateTokens(tokens, { "TGS", "PGS" }, "solver type");
    this->setAttributeValues("physxScene:solverType", tokens, indices);
}

std::vector<std::string> PhysxScene::getSolverTypes(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("physxScene:solverType", indices));
}

void PhysxScene::setBroadphaseTypes(const std::variant<std::string, std::vector<std::string>>& broadphaseTypes,
                                    const std::optional<array::Array>& indices)
{
    const std::vector<std::string> tokens = _resolveStringList(broadphaseTypes, indices);
    details::validateTokens(tokens, { "MBP", "GPU", "SAP" }, "broadphase type");
    this->setAttributeValues("physxScene:broadphaseType", tokens, indices);
}

std::vector<std::string> PhysxScene::getBroadphaseTypes(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("physxScene:broadphaseType", indices));
}

void PhysxScene::setEnabledGpuDynamics(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("physxScene:enableGPUDynamics", enabled, indices);
    // GPU dynamics does not support CCD, so the two settings are mutually exclusive: turning this one
    // on turns CCD off on those same prims. The multiplication of the two boolean masks is a logical
    // AND, broadcast over the selection. The attribute is written directly rather than through
    // setEnabledCcds, which would apply the reciprocal rule and undo this one.
    if (array::any(enabled).item<bool>())
    {
        this->setAttributeValues("physxScene:enableCCD",
                                 array::multiply(array::logicalNot(enabled), this->getEnabledCcds(indices)), indices);
    }
}

array::Array PhysxScene::getEnabledGpuDynamics(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("physxScene:enableGPUDynamics", indices));
}

void PhysxScene::setEnabledCcds(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("physxScene:enableCCD", enabled, indices);
    // The reciprocal of the rule in setEnabledGpuDynamics: turning CCD on turns GPU dynamics off on
    // those same prims, so the wrapper cannot leave both enabled at once.
    if (array::any(enabled).item<bool>())
    {
        this->setAttributeValues("physxScene:enableGPUDynamics",
                                 array::multiply(array::logicalNot(enabled), this->getEnabledGpuDynamics(indices)),
                                 indices);
    }
}

array::Array PhysxScene::getEnabledCcds(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("physxScene:enableCCD", indices));
}

void PhysxScene::setEnabledStabilizations(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("physxScene:enableStabilization", enabled, indices);
}

array::Array PhysxScene::getEnabledStabilizations(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("physxScene:enableStabilization", indices));
}

void PhysxScene::setGpuConfiguration(const PhysxGpuConfiguration& configuration,
                                     const std::optional<array::Array>& indices)
{
    for (const auto& field : g_kGpuConfigurationFields)
    {
        const std::optional<array::Array>& values = configuration.*(field.member);
        if (values.has_value())
        {
            this->setAttributeValues(field.attributeName, *values, indices);
        }
    }
}

PhysxGpuConfiguration PhysxScene::getGpuConfiguration(const std::optional<array::Array>& indices)
{
    PhysxGpuConfiguration configuration;
    for (const auto& field : g_kGpuConfigurationFields)
    {
        configuration.*(field.member) = std::get<array::Array>(this->getAttributeValues(field.attributeName, indices));
    }
    return configuration;
}

} // namespace physics_scenes
} // namespace objects
} // namespace foundation
} // namespace isaacsim

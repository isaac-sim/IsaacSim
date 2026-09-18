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

#include <isaacsim/common/logging/Logging.hpp>
#include <isaacsim/foundation/objects/Prim.hpp>
#include <isaacsim/foundation/prims/physics/RigidBody.hpp>

#include <sstream>
#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace prims
{
namespace physics
{

namespace array = isaacsim::common::array;

namespace
{

constexpr float g_kRadiansToDegrees = 57.29577951308f;
constexpr float g_kDegreesToRadians = 0.01745329252f;

std::string joinPaths(const std::vector<std::string>& v)
{
    std::ostringstream ss;
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        if (i)
            ss << ", ";
        ss << "'" << v[i] << "'";
    }
    return ss.str();
}

} // namespace

RigidBody::RigidBody(const std::variant<std::string, std::vector<std::string>>& paths,
                     const std::optional<array::Array>& masses,
                     const std::optional<array::Array>& densities,
                     bool applyPhysicsApis,
                     bool resetXformOpProperties)
    : isaacsim::foundation::objects::Xform()
{
    // Get prims.
    auto [existentPaths, nonexistentPaths] = this->resolvePaths(paths);
    if (!nonexistentPaths.empty())
    {
        throw std::runtime_error("Specified paths must correspond to existing prims: " + joinPaths(nonexistentPaths));
    }
    m_paths = std::move(existentPaths);
    // Initialize instance from arguments.
    _initialize(resetXformOpProperties);
    if (applyPhysicsApis)
    {
        this->applyPhysicsApis();
    }
    if (masses.has_value())
    {
        this->setMasses(*masses);
    }
    if (densities.has_value())
    {
        this->setDensities(*densities);
    }
}

void RigidBody::applyPhysicsApis(const std::optional<array::Array>& indices)
{
    this->applyApi("PhysicsMassAPI", std::nullopt, indices);
    this->applyApi("PhysicsRigidBodyAPI", std::nullopt, indices);
    this->applyApi("PhysxRigidBodyAPI", std::nullopt, indices);
}

void RigidBody::removePhysicsApis(const std::optional<array::Array>& indices)
{
    this->removeApi("PhysicsRigidBodyAPI", std::nullopt, indices);
    this->removeApi("PhysxRigidBodyAPI", std::nullopt, indices);
}

void RigidBody::setVelocities(const std::optional<array::Array>& linearVelocities,
                              const std::optional<array::Array>& angularVelocities,
                              const std::optional<array::Array>& indices)
{
    if (!linearVelocities.has_value() && !angularVelocities.has_value())
    {
        throw std::invalid_argument(
            "Both 'linearVelocities' and 'angularVelocities' are not defined. Define at least one of them");
    }
    const int64_t batchSize = _resolveIndexedSize(indices);
    if (linearVelocities.has_value())
    {
        this->setAttributeValues("physics:velocity",
                                 linearVelocities->reshape(array::Shape({ int64_t{ -1 }, int64_t{ 3 } }))
                                     .broadcastTo(array::Shape({ batchSize, int64_t{ 3 } })),
                                 indices);
    }
    if (angularVelocities.has_value())
    {
        auto values = angularVelocities->reshape(array::Shape({ int64_t{ -1 }, int64_t{ 3 } }))
                          .broadcastTo(array::Shape({ batchSize, int64_t{ 3 } }))
                          .flatten()
                          .get<std::vector<float>>();
        for (float& value : values)
        {
            value *= g_kRadiansToDegrees;
        }
        this->setAttributeValues("physics:angularVelocity",
                                 array::Array(values).reshape(array::Shape({ batchSize, int64_t{ 3 } })), indices);
    }
}

std::tuple<array::Array, array::Array> RigidBody::getVelocities(const std::optional<array::Array>& indices)
{
    auto linearVelocities = std::get<array::Array>(this->getAttributeValues("physics:velocity", indices));
    auto angularVelocities = std::get<array::Array>(this->getAttributeValues("physics:angularVelocity", indices));
    auto values = angularVelocities.flatten().get<std::vector<float>>();
    for (float& value : values)
    {
        value *= g_kDegreesToRadians;
    }
    return { linearVelocities, array::Array(values).reshape(angularVelocities.shape()) };
}

array::Array RigidBody::getMasses(const std::optional<array::Array>& indices, bool inverse)
{
    this->applyApi("PhysicsMassAPI", std::nullopt, indices);
    auto result = std::get<array::Array>(this->getAttributeValues("physics:mass", indices));
    if (!inverse)
    {
        return result;
    }
    auto values = result.flatten().get<std::vector<float>>();
    for (float& value : values)
    {
        value = 1.0f / (value + 1e-8f);
    }
    return array::Array(values).reshape(result.shape());
}

void RigidBody::setMasses(const array::Array& masses, const std::optional<array::Array>& indices)
{
    this->applyApi("PhysicsMassAPI", std::nullopt, indices);
    const int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues(
        "physics:mass",
        masses.reshape(array::Shape({ int64_t{ -1 }, int64_t{ 1 } })).broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })),
        indices);
}

array::Array RigidBody::getDensities(const std::optional<array::Array>& indices)
{
    this->applyApi("PhysicsMassAPI", std::nullopt, indices);
    return std::get<array::Array>(this->getAttributeValues("physics:density", indices));
}

void RigidBody::setDensities(const array::Array& densities, const std::optional<array::Array>& indices)
{
    this->applyApi("PhysicsMassAPI", std::nullopt, indices);
    const int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("physics:density",
                             densities.reshape(array::Shape({ int64_t{ -1 }, int64_t{ 1 } }))
                                 .broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })),
                             indices);
}

array::Array RigidBody::getSleepThresholds(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("physxRigidBody:sleepThreshold", indices));
}

void RigidBody::setSleepThresholds(const array::Array& thresholds, const std::optional<array::Array>& indices)
{
    const int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("physxRigidBody:sleepThreshold",
                             thresholds.reshape(array::Shape({ int64_t{ -1 }, int64_t{ 1 } }))
                                 .broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })),
                             indices);
}

void RigidBody::setEnabledRigidBodies(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    const int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("physics:rigidBodyEnabled",
                             enabled.reshape(array::Shape({ int64_t{ -1 }, int64_t{ 1 } }))
                                 .broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })),
                             indices);
}

array::Array RigidBody::getEnabledRigidBodies(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("physics:rigidBodyEnabled", indices));
}

void RigidBody::setEnabledGravities(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    const int64_t batchSize = _resolveIndexedSize(indices);
    auto values = enabled.flatten().broadcastTo(array::Shape({ batchSize })).get<std::vector<bool>>();
    std::vector<bool> disabled(values.size());
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        disabled[i] = !values[i];
    }
    this->setAttributeValues("physxRigidBody:disableGravity",
                             array::Array(disabled).reshape(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array RigidBody::getEnabledGravities(const std::optional<array::Array>& indices)
{
    auto disabled = std::get<array::Array>(this->getAttributeValues("physxRigidBody:disableGravity", indices));
    auto values = disabled.flatten().get<std::vector<bool>>();
    std::vector<bool> enabled(values.size());
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        enabled[i] = !values[i];
    }
    return array::Array(enabled).reshape(disabled.shape());
}

array::Array RigidBody::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    // A rigid body is a transformable prim carrying the Rigid Body API.
    const isaacsim::foundation::objects::Prim prims(paths);
    const std::vector<bool> areXformable = prims.isA("Xformable").flatten().get<std::vector<bool>>();
    const std::vector<bool> haveApi = prims.hasApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>();
    std::vector<bool> result(areXformable.size());
    for (std::size_t i = 0; i < result.size(); ++i)
    {
        result[i] = areXformable[i] && haveApi[i];
    }
    return array::Array(result).reshape(array::Shape({ -1, 1 }));
}

} // namespace physics
} // namespace prims
} // namespace foundation
} // namespace isaacsim

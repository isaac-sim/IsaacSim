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

#include "details/EntityUtils.hpp"

#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/physics/entities/RigidBodyEntity.hpp>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <utility>
#include <variant>

namespace isaacsim
{
namespace physics
{
namespace entities
{

namespace
{

std::tuple<array::Array, array::Array> getSplitData(PhysicsEntity& entity,
                                                    const std::string& name,
                                                    size_t index,
                                                    const std::optional<array::Array>& indices)
{
    std::vector<array::Array> components = details::splitArray(entity.getData(name, indices), index);
    return { std::move(components[0]), std::move(components[1]) };
}

void setSplitData(PhysicsEntity& entity,
                  const std::string& name,
                  size_t index,
                  const std::optional<array::Array>& first,
                  const std::optional<array::Array>& second,
                  const std::optional<array::Array>& indices)
{
    std::vector<array::Array> components = details::splitArray(entity.getData(name, indices), index);
    if (first.has_value())
    {
        components[0].set(*first);
    }
    if (second.has_value())
    {
        components[1].set(*second);
    }
    entity.setData(name, details::joinArrays(components), indices);
}

// Build a zero-filled (count, 3) component, overwritten by the given data when it is defined.
array::Array buildVectorComponent(size_t count, const std::optional<array::Array>& data)
{
    array::Array component = array::Array(std::vector<float>(count * 3, 0.0f), array::Dtype::Float32())
                                 .reshape(array::Shape(std::vector<int64_t>{ static_cast<int64_t>(count), 3 }));
    if (data.has_value())
    {
        component.set(*data);
    }
    return component;
}

} // namespace

RigidBodyEntity::RigidBodyEntity(const std::string& engine,
                                 const std::variant<std::string, std::vector<std::string>>& paths)
    : PhysicsEntity(engine, "rigid-body", paths)
{
}

size_t RigidBodyEntity::numShapes() const
{
    auto metadata = m_entityView->getMetadata("num-shapes");
    if (std::holds_alternative<int64_t>(metadata))
    {
        return static_cast<size_t>(std::get<int64_t>(metadata));
    }
    throw std::runtime_error("Unexpected metadata type for `num-shapes`");
}

std::tuple<array::Array, array::Array> RigidBodyEntity::getWorldPoses(const std::optional<array::Array>& indices)
{
    return getSplitData(*this, "transforms", 3, indices);
}

void RigidBodyEntity::setWorldPoses(const std::optional<array::Array>& positions,
                                    const std::optional<array::Array>& orientations,
                                    const std::optional<array::Array>& indices)
{
    if (!positions.has_value() && !orientations.has_value())
    {
        throw std::invalid_argument("Both `positions` and `orientations` are undefined. Define at least one of them");
    }
    setSplitData(*this, "transforms", 3, positions, orientations, indices);
}

std::tuple<array::Array, array::Array> RigidBodyEntity::getVelocities(const std::optional<array::Array>& indices)
{
    return getSplitData(*this, "velocities", 3, indices);
}

void RigidBodyEntity::setVelocities(const std::optional<array::Array>& linearVelocities,
                                    const std::optional<array::Array>& angularVelocities,
                                    const std::optional<array::Array>& indices)
{
    if (!linearVelocities.has_value() && !angularVelocities.has_value())
    {
        throw std::invalid_argument(
            "Both `linear_velocities` and `angular_velocities` are undefined. Define at least one of them");
    }
    setSplitData(*this, "velocities", 3, linearVelocities, angularVelocities, indices);
}

void RigidBodyEntity::applyForces(const array::Array& forces, const std::optional<array::Array>& indices)
{
    setData("forces", forces.reshape(array::Shape({ -1, 3 })), indices);
}

void RigidBodyEntity::applyForcesAndTorquesAtPositions(const std::optional<array::Array>& forces,
                                                       const std::optional<array::Array>& torques,
                                                       const std::optional<array::Array>& positions,
                                                       const std::optional<array::Array>& indices)
{
    if (!forces.has_value() && !torques.has_value())
    {
        throw std::invalid_argument("Both `forces` and `torques` are undefined. Define at least one of them");
    }
    // The wrench tensor is write-only, so undefined components default to zero rather than to their current values
    const size_t count = indices.has_value() ? indices->size() : numPrims();
    array::Array wrenches =
        details::joinArrays({ buildVectorComponent(count, forces), buildVectorComponent(count, torques),
                              buildVectorComponent(count, positions) });
    setData("apply-forces-and-torques-at-position", wrenches, indices);
}

array::Array RigidBodyEntity::getMasses(const std::optional<array::Array>& indices, bool inverse)
{
    return getData(inverse ? "inv-masses" : "masses", indices).reshape(array::Shape({ -1, 1 }));
}

void RigidBodyEntity::setMasses(const array::Array& masses, const std::optional<array::Array>& indices)
{
    setData("masses", masses.reshape(array::Shape({ -1 })), indices);
}

array::Array RigidBodyEntity::getInertias(const std::optional<array::Array>& indices, bool inverse)
{
    return getData(inverse ? "inv-inertias" : "inertias", indices);
}

void RigidBodyEntity::setInertias(const array::Array& inertias, const std::optional<array::Array>& indices)
{
    setData("inertias", inertias.reshape(array::Shape({ -1, 9 })), indices);
}

std::tuple<array::Array, array::Array> RigidBodyEntity::getComs(const std::optional<array::Array>& indices)
{
    return getSplitData(*this, "coms", 3, indices);
}

void RigidBodyEntity::setComs(const std::optional<array::Array>& positions,
                              const std::optional<array::Array>& orientations,
                              const std::optional<array::Array>& indices)
{
    if (!positions.has_value() && !orientations.has_value())
    {
        throw std::invalid_argument("Both `positions` and `orientations` are undefined. Define at least one of them");
    }
    setSplitData(*this, "coms", 3, positions, orientations, indices);
}

array::Array RigidBodyEntity::getEnabledRigidBodies(const std::optional<array::Array>& indices)
{
    const array::Array disabled = getData("disable-simulations", indices);
    return array::logicalNot(disabled).toDtype(disabled.dtype()).reshape(array::Shape({ -1, 1 }));
}

void RigidBodyEntity::setEnabledRigidBodies(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    setData("disable-simulations", array::logicalNot(enabled.reshape(array::Shape({ -1 }))).toDtype(enabled.dtype()),
            indices);
}

array::Array RigidBodyEntity::getEnabledGravities(const std::optional<array::Array>& indices)
{
    const array::Array disabled = getData("disable-gravities", indices);
    return array::logicalNot(disabled).toDtype(disabled.dtype()).reshape(array::Shape({ -1, 1 }));
}

void RigidBodyEntity::setEnabledGravities(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    setData("disable-gravities", array::logicalNot(enabled.reshape(array::Shape({ -1 }))).toDtype(enabled.dtype()),
            indices);
}

} // namespace entities
} // namespace physics
} // namespace isaacsim

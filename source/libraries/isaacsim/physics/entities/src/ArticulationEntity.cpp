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
#include <isaacsim/physics/entities/ArticulationEntity.hpp>
#include <isaacsim/physics/registration/tensors/TensorTypes.hpp>

#include <algorithm>
#include <cstring>
#include <numeric>
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

size_t getMetadataInt(const std::shared_ptr<tensors::EntityView>& view, const std::string& key)
{
    tensors::Metadata metadata = view->getMetadata(key);
    if (std::holds_alternative<int64_t>(metadata))
    {
        return static_cast<size_t>(std::get<int64_t>(metadata));
    }
    throw std::runtime_error("Unexpected metadata type for `" + key + "`");
}

std::vector<std::string> getMetadataStrings(const std::shared_ptr<tensors::EntityView>& view, const std::string& key)
{
    tensors::Metadata metadata = view->getMetadata(key);
    if (std::holds_alternative<std::vector<std::string>>(metadata))
    {
        return std::get<std::vector<std::string>>(metadata);
    }
    throw std::runtime_error("Unexpected metadata type for `" + key + "`");
}

std::tuple<array::Array, array::Array> getSplitData(PhysicsEntity& entity,
                                                    const std::string& name,
                                                    size_t splitIndex,
                                                    const std::optional<array::Array>& indices)
{
    std::vector<array::Array> components = details::splitArray(entity.getData(name, indices), splitIndex);
    return { std::move(components[0]), std::move(components[1]) };
}

void setSplitData(PhysicsEntity& entity,
                  const std::string& name,
                  size_t splitIndex,
                  const std::optional<array::Array>& first,
                  const std::optional<array::Array>& second,
                  const std::optional<array::Array>& indices)
{
    std::vector<array::Array> components = details::splitArray(entity.getData(name, indices), splitIndex);
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

// Extracts slice [:, :, componentIndex] from a 3D array (N, D, K) → (N, D).
array::Array extractComponent(const array::Array& data, size_t componentIndex)
{
    if (!data.device().isCpu())
    {
        throw std::logic_error("extractComponent: not implemented for non-CPU devices");
    }
    const auto shape = data.shape().shape();
    const int64_t entityCount = shape[0];
    const int64_t itemCount = shape[1];
    const int64_t componentCount = shape[2];
    const size_t elemSize = data.dtype().size();

    auto buffer = std::shared_ptr<std::byte[]>(new std::byte[static_cast<size_t>(entityCount * itemCount) * elemSize]);
    const auto* src = static_cast<const std::byte*>(data.data());

    for (int64_t i = 0; i < entityCount * itemCount; ++i)
    {
        std::memcpy(buffer.get() + static_cast<size_t>(i) * elemSize,
                    src + static_cast<size_t>(i * componentCount + static_cast<int64_t>(componentIndex)) * elemSize,
                    elemSize);
    }

    return array::Array::fromBuffer(
        std::move(buffer), array::Shape({ entityCount, itemCount }), data.dtype(), data.device());
}

// Writes values (N, D) into slice [:, :, componentIndex] of a mutable 3D array (N, D, K).
void replaceComponent(array::Array& data, size_t componentIndex, const array::Array& values)
{
    if (!data.device().isCpu())
    {
        throw std::logic_error("replaceComponent: not implemented for non-CPU devices");
    }
    const auto shape = data.shape().shape();
    const int64_t entityCount = shape[0];
    const int64_t itemCount = shape[1];
    const int64_t componentCount = shape[2];
    const size_t elemSize = data.dtype().size();

    auto* dst = static_cast<std::byte*>(data.data());
    const auto* src = static_cast<const std::byte*>(values.data());

    for (int64_t i = 0; i < entityCount * itemCount; ++i)
    {
        std::memcpy(dst + static_cast<size_t>(i * componentCount + static_cast<int64_t>(componentIndex)) * elemSize,
                    src + static_cast<size_t>(i) * elemSize, elemSize);
    }
}

array::Array resolveNameIndices(const std::variant<std::string, std::vector<std::string>>& names,
                                const std::vector<std::string>& nameList,
                                const std::string& entityKind)
{
    const std::vector<std::string>& nameVector = std::holds_alternative<std::string>(names) ?
                                                     std::vector<std::string>{ std::get<std::string>(names) } :
                                                     std::get<std::vector<std::string>>(names);

    std::vector<int32_t> indices;
    indices.reserve(nameVector.size());
    for (const auto& name : nameVector)
    {
        auto it = std::find(nameList.begin(), nameList.end(), name);
        if (it == nameList.end())
        {
            throw std::invalid_argument("Invalid " + entityKind + " name `" + name + "`");
        }
        indices.push_back(static_cast<int32_t>(std::distance(nameList.begin(), it)));
    }
    return array::Array(indices, array::Dtype::Int32());
}

// Selects the rows of a (N, ...) array at the given indices, returning an (M, ...) array.
array::Array gatherRows(const array::Array& data, const std::optional<array::Array>& indices)
{
    if (!indices.has_value())
    {
        return data;
    }
    if (!data.device().isCpu())
    {
        throw std::logic_error("gatherRows: not implemented for non-CPU devices");
    }
    std::vector<int64_t> dimensions = data.shape().shape();
    const int64_t count = dimensions[0];
    const size_t rowSize = count > 0 ? data.nbytes() / static_cast<size_t>(count) : 0;

    const std::vector<int64_t> rows = indices->toDtype(array::Dtype::Int64()).get<std::vector<int64_t>>();
    dimensions[0] = static_cast<int64_t>(rows.size());

    auto buffer = std::shared_ptr<std::byte[]>(new std::byte[rows.size() * rowSize]);
    const auto* source = static_cast<const std::byte*>(data.data());
    for (size_t i = 0; i < rows.size(); ++i)
    {
        if (rows[i] < 0 || rows[i] >= count)
        {
            throw std::out_of_range("Index " + std::to_string(rows[i]) + " is out of range for " +
                                    std::to_string(count) + " articulations");
        }
        std::memcpy(buffer.get() + i * rowSize, source + static_cast<size_t>(rows[i]) * rowSize, rowSize);
    }

    return array::Array::fromBuffer(std::move(buffer), array::Shape(dimensions), data.dtype(), data.device());
}

array::Array makeZeros(const array::Array& shapeLike)
{
    const size_t count = shapeLike.size();
    std::vector<float> zeros(count, 0.0f);
    return array::Array(zeros, array::Dtype::Float32()).reshape(shapeLike.shape());
}

} // namespace

ArticulationEntity::ArticulationEntity(const std::string& engine,
                                       const std::variant<std::string, std::vector<std::string>>& paths)
    : PhysicsEntity(engine, "articulation", paths)
{
}

size_t ArticulationEntity::numDofs() const
{
    return getMetadataInt(m_entityView, "num-dofs");
}

std::vector<std::string> ArticulationEntity::dofNames() const
{
    return getMetadataStrings(m_entityView, "dof-names");
}

std::vector<std::vector<std::string>> ArticulationEntity::dofPaths() const
{
    throw std::runtime_error("`dof-paths` is not available via the tensor API");
}

array::Array ArticulationEntity::dofTypes() const
{
    throw std::runtime_error("`dof-types` is not available via the tensor API");
}

size_t ArticulationEntity::numJoints() const
{
    return getMetadataInt(m_entityView, "num-joints");
}

std::vector<std::string> ArticulationEntity::jointNames() const
{
    return getMetadataStrings(m_entityView, "joint-names");
}

std::vector<std::vector<std::string>> ArticulationEntity::jointPaths() const
{
    throw std::runtime_error("`joint-paths` is not available via the tensor API");
}

array::Array ArticulationEntity::jointTypes() const
{
    throw std::runtime_error("`joint-types` is not available via the tensor API");
}

size_t ArticulationEntity::numLinks() const
{
    return getMetadataInt(m_entityView, "num-links");
}

std::vector<std::string> ArticulationEntity::linkNames() const
{
    return getMetadataStrings(m_entityView, "link-names");
}

std::vector<std::vector<std::string>> ArticulationEntity::linkPaths() const
{
    throw std::runtime_error("`link-paths` is not available via the tensor API");
}

size_t ArticulationEntity::numShapes() const
{
    return getMetadataInt(m_entityView, "num-shapes");
}

size_t ArticulationEntity::numFixedTendons() const
{
    return getMetadataInt(m_entityView, "num-fixed-tendons");
}

std::tuple<size_t, size_t, size_t> ArticulationEntity::jacobianMatrixShape() const
{
    const auto specification =
        m_entityView->getImplementationSpecification("jacobians", tensors::ImplementationKind::eGet);
    // Shape hint: (A, R, C) where R = (L-1)*6 for fixed base, L*6 for floating base.
    const size_t rowCount = static_cast<size_t>(specification.shapeHint[1]);
    const size_t columnCount = static_cast<size_t>(specification.shapeHint[2]);
    return { rowCount / 6, 6, columnCount };
}

std::tuple<size_t, size_t> ArticulationEntity::massMatrixShape() const
{
    const auto specification =
        m_entityView->getImplementationSpecification("generalized-mass-matrices", tensors::ImplementationKind::eGet);
    // Shape hint: (A, M, M).
    const size_t matrixSize = static_cast<size_t>(specification.shapeHint[1]);
    return { matrixSize, matrixSize };
}

array::Array ArticulationEntity::getLinkIndices(const std::variant<std::string, std::vector<std::string>>& names)
{
    return resolveNameIndices(names, linkNames(), "link");
}

array::Array ArticulationEntity::getJointIndices(const std::variant<std::string, std::vector<std::string>>& names)
{
    return resolveNameIndices(names, jointNames(), "joint");
}

array::Array ArticulationEntity::getDofIndices(const std::variant<std::string, std::vector<std::string>>& names)
{
    return resolveNameIndices(names, dofNames(), "DOF");
}

std::tuple<array::Array, array::Array> ArticulationEntity::getDofLimits(const std::optional<array::Array>& indices,
                                                                        const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    array::Array data = getData("dof-limits", indices); // (N, D, 2)
    return { extractComponent(data, 0), extractComponent(data, 1) };
}

void ArticulationEntity::setDofLimits(const std::optional<array::Array>& lower,
                                      const std::optional<array::Array>& upper,
                                      const std::optional<array::Array>& indices,
                                      const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    if (!lower.has_value() && !upper.has_value())
    {
        throw std::invalid_argument("Both `lower` and `upper` are undefined. Define at least one of them");
    }
    array::Array data = getData("dof-limits", indices); // (N, D, 2)
    if (lower.has_value())
    {
        replaceComponent(data, 0, *lower);
    }
    if (upper.has_value())
    {
        replaceComponent(data, 1, *upper);
    }
    setData("dof-limits", data, indices);
}

std::tuple<array::Array, array::Array, array::Array> ArticulationEntity::getDofFrictionProperties(
    const std::optional<array::Array>& indices, const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    array::Array data = getData("dof-friction-properties", indices); // (N, D, 3)
    return { extractComponent(data, 0), extractComponent(data, 1), extractComponent(data, 2) };
}

void ArticulationEntity::setDofFrictionProperties(const std::optional<array::Array>& staticFrictions,
                                                  const std::optional<array::Array>& dynamicFrictions,
                                                  const std::optional<array::Array>& viscousFrictions,
                                                  const std::optional<array::Array>& indices,
                                                  const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    if (!staticFrictions.has_value() && !dynamicFrictions.has_value() && !viscousFrictions.has_value())
    {
        throw std::invalid_argument("All friction properties are undefined. Define at least one of them");
    }
    array::Array data = getData("dof-friction-properties", indices); // (N, D, 3)
    if (staticFrictions.has_value())
    {
        replaceComponent(data, 0, *staticFrictions);
    }
    if (dynamicFrictions.has_value())
    {
        replaceComponent(data, 1, *dynamicFrictions);
    }
    if (viscousFrictions.has_value())
    {
        replaceComponent(data, 2, *viscousFrictions);
    }
    setData("dof-friction-properties", data, indices);
}

std::tuple<array::Array, array::Array, array::Array> ArticulationEntity::getDofDriveModelProperties(
    const std::optional<array::Array>& indices, const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    array::Array data = getData("dof-drive-model-properties", indices); // (N, D, 3)
    return { extractComponent(data, 0), extractComponent(data, 1), extractComponent(data, 2) };
}

void ArticulationEntity::setDofDriveModelProperties(const std::optional<array::Array>& speedEffortGradients,
                                                    const std::optional<array::Array>& maximumActuatorVelocities,
                                                    const std::optional<array::Array>& velocityDependentResistances,
                                                    const std::optional<array::Array>& indices,
                                                    const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    if (!speedEffortGradients.has_value() && !maximumActuatorVelocities.has_value() &&
        !velocityDependentResistances.has_value())
    {
        throw std::invalid_argument("All drive model properties are undefined. Define at least one of them");
    }
    array::Array data = getData("dof-drive-model-properties", indices); // (N, D, 3)
    if (speedEffortGradients.has_value())
    {
        replaceComponent(data, 0, *speedEffortGradients);
    }
    if (maximumActuatorVelocities.has_value())
    {
        replaceComponent(data, 1, *maximumActuatorVelocities);
    }
    if (velocityDependentResistances.has_value())
    {
        replaceComponent(data, 2, *velocityDependentResistances);
    }
    setData("dof-drive-model-properties", data, indices);
}

array::Array ArticulationEntity::getDofArmatures(const std::optional<array::Array>& indices,
                                                 const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("dof-armatures", indices);
}

void ArticulationEntity::setDofArmatures(const array::Array& armatures,
                                         const std::optional<array::Array>& indices,
                                         const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    setData("dof-armatures", armatures, indices);
}

void ArticulationEntity::setDofPositionTargets(const array::Array& positions,
                                               const std::optional<array::Array>& indices,
                                               const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    setData("dof-position-targets", positions, indices);
}

array::Array ArticulationEntity::getDofPositionTargets(const std::optional<array::Array>& indices,
                                                       const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("dof-position-targets", indices);
}

void ArticulationEntity::setDofPositions(const array::Array& positions,
                                         const std::optional<array::Array>& indices,
                                         const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    setData("dof-positions", positions, indices);
}

array::Array ArticulationEntity::getDofPositions(const std::optional<array::Array>& indices,
                                                 const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("dof-positions", indices);
}

void ArticulationEntity::setDofVelocityTargets(const array::Array& velocities,
                                               const std::optional<array::Array>& indices,
                                               const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    setData("dof-velocity-targets", velocities, indices);
}

array::Array ArticulationEntity::getDofVelocityTargets(const std::optional<array::Array>& indices,
                                                       const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("dof-velocity-targets", indices);
}

void ArticulationEntity::setDofVelocities(const array::Array& velocities,
                                          const std::optional<array::Array>& indices,
                                          const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    setData("dof-velocities", velocities, indices);
}

array::Array ArticulationEntity::getDofVelocities(const std::optional<array::Array>& indices,
                                                  const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("dof-velocities", indices);
}

void ArticulationEntity::setDofEfforts(const array::Array& efforts,
                                       const std::optional<array::Array>& indices,
                                       const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    setData("dof-actuation-forces", efforts, indices);
}

array::Array ArticulationEntity::getDofEfforts(const std::optional<array::Array>& indices,
                                               const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("dof-actuation-forces", indices);
}

array::Array ArticulationEntity::getDofProjectedJointForces(const std::optional<array::Array>& indices,
                                                            const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("dof-projected-joint-forces", indices);
}

std::tuple<array::Array, array::Array> ArticulationEntity::getDofGains(const std::optional<array::Array>& indices,
                                                                       const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return { getData("dof-stiffnesses", indices), getData("dof-dampings", indices) };
}

void ArticulationEntity::setDofGains(const std::optional<array::Array>& stiffnesses,
                                     const std::optional<array::Array>& dampings,
                                     const std::optional<array::Array>& indices,
                                     const std::optional<array::Array>& dofIndices,
                                     bool updateDefaultGains)
{
    (void)dofIndices;
    if (!stiffnesses.has_value() && !dampings.has_value())
    {
        throw std::invalid_argument("Both `stiffnesses` and `dampings` are undefined. Define at least one of them");
    }
    if (stiffnesses.has_value())
    {
        setData("dof-stiffnesses", *stiffnesses, indices);
    }
    if (dampings.has_value())
    {
        setData("dof-dampings", *dampings, indices);
    }
    // Snapshot the gains of every articulation, since only a subset may have just been written
    if (updateDefaultGains)
    {
        auto [defaultStiffnesses, defaultDampings] = getDofGains();
        m_defaultDofStiffnesses = std::move(defaultStiffnesses);
        m_defaultDofDampings = std::move(defaultDampings);
    }
}

void ArticulationEntity::switchDofControlMode(const std::string& mode,
                                              const std::optional<array::Array>& indices,
                                              const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    if (mode != "position" && mode != "velocity" && mode != "effort")
    {
        throw std::invalid_argument("Invalid control mode `" + mode +
                                    "`. Supported modes are: position, velocity, effort");
    }
    // Adopt the gains in effect before any mode switch as the defaults to restore
    if (!m_defaultDofStiffnesses.has_value() || !m_defaultDofDampings.has_value())
    {
        auto [stiffnesses, dampings] = getDofGains();
        if (!m_defaultDofStiffnesses.has_value())
        {
            m_defaultDofStiffnesses = std::move(stiffnesses);
        }
        if (!m_defaultDofDampings.has_value())
        {
            m_defaultDofDampings = std::move(dampings);
        }
    }

    const array::Array defaultStiffnesses = gatherRows(*m_defaultDofStiffnesses, indices);
    const array::Array defaultDampings = gatherRows(*m_defaultDofDampings, indices);
    if (mode == "position")
    {
        setDofGains(defaultStiffnesses, defaultDampings, indices, std::nullopt, false);
    }
    else if (mode == "velocity")
    {
        setDofGains(makeZeros(defaultStiffnesses), defaultDampings, indices, std::nullopt, false);
    }
    else
    {
        setDofGains(makeZeros(defaultStiffnesses), makeZeros(defaultDampings), indices, std::nullopt, false);
    }
}

array::Array ArticulationEntity::getDofMaxEfforts(const std::optional<array::Array>& indices,
                                                  const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("dof-max-forces", indices);
}

void ArticulationEntity::setDofMaxEfforts(const array::Array& maxEfforts,
                                          const std::optional<array::Array>& indices,
                                          const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    setData("dof-max-forces", maxEfforts, indices);
}

array::Array ArticulationEntity::getDofMaxVelocities(const std::optional<array::Array>& indices,
                                                     const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("dof-max-velocities", indices);
}

void ArticulationEntity::setDofMaxVelocities(const array::Array& maxVelocities,
                                             const std::optional<array::Array>& indices,
                                             const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    setData("dof-max-velocities", maxVelocities, indices);
}

std::vector<std::vector<std::string>> ArticulationEntity::getDofDriveTypes(const std::optional<array::Array>& indices,
                                                                           const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    array::Array data = getData("drive-types", indices); // (N, D) uint8.
    const auto shape = data.shape().shape();
    const int64_t entityCount = shape[0];
    const int64_t dofCount = shape[1];
    const auto rawData = data.get<std::vector<uint8_t>>();

    std::vector<std::vector<std::string>> result(
        static_cast<size_t>(entityCount), std::vector<std::string>(static_cast<size_t>(dofCount)));
    for (int64_t i = 0; i < entityCount; ++i)
    {
        for (int64_t j = 0; j < dofCount; ++j)
        {
            const uint8_t value = rawData[static_cast<size_t>(i * dofCount + j)];
            if (value == 1)
            {
                result[i][j] = "force";
            }
            else if (value == 2)
            {
                result[i][j] = "acceleration";
            }
            else
            {
                result[i][j] = "none";
            }
        }
    }
    return result;
}

std::tuple<array::Array, array::Array> ArticulationEntity::getWorldPoses(const std::optional<array::Array>& indices)
{
    return getSplitData(*this, "root-transforms", 3, indices);
}

void ArticulationEntity::setWorldPoses(const std::optional<array::Array>& positions,
                                       const std::optional<array::Array>& orientations,
                                       const std::optional<array::Array>& indices)
{
    if (!positions.has_value() && !orientations.has_value())
    {
        throw std::invalid_argument("Both `positions` and `orientations` are undefined. Define at least one of them");
    }
    setSplitData(*this, "root-transforms", 3, positions, orientations, indices);
}

std::tuple<array::Array, array::Array> ArticulationEntity::getVelocities(const std::optional<array::Array>& indices)
{
    return getSplitData(*this, "root-velocities", 3, indices);
}

void ArticulationEntity::setVelocities(const std::optional<array::Array>& linearVelocities,
                                       const std::optional<array::Array>& angularVelocities,
                                       const std::optional<array::Array>& indices)
{
    if (!linearVelocities.has_value() && !angularVelocities.has_value())
    {
        throw std::invalid_argument(
            "Both `linearVelocities` and `angularVelocities` are undefined. Define at least one of them");
    }
    setSplitData(*this, "root-velocities", 3, linearVelocities, angularVelocities, indices);
}

std::tuple<array::Array, array::Array> ArticulationEntity::getLinkIncomingJointForce(
    const std::optional<array::Array>& indices, const std::optional<array::Array>& linkIndices)
{
    (void)linkIndices;
    // shape: (N, L, 6) — split last dim at 3 into forces (N, L, 3) and torques (N, L, 3)
    return getSplitData(*this, "link-incoming-joint-force", 3, indices);
}

array::Array ArticulationEntity::getJacobianMatrices(const std::optional<array::Array>& indices)
{
    return getData("jacobians", indices);
}

array::Array ArticulationEntity::getMassMatrices(const std::optional<array::Array>& indices)
{
    return getData("generalized-mass-matrices", indices);
}

array::Array ArticulationEntity::getDofCoriolisAndCentrifugalCompensationForces(
    const std::optional<array::Array>& indices, const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("coriolis-and-centrifugal-compensation-forces", indices);
}

array::Array ArticulationEntity::getDofGravityCompensationForces(const std::optional<array::Array>& indices,
                                                                 const std::optional<array::Array>& dofIndices)
{
    (void)dofIndices;
    return getData("gravity-compensation-forces", indices);
}

array::Array ArticulationEntity::getLinkMasses(const std::optional<array::Array>& indices,
                                               const std::optional<array::Array>&,
                                               bool inverse)
{
    return getData(inverse ? "inv-masses" : "masses", indices);
}

void ArticulationEntity::setLinkMasses(const array::Array& masses,
                                       const std::optional<array::Array>& indices,
                                       const std::optional<array::Array>& linkIndices)
{
    (void)linkIndices;
    setData("masses", masses, indices);
}

std::tuple<array::Array, array::Array> ArticulationEntity::getLinkComs(const std::optional<array::Array>& indices,
                                                                       const std::optional<array::Array>& linkIndices)
{
    (void)linkIndices;
    return getSplitData(*this, "coms", 3, indices);
}

void ArticulationEntity::setLinkComs(const std::optional<array::Array>& positions,
                                     const std::optional<array::Array>& orientations,
                                     const std::optional<array::Array>& indices,
                                     const std::optional<array::Array>& linkIndices)
{
    (void)linkIndices;
    if (!positions.has_value() && !orientations.has_value())
    {
        throw std::invalid_argument("Both `positions` and `orientations` are undefined. Define at least one of them");
    }
    setSplitData(*this, "coms", 3, positions, orientations, indices);
}

array::Array ArticulationEntity::getLinkInertias(const std::optional<array::Array>& indices,
                                                 const std::optional<array::Array>&,
                                                 bool inverse)
{
    return getData(inverse ? "inv-inertias" : "inertias", indices);
}

void ArticulationEntity::setLinkInertias(const array::Array& inertias,
                                         const std::optional<array::Array>& indices,
                                         const std::optional<array::Array>& linkIndices)
{
    (void)linkIndices;
    setData("inertias", inertias, indices);
}

array::Array ArticulationEntity::getLinkEnabledGravities(const std::optional<array::Array>& indices,
                                                         const std::optional<array::Array>& linkIndices)
{
    (void)linkIndices;
    const array::Array disabled = getData("disable-gravities", indices);
    return array::logicalNot(disabled).toDtype(disabled.dtype());
}

void ArticulationEntity::setLinkEnabledGravities(const array::Array& enabled,
                                                 const std::optional<array::Array>& indices,
                                                 const std::optional<array::Array>& linkIndices)
{
    (void)linkIndices;
    setData("disable-gravities", array::logicalNot(enabled).toDtype(enabled.dtype()), indices);
}

array::Array ArticulationEntity::getFixedTendonStiffnesses(const std::optional<array::Array>& indices,
                                                           const std::optional<array::Array>& tendonIndices)
{
    (void)tendonIndices;
    return getData("fixed-tendon-stiffnesses", indices);
}

array::Array ArticulationEntity::getFixedTendonDampings(const std::optional<array::Array>& indices,
                                                        const std::optional<array::Array>& tendonIndices)
{
    (void)tendonIndices;
    return getData("fixed-tendon-dampings", indices);
}

array::Array ArticulationEntity::getFixedTendonLimitStiffnesses(const std::optional<array::Array>& indices,
                                                                const std::optional<array::Array>& tendonIndices)
{
    (void)tendonIndices;
    return getData("fixed-tendon-limit-stiffnesses", indices);
}

std::tuple<array::Array, array::Array> ArticulationEntity::getFixedTendonLimits(
    const std::optional<array::Array>& indices, const std::optional<array::Array>& tendonIndices)
{
    (void)tendonIndices;
    array::Array data = getData("fixed-tendon-limits", indices); // (N, T, 2)
    return { extractComponent(data, 0), extractComponent(data, 1) };
}

array::Array ArticulationEntity::getFixedTendonRestLengths(const std::optional<array::Array>& indices,
                                                           const std::optional<array::Array>& tendonIndices)
{
    (void)tendonIndices;
    return getData("fixed-tendon-rest-lengths", indices);
}

array::Array ArticulationEntity::getFixedTendonOffsets(const std::optional<array::Array>& indices,
                                                       const std::optional<array::Array>& tendonIndices)
{
    (void)tendonIndices;
    return getData("fixed-tendon-offsets", indices);
}

void ArticulationEntity::setFixedTendonProperties(const std::optional<array::Array>& stiffnesses,
                                                  const std::optional<array::Array>& dampings,
                                                  const std::optional<array::Array>& limitStiffnesses,
                                                  const std::optional<array::Array>& lowerLimits,
                                                  const std::optional<array::Array>& upperLimits,
                                                  const std::optional<array::Array>& restLengths,
                                                  const std::optional<array::Array>& offsets,
                                                  const std::optional<array::Array>& indices,
                                                  const std::optional<array::Array>& tendonIndices)
{
    (void)tendonIndices;
    if (!stiffnesses.has_value() && !dampings.has_value() && !limitStiffnesses.has_value() &&
        !lowerLimits.has_value() && !upperLimits.has_value() && !restLengths.has_value() && !offsets.has_value())
    {
        throw std::invalid_argument("All fixed tendon properties are undefined. Define at least one of them");
    }
    if (stiffnesses.has_value())
    {
        setData("fixed-tendon-stiffnesses", *stiffnesses, indices);
    }
    if (dampings.has_value())
    {
        setData("fixed-tendon-dampings", *dampings, indices);
    }
    if (limitStiffnesses.has_value())
    {
        setData("fixed-tendon-limit-stiffnesses", *limitStiffnesses, indices);
    }
    if (lowerLimits.has_value() || upperLimits.has_value())
    {
        array::Array data = getData("fixed-tendon-limits", indices); // (N, T, 2)
        if (lowerLimits.has_value())
        {
            replaceComponent(data, 0, *lowerLimits);
        }
        if (upperLimits.has_value())
        {
            replaceComponent(data, 1, *upperLimits);
        }
        setData("fixed-tendon-limits", data, indices);
    }
    if (restLengths.has_value())
    {
        setData("fixed-tendon-rest-lengths", *restLengths, indices);
    }
    if (offsets.has_value())
    {
        setData("fixed-tendon-offsets", *offsets, indices);
    }
}

} // namespace entities
} // namespace physics
} // namespace isaacsim

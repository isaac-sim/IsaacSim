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

#include <isaacsim/physics/entities/Export.h>
#include <isaacsim/physics/entities/PhysicsEntity.hpp>

#include <cstddef>
#include <tuple>

namespace isaacsim
{
namespace physics
{
namespace entities
{

/**
 * @class RigidBodyEntity
 * @brief Wrapper over a batch of simulated rigid body prims.
 * @details
 * Extends PhysicsEntity with rigid body specific state and properties: world poses, linear and
 * angular velocities, external force/torque application, mass and inertia properties, centers of
 * mass, and per-body simulation and gravity toggles.
 *
 * All methods that accept an @p indices parameter operate only on the prims at those positions;
 * when @p indices is omitted, all wrapped prims are processed. Inputs smaller than the expected
 * shape are broadcast.
 */
class ISAACSIM_PHYSICS_ENTITIES_API RigidBodyEntity : public PhysicsEntity
{
public:
    /**
     * @brief Construct a rigid body entity wrapper for one or more USD prim paths.
     * @param[in] engine Name of the physics engine that simulates the prims.
     * @param[in] paths  Single path string or list of path strings. May include regular expressions.
     * @throws std::runtime_error if the entity cannot be created for the given engine and paths.
     */
    RigidBodyEntity(const std::string& engine, const std::variant<std::string, std::vector<std::string>>& paths);
    ~RigidBodyEntity() = default;

    /**
     * @brief Return the number of collision shapes per wrapped rigid body.
     * @return Count of collision shapes.
     * @throws std::runtime_error if the shape count cannot be queried from the physics engine.
     */
    size_t numShapes() const;

    /**
     * @brief Get the world-frame poses of the selected rigid bodies.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @return A pair of (positions, orientations). Positions have shape @c (N,3);
     *         orientations are quaternions @c xyzw with shape @c (N,4).
     */
    std::tuple<array::Array, array::Array> getWorldPoses(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the world-frame poses of the selected rigid bodies.
     * @details At least one of @p positions or @p orientations must be provided.
     *          This method teleports the bodies to the specified poses.
     * @param[in] positions    World-frame positions (shape @c (N,3)). Optional.
     * @param[in] orientations Orientations as quaternions @c xyzw (shape @c (N,4)). Optional.
     * @param[in] indices      Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if both @p positions and @p orientations are undefined.
     */
    void setWorldPoses(const std::optional<array::Array>& positions = std::nullopt,
                       const std::optional<array::Array>& orientations = std::nullopt,
                       const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the world-frame velocities of the selected rigid bodies.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @return A pair of (linear velocities, angular velocities), both with shape @c (N,3),
     *         in distance/second and radians/second respectively.
     */
    std::tuple<array::Array, array::Array> getVelocities(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the world-frame velocities of the selected rigid bodies.
     * @details At least one of @p linearVelocities or @p angularVelocities must be provided.
     * @param[in] linearVelocities  Linear velocities in distance/second (shape @c (N,3)). Optional.
     * @param[in] angularVelocities Angular velocities in radians/second (shape @c (N,3)). Optional.
     * @param[in] indices           Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                              processed.
     * @throws std::invalid_argument if both @p linearVelocities and @p angularVelocities are undefined.
     */
    void setVelocities(const std::optional<array::Array>& linearVelocities = std::nullopt,
                       const std::optional<array::Array>& angularVelocities = std::nullopt,
                       const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Apply forces at the centers of mass of the selected rigid bodies.
     * @details The applied forces are consumed by the next simulation step.
     * @param[in] forces  Forces to apply (shape @c (N,3)).
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     */
    void applyForces(const array::Array& forces, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Apply forces and torques at given world-frame positions of the selected rigid bodies.
     * @details At least one of @p forces or @p torques must be provided. Undefined components are
     *          applied as zero rather than retaining their previous values. The applied wrench is
     *          consumed by the next simulation step.
     * @param[in] forces    Forces to apply (shape @c (N,3)). Optional.
     * @param[in] torques   Torques to apply (shape @c (N,3)). Optional.
     * @param[in] positions World-frame positions at which the forces are applied (shape @c (N,3)).
     *                      Optional; defaults to the bodies' origins.
     * @param[in] indices   Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if both @p forces and @p torques are undefined.
     */
    void applyForcesAndTorquesAtPositions(const std::optional<array::Array>& forces = std::nullopt,
                                          const std::optional<array::Array>& torques = std::nullopt,
                                          const std::optional<array::Array>& positions = std::nullopt,
                                          const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the masses of the selected rigid bodies.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] inverse Whether to return the inverse masses (@c true) or the masses (@c false).
     * @return Masses or inverse masses (shape @c (N,1)).
     */
    array::Array getMasses(const std::optional<array::Array>& indices = std::nullopt, bool inverse = false);

    /**
     * @brief Set the masses of the selected rigid bodies.
     * @param[in] masses  Masses to apply (shape @c (N,1)).
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     */
    void setMasses(const array::Array& masses, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the inertia tensors of the selected rigid bodies.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] inverse Whether to return the inverse inertia tensors (@c true) or the inertia tensors (@c false).
     * @return Flattened row-major 3x3 inertia tensors (shape @c (N,9)).
     */
    array::Array getInertias(const std::optional<array::Array>& indices = std::nullopt, bool inverse = false);

    /**
     * @brief Set the inertia tensors of the selected rigid bodies.
     * @param[in] inertias Flattened row-major 3x3 inertia tensors (shape @c (N,9)).
     * @param[in] indices  Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     */
    void setInertias(const array::Array& inertias, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the centers of mass of the selected rigid bodies, expressed in their local frames.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @return A pair of (positions, orientations). Positions have shape @c (N,3);
     *         orientations are quaternions @c xyzw with shape @c (N,4).
     */
    std::tuple<array::Array, array::Array> getComs(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the centers of mass of the selected rigid bodies, expressed in their local frames.
     * @details At least one of @p positions or @p orientations must be provided.
     * @param[in] positions    Center of mass positions (shape @c (N,3)). Optional.
     * @param[in] orientations Center of mass orientations as quaternions @c xyzw (shape @c (N,4)). Optional.
     * @param[in] indices      Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if both @p positions and @p orientations are undefined.
     */
    void setComs(const std::optional<array::Array>& positions = std::nullopt,
                 const std::optional<array::Array>& orientations = std::nullopt,
                 const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the simulation-enabled state of the selected rigid bodies.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @return Boolean flags (shape @c (N,1)). @c true if the body is simulated, @c false otherwise.
     */
    array::Array getEnabledRigidBodies(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable the simulation of the selected rigid bodies.
     * @param[in] enabled Boolean flags (shape @c (N,1)). @c true simulates the body, @c false excludes it.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     */
    void setEnabledRigidBodies(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the gravity-enabled state of the selected rigid bodies.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @return Boolean flags (shape @c (N,1)). @c true if gravity affects the body, @c false otherwise.
     */
    array::Array getEnabledGravities(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable gravity for the selected rigid bodies.
     * @param[in] enabled Boolean flags (shape @c (N,1)). @c true applies gravity, @c false excludes it.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     */
    void setEnabledGravities(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);
};

} // namespace entities
} // namespace physics
} // namespace isaacsim

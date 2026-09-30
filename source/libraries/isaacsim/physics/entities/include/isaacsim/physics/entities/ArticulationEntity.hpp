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
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace entities
{

/**
 * @class ArticulationEntity
 * @brief Wrapper over a batch of simulated articulation prims.
 * @details
 * Extends PhysicsEntity with articulation specific state and properties: kinematic tree topology
 * (links, joints, degrees of freedom), root pose and velocity, DOF state and drive configuration,
 * link mass properties, dynamics quantities (Jacobian and mass matrices, compensation forces),
 * and fixed tendon properties.
 *
 * Batched quantities are laid out with @c N as the number of selected prims, @c D as the number of
 * degrees of freedom, @c L as the number of links, and @c T as the number of fixed tendons.
 * All methods that accept an @p indices parameter operate only on the prims at those positions;
 * when @p indices is omitted, all wrapped prims are processed. Inputs smaller than the expected
 * shape are broadcast.
 */
class ISAACSIM_PHYSICS_ENTITIES_API ArticulationEntity : public PhysicsEntity
{
public:
    /**
     * @brief Construct an articulation entity wrapper for one or more USD prim paths.
     * @param[in] engine Name of the physics engine that simulates the prims.
     * @param[in] paths  Single path string or list of path strings. May include regular expressions.
     * @throws std::runtime_error if the entity cannot be created for the given engine and paths.
     */
    ArticulationEntity(const std::string& engine, const std::variant<std::string, std::vector<std::string>>& paths);
    ~ArticulationEntity() = default;

    /**
     * @brief Return the number of degrees of freedom (DOFs) per wrapped articulation.
     * @return Count of DOFs.
     * @throws std::runtime_error if the DOF count cannot be queried from the physics engine.
     */
    size_t numDofs() const;

    /**
     * @brief Return the names of the degrees of freedom (DOFs), in DOF index order.
     * @return Ordered list of DOF names (size @c D).
     * @throws std::runtime_error if the DOF names cannot be queried from the physics engine.
     */
    std::vector<std::string> dofNames() const;

    /**
     * @brief Return the USD prim paths of the degrees of freedom (DOFs) of each wrapped articulation.
     * @return Ordered list of DOF path lists, one list per articulation.
     * @throws std::runtime_error DOF paths are not exposed by the tensor API.
     */
    std::vector<std::vector<std::string>> dofPaths() const;

    /**
     * @brief Return the joint types of the degrees of freedom (DOFs), in DOF index order.
     * @return DOF types (shape @c (N,D)).
     * @throws std::runtime_error DOF types are not exposed by the tensor API.
     */
    array::Array dofTypes() const;

    /**
     * @brief Return the number of joints per wrapped articulation.
     * @return Count of joints.
     * @throws std::runtime_error if the joint count cannot be queried from the physics engine.
     */
    size_t numJoints() const;

    /**
     * @brief Return the names of the joints, in joint index order.
     * @return Ordered list of joint names.
     * @throws std::runtime_error if the joint names cannot be queried from the physics engine.
     */
    std::vector<std::string> jointNames() const;

    /**
     * @brief Return the USD prim paths of the joints of each wrapped articulation.
     * @return Ordered list of joint path lists, one list per articulation.
     * @throws std::runtime_error Joint paths are not exposed by the tensor API.
     */
    std::vector<std::vector<std::string>> jointPaths() const;

    /**
     * @brief Return the joint types, in joint index order.
     * @return Joint types.
     * @throws std::runtime_error Joint types are not exposed by the tensor API.
     */
    array::Array jointTypes() const;

    /**
     * @brief Return the number of links per wrapped articulation.
     * @return Count of links.
     * @throws std::runtime_error if the link count cannot be queried from the physics engine.
     */
    size_t numLinks() const;

    /**
     * @brief Return the names of the links, in link index order.
     * @return Ordered list of link names (size @c L).
     * @throws std::runtime_error if the link names cannot be queried from the physics engine.
     */
    std::vector<std::string> linkNames() const;

    /**
     * @brief Return the USD prim paths of the links of each wrapped articulation.
     * @return Ordered list of link path lists, one list per articulation.
     * @throws std::runtime_error Link paths are not exposed by the tensor API.
     */
    std::vector<std::vector<std::string>> linkPaths() const;

    /**
     * @brief Return the number of collision shapes per wrapped articulation.
     * @return Count of collision shapes.
     * @throws std::runtime_error if the shape count cannot be queried from the physics engine.
     */
    size_t numShapes() const;

    /**
     * @brief Return the number of fixed tendons per wrapped articulation.
     * @return Count of fixed tendons.
     * @throws std::runtime_error if the tendon count cannot be queried from the physics engine.
     */
    size_t numFixedTendons() const;

    /**
     * @brief Return the shape of the Jacobian matrix of a single articulation.
     * @details The shape depends on the number of links, the number of DOFs, and whether the
     *          articulation base is fixed:
     *          - Fixed base: @c (L-1, 6, D)
     *          - Floating base: @c (L, 6, D+6)
     *
     *          Each link contributes 6 rows representing its linear and angular motion along the
     *          three coordinate axes. For floating bases, the 6 extra columns correspond to the
     *          linear and angular degrees of freedom of the free root link.
     * @return Jacobian matrix shape as (rows, 6, columns).
     */
    std::tuple<size_t, size_t, size_t> jacobianMatrixShape() const;

    /**
     * @brief Return the shape of the generalized mass matrix of a single articulation.
     * @details The shape depends on the number of DOFs and whether the articulation base is fixed:
     *          - Fixed base: @c (D, D)
     *          - Floating base: @c (D+6, D+6)
     * @return Mass matrix shape as (rows, columns).
     */
    std::tuple<size_t, size_t> massMatrixShape() const;

    /**
     * @brief Resolve link names to their link indices.
     * @param[in] names Single link name or list of link names.
     * @return Link indices, in the order the names were given.
     * @throws std::invalid_argument if any name does not match a link of the articulation.
     */
    array::Array getLinkIndices(const std::variant<std::string, std::vector<std::string>>& names);

    /**
     * @brief Resolve joint names to their joint indices.
     * @param[in] names Single joint name or list of joint names.
     * @return Joint indices, in the order the names were given.
     * @throws std::invalid_argument if any name does not match a joint of the articulation.
     */
    array::Array getJointIndices(const std::variant<std::string, std::vector<std::string>>& names);

    /**
     * @brief Resolve degree of freedom (DOF) names to their DOF indices.
     * @param[in] names Single DOF name or list of DOF names.
     * @return DOF indices, in the order the names were given.
     * @throws std::invalid_argument if any name does not match a DOF of the articulation.
     */
    array::Array getDofIndices(const std::variant<std::string, std::vector<std::string>>& names);

    /**
     * @brief Get the position limits of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return A pair of (lower limits, upper limits), both with shape @c (N,D).
     */
    std::tuple<array::Array, array::Array> getDofLimits(const std::optional<array::Array>& indices = std::nullopt,
                                                        const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the position limits of the degrees of freedom (DOFs) of the selected articulations.
     * @details At least one of @p lower or @p upper must be provided.
     * @param[in] lower      Lower limits (shape @c (N,D)). Optional.
     * @param[in] upper      Upper limits (shape @c (N,D)). Optional.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @throws std::invalid_argument if both @p lower and @p upper are undefined.
     */
    void setDofLimits(const std::optional<array::Array>& lower = std::nullopt,
                      const std::optional<array::Array>& upper = std::nullopt,
                      const std::optional<array::Array>& indices = std::nullopt,
                      const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the friction properties of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return A triple of (static friction efforts, dynamic friction efforts, viscous friction coefficients),
     *         all with shape @c (N,D).
     */
    std::tuple<array::Array, array::Array, array::Array> getDofFrictionProperties(
        const std::optional<array::Array>& indices = std::nullopt,
        const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the friction properties of the degrees of freedom (DOFs) of the selected articulations.
     * @details At least one of the friction properties must be provided.
     * @param[in] staticFrictions  Static friction efforts (shape @c (N,D)). Optional.
     * @param[in] dynamicFrictions Dynamic friction efforts (shape @c (N,D)). Optional.
     * @param[in] viscousFrictions Viscous friction coefficients (shape @c (N,D)). Optional.
     * @param[in] indices          Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                             processed.
     * @param[in] dofIndices       Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @throws std::invalid_argument if all friction properties are undefined.
     * @warning The static frictions must be greater than or equal to the dynamic frictions.
     */
    void setDofFrictionProperties(const std::optional<array::Array>& staticFrictions = std::nullopt,
                                  const std::optional<array::Array>& dynamicFrictions = std::nullopt,
                                  const std::optional<array::Array>& viscousFrictions = std::nullopt,
                                  const std::optional<array::Array>& indices = std::nullopt,
                                  const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the drive model properties of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return A triple of (speed effort gradients, maximum actuator velocities, velocity-dependent resistances),
     *         all with shape @c (N,D).
     */
    std::tuple<array::Array, array::Array, array::Array> getDofDriveModelProperties(
        const std::optional<array::Array>& indices = std::nullopt,
        const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the drive model properties of the degrees of freedom (DOFs) of the selected articulations.
     * @details At least one of the drive model properties must be provided.
     * @param[in] speedEffortGradients         Speed effort gradients (shape @c (N,D)). Optional.
     * @param[in] maximumActuatorVelocities    Maximum actuator velocities (shape @c (N,D)). Optional.
     * @param[in] velocityDependentResistances Velocity-dependent resistances (shape @c (N,D)). Optional.
     * @param[in] indices                      Indices of prims to process (shape @c (N,)). If omitted, all wrapped
     *                                         prims are processed.
     * @param[in] dofIndices                   Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are
     *                                         processed.
     * @throws std::invalid_argument if all drive model properties are undefined.
     */
    void setDofDriveModelProperties(const std::optional<array::Array>& speedEffortGradients = std::nullopt,
                                    const std::optional<array::Array>& maximumActuatorVelocities = std::nullopt,
                                    const std::optional<array::Array>& velocityDependentResistances = std::nullopt,
                                    const std::optional<array::Array>& indices = std::nullopt,
                                    const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the armatures of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Armatures (shape @c (N,D)).
     */
    array::Array getDofArmatures(const std::optional<array::Array>& indices = std::nullopt,
                                 const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the armatures of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] armatures  Armatures to apply (shape @c (N,D)).
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     */
    void setDofArmatures(const array::Array& armatures,
                         const std::optional<array::Array>& indices = std::nullopt,
                         const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the position targets of the degrees of freedom (DOFs) of the selected articulations.
     * @details The targets are tracked by the implicit PD controller of each DOF drive.
     * @param[in] positions  Position targets in distance or radians, depending on the DOF type (shape @c (N,D)).
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     */
    void setDofPositionTargets(const array::Array& positions,
                               const std::optional<array::Array>& indices = std::nullopt,
                               const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the position targets of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Position targets in distance or radians, depending on the DOF type (shape @c (N,D)).
     */
    array::Array getDofPositionTargets(const std::optional<array::Array>& indices = std::nullopt,
                                       const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the positions of the degrees of freedom (DOFs) of the selected articulations.
     * @details This method teleports the DOFs to the specified positions.
     * @param[in] positions  Positions in distance or radians, depending on the DOF type (shape @c (N,D)).
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     */
    void setDofPositions(const array::Array& positions,
                         const std::optional<array::Array>& indices = std::nullopt,
                         const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the positions of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Positions in distance or radians, depending on the DOF type (shape @c (N,D)).
     */
    array::Array getDofPositions(const std::optional<array::Array>& indices = std::nullopt,
                                 const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the velocity targets of the degrees of freedom (DOFs) of the selected articulations.
     * @details The targets are tracked by the implicit PD controller of each DOF drive.
     * @param[in] velocities Velocity targets in distance/second or radians/second, depending on the DOF type
     *                       (shape @c (N,D)).
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     */
    void setDofVelocityTargets(const array::Array& velocities,
                               const std::optional<array::Array>& indices = std::nullopt,
                               const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the velocity targets of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Velocity targets in distance/second or radians/second, depending on the DOF type (shape @c (N,D)).
     */
    array::Array getDofVelocityTargets(const std::optional<array::Array>& indices = std::nullopt,
                                       const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the velocities of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] velocities Velocities in distance/second or radians/second, depending on the DOF type
     *                       (shape @c (N,D)).
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     */
    void setDofVelocities(const array::Array& velocities,
                          const std::optional<array::Array>& indices = std::nullopt,
                          const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the velocities of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Velocities in distance/second or radians/second, depending on the DOF type (shape @c (N,D)).
     */
    array::Array getDofVelocities(const std::optional<array::Array>& indices = std::nullopt,
                                  const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the actuation efforts of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] efforts    Actuation forces or torques, depending on the DOF type (shape @c (N,D)).
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     */
    void setDofEfforts(const array::Array& efforts,
                       const std::optional<array::Array>& indices = std::nullopt,
                       const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the actuation efforts of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Actuation forces or torques, depending on the DOF type (shape @c (N,D)).
     */
    array::Array getDofEfforts(const std::optional<array::Array>& indices = std::nullopt,
                               const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the projected joint forces of the degrees of freedom (DOFs) of the selected articulations.
     * @details These are the incoming joint forces of the links projected onto the motion direction,
     *          that is, the active component of the force.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Projected joint forces (shape @c (N,D)).
     */
    array::Array getDofProjectedJointForces(const std::optional<array::Array>& indices = std::nullopt,
                                            const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the implicit PD controller gains of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return A pair of (stiffnesses, dampings), both with shape @c (N,D).
     */
    std::tuple<array::Array, array::Array> getDofGains(const std::optional<array::Array>& indices = std::nullopt,
                                                       const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the implicit PD controller gains of the degrees of freedom (DOFs) of the selected articulations.
     * @details At least one of @p stiffnesses or @p dampings must be provided.
     * @param[in] stiffnesses        Stiffnesses (shape @c (N,D)). Optional.
     * @param[in] dampings           Dampings (shape @c (N,D)). Optional.
     * @param[in] indices            Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                               processed.
     * @param[in] dofIndices         Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @param[in] updateDefaultGains Whether to record the resulting gains of all wrapped articulations as the new
     *                               defaults used by @c switchDofControlMode.
     * @throws std::invalid_argument if both @p stiffnesses and @p dampings are undefined.
     * @see switchDofControlMode
     */
    void setDofGains(const std::optional<array::Array>& stiffnesses = std::nullopt,
                     const std::optional<array::Array>& dampings = std::nullopt,
                     const std::optional<array::Array>& indices = std::nullopt,
                     const std::optional<array::Array>& dofIndices = std::nullopt,
                     bool updateDefaultGains = true);

    /**
     * @brief Switch the control mode of the degrees of freedom (DOFs) of the selected articulations.
     * @details The mode is expressed as an adjustment of the implicit PD controller gains:
     *          - @c "position": default stiffnesses and default dampings.
     *          - @c "velocity": zero stiffnesses and default dampings.
     *          - @c "effort": zero stiffnesses and zero dampings.
     *
     *          The default gains are those recorded by the most recent @c setDofGains call with
     *          @c updateDefaultGains enabled. If none has been made, the gains in effect on the first
     *          call to this method are adopted as the defaults, so that switching away from and back to
     *          @c "position" restores the original gains.
     * @param[in] mode       Control mode. Supported modes are @c "position", @c "velocity" and @c "effort".
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @throws std::invalid_argument if the control mode is not supported.
     * @see setDofGains
     */
    void switchDofControlMode(const std::string& mode,
                              const std::optional<array::Array>& indices = std::nullopt,
                              const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the maximum efforts of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Maximum forces or torques, depending on the DOF type (shape @c (N,D)).
     */
    array::Array getDofMaxEfforts(const std::optional<array::Array>& indices = std::nullopt,
                                  const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the maximum efforts of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] maxEfforts Maximum forces or torques, depending on the DOF type (shape @c (N,D)).
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     */
    void setDofMaxEfforts(const array::Array& maxEfforts,
                          const std::optional<array::Array>& indices = std::nullopt,
                          const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the maximum velocities of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Maximum velocities in distance/second or radians/second, depending on the DOF type (shape @c (N,D)).
     */
    array::Array getDofMaxVelocities(const std::optional<array::Array>& indices = std::nullopt,
                                     const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the maximum velocities of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] maxVelocities Maximum velocities in distance/second or radians/second, depending on the DOF type
     *                          (shape @c (N,D)).
     * @param[in] indices       Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                          processed.
     * @param[in] dofIndices    Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     */
    void setDofMaxVelocities(const array::Array& maxVelocities,
                             const std::optional<array::Array>& indices = std::nullopt,
                             const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the drive types of the degrees of freedom (DOFs) of the selected articulations.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Drive type names (shape @c (N,D)). Possible values are @c "force", @c "acceleration",
     *         and @c "none" when the drive type is not set.
     */
    std::vector<std::vector<std::string>> getDofDriveTypes(const std::optional<array::Array>& indices = std::nullopt,
                                                           const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the world-frame poses of the root links of the selected articulations.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @return A pair of (positions, orientations). Positions have shape @c (N,3);
     *         orientations are quaternions @c xyzw with shape @c (N,4).
     */
    std::tuple<array::Array, array::Array> getWorldPoses(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the world-frame poses of the root links of the selected articulations.
     * @details At least one of @p positions or @p orientations must be provided.
     *          This method teleports the articulations to the specified poses.
     * @param[in] positions    World-frame positions (shape @c (N,3)). Optional.
     * @param[in] orientations Orientations as quaternions @c xyzw (shape @c (N,4)). Optional.
     * @param[in] indices      Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if both @p positions and @p orientations are undefined.
     */
    void setWorldPoses(const std::optional<array::Array>& positions = std::nullopt,
                       const std::optional<array::Array>& orientations = std::nullopt,
                       const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the world-frame velocities of the root links of the selected articulations.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @return A pair of (linear velocities, angular velocities), both with shape @c (N,3),
     *         in distance/second and radians/second respectively.
     */
    std::tuple<array::Array, array::Array> getVelocities(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the world-frame velocities of the root links of the selected articulations.
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
     * @brief Get the incoming joint forces and torques of the links of the selected articulations.
     * @details In a kinematic tree each link has a single incoming joint. This method reports the
     *          total 6D force/torque transmitted through that joint under external loads.
     * @param[in] indices     Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process (shape @c (L,)). If omitted, all links are processed.
     * @return A pair of (forces, torques), both with shape @c (N,L,3).
     */
    std::tuple<array::Array, array::Array> getLinkIncomingJointForce(
        const std::optional<array::Array>& indices = std::nullopt,
        const std::optional<array::Array>& linkIndices = std::nullopt);

    /**
     * @brief Get the Jacobian matrices of the selected articulations.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @return Jacobian matrices, with the leading dimension @c N followed by the per-articulation
     *         Jacobian matrix shape.
     * @see jacobianMatrixShape
     */
    array::Array getJacobianMatrices(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the generalized mass matrices of the selected articulations.
     * @param[in] indices Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @return Mass matrices, with the leading dimension @c N followed by the per-articulation
     *         mass matrix shape.
     * @see massMatrixShape
     */
    array::Array getMassMatrices(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the Coriolis and centrifugal compensation forces of the selected articulations.
     * @details These are the DOF forces required to counteract the Coriolis and centrifugal forces
     *          for the current articulation state.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Compensation forces. Shape is @c (N,D) for a fixed articulation base, and @c (N,D+6) for a
     *         floating base, where the extra components are the forces acting on the root link.
     */
    array::Array getDofCoriolisAndCentrifugalCompensationForces(
        const std::optional<array::Array>& indices = std::nullopt,
        const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the gravity compensation forces of the selected articulations.
     * @details These are the DOF forces required to counteract gravity for the current articulation pose.
     * @param[in] indices    Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process (shape @c (D,)). If omitted, all DOFs are processed.
     * @return Compensation forces. Shape is @c (N,D) for a fixed articulation base, and @c (N,D+6) for a
     *         floating base, where the extra components are the forces acting on the root link.
     */
    array::Array getDofGravityCompensationForces(const std::optional<array::Array>& indices = std::nullopt,
                                                 const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the masses of the links of the selected articulations.
     * @param[in] indices     Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process (shape @c (L,)). If omitted, all links are processed.
     * @param[in] inverse     Whether to return the inverse masses (@c true) or the masses (@c false).
     * @return Masses or inverse masses (shape @c (N,L)).
     */
    array::Array getLinkMasses(const std::optional<array::Array>& indices = std::nullopt,
                               const std::optional<array::Array>& linkIndices = std::nullopt,
                               bool inverse = false);

    /**
     * @brief Set the masses of the links of the selected articulations.
     * @param[in] masses      Masses to apply (shape @c (N,L)).
     * @param[in] indices     Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process (shape @c (L,)). If omitted, all links are processed.
     */
    void setLinkMasses(const array::Array& masses,
                       const std::optional<array::Array>& indices = std::nullopt,
                       const std::optional<array::Array>& linkIndices = std::nullopt);

    /**
     * @brief Get the centers of mass of the links of the selected articulations, expressed in their local frames.
     * @param[in] indices     Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process (shape @c (L,)). If omitted, all links are processed.
     * @return A pair of (positions, orientations). Positions have shape @c (N,L,3);
     *         orientations are quaternions @c xyzw with shape @c (N,L,4).
     */
    std::tuple<array::Array, array::Array> getLinkComs(const std::optional<array::Array>& indices = std::nullopt,
                                                       const std::optional<array::Array>& linkIndices = std::nullopt);

    /**
     * @brief Set the centers of mass of the links of the selected articulations, expressed in their local frames.
     * @details At least one of @p positions or @p orientations must be provided.
     * @param[in] positions    Center of mass positions (shape @c (N,L,3)). Optional.
     * @param[in] orientations Center of mass orientations as quaternions @c xyzw (shape @c (N,L,4)). Optional.
     * @param[in] indices      Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] linkIndices  Indices of links to process (shape @c (L,)). If omitted, all links are processed.
     * @throws std::invalid_argument if both @p positions and @p orientations are undefined.
     */
    void setLinkComs(const std::optional<array::Array>& positions = std::nullopt,
                     const std::optional<array::Array>& orientations = std::nullopt,
                     const std::optional<array::Array>& indices = std::nullopt,
                     const std::optional<array::Array>& linkIndices = std::nullopt);

    /**
     * @brief Get the inertia tensors of the links of the selected articulations.
     * @param[in] indices     Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process (shape @c (L,)). If omitted, all links are processed.
     * @param[in] inverse     Whether to return the inverse inertia tensors (@c true) or the inertia tensors
     *                        (@c false).
     * @return Flattened row-major 3x3 inertia tensors (shape @c (N,L,9)).
     */
    array::Array getLinkInertias(const std::optional<array::Array>& indices = std::nullopt,
                                 const std::optional<array::Array>& linkIndices = std::nullopt,
                                 bool inverse = false);

    /**
     * @brief Set the inertia tensors of the links of the selected articulations.
     * @param[in] inertias    Flattened row-major 3x3 inertia tensors (shape @c (N,L,9)).
     * @param[in] indices     Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process (shape @c (L,)). If omitted, all links are processed.
     */
    void setLinkInertias(const array::Array& inertias,
                         const std::optional<array::Array>& indices = std::nullopt,
                         const std::optional<array::Array>& linkIndices = std::nullopt);

    /**
     * @brief Get the gravity-enabled state of the links of the selected articulations.
     * @param[in] indices     Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process (shape @c (L,)). If omitted, all links are processed.
     * @return Boolean flags (shape @c (N,L)). @c true if gravity affects the link, @c false otherwise.
     */
    array::Array getLinkEnabledGravities(const std::optional<array::Array>& indices = std::nullopt,
                                         const std::optional<array::Array>& linkIndices = std::nullopt);

    /**
     * @brief Enable or disable gravity for the links of the selected articulations.
     * @param[in] enabled     Boolean flags (shape @c (N,L)). @c true applies gravity, @c false excludes it.
     * @param[in] indices     Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process (shape @c (L,)). If omitted, all links are processed.
     */
    void setLinkEnabledGravities(const array::Array& enabled,
                                 const std::optional<array::Array>& indices = std::nullopt,
                                 const std::optional<array::Array>& linkIndices = std::nullopt);

    /**
     * @brief Get the stiffnesses of the fixed tendons of the selected articulations.
     * @param[in] indices       Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                          processed.
     * @param[in] tendonIndices Indices of tendons to process (shape @c (T,)). If omitted, all tendons are processed.
     * @return Stiffnesses (shape @c (N,T)).
     */
    array::Array getFixedTendonStiffnesses(const std::optional<array::Array>& indices = std::nullopt,
                                           const std::optional<array::Array>& tendonIndices = std::nullopt);

    /**
     * @brief Get the dampings of the fixed tendons of the selected articulations.
     * @param[in] indices       Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                          processed.
     * @param[in] tendonIndices Indices of tendons to process (shape @c (T,)). If omitted, all tendons are processed.
     * @return Dampings (shape @c (N,T)).
     */
    array::Array getFixedTendonDampings(const std::optional<array::Array>& indices = std::nullopt,
                                        const std::optional<array::Array>& tendonIndices = std::nullopt);

    /**
     * @brief Get the limit stiffnesses of the fixed tendons of the selected articulations.
     * @param[in] indices       Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                          processed.
     * @param[in] tendonIndices Indices of tendons to process (shape @c (T,)). If omitted, all tendons are processed.
     * @return Limit stiffnesses (shape @c (N,T)).
     */
    array::Array getFixedTendonLimitStiffnesses(const std::optional<array::Array>& indices = std::nullopt,
                                                const std::optional<array::Array>& tendonIndices = std::nullopt);

    /**
     * @brief Get the length limits of the fixed tendons of the selected articulations.
     * @param[in] indices       Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                          processed.
     * @param[in] tendonIndices Indices of tendons to process (shape @c (T,)). If omitted, all tendons are processed.
     * @return A pair of (lower limits, upper limits), both with shape @c (N,T).
     */
    std::tuple<array::Array, array::Array> getFixedTendonLimits(
        const std::optional<array::Array>& indices = std::nullopt,
        const std::optional<array::Array>& tendonIndices = std::nullopt);

    /**
     * @brief Get the rest lengths of the fixed tendons of the selected articulations.
     * @param[in] indices       Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                          processed.
     * @param[in] tendonIndices Indices of tendons to process (shape @c (T,)). If omitted, all tendons are processed.
     * @return Rest lengths (shape @c (N,T)).
     */
    array::Array getFixedTendonRestLengths(const std::optional<array::Array>& indices = std::nullopt,
                                           const std::optional<array::Array>& tendonIndices = std::nullopt);

    /**
     * @brief Get the length offsets of the fixed tendons of the selected articulations.
     * @param[in] indices       Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                          processed.
     * @param[in] tendonIndices Indices of tendons to process (shape @c (T,)). If omitted, all tendons are processed.
     * @return Offsets (shape @c (N,T)).
     */
    array::Array getFixedTendonOffsets(const std::optional<array::Array>& indices = std::nullopt,
                                       const std::optional<array::Array>& tendonIndices = std::nullopt);

    /**
     * @brief Set the properties of the fixed tendons of the selected articulations.
     * @details At least one of the tendon properties must be provided.
     * @param[in] stiffnesses      Stiffnesses (shape @c (N,T)). Optional.
     * @param[in] dampings         Dampings (shape @c (N,T)). Optional.
     * @param[in] limitStiffnesses Limit stiffnesses (shape @c (N,T)). Optional.
     * @param[in] lowerLimits      Lower length limits (shape @c (N,T)). Optional.
     * @param[in] upperLimits      Upper length limits (shape @c (N,T)). Optional.
     * @param[in] restLengths      Rest lengths (shape @c (N,T)). Optional.
     * @param[in] offsets          Length offsets (shape @c (N,T)). Optional.
     * @param[in] indices          Indices of prims to process (shape @c (N,)). If omitted, all wrapped prims are
     *                             processed.
     * @param[in] tendonIndices    Indices of tendons to process (shape @c (T,)). If omitted, all tendons are
     *                             processed.
     * @throws std::invalid_argument if all tendon properties are undefined.
     */
    void setFixedTendonProperties(const std::optional<array::Array>& stiffnesses = std::nullopt,
                                  const std::optional<array::Array>& dampings = std::nullopt,
                                  const std::optional<array::Array>& limitStiffnesses = std::nullopt,
                                  const std::optional<array::Array>& lowerLimits = std::nullopt,
                                  const std::optional<array::Array>& upperLimits = std::nullopt,
                                  const std::optional<array::Array>& restLengths = std::nullopt,
                                  const std::optional<array::Array>& offsets = std::nullopt,
                                  const std::optional<array::Array>& indices = std::nullopt,
                                  const std::optional<array::Array>& tendonIndices = std::nullopt);

private:
    /**
     * @brief Default stiffnesses of all DOFs of all wrapped articulations (shape @c (N,D)).
     * @details Empty until the gains are first recorded, either by @c setDofGains with
     *          @c updateDefaultGains enabled or by the first call to @c switchDofControlMode.
     */
    std::optional<array::Array> m_defaultDofStiffnesses;

    /**
     * @brief Default dampings of all DOFs of all wrapped articulations (shape @c (N,D)).
     * @details Empty until the gains are first recorded, either by @c setDofGains with
     *          @c updateDefaultGains enabled or by the first call to @c switchDofControlMode.
     */
    std::optional<array::Array> m_defaultDofDampings;
};

} // namespace entities
} // namespace physics
} // namespace isaacsim

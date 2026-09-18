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

#include <isaacsim/foundation/objects/Xform.hpp>
#include <isaacsim/foundation/prims/Export.h>

namespace isaacsim
{
namespace foundation
{
namespace prims
{
namespace physics
{

namespace array = isaacsim::common::array;

/**
 * @class Articulation
 * @brief High-level wrapper over one or more USD prims with the Articulation Root API applied.
 * @details
 * Extends @c Xform with a unified interface for querying and modifying articulation structure
 * (links, joints, and degrees of freedom) and their physical properties.
 *
 * Paths may contain regular expressions; the constructor resolves them against the active stage
 * and locates the nearest ancestor or descendant prim that carries the Articulation Root API.
 * All methods that accept an @p indices parameter operate only on the prims at those positions;
 * when @p indices is omitted, all wrapped prims are processed. Methods that additionally accept
 * a @p dofIndices or @p linkIndices parameter follow the same convention for the corresponding
 * sub-entity dimension.
 *
 * @note Articulation metadata (DOF/joint/link names, counts, paths, and types) is parsed once
 *       from the USD stage at construction time and cached for the lifetime of the object.
 */
class ISAACSIM_FOUNDATION_PRIMS_API Articulation : public isaacsim::foundation::objects::Xform
{
public:
    /**
     * @brief Construct an Articulation wrapper for one or more USD prim paths.
     * @details
     * Resolves each path against the active stage to find the prim carrying the
     * Articulation Root API, then parses the articulation metadata (DOF names, joint
     * names, link names, and their respective counts and types).
     *
     * @param[in] paths               Single path string or list of path strings. May include regular
     *                                expressions that are expanded against the active stage.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     *
     * @throws std::runtime_error if no active or default stage has been set.
     */
    Articulation(const std::variant<std::string, std::vector<std::string>>& paths,
                 // Xform
                 bool resetXformOpProperties = true);
    ~Articulation() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details A prim matches when it is @c Xformable and it or any of its descendants has the
     *          @c PhysicsArticulationRootAPI schema applied, mirroring how the constructor locates the
     *          articulation root. Testing the prim itself is not enough, since paths pointing at a
     *          wrapper above the root are accepted as well.
     *          The paths are resolved against the active stage before being checked.
     *          Since this method is static, the returned array is always allocated on the CPU.
     * @param[in] paths Single path string or list of path strings. May include regular
     *                  expressions that are expanded against the active stage.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per resolved prim.
     * @throws std::runtime_error if the given paths do not correspond to existing prims.
     */
    static array::Array areOfType(const std::variant<std::string, std::vector<std::string>>& paths);

    // -- Articulation metadata --

    /**
     * @brief Return the USD paths of the articulation root prims for each wrapped articulation.
     * @return List of absolute root prim paths, one per wrapped articulation.
     */
    std::vector<std::string> rootPaths() const;

    /**
     * @brief Return the total number of degrees of freedom (DOFs) in the articulations.
     * @return DOF count shared by all wrapped articulations.
     */
    int numDofs() const;

    /**
     * @brief Return the names of all degrees of freedom (DOFs) in the articulations.
     * @return List of DOF name strings.
     */
    std::vector<std::string> dofNames() const;

    /**
     * @brief Return the USD paths of all degrees of freedom (DOFs) for each wrapped articulation.
     * @return List of DOF path lists, one inner list per wrapped articulation.
     */
    std::vector<std::vector<std::string>> dofPaths() const;

    /**
     * @brief Return the type identifiers of all degrees of freedom (DOFs) in the articulations.
     * @return List of DOF type strings (e.g., @c "rotation", @c "translation").
     */
    std::vector<std::string> dofTypes() const;

    /**
     * @brief Return the total number of joints in the articulations.
     * @return Joint count shared by all wrapped articulations.
     */
    int numJoints() const;

    /**
     * @brief Return the names of all joints in the articulations.
     * @return List of joint name strings.
     */
    std::vector<std::string> jointNames() const;

    /**
     * @brief Return the USD paths of all joints for each wrapped articulation.
     * @return List of joint path lists, one inner list per wrapped articulation.
     */
    std::vector<std::vector<std::string>> jointPaths() const;

    /**
     * @brief Return the type identifiers of all joints in the articulations.
     * @return List of joint type strings (e.g., @c "revolute", @c "prismatic").
     */
    std::vector<std::string> jointTypes() const;

    /**
     * @brief Return the total number of links in the articulations.
     * @return Link count shared by all wrapped articulations.
     */
    int numLinks() const;

    /**
     * @brief Return the names of all links in the articulations.
     * @return List of link name strings.
     */
    std::vector<std::string> linkNames() const;

    /**
     * @brief Return the USD paths of all links for each wrapped articulation.
     * @return List of link path lists, one inner list per wrapped articulation.
     */
    std::vector<std::vector<std::string>> linkPaths() const;

    /**
     * @brief Return the zero-based indices of one or more DOFs by name.
     * @param[in] names Single DOF name or list of DOF names to look up.
     * @return Array of indices corresponding to the specified DOF names, in input order.
     */
    array::Array getDofIndices(const std::variant<std::string, std::vector<std::string>>& names) const;

    /**
     * @brief Return the zero-based indices of one or more joints by name.
     * @param[in] names Single joint name or list of joint names to look up.
     * @return Array of indices corresponding to the specified joint names, in input order.
     */
    array::Array getJointIndices(const std::variant<std::string, std::vector<std::string>>& names) const;

    /**
     * @brief Return the zero-based indices of one or more links by name.
     * @param[in] names Single link name or list of link names to look up.
     * @return Array of indices corresponding to the specified link names, in input order.
     */
    array::Array getLinkIndices(const std::variant<std::string, std::vector<std::string>>& names) const;

    // -- DOF-related methods --

    /**
     * @brief Get the lower and upper position limits of the selected DOFs.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return Two-element tuple: 1) lower limits, shape @c (N, D); 2) upper limits, shape @c (N, D).
     *         Angular DOF limits are in radians; prismatic DOF limits are in stage length units.
     */
    std::tuple<array::Array, array::Array> getDofLimits(const std::optional<array::Array>& indices = std::nullopt,
                                                        const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the lower and/or upper position limits of the selected DOFs.
     * @details
     * Angular DOF limits must be provided in radians; prismatic DOF limits in stage length units.
     * At least one of @p lower or @p upper must be specified.
     *
     * @param[in] lower      Lower position limits, shape @c (N, D). If omitted, lower limits are unchanged.
     * @param[in] upper      Upper position limits, shape @c (N, D). If omitted, upper limits are unchanged.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     */
    void setDofLimits(const std::optional<array::Array>& lower = std::nullopt,
                      const std::optional<array::Array>& upper = std::nullopt,
                      const std::optional<array::Array>& indices = std::nullopt,
                      const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the friction properties of the selected DOFs.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return Three-element tuple: 1) static friction efforts, shape @c (N, D);
     *         2) dynamic friction efforts, shape @c (N, D);
     *         3) viscous friction coefficients, shape @c (N, D).
     */
    std::tuple<array::Array, array::Array, array::Array> getDofFrictionProperties(
        const std::optional<array::Array>& indices = std::nullopt,
        const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the friction properties of the selected DOFs.
     * @details
     * At least one of the friction parameters must be specified. Unspecified values are left unchanged.
     *
     * @param[in] staticFrictions    Static friction efforts, shape @c (N, D). If omitted, unchanged.
     * @param[in] dynamicFrictions   Dynamic friction efforts, shape @c (N, D). If omitted, unchanged.
     * @param[in] viscousFrictions   Viscous friction coefficients, shape @c (N, D). If omitted, unchanged.
     * @param[in] indices            Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices         Indices of DOFs to process. If omitted, all DOFs are processed.
     *
     * @warning Static friction efforts must be greater than or equal to the corresponding dynamic
     *          friction efforts.
     */
    void setDofFrictionProperties(const std::optional<array::Array>& staticFrictions = std::nullopt,
                                  const std::optional<array::Array>& dynamicFrictions = std::nullopt,
                                  const std::optional<array::Array>& viscousFrictions = std::nullopt,
                                  const std::optional<array::Array>& indices = std::nullopt,
                                  const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the drive model properties of the selected DOFs.
     * @details
     * These properties define the actuator performance envelope used by PhysX.
     *
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return Three-element tuple: 1) speed-effort gradients, shape @c (N, D);
     *         2) maximum actuator velocities, shape @c (N, D);
     *         3) velocity-dependent resistances, shape @c (N, D).
     */
    std::tuple<array::Array, array::Array, array::Array> getDofDriveModelProperties(
        const std::optional<array::Array>& indices = std::nullopt,
        const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the drive model properties of the selected DOFs.
     * @details
     * These properties define the actuator performance envelope used by PhysX.
     * At least one parameter must be specified; unspecified values are left unchanged.
     *
     * @param[in] speedEffortGradients        Speed-effort gradients, shape @c (N, D). If omitted, unchanged.
     * @param[in] maximumActuatorVelocities   Maximum actuator velocities, shape @c (N, D). If omitted, unchanged.
     * @param[in] velocityDependentResistances Velocity-dependent resistances, shape @c (N, D). If omitted, unchanged.
     * @param[in] indices                     Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices                  Indices of DOFs to process. If omitted, all DOFs are processed.
     */
    void setDofDriveModelProperties(const std::optional<array::Array>& speedEffortGradients = std::nullopt,
                                    const std::optional<array::Array>& maximumActuatorVelocities = std::nullopt,
                                    const std::optional<array::Array>& velocityDependentResistances = std::nullopt,
                                    const std::optional<array::Array>& indices = std::nullopt,
                                    const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the armatures of the selected DOFs.
     * @details
     * Armatures are additional inertia-like terms added to each DOF to improve solver stability.
     *
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return Armatures, shape @c (N, D).
     */
    array::Array getDofArmatures(const std::optional<array::Array>& indices = std::nullopt,
                                 const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the armatures of the selected DOFs.
     * @details
     * Armatures are additional inertia-like terms added to each DOF to improve solver stability.
     *
     * @param[in] armatures  Armatures, shape @c (N, D).
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     */
    void setDofArmatures(const array::Array& armatures,
                         const std::optional<array::Array>& indices = std::nullopt,
                         const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the drive types of the selected DOFs.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return List of drive type string lists, one inner list per wrapped prim, one entry per DOF.
     */
    std::vector<std::vector<std::string>> getDofDriveTypes(const std::optional<array::Array>& indices = std::nullopt,
                                                           const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the drive types of the selected DOFs.
     * @param[in] types      A single drive type string applied to all DOFs, or a per-prim per-DOF list.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     */
    void setDofDriveTypes(const std::variant<std::string, std::vector<std::vector<std::string>>>& types,
                          const std::optional<array::Array>& indices = std::nullopt,
                          const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the maximum velocity limits of the selected DOFs.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return Maximum velocities, shape @c (N, D). Angular DOFs in rad/s, prismatic DOFs in m/s.
     */
    array::Array getDofMaxVelocities(const std::optional<array::Array>& indices = std::nullopt,
                                     const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the maximum velocity limits of the selected DOFs.
     * @param[in] maxVelocities Maximum velocities, shape @c (N, D). Angular DOFs in rad/s, prismatic DOFs in m/s.
     * @param[in] indices       Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices    Indices of DOFs to process. If omitted, all DOFs are processed.
     */
    void setDofMaxVelocities(const array::Array& maxVelocities,
                             const std::optional<array::Array>& indices = std::nullopt,
                             const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the maximum effort limits of the selected DOFs.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return Maximum efforts, shape @c (N, D). Angular DOFs in N·m, prismatic DOFs in N.
     */
    array::Array getDofMaxEfforts(const std::optional<array::Array>& indices = std::nullopt,
                                  const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the maximum effort limits of the selected DOFs.
     * @param[in] maxEfforts Maximum efforts, shape @c (N, D). Angular DOFs in N·m, prismatic DOFs in N.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     */
    void setDofMaxEfforts(const array::Array& maxEfforts,
                          const std::optional<array::Array>& indices = std::nullopt,
                          const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the drive gains (stiffness and damping) of the selected DOFs.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return Two-element tuple: 1) stiffnesses, shape @c (N, D); 2) dampings, shape @c (N, D).
     */
    std::tuple<array::Array, array::Array> getDofGains(const std::optional<array::Array>& indices = std::nullopt,
                                                       const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the drive gains (stiffness and/or damping) of the selected DOFs.
     * @details
     * At least one of @p stiffnesses or @p dampings must be specified.
     *
     * @param[in] stiffnesses        Stiffness values, shape @c (N, D). If omitted, stiffnesses are unchanged.
     * @param[in] dampings           Damping values, shape @c (N, D). If omitted, dampings are unchanged.
     * @param[in] indices            Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices         Indices of DOFs to process. If omitted, all DOFs are processed.
     * @param[in] updateDefaultGains Whether to update the internally cached default gain values.
     */
    void setDofGains(const std::optional<array::Array>& stiffnesses = std::nullopt,
                     const std::optional<array::Array>& dampings = std::nullopt,
                     const std::optional<array::Array>& indices = std::nullopt,
                     const std::optional<array::Array>& dofIndices = std::nullopt,
                     bool updateDefaultGains = true);

    /**
     * @brief Switch the drive control mode of the selected DOFs.
     * @details
     * Adjusts stiffness and damping to preset values that correspond to the requested mode.
     * Common modes include position control and velocity control.
     *
     * @param[in] mode       Name of the control mode to activate (e.g., @c "position", @c "velocity").
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     */
    void switchDofControlMode(const std::string& mode,
                              const std::optional<array::Array>& indices = std::nullopt,
                              const std::optional<array::Array>& dofIndices = std::nullopt);

    // -- DOF targets --

    /**
     * @brief Get the current position targets of the selected DOFs.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return Position targets, shape @c (N, D). Angular DOFs in radians, prismatic DOFs in stage length units.
     */
    array::Array getDofPositionTargets(const std::optional<array::Array>& indices = std::nullopt,
                                       const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the desired position targets of the selected DOFs.
     * @details
     * Sets the drive target positions; the simulation may take several steps to reach them
     * depending on the DOFs' stiffness and damping values.
     *
     * @param[in] positions  Position targets, shape @c (N, D). Angular DOFs in radians,
     *                       prismatic DOFs in stage length units.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     */
    void setDofPositionTargets(const array::Array& positions,
                               const std::optional<array::Array>& indices = std::nullopt,
                               const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Get the current velocity targets of the selected DOFs.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     * @return Velocity targets, shape @c (N, D). Angular DOFs in rad/s, prismatic DOFs in m/s.
     */
    array::Array getDofVelocityTargets(const std::optional<array::Array>& indices = std::nullopt,
                                       const std::optional<array::Array>& dofIndices = std::nullopt);

    /**
     * @brief Set the desired velocity targets of the selected DOFs.
     * @details
     * Sets the drive target velocities; the simulation may take several steps to reach them
     * depending on the DOFs' damping values.
     *
     * @param[in] velocities Velocity targets, shape @c (N, D). Angular DOFs in rad/s,
     *                       prismatic DOFs in m/s.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] dofIndices Indices of DOFs to process. If omitted, all DOFs are processed.
     */
    void setDofVelocityTargets(const array::Array& velocities,
                               const std::optional<array::Array>& indices = std::nullopt,
                               const std::optional<array::Array>& dofIndices = std::nullopt);

    // -- Link-related methods --

    /**
     * @brief Get the masses of the selected links.
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process. If omitted, all links are processed.
     * @param[in] inverse     If @c true, returns inverse masses (1/mass) instead of masses.
     * @return Masses (or inverse masses) in kg, shape @c (N, L).
     */
    array::Array getLinkMasses(const std::optional<array::Array>& indices = std::nullopt,
                               const std::optional<array::Array>& linkIndices = std::nullopt,
                               bool inverse = false);

    /**
     * @brief Set the masses of the selected links.
     * @param[in] masses      Masses in kg, shape @c (N, L).
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process. If omitted, all links are processed.
     */
    void setLinkMasses(const array::Array& masses,
                       const std::optional<array::Array>& indices = std::nullopt,
                       const std::optional<array::Array>& linkIndices = std::nullopt);

    /**
     * @brief Get the gravity-enabled flags of the selected links.
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process. If omitted, all links are processed.
     * @return Boolean flags indicating whether gravity is enabled, shape @c (N, L).
     */
    array::Array getLinkEnabledGravities(const std::optional<array::Array>& indices = std::nullopt,
                                         const std::optional<array::Array>& linkIndices = std::nullopt);

    /**
     * @brief Enable or disable gravity on the selected links.
     * @param[in] enabled     Boolean flags, shape @c (N, L). @c true to enable gravity, @c false to disable.
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] linkIndices Indices of links to process. If omitted, all links are processed.
     */
    void setLinkEnabledGravities(const array::Array& enabled,
                                 const std::optional<array::Array>& indices = std::nullopt,
                                 const std::optional<array::Array>& linkIndices = std::nullopt);

    // -- PhysX-specific methods --

    /**
     * @brief Get the PhysX solver iteration counts for the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Two-element tuple: 1) position iteration counts, shape @c (N, 1);
     *         2) velocity iteration counts, shape @c (N, 1).
     */
    std::tuple<array::Array, array::Array> getSolverIterationCounts(
        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the PhysX solver iteration counts for the selected prims.
     * @details
     * Higher iteration counts improve simulation accuracy at the cost of performance.
     * At least one of @p positionCounts or @p velocityCounts must be specified.
     *
     * @param[in] positionCounts  Position solver iteration counts, shape @c (N, 1). If omitted, unchanged.
     * @param[in] velocityCounts  Velocity solver iteration counts, shape @c (N, 1). If omitted, unchanged.
     * @param[in] indices         Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setSolverIterationCounts(const std::optional<array::Array>& positionCounts = std::nullopt,
                                  const std::optional<array::Array>& velocityCounts = std::nullopt,
                                  const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the stabilization thresholds of the selected prims.
     * @details
     * The stabilization threshold controls the energy level below which PhysX applies
     * position-based stabilization to help reduce jitter on resting articulations.
     *
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Stabilization thresholds, shape @c (N, 1).
     */
    array::Array getStabilizationThresholds(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the stabilization thresholds of the selected prims.
     * @details
     * The stabilization threshold controls the energy level below which PhysX applies
     * position-based stabilization to help reduce jitter on resting articulations.
     *
     * @param[in] thresholds Stabilization thresholds, shape @c (N, 1).
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setStabilizationThresholds(const array::Array& thresholds,
                                    const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the self-collision enabled flags of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating whether self-collision is enabled, shape @c (N, 1).
     */
    array::Array getEnabledSelfCollisions(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable self-collision for the selected prims.
     * @param[in] enabled Boolean flags, shape @c (N, 1). @c true to enable, @c false to disable.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledSelfCollisions(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the sleep thresholds of the selected prims.
     * @details
     * An articulation is put to sleep by the solver when its kinetic energy per unit mass
     * falls below this threshold.
     *
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Sleep thresholds, shape @c (N, 1).
     */
    array::Array getSleepThresholds(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the sleep thresholds of the selected prims.
     * @details
     * An articulation is put to sleep by the solver when its kinetic energy per unit mass
     * falls below this threshold.
     *
     * @param[in] thresholds Sleep thresholds, shape @c (N, 1).
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setSleepThresholds(const array::Array& thresholds, const std::optional<array::Array>& indices = std::nullopt);

private:
    void _fetchRootPaths();
    void _parseMetadata();

    array::Array _getDofDriveTargets(const std::string& property,
                                     const std::optional<array::Array>& indices,
                                     const std::optional<array::Array>& dofIndices);
    void _setDofDriveTargets(const std::string& property,
                             const array::Array& targets,
                             const std::optional<array::Array>& indices,
                             const std::optional<array::Array>& dofIndices);

    int m_numDofs = 0;
    std::vector<std::string> m_dofNames;
    std::vector<std::vector<std::string>> m_dofPaths;
    std::vector<std::string> m_dofTypes;

    int m_numJoints = 0;
    std::vector<std::string> m_jointNames;
    std::vector<std::vector<std::string>> m_jointPaths;
    std::vector<std::string> m_jointTypes;

    int m_numLinks = 0;
    std::vector<std::string> m_linkNames;
    std::vector<std::vector<std::string>> m_linkPaths;

    std::optional<array::Array> m_defaultDofStiffnesses;
    std::optional<array::Array> m_defaultDofDampings;

    std::vector<std::string> m_rootPaths;
};

} // namespace physics
} // namespace prims
} // namespace foundation
} // namespace isaacsim

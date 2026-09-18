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
 * @class RigidBody
 * @brief High-level wrapper over one or more USD prims with the Rigid Body API applied.
 * @details
 * Extends @c Xform with a unified interface for querying and modifying rigid body physical
 * properties such as mass, density, linear and angular velocity, gravity, and sleep behavior.
 *
 * Paths may contain regular expressions; the constructor resolves them against the active stage.
 * All methods that accept an @p indices parameter operate only on the prims at those positions;
 * when @p indices is omitted, all wrapped prims are processed.
 */
class ISAACSIM_FOUNDATION_PRIMS_API RigidBody : public isaacsim::foundation::objects::Xform
{
public:
    /**
     * @brief Construct a RigidBody wrapper for one or more USD prim paths.
     * @details
     * Applies the Rigid Body API to the resolved prims and optionally initializes their
     * mass, density, and transform properties.
     *
     * @param[in] paths               Single path string or list of path strings. May include regular
     *                                expressions that are expanded against the active stage.
     * @param[in] masses              Initial masses in kg, shape @c (N, 1). If omitted, existing values
     *                                are preserved.
     * @param[in] densities           Initial densities in kg/m³, shape @c (N, 1). If omitted, existing
     *                                values are preserved.
     * @param[in] applyPhysicsApis    Whether to apply the Rigid Body and Mass physics APIs during initialization.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     *
     * @throws std::runtime_error if no active or default stage has been set.
     */
    RigidBody(const std::variant<std::string, std::vector<std::string>>& paths,
              // RigidBody
              const std::optional<array::Array>& masses = std::nullopt,
              const std::optional<array::Array>& densities = std::nullopt,
              bool applyPhysicsApis = true,
              // Xform
              bool resetXformOpProperties = true);
    ~RigidBody() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details A prim matches when it is @c Xformable and has the @c PhysicsRigidBodyAPI schema applied.
     *          The constructor applies that schema by default, so a @c false flag means the prim is not
     *          a rigid body yet, not that it cannot be wrapped.
     *          The paths are resolved against the active stage before being checked.
     *          Since this method is static, the returned array is always allocated on the CPU.
     * @param[in] paths Single path string or list of path strings. May include regular
     *                  expressions that are expanded against the active stage.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per resolved prim.
     * @throws std::runtime_error if the given paths do not correspond to existing prims.
     */
    static array::Array areOfType(const std::variant<std::string, std::vector<std::string>>& paths);

    /**
     * @brief Apply the Rigid Body and Mass physics APIs to the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void applyPhysicsApis(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Remove the Rigid Body and Mass physics APIs from the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void removePhysicsApis(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the linear and/or angular velocities of the selected prims.
     * @details
     * At least one of @p linearVelocities or @p angularVelocities must be specified.
     *
     * @param[in] linearVelocities  Linear velocities in m/s, shape @c (N, 3). If omitted, unchanged.
     * @param[in] angularVelocities Angular velocities in rad/s, shape @c (N, 3). If omitted, unchanged.
     * @param[in] indices           Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setVelocities(const std::optional<array::Array>& linearVelocities = std::nullopt,
                       const std::optional<array::Array>& angularVelocities = std::nullopt,
                       const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the linear and angular velocities of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Two-element tuple: 1) linear velocities in m/s, shape @c (N, 3);
     *         2) angular velocities in rad/s, shape @c (N, 3).
     */
    std::tuple<array::Array, array::Array> getVelocities(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the masses of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] inverse If @c true, returns inverse masses (1/mass) instead of masses.
     * @return Masses (or inverse masses) in kg, shape @c (N, 1).
     */
    array::Array getMasses(const std::optional<array::Array>& indices = std::nullopt, bool inverse = false);

    /**
     * @brief Set the masses of the selected prims.
     * @param[in] masses  Masses in kg, shape @c (N, 1).
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setMasses(const array::Array& masses, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the densities of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Densities in kg/m³, shape @c (N, 1).
     */
    array::Array getDensities(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the densities of the selected prims.
     * @param[in] densities Densities in kg/m³, shape @c (N, 1).
     * @param[in] indices   Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setDensities(const array::Array& densities, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the sleep thresholds of the selected prims.
     * @details
     * A rigid body is put to sleep by the solver when its kinetic energy per unit mass
     * falls below this threshold.
     *
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Sleep thresholds, shape @c (N, 1).
     */
    array::Array getSleepThresholds(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the sleep thresholds of the selected prims.
     * @details
     * A rigid body is put to sleep by the solver when its kinetic energy per unit mass
     * falls below this threshold.
     *
     * @param[in] thresholds Sleep thresholds, shape @c (N, 1).
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setSleepThresholds(const array::Array& thresholds, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable rigid body dynamics for the selected prims.
     * @details
     * Disabling a rigid body freezes it in place; it still participates in collision detection
     * but is not moved by the physics solver.
     *
     * @param[in] enabled Boolean flags, shape @c (N, 1). @c true to enable, @c false to disable.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledRigidBodies(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the rigid body dynamics enabled flags of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating whether rigid body dynamics are enabled, shape @c (N, 1).
     */
    array::Array getEnabledRigidBodies(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable gravity for the selected prims.
     * @param[in] enabled Boolean flags, shape @c (N, 1). @c true to enable gravity, @c false to disable.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledGravities(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the gravity-enabled flags of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating whether gravity is enabled, shape @c (N, 1).
     */
    array::Array getEnabledGravities(const std::optional<array::Array>& indices = std::nullopt);
};

} // namespace physics
} // namespace prims
} // namespace foundation
} // namespace isaacsim

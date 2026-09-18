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
 * @class ColliderBody
 * @brief High-level wrapper over one or more USD prims with the Collision API applied.
 * @details
 * Extends @c Xform with a unified interface for querying and modifying collision properties
 * such as contact offsets, rest offsets, torsional patch radii, and collision mesh approximations.
 *
 * Paths may contain regular expressions; the constructor resolves them against the active stage.
 * All methods that accept an @p indices parameter operate only on the prims at those positions;
 * when @p indices is omitted, all wrapped prims are processed.
 */
class ISAACSIM_FOUNDATION_PRIMS_API ColliderBody : public isaacsim::foundation::objects::Xform
{
public:
    /**
     * @brief Construct a ColliderBody wrapper for one or more USD prim paths.
     * @details
     * Applies the Collision API to the resolved prims and optionally sets the initial
     * collision mesh approximation and transform properties.
     *
     * @param[in] paths               Single path string or list of path strings. May include regular
     *                                expressions that are expanded against the active stage.
     * @param[in] approximations      Collision mesh approximation type(s) to set on construction
     *                                (e.g., @c "convexHull", @c "boundingCube"). A single string
     *                                is applied to all prims; a list assigns one value per prim.
     *                                If omitted, existing values are preserved.
     * @param[in] applyCollisionApis  Whether to apply the Collision API during initialization.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     *
     * @throws std::runtime_error if no active or default stage has been set.
     */
    ColliderBody(const std::variant<std::string, std::vector<std::string>>& paths,
                 // ColliderBody
                 const std::optional<std::variant<std::string, std::vector<std::string>>>& approximations = std::nullopt,
                 bool applyCollisionApis = true,
                 // Xform
                 bool resetXformOpProperties = true);
    ~ColliderBody() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details A prim matches when it is @c Xformable and has the @c PhysicsCollisionAPI schema applied.
     *          The constructor applies that schema by default, so a @c false flag means the prim is not
     *          a collider body yet, not that it cannot be wrapped.
     *          The paths are resolved against the active stage before being checked.
     *          Since this method is static, the returned array is always allocated on the CPU.
     * @param[in] paths Single path string or list of path strings. May include regular
     *                  expressions that are expanded against the active stage.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per resolved prim.
     * @throws std::runtime_error if the given paths do not correspond to existing prims.
     */
    static array::Array areOfType(const std::variant<std::string, std::vector<std::string>>& paths);

    /**
     * @brief Apply the Collision API to the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void applyCollisionApis(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Remove the Collision API from the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void removeCollisionApis(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the contact and/or rest offsets of the selected prims.
     * @details
     * Two shapes generate contacts when their distance falls below the sum of their contact offsets.
     * The rest offset determines the distance at which two shapes settle into a resting state.
     * At least one of @p contactOffsets or @p restOffsets must be specified.
     *
     * @param[in] contactOffsets Contact offsets in stage length units, shape @c (N, 1).
     *                           If omitted, existing values are preserved.
     * @param[in] restOffsets    Rest offsets in stage length units, shape @c (N, 1).
     *                           If omitted, existing values are preserved.
     * @param[in] indices        Indices of prims to process. If omitted, all wrapped prims are processed.
     *
     * @warning The contact offset must be positive and greater than the rest offset.
     */
    void setOffsets(const std::optional<array::Array>& contactOffsets = std::nullopt,
                    const std::optional<array::Array>& restOffsets = std::nullopt,
                    const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the contact and rest offsets of the selected prims.
     * @details
     * Two shapes generate contacts when their distance falls below the sum of their contact offsets.
     * The rest offset determines the distance at which two shapes settle into a resting state.
     *
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Two-element tuple: 1) contact offsets in stage length units, shape @c (N, 1);
     *         2) rest offsets in stage length units, shape @c (N, 1).
     */
    std::tuple<array::Array, array::Array> getOffsets(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the torsional patch radii of the contact patches.
     * @details
     * Torsional patch radii are used to compute torsional friction on contact patches.
     * The @p minimum flag selects between the standard patch radius and the minimum patch radius
     * attributes on the PhysX collision API.
     *
     * @param[in] radii   Patch radii in stage length units, shape @c (N, 1).
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] minimum If @c true, sets the minimum torsional patch radii instead of the standard ones.
     */
    void setTorsionalPatchRadii(const array::Array& radii,
                                const std::optional<array::Array>& indices = std::nullopt,
                                bool minimum = false);

    /**
     * @brief Get the torsional patch radii of the contact patches.
     * @details
     * Torsional patch radii are used to compute torsional friction on contact patches.
     *
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] minimum If @c true, returns the minimum torsional patch radii instead of the standard ones.
     * @return Patch radii in stage length units, shape @c (N, 1).
     */
    array::Array getTorsionalPatchRadii(const std::optional<array::Array>& indices = std::nullopt, bool minimum = false);

    /**
     * @brief Set the collision mesh approximation of the selected prims.
     * @details
     * The approximation determines the collision geometry representation used by the physics solver.
     * Common values include @c "none", @c "convexHull", @c "convexDecomposition",
     * @c "meshSimplification", @c "boundingSphere", and @c "boundingCube".
     *
     * @param[in] approximations A single approximation string applied to all selected prims, or
     *                           a list of strings assigning one value per prim.
     * @param[in] indices        Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setCollisionApproximations(const std::variant<std::string, std::vector<std::string>>& approximations,
                                    const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the collision mesh approximation of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Ordered list of approximation type strings, one per selected prim.
     */
    std::vector<std::string> getCollisionApproximations(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable collision for the selected prims.
     * @param[in] enabled Boolean flags, shape @c (N, 1). @c true to enable collision, @c false to disable.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledCollisions(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the collision-enabled flags of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating whether collision is enabled, shape @c (N, 1).
     */
    array::Array getEnabledCollisions(const std::optional<array::Array>& indices = std::nullopt);
};

} // namespace physics
} // namespace prims
} // namespace foundation
} // namespace isaacsim

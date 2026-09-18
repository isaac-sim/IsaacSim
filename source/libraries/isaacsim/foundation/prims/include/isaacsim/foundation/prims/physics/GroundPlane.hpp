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

#include <isaacsim/foundation/objects/Mesh.hpp>
#include <isaacsim/foundation/objects/Xform.hpp>
#include <isaacsim/foundation/objects/shapes/Plane.hpp>
#include <isaacsim/foundation/prims/Export.h>
#include <isaacsim/foundation/prims/physics/ColliderBody.hpp>

#include <optional>

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
 * @class GroundPlane
 * @brief High-level wrapper over one or more ground plane prims.
 * @details
 * A ground plane is a composite prim made of an @c Xform root, two required children, and an optional
 * visual material:
 *
 * @code{.txt}
 * Xform           // GroundPlane instance
 *   |-- Plane     // - for collision and physics
 *   |-- Mesh      // - for rendering, since Plane is unsupported by Hydra rendering
 *   |-- Material  // - visual material bound to the Mesh (optional)
 * @endcode
 *
 * The class creates or wraps (one of both) ground plane prims according to the following rules:
 *
 * - If the prim paths exist, a wrapper is placed over the ground plane prims. The children are matched
 *   by USD type rather than by name, so ground planes authored by other tools are wrapped as well.
 * - If the prim paths do not exist, ground plane prims are created at each path and a wrapper is placed
 *   over them.
 */
class ISAACSIM_FOUNDATION_PRIMS_API GroundPlane : public isaacsim::foundation::objects::Xform
{
public:
    /**
     * @brief Construct a GroundPlane wrapper for one or more USD prim paths.
     *
     * @param[in] paths                  Single path string or list of path strings to existing or
     *                                   non-existing (one of both) ground plane prims. May include regular
     *                                   expressions that are expanded against the active stage.
     * @param[in] sizes                  Sizes (full extent, in stage units) of the ground planes, shape @c (N, 1).
     *                                   Broadcast rules apply. Defaults to @c 100.0. Used only when creating
     *                                   prims; ignored when wrapping existing ones.
     * @param[in] colors                 Display colors as normalized RGB triples (shape @c (N, 3)) or as
     *                                   case-insensitive color token strings. Broadcast rules apply.
     *                                   Defaults to @c (0.5, 0.5, 0.5). Used only when creating prims;
     *                                   ignored when wrapping existing ones.
     * @param[in] axes                   Normal axis per ground plane (@c "X", @c "Y", or @c "Z"). A single
     *                                   string is applied to all prims; a list assigns one value per prim.
     *                                   Defaults to the stage up-axis. Used only when creating prims;
     *                                   ignored when wrapping existing ones.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     *
     * @throws std::runtime_error if no active or default stage has been set, if a wrapped prim does not have
     *         2 or 3 children, or if it is missing either its @c Plane child or its @c Mesh child.
     * @throws std::invalid_argument if an axis is neither @c "X", @c "Y" nor @c "Z".
     */
    GroundPlane(const std::variant<std::string, std::vector<std::string>>& paths,
                // GroundPlane
                const std::optional<array::Array>& sizes = std::nullopt,
                const std::optional<isaacsim::foundation::objects::ColorType>& colors = std::nullopt,
                const std::optional<std::variant<std::string, std::vector<std::string>>>& axes = std::nullopt,
                // Xform
                bool resetXformOpProperties = true);
    ~GroundPlane() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details A ground plane has no USD schema of its own, so its composite structure is tested instead:
     *          a prim matches when it is @c Xformable and has 2 or 3 children including a @c Plane child
     *          (collision) and a @c Mesh child (rendering), the optional third child being the visual
     *          material. This mirrors what the constructor accepts when wrapping existing prims.
     *          The paths are resolved against the active stage before being checked.
     *          Since this method is static, the returned array is always allocated on the CPU.
     * @param[in] paths Single path string or list of path strings. May include regular
     *                  expressions that are expanded against the active stage.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per resolved prim.
     * @throws std::runtime_error if the given paths do not correspond to existing prims.
     */
    static array::Array areOfType(const std::variant<std::string, std::vector<std::string>>& paths);

    /**
     * @brief Get the wrapper over the USD Plane prims that provide collision for the ground planes.
     * @return Reference to the Plane wrapper, whose prims are ordered as the wrapped ground planes.
     */
    isaacsim::foundation::objects::shapes::Plane& planes();

    /**
     * @brief Get the wrapper over the USD Mesh prims that provide rendering for the ground planes.
     * @return Reference to the Mesh wrapper, whose prims are ordered as the wrapped ground planes.
     */
    isaacsim::foundation::objects::Mesh& meshes();

    /**
     * @brief Set the contact and/or rest offsets of the selected ground planes.
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
     * @brief Get the contact and rest offsets of the selected ground planes.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Two-element tuple: 1) contact offsets in stage length units, shape @c (N, 1);
     *         2) rest offsets in stage length units, shape @c (N, 1).
     */
    std::tuple<array::Array, array::Array> getOffsets(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the torsional patch radii of the contact patches of the selected ground planes.
     * @param[in] radii   Patch radii in stage length units, shape @c (N, 1).
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] minimum If @c true, sets the minimum torsional patch radii instead of the standard ones.
     */
    void setTorsionalPatchRadii(const array::Array& radii,
                                const std::optional<array::Array>& indices = std::nullopt,
                                bool minimum = false);

    /**
     * @brief Get the torsional patch radii of the contact patches of the selected ground planes.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] minimum If @c true, returns the minimum torsional patch radii instead of the standard ones.
     * @return Patch radii in stage length units, shape @c (N, 1).
     */
    array::Array getTorsionalPatchRadii(const std::optional<array::Array>& indices = std::nullopt, bool minimum = false);

    /**
     * @brief Enable or disable collision for the selected ground planes.
     * @param[in] enabled Boolean flags, shape @c (N, 1). @c true to enable collision, @c false to disable.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledCollisions(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the collision-enabled flags of the selected ground planes.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating whether collision is enabled, shape @c (N, 1).
     */
    array::Array getEnabledCollisions(const std::optional<array::Array>& indices = std::nullopt);

private:
    /** @brief Wrapper over the Plane children, in the order of the wrapped ground planes. */
    std::optional<isaacsim::foundation::objects::shapes::Plane> m_planes;
    /** @brief Wrapper over the Mesh children, in the order of the wrapped ground planes. */
    std::optional<isaacsim::foundation::objects::Mesh> m_meshes;
    /** @brief Collider wrapper over the Plane children, which carry the collision APIs. */
    std::optional<ColliderBody> m_colliders;
};

} // namespace physics
} // namespace prims
} // namespace foundation
} // namespace isaacsim

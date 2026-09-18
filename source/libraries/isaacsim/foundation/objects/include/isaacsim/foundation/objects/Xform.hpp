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

#include <isaacsim/foundation/objects/Prim.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{

/**
 * @class Xform
 * @brief High-level wrapper over one or more USD Xform prims for transform and visibility operations.
 * @details
 * Extends Prim with pose (position/orientation) and scale management in both world and local
 * frames, visibility control, and a default-state mechanism for resetting prims to a previously
 * recorded pose.
 *
 * @note Transformation operations on non-root articulation links are not supported.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API Xform : public Prim
{
public:
    /**
     * @brief Construct an Xform wrapper.
     * @param[in] paths                  Single path or list of paths to USD Xform prims.
     *                                   May include regular expressions.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     * @param[in] resolvePaths           Whether to resolve and expand path expressions.
     */
    Xform(const std::variant<std::string, std::vector<std::string>>& paths,
          // Xform
          bool resetXformOpProperties = true,
          // Prim
          bool resolvePaths = true);
    ~Xform() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details As the base class of all transformable prims, this method tests for the
     *          @c UsdGeomXformable schema, which every shape, light, camera and mesh derives from.
     *          A @c true flag therefore does not imply the prim is a plain @c Xform; use the
     *          subclass-specific @c areOfType to narrow it down.
     *
     *          The paths are resolved against the active stage before being checked.
     *          Since this method is static, the returned array is always allocated on the CPU.
     * @param[in] paths Single path string or list of path strings. May include regular
     *                  expressions that are expanded against the active stage.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per resolved prim.
     * @throws std::runtime_error if the given paths do not correspond to existing prims.
     */
    static array::Array areOfType(const std::variant<std::string, std::vector<std::string>>& paths);

    /**
     * @brief Set the visibility of the selected prims.
     * @param[in] visibilities Boolean flags (shape @c (N,1)). @c true makes a prim visible,
     *                         @c false makes it invisible. Broadcast rules apply for smaller inputs.
     * @param[in] indices      Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setVisibilities(const array::Array& visibilities, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the visibility state of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating visibility (shape @c (N,1)).
     */
    array::Array getVisibilities(const std::optional<array::Array>& indices = std::nullopt);

    // TODO: applyVisualMaterials
    // TODO: getAppliedVisualMaterials

    /**
     * @brief Get the world-frame poses (positions and orientations) of the selected prims.
     * @param[in] indices        Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] rotationFormat Component order of the returned quaternions: @c "xyzw" or @c "wxyz".
     * @return A pair of (positions, orientations). Positions have shape @c (N,3);
     *         orientations are quaternions in @p rotationFormat order with shape @c (N,4).
     * @throws std::invalid_argument if @p rotationFormat is neither @c "xyzw" nor @c "wxyz".
     */
    std::tuple<array::Array, array::Array> getWorldPoses(const std::optional<array::Array>& indices = std::nullopt,
                                                         const std::string& rotationFormat = "xyzw");

    /**
     * @brief Get the local-frame poses (translations and orientations) of the selected prims.
     * @param[in] indices        Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] rotationFormat Component order of the returned quaternions: @c "xyzw" or @c "wxyz".
     * @return A pair of (translations, orientations). Translations have shape @c (N,3);
     *         orientations are quaternions in @p rotationFormat order with shape @c (N,4).
     * @throws std::invalid_argument if @p rotationFormat is neither @c "xyzw" nor @c "wxyz".
     */
    std::tuple<array::Array, array::Array> getLocalPoses(const std::optional<array::Array>& indices = std::nullopt,
                                                         const std::string& rotationFormat = "xyzw");

    /**
     * @brief Get the local scales of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Scale values (shape @c (N,3)).
     */
    array::Array getLocalScales(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the local scales of the selected prims.
     * @param[in] scales  Scale values to apply (shape @c (N,3)). Broadcast rules apply for smaller inputs.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setLocalScales(const array::Array& scales, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the local-frame poses (translations and/or orientations) of the selected prims.
     * @details At least one of @p translations or @p orientations must be provided.
     *          This method teleports prims to the specified poses.
     * @param[in] translations   Local-frame translations (shape @c (N,3)). Optional.
     * @param[in] orientations   Orientations as quaternions in @p rotationFormat order (shape @c (N,4)). Optional.
     * @param[in] indices        Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] rotationFormat Component order of @p orientations: @c "xyzw" or @c "wxyz".
     * @throws std::invalid_argument if @p rotationFormat is neither @c "xyzw" nor @c "wxyz".
     */
    void setLocalPoses(const std::optional<array::Array>& translations = std::nullopt,
                       const std::optional<array::Array>& orientations = std::nullopt,
                       const std::optional<array::Array>& indices = std::nullopt,
                       const std::string& rotationFormat = "xyzw");

    /**
     * @brief Set the world-frame poses (positions and/or orientations) of the selected prims.
     * @details At least one of @p positions or @p orientations must be provided.
     *          This method teleports prims to the specified poses.
     * @param[in] positions      World-frame positions (shape @c (N,3)). Optional.
     * @param[in] orientations   Orientations as quaternions in @p rotationFormat order (shape @c (N,4)). Optional.
     * @param[in] indices        Indices of prims to process. If omitted, all wrapped prims are processed.
     * @param[in] rotationFormat Component order of @p orientations: @c "xyzw" or @c "wxyz".
     * @throws std::invalid_argument if @p rotationFormat is neither @c "xyzw" nor @c "wxyz".
     */
    void setWorldPoses(const std::optional<array::Array>& positions = std::nullopt,
                       const std::optional<array::Array>& orientations = std::nullopt,
                       const std::optional<array::Array>& indices = std::nullopt,
                       const std::string& rotationFormat = "xyzw");

    /**
     * @brief Normalize the xformOp stack of all wrapped prims to the standard translate/orient/scale order.
     * @details Removes unsupported or redundant xformOp attributes and ensures only
     *          @c xformOp:translate, @c xformOp:orient, and @c xformOp:scale remain, in that order.
     *          World-frame poses are preserved through the normalization.
     */
    void resetXformOpProperties();

protected:
    Xform();

    /** @brief Normalizes the xformOp stack of the wrapped prims when requested. */
    void _initialize(bool resetXformOpProperties = false);

private:
    struct XformOps
    {
        void (*resetXformOpProperties)(int64_t, const std::string&);
        array::Array (*getXformLocalScales)(int64_t, const std::vector<std::string>&);
        void (*setXformLocalScales)(int64_t, const std::vector<std::string>&, const array::Array&);
        std::tuple<array::Array, array::Array> (*getXformLocalPoses)(int64_t, const std::vector<std::string>&);
        void (*setXformLocalPoses)(int64_t,
                                   const std::vector<std::string>&,
                                   const std::optional<array::Array>&,
                                   const std::optional<array::Array>&);
        std::tuple<array::Array, array::Array> (*getXformWorldPoses)(int64_t, const std::vector<std::string>&);
        void (*setXformWorldPoses)(int64_t,
                                   const std::vector<std::string>&,
                                   const std::optional<array::Array>&,
                                   const std::optional<array::Array>&);
    };

    static XformOps _populateXformOps(const std::string& backend);

    bool m_nonRootArticulationLink = false;
    XformOps m_xformOps;
};

} // namespace objects
} // namespace foundation
} // namespace isaacsim

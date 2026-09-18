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

#include <isaacsim/foundation/objects/shapes/Shape.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace shapes
{

/**
 * @class Plane
 * @brief Wrapper over one or more USD plane geometry prims.
 * @details
 * The @p axes parameter controls the normal direction of each plane: the plane surface lies in
 * the two axes perpendicular to the specified axis.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API Plane : public Shape
{
public:
    /**
     * @brief Construct a Plane wrapper and optionally configure the initial geometry.
     * @param[in] paths                  Single path or list of paths to USD plane prims.
     * @param[in] widths                 Initial widths in scene units (shape @c (N,)). Optional.
     * @param[in] lengths                Initial lengths in scene units (shape @c (N,)). Optional.
     * @param[in] axes                   Normal axis per plane (@c "X", @c "Y", or @c "Z").
     *                                   Single value or list. Optional.
     * @param[in] colors                 Initial display colors. Optional.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     */
    Plane(const std::variant<std::string, std::vector<std::string>>& paths,
          // Plane
          const std::optional<array::Array>& widths = std::nullopt,
          const std::optional<array::Array>& lengths = std::nullopt,
          const std::optional<std::variant<std::string, std::vector<std::string>>>& axes = std::nullopt,
          // Shape
          const std::optional<ColorType>& colors = std::nullopt,
          // Xform
          bool resetXformOpProperties = true);
    ~Plane() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details The paths are resolved against the active stage before being checked.
     *          Since this method is static, the returned array is always allocated on the CPU.
     * @param[in] paths Single path string or list of path strings. May include regular
     *                  expressions that are expanded against the active stage.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per resolved prim.
     * @throws std::runtime_error if the given paths do not correspond to existing prims.
     */
    static array::Array areOfType(const std::variant<std::string, std::vector<std::string>>& paths);

    /**
     * @brief Set the widths (in scene units) of the selected plane prims.
     * @param[in] widths  Width values (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setWidths(const array::Array& widths, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the widths (in scene units) of the selected plane prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Width values (shape @c (N,)).
     */
    array::Array getWidths(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the lengths (in scene units) of the selected plane prims.
     * @param[in] lengths Length values (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setLengths(const array::Array& lengths, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the lengths (in scene units) of the selected plane prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Length values (shape @c (N,)).
     */
    array::Array getLengths(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the normal axes of the selected plane prims.
     * @param[in] axes    Axis tokens (@c "X", @c "Y", or @c "Z"). Single value or list; broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setAxes(const std::variant<std::string, std::vector<std::string>>& axes,
                 const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the normal axes of the selected plane prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Axis token strings, one per selected prim.
     */
    std::vector<std::string> getAxes(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Update the USD extent attribute of all wrapped plane prims to match their current geometry.
     */
    void updateExtents();
};

} // namespace shapes
} // namespace objects
} // namespace foundation
} // namespace isaacsim

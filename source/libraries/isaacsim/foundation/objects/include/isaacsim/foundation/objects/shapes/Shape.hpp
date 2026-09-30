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

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace shapes
{

/// @brief Variant type for display colors: a single color token, a list of color tokens, or a numeric RGB array.
using ColorType = std::variant<std::string, std::vector<std::string>, array::Array>;

/**
 * @class Shape
 * @brief Base class for USD geometry shape prim wrappers.
 * @details
 * Extends Xform with display color control and the @c updateExtents interface that concrete shape subclasses
 * must implement to keep USD extents synchronized with their geometry attributes.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API Shape : public Xform
{
public:
    ~Shape() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details As the base class of all shapes, this method tests for the @c UsdGeomGprim schema.
     *          Since @c Gprim is also the base of other geometry types (e.g. @c Mesh, @c Points),
     *          a @c true flag does not imply the prim can be wrapped by a concrete Shape subclass;
     *          use the subclass-specific @c areOfType for that.
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
     * @brief Set the display colors of the selected prims.
     * @param[in] colors  Color values as RGB triples (shape @c (N,3)), a single color token string,
     *                    or a list of color token strings. Broadcast rules apply for smaller inputs.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setDisplayColors(const ColorType& colors, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the display colors of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Display color values as RGB triples (shape @c (N,3)).
     */
    array::Array getDisplayColors(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Update the USD extent attribute of the wrapped prims to match their current geometry.
     * @details Must be called after changing shape-specific geometry attributes (e.g. radii, sizes)
     *          to keep the USD stage consistent.
     */
    void updateExtents();

protected:
    /**
     * @brief Construct a Shape wrapper.
     * @param[in] paths                  Single path or list of paths to USD shape prims.
     * @param[in] shapeType              USD geometry type name used when creating new prims.
     * @param[in] colors                 Initial display colors. Optional.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     */
    Shape(const std::variant<std::string, std::vector<std::string>>& paths,
          // Shape (internal)
          const std::string& shapeType,
          // Shape
          const std::optional<ColorType>& colors = std::nullopt,
          // Xform
          bool resetXformOpProperties = true);

    /** @brief Expands and validates one or more requested shape axes. */
    std::vector<std::string> _resolveAxes(const std::variant<std::string, std::vector<std::string>>& axes);
};

} // namespace shapes
} // namespace objects
} // namespace foundation
} // namespace isaacsim

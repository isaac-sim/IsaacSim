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
 * @class Sphere
 * @brief Wrapper over one or more USD sphere geometry prims.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API Sphere : public Shape
{
public:
    /**
     * @brief Construct a Sphere wrapper and optionally configure the initial geometry.
     * @param[in] paths                  Single path or list of paths to USD sphere prims.
     * @param[in] radii                  Initial sphere radii in scene units (shape @c (N,)). Optional.
     * @param[in] colors                 Initial display colors. Optional.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     */
    Sphere(const std::variant<std::string, std::vector<std::string>>& paths,
           // Sphere
           const std::optional<array::Array>& radii = std::nullopt,
           // Shape
           const std::optional<ColorType>& colors = std::nullopt,
           // Xform
           bool resetXformOpProperties = true);
    ~Sphere() = default;

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
     * @brief Set the radii (in scene units) of the selected sphere prims.
     * @param[in] radii   Radius values (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setRadii(const array::Array& radii, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the radii (in scene units) of the selected sphere prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Radius values (shape @c (N,)).
     */
    array::Array getRadii(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Update the USD extent attribute of all wrapped sphere prims to match their current radii.
     */
    void updateExtents();
};

} // namespace shapes
} // namespace objects
} // namespace foundation
} // namespace isaacsim

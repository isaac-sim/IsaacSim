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

#include <isaacsim/foundation/objects/lights/Light.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace lights
{

/**
 * @class DistantLight
 * @brief Wrapper over one or more USD distant (directional) light prims.
 * @details
 * A distant light simulates illumination from an infinitely far source, such as the sun.
 * The @p angles parameter controls the angular diameter of the source, which affects the
 * softness of shadows.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API DistantLight : public Light
{
public:
    /**
     * @brief Construct a DistantLight wrapper and optionally configure the initial angle.
     * @param[in] paths                  Single path or list of paths to USD distant light prims.
     * @param[in] angles                 Initial angular diameters in degrees (shape @c (N,)). Optional.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     */
    DistantLight(const std::variant<std::string, std::vector<std::string>>& paths,
                 // DistantLight
                 const std::optional<array::Array>& angles = std::nullopt,
                 // Xform
                 bool resetXformOpProperties = true);
    ~DistantLight() = default;

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
     * @brief Set the angular diameters (in degrees) of the selected distant light prims.
     * @param[in] angles  Angular diameter values in degrees (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setAngles(const array::Array& angles, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the angular diameters (in degrees) of the selected distant light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Angular diameter values in degrees (shape @c (N,)).
     */
    array::Array getAngles(const std::optional<array::Array>& indices = std::nullopt);
};

} // namespace lights
} // namespace objects
} // namespace foundation
} // namespace isaacsim

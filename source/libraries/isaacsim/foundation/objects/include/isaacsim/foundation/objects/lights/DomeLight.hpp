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
 * @class DomeLight
 * @brief Wrapper over one or more USD dome (environment) light prims.
 * @details
 * A dome light illuminates the scene from all directions using a spherical environment map.
 * The @p textureFiles parameter specifies the HDR image file used as the environment map, and
 * @p textureFormats controls how the image is interpreted (e.g. latlong, mirroredBall, angular).
 */
class ISAACSIM_FOUNDATION_OBJECTS_API DomeLight : public Light
{
public:
    /**
     * @brief Construct a DomeLight wrapper and optionally configure the initial environment map.
     * @param[in] paths                  Single path or list of paths to USD dome light prims.
     * @param[in] radii                  Initial guide sphere radii in scene units (shape @c (N,)). Optional.
     * @param[in] textureFiles           Paths to HDR environment map files. Single value or list. Optional.
     * @param[in] textureFormats         Texture mapping format tokens (e.g. @c "latlong", @c "mirroredBall",
     *                                   @c "angular"). Single value or list. Optional.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     */
    DomeLight(const std::variant<std::string, std::vector<std::string>>& paths,
              // DomeLight
              const std::optional<array::Array>& radii = std::nullopt,
              const std::optional<std::variant<std::string, std::vector<std::string>>>& textureFiles = std::nullopt,
              const std::optional<std::variant<std::string, std::vector<std::string>>>& textureFormats = std::nullopt,
              // Xform
              bool resetXformOpProperties = true);
    ~DomeLight() = default;

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
     * @brief Set the guide sphere radii (in scene units) of the selected dome light prims.
     * @details The guide sphere is a visual aid rendered in the viewport; it does not affect lighting.
     * @param[in] radii   Radius values (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setGuideRadii(const array::Array& radii, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the guide sphere radii (in scene units) of the selected dome light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Radius values (shape @c (N,)).
     */
    array::Array getGuideRadii(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the HDR environment map texture file paths for the selected dome light prims.
     * @param[in] textureFiles Filesystem or Omniverse URI paths. Single value or list; broadcast rules apply.
     * @param[in] indices      Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setTextureFiles(const std::variant<std::string, std::vector<std::string>>& textureFiles,
                         const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the HDR environment map texture file paths of the selected dome light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Texture file path strings, one per selected prim.
     */
    std::vector<std::string> getTextureFiles(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the texture mapping format tokens for the selected dome light prims.
     * @param[in] textureFormats Format tokens (e.g. @c "latlong", @c "mirroredBall", @c "angular").
     *                           Single value or list; broadcast rules apply.
     * @param[in] indices        Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setTextureFormats(const std::variant<std::string, std::vector<std::string>>& textureFormats,
                           const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the texture mapping format tokens of the selected dome light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Format token strings, one per selected prim.
     */
    std::vector<std::string> getTextureFormats(const std::optional<array::Array>& indices = std::nullopt);
};

} // namespace lights
} // namespace objects
} // namespace foundation
} // namespace isaacsim

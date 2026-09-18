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

/**
 * @class Camera
 * @brief High-level wrapper over one or more USD Camera prims.
 * @details
 * Extends Xform with camera-specific attributes: focal length, focus distance, aperture,
 * aperture offset, f-stop, projection type, clipping range, shutter timing, and stereo role.
 * All set/get methods operate in batch over the wrapped prims and accept an optional @p indices
 * parameter to restrict processing to a subset.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API Camera : public Xform
{
public:
    /**
     * @brief Construct a Camera wrapper.
     * @param[in] paths                  Single path or list of paths to USD Camera prims.
     *                                   May include regular expressions.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     */
    Camera(const std::variant<std::string, std::vector<std::string>>& paths,
           // Xform
           bool resetXformOpProperties = true);
    ~Camera() = default;

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
     * @brief Set the focal lengths (in scene units) of the selected cameras.
     * @param[in] focalLengths Focal length values (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices      Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setFocalLengths(const array::Array& focalLengths, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the focal lengths (in scene units) of the selected cameras.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Focal length values (shape @c (N,)).
     */
    array::Array getFocalLengths(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the focus distances (in scene units) of the selected cameras.
     * @param[in] focusDistances Focus distance values (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices        Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setFocusDistances(const array::Array& focusDistances, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the focus distances (in scene units) of the selected cameras.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Focus distance values (shape @c (N,)).
     */
    array::Array getFocusDistances(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the stereo roles of the selected cameras.
     * @param[in] roles   Stereo role tokens (e.g. @c "mono", @c "left", @c "right").
     *                    Single value or list; broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setStereoRoles(const std::variant<std::string, std::vector<std::string>>& roles,
                        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the stereo roles of the selected cameras.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Stereo role token strings, one per selected camera.
     */
    std::vector<std::string> getStereoRoles(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the f-stop (aperture number) values of the selected cameras.
     * @param[in] fstops  F-stop values (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setFStops(const array::Array& fstops, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the f-stop (aperture number) values of the selected cameras.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return F-stop values (shape @c (N,)).
     */
    array::Array getFStops(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the horizontal and/or vertical aperture sizes (in scene units) of the selected cameras.
     * @param[in] horizontalApertures Horizontal aperture values (shape @c (N,)). Optional.
     * @param[in] verticalApertures   Vertical aperture values (shape @c (N,)). Optional.
     * @param[in] indices             Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setApertures(const std::optional<array::Array>& horizontalApertures = std::nullopt,
                      const std::optional<array::Array>& verticalApertures = std::nullopt,
                      const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the horizontal and vertical aperture sizes (in scene units) of the selected cameras.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return A pair of (horizontalApertures, verticalApertures), each of shape @c (N,).
     */
    std::tuple<array::Array, array::Array> getApertures(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the horizontal and/or vertical aperture offsets (in scene units) of the selected cameras.
     * @param[in] horizontalOffsets Horizontal aperture offset values (shape @c (N,)). Optional.
     * @param[in] verticalOffsets   Vertical aperture offset values (shape @c (N,)). Optional.
     * @param[in] indices           Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setApertureOffsets(const std::optional<array::Array>& horizontalOffsets = std::nullopt,
                            const std::optional<array::Array>& verticalOffsets = std::nullopt,
                            const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the horizontal and vertical aperture offsets (in scene units) of the selected cameras.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return A pair of (horizontalOffsets, verticalOffsets), each of shape @c (N,).
     */
    std::tuple<array::Array, array::Array> getApertureOffsets(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the projection types of the selected cameras.
     * @param[in] projections Projection type tokens (e.g. @c "perspective", @c "orthographic").
     *                        Single value or list; broadcast rules apply.
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setProjections(const std::variant<std::string, std::vector<std::string>>& projections,
                        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the projection types of the selected cameras.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Projection type token strings, one per selected camera.
     */
    std::vector<std::string> getProjections(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the near and/or far clipping plane distances (in scene units) of the selected cameras.
     * @param[in] nearDistances Near clipping plane distances (shape @c (N,)). Optional.
     * @param[in] farDistances  Far clipping plane distances (shape @c (N,)). Optional.
     * @param[in] indices       Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setClippingRanges(const std::optional<array::Array>& nearDistances = std::nullopt,
                           const std::optional<array::Array>& farDistances = std::nullopt,
                           const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the near and far clipping plane distances (in scene units) of the selected cameras.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return A pair of (nearDistances, farDistances), each of shape @c (N,).
     */
    std::tuple<array::Array, array::Array> getClippingRanges(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the shutter open and/or close times of the selected cameras.
     * @param[in] openTimes  Shutter open times relative to the current frame (shape @c (N,)). Optional.
     * @param[in] closeTimes Shutter close times relative to the current frame (shape @c (N,)). Optional.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setShutterTimes(const std::optional<array::Array>& openTimes = std::nullopt,
                         const std::optional<array::Array>& closeTimes = std::nullopt,
                         const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the shutter open and close times of the selected cameras.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return A pair of (openTimes, closeTimes), each of shape @c (N,).
     */
    std::tuple<array::Array, array::Array> getShutterTimes(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Adjust aperture so that the given pixel resolutions map to square pixels.
     * @details Modifies the horizontal or vertical aperture (depending on @p modes) to achieve
     *          square pixels for the specified output resolutions.
     * @param[in] resolutions Target pixel resolutions as @c (height, width) pairs (shape @c (N,2)).
     * @param[in] modes       Fitting mode per camera: @c "horizontal" adjusts vertical aperture,
     *                        @c "vertical" adjusts horizontal aperture. Single value or list.
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void enforceSquarePixels(const array::Array& resolutions,
                             const std::variant<std::string, std::vector<std::string>>& modes = std::string("horizontal"),
                             const std::optional<array::Array>& indices = std::nullopt);
};

} // namespace objects
} // namespace foundation
} // namespace isaacsim

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
namespace lights
{

/**
 * @class Light
 * @brief Base class for USD light prim wrappers.
 * @details
 * Extends Xform with common UsdLux light attributes: intensity, exposure, diffuse/specular
 * multipliers, power normalization, color, and color-temperature control.
 * All set/get methods operate in batch over the wrapped prims and accept an optional @p indices
 * parameter to restrict processing to a subset.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API Light : public Xform
{
public:
    ~Light() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details As the base class of all lights, this method tests for the applied @c UsdLuxLightAPI
     *          schema rather than for a typed schema. A @c true flag does not imply the prim can be
     *          wrapped by a concrete Light subclass; use the subclass-specific @c areOfType for that.
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
     * @brief Set the intensities (linear power scale) of the selected light prims.
     * @param[in] intensities Intensity values (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setIntensities(const array::Array& intensities, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the intensities (linear power scale) of the selected light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Intensity values (shape @c (N,)).
     */
    array::Array getIntensities(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the exposures (power-of-2 exponential power scale) of the selected light prims.
     * @param[in] exposures Exposure values in stops (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices   Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setExposures(const array::Array& exposures, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the exposures (power-of-2 exponential power scale) of the selected light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Exposure values in stops (shape @c (N,)).
     */
    array::Array getExposures(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the diffuse and/or specular response multipliers of the selected light prims.
     * @param[in] diffuseMultipliers  Diffuse multiplier values (shape @c (N,)). Optional.
     * @param[in] specularMultipliers Specular multiplier values (shape @c (N,)). Optional.
     * @param[in] indices             Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setMultipliers(const std::optional<array::Array>& diffuseMultipliers = std::nullopt,
                        const std::optional<array::Array>& specularMultipliers = std::nullopt,
                        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the diffuse and specular response multipliers of the selected light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return A pair of (diffuseMultipliers, specularMultipliers), each of shape @c (N,).
     */
    std::tuple<array::Array, array::Array> getMultipliers(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable power normalization by surface area for the selected light prims.
     * @details When enabled, total emitted power is held constant regardless of the light's size.
     * @param[in] enabled Boolean flags (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledNormalizations(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the enabled state of power normalization for the selected light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags (shape @c (N,)).
     */
    array::Array getEnabledNormalizations(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable color-temperature mode for the selected light prims.
     * @details When enabled, the light color is derived from the color temperature rather than
     *          the color attribute.
     * @param[in] enabled Boolean flags (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledColorTemperatures(const array::Array& enabled,
                                     const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the enabled state of color-temperature mode for the selected light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags (shape @c (N,)).
     */
    array::Array getEnabledColorTemperatures(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the color temperatures (in degrees Kelvin) of the selected light prims.
     * @note Takes effect only when color-temperature mode is enabled via @c setEnabledColorTemperatures.
     * @param[in] colorTemperatures Temperature values in Kelvin (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices           Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setColorTemperatures(const array::Array& colorTemperatures,
                              const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the color temperatures (in degrees Kelvin) of the selected light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Temperature values in Kelvin (shape @c (N,)).
     */
    array::Array getColorTemperatures(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the emitted light colors of the selected light prims.
     * @note When color-temperature mode is enabled, this value is combined with the
     *       color-temperature color.
     * @param[in] colors  RGB color values (shape @c (N,3)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setColors(const array::Array& colors, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the emitted light colors of the selected light prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return RGB color values (shape @c (N,3)).
     */
    array::Array getColors(const std::optional<array::Array>& indices = std::nullopt);

protected:
    /**
     * @brief Construct a Light wrapper.
     * @param[in] paths                  Single path or list of paths to USD light prims.
     * @param[in] lightType              USD light type name used when creating new prims.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     */
    Light(const std::variant<std::string, std::vector<std::string>>& paths,
          // Light (internal)
          const std::string& lightType,
          // Xform
          bool resetXformOpProperties = true);
};

} // namespace lights
} // namespace objects
} // namespace foundation
} // namespace isaacsim

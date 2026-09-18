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

#include <isaacsim/foundation/objects/lights/Light.hpp>

#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace lights
{

Light::Light(const std::variant<std::string, std::vector<std::string>>& paths,
             const std::string& lightType,
             bool resetXformOpProperties)
    : Xform()
{
    // Get or create light prims.
    auto [existentPaths, nonexistentPaths] = this->resolvePaths(paths);
    // Get light prims.
    if (!existentPaths.empty())
    {
        m_paths = std::move(existentPaths);
        const std::vector<bool> isLight = this->isA(lightType).flatten().get<std::vector<bool>>();
        for (std::size_t i = 0; i < m_paths.size(); ++i)
        {
            if (!isLight[i])
            {
                throw std::runtime_error("The wrapped prim at path '" + m_paths[i] + "' is not a USD" + lightType);
            }
        }
    }
    // Create light prims.
    else
    {
        m_paths = std::move(nonexistentPaths);
        for (const auto& path : m_paths)
        {
            this->getStage().definePrim(path, lightType);
        }
    }
    // Initialize instance from arguments.
    _initialize(resetXformOpProperties);
}

void Light::setIntensities(const array::Array& intensities, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues(
        "inputs:intensity", intensities.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Light::getIntensities(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:intensity", indices));
}

void Light::setExposures(const array::Array& exposures, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues(
        "inputs:exposure", exposures.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Light::getExposures(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:exposure", indices));
}

void Light::setMultipliers(const std::optional<array::Array>& diffuseMultipliers,
                           const std::optional<array::Array>& specularMultipliers,
                           const std::optional<array::Array>& indices)
{
    if (!diffuseMultipliers.has_value() && !specularMultipliers.has_value())
    {
        throw std::invalid_argument(
            "Both 'diffuseMultipliers' and 'specularMultipliers' are not defined. Define at least one of them");
    }
    int64_t batchSize = _resolveIndexedSize(indices);
    if (diffuseMultipliers.has_value())
    {
        this->setAttributeValues(
            "inputs:diffuse", (*diffuseMultipliers).broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
    }
    if (specularMultipliers.has_value())
    {
        this->setAttributeValues(
            "inputs:specular", (*specularMultipliers).broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
    }
}

std::tuple<array::Array, array::Array> Light::getMultipliers(const std::optional<array::Array>& indices)
{
    return { std::get<array::Array>(this->getAttributeValues("inputs:diffuse", indices)),
             std::get<array::Array>(this->getAttributeValues("inputs:specular", indices)) };
}

void Light::setEnabledNormalizations(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("inputs:normalize", enabled.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Light::getEnabledNormalizations(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:normalize", indices));
}

void Light::setEnabledColorTemperatures(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues(
        "inputs:enableColorTemperature", enabled.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Light::getEnabledColorTemperatures(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:enableColorTemperature", indices));
}

void Light::setColorTemperatures(const array::Array& colorTemperatures, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues(
        "inputs:colorTemperature", colorTemperatures.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Light::getColorTemperatures(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:colorTemperature", indices));
}

void Light::setColors(const array::Array& colors, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("inputs:color", colors.broadcastTo(array::Shape({ batchSize, int64_t{ 3 } })), indices);
}

array::Array Light::getColors(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:color", indices));
}

array::Array Light::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).hasApi("LightAPI");
}

} // namespace lights
} // namespace objects
} // namespace foundation
} // namespace isaacsim

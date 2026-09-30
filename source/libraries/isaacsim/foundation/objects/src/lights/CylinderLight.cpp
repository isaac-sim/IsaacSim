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

#include <isaacsim/foundation/objects/lights/CylinderLight.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace lights
{

CylinderLight::CylinderLight(const std::variant<std::string, std::vector<std::string>>& paths,
                             const std::optional<array::Array>& radii,
                             const std::optional<array::Array>& lengths,
                             bool resetXformOpProperties)
    : Light(paths, /*lightType=*/"CylinderLight", resetXformOpProperties)
{
    // Initialize instance from arguments.
    if (radii.has_value())
    {
        this->setRadii(*radii);
    }
    if (lengths.has_value())
    {
        this->setLengths(*lengths);
    }
}

void CylinderLight::setRadii(const array::Array& radii, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("inputs:radius", radii.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array CylinderLight::getRadii(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:radius", indices));
}

void CylinderLight::setLengths(const array::Array& lengths, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("inputs:length", lengths.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array CylinderLight::getLengths(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:length", indices));
}

void CylinderLight::setEnabledTreatAsLines(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("treatAsLine", enabled.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array CylinderLight::getEnabledTreatAsLines(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("treatAsLine", indices));
}

array::Array CylinderLight::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("CylinderLight");
}

} // namespace lights
} // namespace objects
} // namespace foundation
} // namespace isaacsim

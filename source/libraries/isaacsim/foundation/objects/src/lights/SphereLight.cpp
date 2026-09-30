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

#include <isaacsim/foundation/objects/lights/SphereLight.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace lights
{

SphereLight::SphereLight(const std::variant<std::string, std::vector<std::string>>& paths,
                         const std::optional<array::Array>& radii,
                         bool resetXformOpProperties)
    : Light(paths, /*lightType=*/"SphereLight", resetXformOpProperties)
{
    // Initialize instance from arguments.
    if (radii.has_value())
    {
        this->setRadii(*radii);
    }
}

void SphereLight::setRadii(const array::Array& radii, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("inputs:radius", radii.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array SphereLight::getRadii(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:radius", indices));
}

void SphereLight::setEnabledTreatAsPoints(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("treatAsPoint", enabled.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array SphereLight::getEnabledTreatAsPoints(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("treatAsPoint", indices));
}

array::Array SphereLight::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("SphereLight");
}

} // namespace lights
} // namespace objects
} // namespace foundation
} // namespace isaacsim

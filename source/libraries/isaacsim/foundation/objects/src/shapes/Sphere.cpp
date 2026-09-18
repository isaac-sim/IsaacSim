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

#include <isaacsim/foundation/objects/shapes/Sphere.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace shapes
{

Sphere::Sphere(const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& radii,
               const std::optional<ColorType>& colors,
               bool resetXformOpProperties)
    : Shape(paths, /*shapeType=*/"Sphere", colors, resetXformOpProperties)
{
    // Initialize instance from arguments.
    if (radii.has_value())
    {
        this->setRadii(*radii);
    }
}

void Sphere::setRadii(const array::Array& radii, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("radius", radii.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Sphere::getRadii(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("radius", indices));
}

void Sphere::updateExtents()
{
    // TODO: Implement and call it when setting values.
}

array::Array Sphere::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("Sphere");
}

} // namespace shapes
} // namespace objects
} // namespace foundation
} // namespace isaacsim

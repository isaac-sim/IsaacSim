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

#include <isaacsim/foundation/objects/shapes/Plane.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace shapes
{

Plane::Plane(const std::variant<std::string, std::vector<std::string>>& paths,
             const std::optional<array::Array>& widths,
             const std::optional<array::Array>& lengths,
             const std::optional<std::variant<std::string, std::vector<std::string>>>& axes,
             const std::optional<ColorType>& colors,
             bool resetXformOpProperties)
    : Shape(paths, /*shapeType=*/"Plane", colors, resetXformOpProperties)
{
    // Initialize instance from arguments.
    if (widths.has_value())
    {
        this->setWidths(*widths);
    }
    if (lengths.has_value())
    {
        this->setLengths(*lengths);
    }
    if (axes.has_value())
    {
        this->setAxes(*axes);
    }
}

void Plane::setWidths(const array::Array& widths, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("width", widths.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Plane::getWidths(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("width", indices));
}

void Plane::setLengths(const array::Array& lengths, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("length", lengths.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Plane::getLengths(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("length", indices));
}

void Plane::setAxes(const std::variant<std::string, std::vector<std::string>>& axes,
                    const std::optional<array::Array>& indices)
{
    this->setAttributeValues("axis", _resolveStringList(_resolveAxes(axes), indices), indices);
}

std::vector<std::string> Plane::getAxes(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("axis", indices));
}

void Plane::updateExtents()
{
    // TODO: Implement and call it when setting values.
}

array::Array Plane::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("Plane");
}

} // namespace shapes
} // namespace objects
} // namespace foundation
} // namespace isaacsim

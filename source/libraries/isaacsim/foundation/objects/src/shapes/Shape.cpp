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

#include <isaacsim/foundation/objects/shapes/Shape.hpp>

#include <cctype>
#include <stdexcept>
#include <type_traits>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace shapes
{

Shape::Shape(const std::variant<std::string, std::vector<std::string>>& paths,
             const std::string& shapeType,
             const std::optional<ColorType>& colors,
             bool resetXformOpProperties)
    : Xform()
{
    // Get or create shape prims.
    auto [existentPaths, nonexistentPaths] = this->resolvePaths(paths);
    // Get shape prims.
    if (!existentPaths.empty())
    {
        m_paths = std::move(existentPaths);
        const std::vector<bool> isShape = this->isA(shapeType).flatten().get<std::vector<bool>>();
        for (std::size_t i = 0; i < m_paths.size(); ++i)
        {
            if (!isShape[i])
            {
                throw std::runtime_error("The wrapped prim at path '" + m_paths[i] + "' is not a USD" + shapeType);
            }
        }
    }
    // Create shape prims.
    else
    {
        m_paths = std::move(nonexistentPaths);
        for (const auto& path : m_paths)
        {
            this->getStage().definePrim(path, shapeType);
        }
    }
    // Initialize instance from arguments.
    _initialize(resetXformOpProperties);
    if (colors.has_value())
    {
        this->setDisplayColors(*colors);
    }
}

void Shape::updateExtents()
{
}

void Shape::setDisplayColors(const ColorType& colors, const std::optional<array::Array>& indices)
{
    // TODO: implementation.
    (void)colors;
    (void)indices;
}

array::Array Shape::getDisplayColors(const std::optional<array::Array>& indices)
{
    // TODO: implementation.
    (void)indices;
    return array::Array(std::vector<double>{ 0.0, 0.0, 0.0 });
}

std::vector<std::string> Shape::_resolveAxes(const std::variant<std::string, std::vector<std::string>>& axes)
{
    // Normalize each axis to uppercase and validate it is exactly one of "X", "Y", or "Z".
    auto normalize = [](const std::string& axis) -> std::string
    {
        if (axis.size() != 1)
        {
            throw std::invalid_argument("Invalid axis '" + axis + "': expected a single character \"X\", \"Y\", or \"Z\"");
        }
        char value = static_cast<char>(std::toupper(static_cast<unsigned char>(axis[0])));
        if (value != 'X' && value != 'Y' && value != 'Z')
        {
            throw std::invalid_argument("Invalid axis '" + axis + "': expected one of \"X\", \"Y\", or \"Z\"");
        }
        return std::string(1, value);
    };

    return std::visit(
        [&normalize](const auto& axes) -> std::vector<std::string>
        {
            using T = std::decay_t<decltype(axes)>;
            if constexpr (std::is_same_v<T, std::string>)
            {
                return { normalize(axes) };
            }
            else
            {
                std::vector<std::string> result;
                result.reserve(axes.size());
                for (const auto& axis : axes)
                {
                    result.push_back(normalize(axis));
                }
                return result;
            }
        },
        axes);
}

array::Array Shape::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("Gprim");
}

} // namespace shapes
} // namespace objects
} // namespace foundation
} // namespace isaacsim

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

#include <isaacsim/foundation/objects/lights/RectLight.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace lights
{

RectLight::RectLight(const std::variant<std::string, std::vector<std::string>>& paths,
                     const std::optional<array::Array>& widths,
                     const std::optional<array::Array>& heights,
                     const std::optional<std::variant<std::string, std::vector<std::string>>>& textureFiles,
                     bool resetXformOpProperties)
    : Light(paths, /*lightType=*/"RectLight", resetXformOpProperties)
{
    // Initialize instance from arguments.
    if (widths.has_value())
    {
        this->setWidths(*widths);
    }
    if (heights.has_value())
    {
        this->setHeights(*heights);
    }
    if (textureFiles.has_value())
    {
        this->setTextureFiles(*textureFiles);
    }
}

void RectLight::setWidths(const array::Array& widths, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("inputs:width", widths.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array RectLight::getWidths(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:width", indices));
}

void RectLight::setHeights(const array::Array& heights, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("inputs:height", heights.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array RectLight::getHeights(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("inputs:height", indices));
}

void RectLight::setTextureFiles(const std::variant<std::string, std::vector<std::string>>& textureFiles,
                                const std::optional<array::Array>& indices)
{
    this->setAttributeValues("inputs:texture:file", _resolveStringList(textureFiles, indices), indices);
}

std::vector<std::string> RectLight::getTextureFiles(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("inputs:texture:file", indices));
}

array::Array RectLight::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("RectLight");
}

} // namespace lights
} // namespace objects
} // namespace foundation
} // namespace isaacsim

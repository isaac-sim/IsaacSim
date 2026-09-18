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

#include <isaacsim/foundation/objects/lights/DomeLight.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace lights
{

DomeLight::DomeLight(const std::variant<std::string, std::vector<std::string>>& paths,
                     const std::optional<array::Array>& radii,
                     const std::optional<std::variant<std::string, std::vector<std::string>>>& textureFiles,
                     const std::optional<std::variant<std::string, std::vector<std::string>>>& textureFormats,
                     bool resetXformOpProperties)
    : Light(paths, /*lightType=*/"DomeLight", resetXformOpProperties)
{
    // Initialize instance from arguments.
    if (radii.has_value())
    {
        this->setGuideRadii(*radii);
    }
    if (textureFiles.has_value())
    {
        this->setTextureFiles(*textureFiles);
    }
    if (textureFormats.has_value())
    {
        this->setTextureFormats(*textureFormats);
    }
}

void DomeLight::setGuideRadii(const array::Array& radii, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("guideRadius", radii.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array DomeLight::getGuideRadii(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("guideRadius", indices));
}

void DomeLight::setTextureFiles(const std::variant<std::string, std::vector<std::string>>& textureFiles,
                                const std::optional<array::Array>& indices)
{
    this->setAttributeValues("inputs:texture:file", _resolveStringList(textureFiles, indices), indices);
}

std::vector<std::string> DomeLight::getTextureFiles(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("inputs:texture:file", indices));
}

void DomeLight::setTextureFormats(const std::variant<std::string, std::vector<std::string>>& textureFormats,
                                  const std::optional<array::Array>& indices)
{
    this->setAttributeValues("inputs:texture:format", _resolveStringList(textureFormats, indices), indices);
}

std::vector<std::string> DomeLight::getTextureFormats(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("inputs:texture:format", indices));
}

array::Array DomeLight::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("DomeLight");
}

} // namespace lights
} // namespace objects
} // namespace foundation
} // namespace isaacsim

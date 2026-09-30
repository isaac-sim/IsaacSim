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

#include <isaacsim/foundation/objects/Prim.hpp>
#include <isaacsim/foundation/usd/openusd/Usd.hpp>
#include <isaacsim/foundation/usd/ovstage/Usd.hpp>

#include <numeric>
#include <sstream>
#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace objects
{

namespace openusd = isaacsim::foundation::usd::openusd;
namespace ovstage = isaacsim::foundation::usd::ovstage;

namespace
{

std::string joinPaths(const std::vector<std::string>& paths)
{
    std::ostringstream stream;
    for (std::size_t index = 0; index < paths.size(); ++index)
    {
        if (index)
            stream << ", ";
        stream << "'" << paths[index] << "'";
    }
    return stream.str();
}

struct AttributeSpecification
{
    int64_t dimensions;
    bool isScalar;
    bool isArray;
};

// clang-format off
const std::unordered_map<std::string, AttributeSpecification> g_kAttributeSpecifications = {
    { "asset",         {  0, false, false } },
    { "asset[]",       {  0, false,  true } },

    { "bool",          {  1,  true, false } },
    { "bool[]",        {  1,  true,  true } },

    { "double",        {  1,  true, false } },
    { "double[]",      {  1,  true,  true } },

    { "float",         {  1,  true, false } },
    { "float[]",       {  1,  true,  true } },

    { "half",          {  1,  true, false } },
    { "half[]",        {  1,  true,  true } },

    { "int",           {  1,  true, false } },
    { "int[]",         {  1,  true,  true } },

    { "int64",         {  1,  true, false } },
    { "int64[]",       {  1,  true,  true } },

    { "uint",          {  1,  true, false } },
    { "uint[]",        {  1,  true,  true } },

    { "uint64",        {  1,  true, false } },
    { "uint64[]",      {  1,  true,  true } },

    { "uchar",         {  1,  true, false } },
    { "uchar[]",       {  1,  true,  true } },

    { "string",        {  0, false, false } },
    { "string[]",      {  0, false,  true } },

    { "token",         {  0, false, false } },
    { "token[]",       {  0, false,  true } },

    { "timecode",      {  1,  true, false } },
    { "timecode[]",    {  1,  true,  true } },

    // GfVec2d
    { "double2",       {  2,  true, false } },
    { "texCoord2d",    {  2,  true, false } },
    { "double2[]",     {  2,  true,  true } },
    { "texCoord2d[]",  {  2,  true,  true } },

    // GfVec3d
    { "double3",       {  3,  true, false } },
    { "color3d",       {  3,  true, false } },
    { "normal3d",      {  3,  true, false } },
    { "point3d",       {  3,  true, false } },
    { "vector3d",      {  3,  true, false } },
    { "texCoord3d",    {  3,  true, false } },
    { "double3[]",     {  3,  true,  true } },
    { "color3d[]",     {  3,  true,  true } },
    { "normal3d[]",    {  3,  true,  true } },
    { "point3d[]",     {  3,  true,  true } },
    { "vector3d[]",    {  3,  true,  true } },
    { "texCoord3d[]",  {  3,  true,  true } },

    // GfVec4d
    { "double4",       {  4,  true, false } },
    { "color4d",       {  4,  true, false } },
    { "double4[]",     {  4,  true,  true } },
    { "color4d[]",     {  4,  true,  true } },

    // GfVec2f
    { "float2",        {  2,  true, false } },
    { "texCoord2f",    {  2,  true, false } },
    { "float2[]",      {  2,  true,  true } },
    { "texCoord2f[]",  {  2,  true,  true } },

    // GfVec3f
    { "float3",        {  3,  true, false } },
    { "color3f",       {  3,  true, false } },
    { "normal3f",      {  3,  true, false } },
    { "point3f",       {  3,  true, false } },
    { "vector3f",      {  3,  true, false } },
    { "texCoord3f",    {  3,  true, false } },
    { "float3[]",      {  3,  true,  true } },
    { "color3f[]",     {  3,  true,  true } },
    { "normal3f[]",    {  3,  true,  true } },
    { "point3f[]",     {  3,  true,  true } },
    { "vector3f[]",    {  3,  true,  true } },
    { "texCoord3f[]",  {  3,  true,  true } },

    // GfVec4f
    { "float4",        {  4,  true, false } },
    { "color4f",       {  4,  true, false } },
    { "float4[]",      {  4,  true,  true } },
    { "color4f[]",     {  4,  true,  true } },

    // GfVec2h
    { "half2",         {  2,  true, false } },
    { "texCoord2h",    {  2,  true, false } },
    { "half2[]",       {  2,  true,  true } },
    { "texCoord2h[]",  {  2,  true,  true } },

    // GfVec3h
    { "half3",         {  3,  true, false } },
    { "color3h",       {  3,  true, false } },
    { "normal3h",      {  3,  true, false } },
    { "point3h",       {  3,  true, false } },
    { "vector3h",      {  3,  true, false } },
    { "texCoord3h",    {  3,  true, false } },
    { "half3[]",       {  3,  true,  true } },
    { "color3h[]",     {  3,  true,  true } },
    { "normal3h[]",    {  3,  true,  true } },
    { "point3h[]",     {  3,  true,  true } },
    { "vector3h[]",    {  3,  true,  true } },
    { "texCoord3h[]",  {  3,  true,  true } },

    // GfVec4h
    { "half4",         {  4,  true, false } },
    { "color4h",       {  4,  true, false } },
    { "half4[]",       {  4,  true,  true } },
    { "color4h[]",     {  4,  true,  true } },

    // GfVec2i
    { "int2",          {  2,  true, false } },
    { "int2[]",        {  2,  true,  true } },

    // GfVec3i
    { "int3",          {  3,  true, false } },
    { "int3[]",        {  3,  true,  true } },

    // GfVec4i
    { "int4",          {  4,  true, false } },
    { "int4[]",        {  4,  true,  true } },

    // GfMatrix2d
    { "matrix2d",      {  4,  true, false } },
    { "matrix2d[]",    {  4,  true,  true } },

    // GfMatrix3d
    { "matrix3d",      {  9,  true, false } },
    { "matrix3d[]",    {  9,  true,  true } },

    // GfMatrix4d
    { "matrix4d",      { 16,  true, false } },
    { "frame4d",       { 16,  true, false } },
    { "matrix4d[]",    { 16,  true,  true } },
    { "frame4d[]",     { 16,  true,  true } },

    // GfQuatd
    { "quatd",         {  4,  true, false } },
    { "quatd[]",       {  4,  true,  true } },

    // GfQuatf
    { "quatf",         {  4,  true, false } },
    { "quatf[]",       {  4,  true,  true } },

    // GfQuath
    { "quath",         {  4,  true, false } },
    { "quath[]",       {  4,  true,  true } },
};
// clang-format on

} // namespace

Prim::Prim(const std::variant<std::string, std::vector<std::string>>& paths, bool resolvePaths)
    : m_stage(details::getActiveStage())
{
    m_primOps = _populatePrimOps(m_stage.getBackend());
    m_paths = std::holds_alternative<std::string>(paths) ? std::vector<std::string>{ std::get<std::string>(paths) } :
                                                           std::get<std::vector<std::string>>(paths);
    // Resolve paths
    if (resolvePaths)
    {
        auto [existingPaths, nonexistentPaths] = this->resolvePaths(m_paths);
        if (!nonexistentPaths.empty())
        {
            throw std::runtime_error("Specified paths must correspond to existing prims: " + joinPaths(nonexistentPaths));
        }
        m_paths = std::move(existingPaths);
    }
}

Prim::Prim() : m_stage(details::getActiveStage())
{
    m_primOps = _populatePrimOps(m_stage.getBackend());
}

Prim::PrimOps Prim::_populatePrimOps(const std::string& backend)
{
    if (backend == "openusd")
    {
        return {
            openusd::findMatchingPrimPaths,
            openusd::isValidPathString,
            openusd::getName,
            openusd::getTypeName,
            openusd::getParent,
            openusd::getChildren,
            openusd::isA,
            openusd::hasApi,
            openusd::applyApi,
            openusd::removeApi,
            openusd::getAppliedSchemas,
            openusd::getVariants,
            openusd::setVariants,
            openusd::createPrimAttribute,
            openusd::removePrimAttribute,
            openusd::getPrimAttributeTypeName,
            openusd::getPrimAttributeValues,
            openusd::setPrimAttributeValues,
        };
    }
    else if (backend == "ovstage")
    {
        return {
            ovstage::findMatchingPrimPaths,
            ovstage::isValidPathString,
            ovstage::getName,
            ovstage::getTypeName,
            ovstage::getParent,
            ovstage::getChildren,
            ovstage::isA,
            ovstage::hasApi,
            ovstage::applyApi,
            ovstage::removeApi,
            ovstage::getAppliedSchemas,
            ovstage::getVariants,
            ovstage::setVariants,
            ovstage::createPrimAttribute,
            ovstage::removePrimAttribute,
            ovstage::getPrimAttributeTypeName,
            ovstage::getPrimAttributeValues,
            ovstage::setPrimAttributeValues,
        };
    }
    throw std::runtime_error("Unknown backend '" + backend + "'. Expected 'openusd' or 'ovstage'.");
}

int64_t Prim::_resolveIndexedSize(const std::optional<array::Array>& indices) const
{
    return indices.has_value() ? indices->shape()[0] : static_cast<int64_t>(m_paths.size());
}

std::vector<int64_t> Prim::_resolveIndexValues(const std::optional<array::Array>& indices) const
{
    if (!indices.has_value())
    {
        std::vector<int64_t> allIndices(static_cast<std::size_t>(m_paths.size()));
        std::iota(allIndices.begin(), allIndices.end(), int64_t{ 0 });
        return allIndices;
    }

    const auto count = static_cast<int64_t>(m_paths.size());
    std::vector<int64_t> indexValues = indices->flatten().get<std::vector<int64_t>>();
    for (int64_t& index : indexValues)
    {
        const int64_t original = index;
        if (index < 0)
        {
            index += count; // Python-style negative indexing
        }
        if (index < 0 || index >= count)
        {
            throw std::out_of_range("Prim index " + std::to_string(original) + " is out of range [0, " +
                                    std::to_string(count) + ")");
        }
    }
    return indexValues;
}

std::vector<std::string> Prim::_resolveIndexedPaths(const std::optional<array::Array>& indices) const
{
    if (!indices.has_value())
    {
        return m_paths;
    }

    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    std::vector<std::string> resolvedPaths;
    resolvedPaths.reserve(indexValues.size());
    for (int64_t index : indexValues)
    {
        resolvedPaths.push_back(m_paths[index]);
    }
    return resolvedPaths;
}

std::vector<std::string> Prim::_resolveStringList(const std::variant<std::string, std::vector<std::string>>& values,
                                                  const std::optional<array::Array>& indices) const
{
    int64_t batchSize = _resolveIndexedSize(indices);
    if (std::holds_alternative<std::string>(values))
    {
        return std::vector<std::string>(batchSize, std::get<std::string>(values));
    }
    else
    {
        auto list = std::get<std::vector<std::string>>(values);
        if (list.empty())
        {
            throw std::invalid_argument("Expected a single string or a list of strings, but got an empty list");
        }
        else if (list.size() == 1)
        {
            return batchSize == 1 ? list : std::vector<std::string>(batchSize, list[0]);
        }
        else if (list.size() != static_cast<std::size_t>(batchSize))
        {
            throw std::invalid_argument("Expected a list of strings with length " + std::to_string(batchSize) +
                                        ", got a list with length " + std::to_string(list.size()));
        }
        return list;
    }
}

size_t Prim::size() const
{
    return m_paths.size();
}

const std::vector<std::string>& Prim::paths() const
{
    return m_paths;
}

const Stage& Prim::getStage() const
{
    return m_stage;
}

std::tuple<std::vector<std::string>, std::vector<std::string>> Prim::resolvePaths(
    const std::variant<std::string, std::vector<std::string>>& paths, bool raiseOnMixedPaths)
{
    const std::vector<std::string> inputPaths = std::holds_alternative<std::string>(paths) ?
                                                    std::vector<std::string>{ std::get<std::string>(paths) } :
                                                    std::get<std::vector<std::string>>(paths);

    std::vector<std::vector<std::string>> existingPathGroups;
    std::vector<std::string> nonexistentPaths;
    std::vector<std::string> invalidPaths;

    const Stage& activeStage = details::getActiveStage();
    const auto ops = _populatePrimOps(activeStage.getBackend());
    int64_t stageId = activeStage.getStageId();
    for (const auto& path : inputPaths)
    {
        auto result = ops.findMatchingPrimPaths(stageId, path, false);
        if (!result.empty())
        {
            existingPathGroups.push_back(std::move(result));
        }
        else if (ops.isValidPathString(path))
        {
            nonexistentPaths.push_back(path);
        }
        else
        {
            invalidPaths.push_back(path);
        }
    }

    if (raiseOnMixedPaths)
    {
        if (!existingPathGroups.empty() && !nonexistentPaths.empty())
        {
            throw std::runtime_error(
                "Specified paths include both existing and non-existing prims.\n"
                "Ensure all the paths correspond to existing or non-existing prims only.\n"
                "Given paths: " +
                joinPaths(inputPaths) +
                "\n"
                "Non-existing paths: " +
                joinPaths(nonexistentPaths) + "\n");
        }
        else if (existingPathGroups.empty() && nonexistentPaths.empty())
        {
            throw std::runtime_error(
                "Specified paths are invalid. Possible reasons:\n"
                " - A regex is used but the stage does not contain any prims matching the regex\n"
                " - A specified path is malformed or contains invalid characters\n"
                "Given paths: " +
                joinPaths(inputPaths) +
                "\n"
                "Invalid paths: " +
                joinPaths(invalidPaths) + "\n");
        }
        else if (!existingPathGroups.empty() && existingPathGroups.size() != inputPaths.size())
        {
            throw std::runtime_error(
                "Specified paths are invalid. Possible reasons:\n"
                " - A regex is used but the stage does not contain any prims matching the regex\n"
                " - A specified path is malformed or contains invalid characters\n"
                "Given paths: " +
                joinPaths(inputPaths) +
                "\n"
                "Invalid paths: " +
                joinPaths(invalidPaths) + "\n");
        }
        else if (!nonexistentPaths.empty() && nonexistentPaths.size() != inputPaths.size())
        {
            throw std::runtime_error(
                "Specified paths are invalid. Possible reasons:\n"
                " - A regex is used but the stage does not contain any prims matching the regex\n"
                " - A specified path is malformed or contains invalid characters\n"
                "Given paths: " +
                joinPaths(inputPaths) +
                "\n"
                "Non-existing paths: " +
                joinPaths(nonexistentPaths) +
                "\n"
                "Invalid paths: " +
                joinPaths(invalidPaths) + "\n");
        }
    }

    std::vector<std::string> existingPaths;
    for (const auto& group : existingPathGroups)
    {
        existingPaths.insert(existingPaths.end(), group.begin(), group.end());
    }

    return { existingPaths, nonexistentPaths };
}

std::vector<std::string> Prim::getName(const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<std::string> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.getName(stageId, path));
    }
    return result;
}

std::vector<std::string> Prim::getTypeName(const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<std::string> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.getTypeName(stageId, path));
    }
    return result;
}

std::vector<std::string> Prim::getParent(const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<std::string> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.getParent(stageId, path));
    }
    return result;
}

std::vector<std::vector<std::string>> Prim::getChildren(const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<std::vector<std::string>> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.getChildren(stageId, path));
    }
    return result;
}

std::vector<std::unordered_map<std::string, std::vector<std::string>>> Prim::getVariantSets(
    const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<std::unordered_map<std::string, std::vector<std::string>>> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.getVariants(stageId, path, false));
    }
    return result;
}

std::vector<std::unordered_map<std::string, std::string>> Prim::getVariantSelection(
    const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<std::unordered_map<std::string, std::string>> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        std::unordered_map<std::string, std::string> selection;
        for (const auto& [name, selections] : m_primOps.getVariants(stageId, path, true))
        {
            selection[name] = selections.empty() ? "" : selections[0];
        }
        result.push_back(std::move(selection));
    }
    return result;
}

void Prim::setVariantSelection(const std::unordered_map<std::string, std::string>& variants,
                               const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::unordered_map<std::string, std::vector<std::string>> wrapped;

    int64_t stageId = m_stage.getStageId();
    for (const auto& [name, selection] : variants)
    {
        wrapped[name] = { selection };
    }
    for (const auto& path : paths)
    {
        m_primOps.setVariants(stageId, path, wrapped);
    }
}

array::Array Prim::isA(const std::string& schemaType, const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<bool> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.isA(stageId, path, schemaType));
    }
    return array::Array(result).reshape(array::Shape({ static_cast<int64_t>(paths.size()), int64_t{ 1 } }));
}

array::Array Prim::hasApi(const std::string& schemaType,
                          const std::optional<std::string>& instanceName,
                          const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<bool> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.hasApi(stageId, path, schemaType, instanceName));
    }
    return array::Array(result).reshape(array::Shape({ static_cast<int64_t>(paths.size()), int64_t{ 1 } }));
}

array::Array Prim::applyApi(const std::string& schemaType,
                            const std::optional<std::string>& instanceName,
                            const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<bool> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.applyApi(stageId, path, schemaType, instanceName));
    }
    return array::Array(result).reshape(array::Shape({ static_cast<int64_t>(paths.size()), int64_t{ 1 } }));
}

array::Array Prim::removeApi(const std::string& schemaType,
                             const std::optional<std::string>& instanceName,
                             const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<bool> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.removeApi(stageId, path, schemaType, instanceName));
    }
    return array::Array(result).reshape(array::Shape({ static_cast<int64_t>(paths.size()), int64_t{ 1 } }));
}

std::vector<std::vector<std::string>> Prim::getAppliedSchemas(const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<std::vector<std::string>> result;
    result.reserve(paths.size());

    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.getAppliedSchemas(stageId, path));
    }
    return result;
}

array::Array Prim::createAttribute(const std::string& attributeName,
                                   const std::string& typeName,
                                   const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<bool> result;
    result.reserve(paths.size());
    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.createPrimAttribute(stageId, path, attributeName, typeName));
    }
    return array::Array(result).reshape(array::Shape({ static_cast<int64_t>(paths.size()), int64_t{ 1 } }));
}

array::Array Prim::removeAttribute(const std::string& attributeName, const std::optional<array::Array>& indices) const
{
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    std::vector<bool> result;
    result.reserve(paths.size());
    int64_t stageId = m_stage.getStageId();
    for (const auto& path : paths)
    {
        result.push_back(m_primOps.removePrimAttribute(stageId, path, attributeName));
    }
    return array::Array(result).reshape(array::Shape({ static_cast<int64_t>(paths.size()), int64_t{ 1 } }));
}

OutputValueType Prim::getAttributeValues(const std::string& attributeName, const std::optional<array::Array>& indices) const
{
    int64_t stageId = m_stage.getStageId();
    return m_primOps.getPrimAttributeValues(stageId, _resolveIndexedPaths(indices), attributeName);
}

void Prim::setAttributeValues(const std::string& attributeName,
                              const InputValueType& values,
                              const std::optional<array::Array>& indices) const
{
    int64_t stageId = m_stage.getStageId();
    std::vector<std::string> paths = _resolveIndexedPaths(indices);
    // Reshape/broadcast Array values.
    if (std::holds_alternative<array::Array>(values))
    {
        // Get attribute specification.
        const std::string typeName = m_primOps.getPrimAttributeTypeName(stageId, paths.front(), attributeName);
        const auto specification = g_kAttributeSpecifications.find(typeName);
        if (specification != g_kAttributeSpecifications.end())
        {
            // Reshape/broadcast Array values that are scalar but not array.
            if (specification->second.isScalar && !specification->second.isArray)
            {
                const auto& sourceValues = std::get<array::Array>(values);
                const int64_t dimension = specification->second.dimensions;
                const array::Shape reshapeShape =
                    sourceValues.size() == 1 ? array::Shape({ 1 }) : array::Shape({ int64_t{ -1 }, dimension });
                const array::Shape broadcastShape = array::Shape({ _resolveIndexedSize(indices), dimension });
                const array::Array resolvedValues = sourceValues.reshape(reshapeShape).broadcastTo(broadcastShape);
                m_primOps.setPrimAttributeValues(stageId, paths, attributeName, resolvedValues);
                return;
            }
        }
    }
    // TODO: broadcast single string or list of strings automatically.
    // A single string token is broadcast to a one-element string list; the backend has no scalar-string.
    OutputValueType resolvedValues = std::visit(
        [](auto&& value) -> OutputValueType
        {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, std::string>)
            {
                return std::vector<std::string>{ value };
            }
            else
            {
                return value;
            }
        },
        values);
    m_primOps.setPrimAttributeValues(stageId, paths, attributeName, resolvedValues);
}

} // namespace objects
} // namespace foundation
} // namespace isaacsim

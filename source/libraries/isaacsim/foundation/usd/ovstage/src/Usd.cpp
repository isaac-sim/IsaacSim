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

#include "details/UsdHelpers.hpp"

#include <isaacsim/common/exceptions/Exceptions.hpp>
#include <isaacsim/foundation/usd/ovstage/Usd.hpp>
#include <ovstage/ovstage_population.h>
#include <ovx/path_dictionary/path_dictionary.h>
#include <ovx/path_dictionary/path_dictionary_utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace isaacsim
{
namespace foundation
{
namespace usd
{
namespace ovstage
{

namespace
{

static constexpr char g_kPrimTypeAttr[] = "usd-prim-type";
// Sentinel written for prims defined with no type name; mapped back to "" on read.
static constexpr char g_kUntypedSentinel[] = "__ovstage_population_untyped__";
static constexpr char g_kSchemasAttr[] = "usd-schemas";

// Writes the full schema set for one prim at `ordinal` (no floor advance — caller seals).
void writeSchemasToPath(ovstage_instance_t* instance,
                        path_dictionary_instance_t* dictionary,
                        const std::string& path,
                        const std::vector<std::string>& schemas,
                        ovstage_ordinal_t ordinal)
{
    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList).status != OVX_API_SUCCESS)
        return;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return;
    }

    std::vector<uint64_t> schemaIdentifiers;
    if (!schemas.empty())
    {
        std::vector<ovx_string_t> nameViews;
        nameViews.reserve(schemas.size());
        for (const auto& schema : schemas)
            nameViews.push_back({ schema.c_str(), schema.size() });

        std::vector<ovx_token_t> tokens(schemas.size(), OVX_INVALID_TOKEN);
        if (path_dictionary_create_tokens_from_strings(dictionary, nameViews.data(), schemas.size(), tokens.data()).status ==
            OVX_API_SUCCESS)
        {
            schemaIdentifiers.reserve(schemas.size());
            for (auto token : tokens)
                schemaIdentifiers.push_back(static_cast<uint64_t>(token));
        }
    }

    const int64_t shape[1] = { static_cast<int64_t>(schemaIdentifiers.size()) };
    const int64_t strides[1] = { 1 };
    DLTensor tensor{};
    tensor.data = schemaIdentifiers.empty() ? nullptr : schemaIdentifiers.data();
    tensor.device = { kDLCPU, 0 };
    tensor.ndim = 1;
    tensor.dtype = { kDLUInt, 64, 1 };
    tensor.shape = const_cast<int64_t*>(shape);
    tensor.strides = const_cast<int64_t*>(strides);

    ovstage_write_data_t writeDescription{};
    writeDescription.tensors = &tensor;
    writeDescription.tensor_count = 1;
    writeDescription.semantic = OVSTAGE_SEMANTIC_NONE;
    writeDescription.is_array = true;

    const ovx_string_or_token_t attributeSpecification{ OVX_INVALID_TOKEN,
                                                        ovx_string_t{ g_kSchemasAttr, sizeof(g_kSchemasAttr) - 1 } };
    details::waitAndRelease(instance, ovstage_write_attribute(instance, query, attributeSpecification, ordinal,
                                                              writeDescription, OVSTAGE_PRIM_MODE_UPSERT));

    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);
}

// Returns the usd-prim-type string for the prim at `path`, or std::nullopt if the
// prim does not exist. Returns "" when the prim exists but was defined with an
// empty type. Does NOT throw.
std::optional<std::string> readPrimType(ovstage_instance_t* instance,
                                        path_dictionary_instance_t* dictionary,
                                        const std::string& path)
{
    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList).status != OVX_API_SUCCESS)
        return std::nullopt;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return std::nullopt;
    }

    const ovx_string_t attributeString{ g_kPrimTypeAttr, sizeof(g_kPrimTypeAttr) - 1 };
    ovx_token_t attributeToken = OVX_INVALID_TOKEN;
    path_dictionary_create_tokens_from_strings(dictionary, &attributeString, 1, &attributeToken);

    std::optional<std::string> result;
    if (attributeToken != OVX_INVALID_TOKEN)
    {
        ovstage_ordinal_range_t range{};
        range.end_ordinal = UINT64_MAX;
        range.has_start_ordinal = false;

        ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
        ovstage_enqueue_result_t enqueueResult =
            ovstage_read_attributes(instance, query, &attributeToken, 1, range, &read);
        if (enqueueResult.status == OVSTAGE_OK)
        {
            details::waitAndRelease(instance, enqueueResult);
            for (;;)
            {
                ovstage_read_group_t group{};
                const ovstage_api_status_t fetched =
                    ovstage_fetch_read_next(instance, read, OVSTAGE_TIMEOUT_INFINITE, &group);
                if (fetched == OVSTAGE_ERROR_END_OF_ITERATION)
                    break;
                if (fetched != OVSTAGE_OK)
                    break;
                if (!group.is_delete && group.data.tensor_count == 1 && group.data.tensors &&
                    group.data.tensors[0].data && group.data.tensors[0].ndim == 1 &&
                    group.data.tensors[0].dtype.code == kDLUInt && group.data.tensors[0].dtype.bits == 64 &&
                    group.data.tensors[0].dtype.lanes == 1 && group.data.tensors[0].shape[0] >= 1)
                {
                    const uint64_t typeId = *static_cast<const uint64_t*>(group.data.tensors[0].data);
                    const ovx_token_t typeToken = static_cast<ovx_token_t>(typeId);
                    ovx_string_t name{};
                    if (path_dictionary_get_strings_from_tokens(dictionary, &typeToken, 1, &name).status ==
                            OVX_API_SUCCESS &&
                        name.ptr)
                    {
                        std::string resolved(name.ptr, name.length);
                        result.emplace(resolved == g_kUntypedSentinel ? "" : resolved);
                    }
                    else
                        result.emplace();
                }
                ovstage_release_group(instance, &group);
            }
            details::waitAndRelease(instance, ovstage_release_read(instance, read));
        }
    }

    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);
    return result;
}

// Writes usd-prim-type for one path at `ordinal` (no floor advance — caller seals).
void writePrimTypeToPath(ovstage_instance_t* instance,
                         path_dictionary_instance_t* dictionary,
                         const std::string& path,
                         const std::string& typeName,
                         ovstage_ordinal_t ordinal)
{
    const ovx_string_t typeString{ typeName.c_str(), typeName.size() };
    ovx_token_t typeToken = OVX_INVALID_TOKEN;
    if (path_dictionary_create_tokens_from_strings(dictionary, &typeString, 1, &typeToken).status != OVX_API_SUCCESS)
        return;

    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList).status != OVX_API_SUCCESS)
        return;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return;
    }

    uint64_t typeId = static_cast<uint64_t>(typeToken);
    const int64_t shape[1] = { 1 };
    const int64_t strides[1] = { 1 };
    DLTensor tensor{};
    tensor.data = &typeId;
    tensor.device = { kDLCPU, 0 };
    tensor.ndim = 1;
    tensor.dtype = { kDLUInt, 64, 1 };
    tensor.shape = const_cast<int64_t*>(shape);
    tensor.strides = const_cast<int64_t*>(strides);

    ovstage_write_data_t writeDescription{};
    writeDescription.tensors = &tensor;
    writeDescription.tensor_count = 1;
    writeDescription.semantic = OVSTAGE_SEMANTIC_NONE;
    writeDescription.is_array = false;

    const ovx_string_or_token_t attributeSpecification{ OVX_INVALID_TOKEN,
                                                        ovx_string_t{ g_kPrimTypeAttr, sizeof(g_kPrimTypeAttr) - 1 } };
    details::waitAndRelease(instance, ovstage_write_attribute(instance, query, attributeSpecification, ordinal,
                                                              writeDescription, OVSTAGE_PRIM_MODE_UPSERT));

    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);
}

struct UsdTypeDescriptor
{
    DLDataType dtype;
    ovstage_attribute_semantic_t semantic;
    bool is_array;
};

static std::optional<UsdTypeDescriptor> parseUsdTypeName(const std::string& name)
{
    static const std::unordered_map<std::string, UsdTypeDescriptor> s_kMap = {
        { "bool", { { kDLUInt, 8, 1 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "uchar", { { kDLUInt, 8, 1 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "int", { { kDLInt, 32, 1 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "uint", { { kDLUInt, 32, 1 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "int64", { { kDLInt, 64, 1 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "uint64", { { kDLUInt, 64, 1 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "half", { { kDLFloat, 16, 1 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "float", { { kDLFloat, 32, 1 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "double", { { kDLFloat, 64, 1 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "string", { { kDLUInt, 8, 1 }, OVSTAGE_SEMANTIC_STRING, true } },
        { "token", { { kDLUInt, 64, 1 }, OVSTAGE_SEMANTIC_TOKEN_ID, false } },
        { "asset", { { kDLUInt, 8, 1 }, OVSTAGE_SEMANTIC_ASSET_STRING, true } },
        { "int2", { { kDLInt, 32, 2 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "int3", { { kDLInt, 32, 3 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "int4", { { kDLInt, 32, 4 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "half2", { { kDLFloat, 16, 2 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "half3", { { kDLFloat, 16, 3 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "half4", { { kDLFloat, 16, 4 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "float2", { { kDLFloat, 32, 2 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "float3", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "float4", { { kDLFloat, 32, 4 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "double2", { { kDLFloat, 64, 2 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "double3", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "double4", { { kDLFloat, 64, 4 }, OVSTAGE_SEMANTIC_NONE, false } },
        { "point2f", { { kDLFloat, 32, 2 }, OVSTAGE_SEMANTIC_POINT, false } },
        { "point3f", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_POINT, false } },
        { "point2d", { { kDLFloat, 64, 2 }, OVSTAGE_SEMANTIC_POINT, false } },
        { "point3d", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_POINT, false } },
        { "vector2f", { { kDLFloat, 32, 2 }, OVSTAGE_SEMANTIC_VECTOR, false } },
        { "vector3f", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_VECTOR, false } },
        { "vector2d", { { kDLFloat, 64, 2 }, OVSTAGE_SEMANTIC_VECTOR, false } },
        { "vector3d", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_VECTOR, false } },
        { "normal3f", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_NORMAL, false } },
        { "normal3d", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_NORMAL, false } },
        { "color3f", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_COLOR, false } },
        { "color3d", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_COLOR, false } },
        { "color4f", { { kDLFloat, 32, 4 }, OVSTAGE_SEMANTIC_COLOR, false } },
        { "color4d", { { kDLFloat, 64, 4 }, OVSTAGE_SEMANTIC_COLOR, false } },
        { "quatf", { { kDLFloat, 32, 4 }, OVSTAGE_SEMANTIC_QUATERNION, false } },
        { "quatd", { { kDLFloat, 64, 4 }, OVSTAGE_SEMANTIC_QUATERNION, false } },
        { "quath", { { kDLFloat, 16, 4 }, OVSTAGE_SEMANTIC_QUATERNION, false } },
        { "texCoord2f", { { kDLFloat, 32, 2 }, OVSTAGE_SEMANTIC_TEXTURE_COORDINATE, false } },
        { "texCoord3f", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_TEXTURE_COORDINATE, false } },
        { "texCoord2d", { { kDLFloat, 64, 2 }, OVSTAGE_SEMANTIC_TEXTURE_COORDINATE, false } },
        { "texCoord3d", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_TEXTURE_COORDINATE, false } },
        { "matrix2d", { { kDLFloat, 64, 4 }, OVSTAGE_SEMANTIC_MATRIX, false } },
        { "matrix3d", { { kDLFloat, 64, 9 }, OVSTAGE_SEMANTIC_MATRIX, false } },
        { "matrix4d", { { kDLFloat, 64, 16 }, OVSTAGE_SEMANTIC_MATRIX, false } },
        // array variants
        { "bool[]", { { kDLUInt, 8, 1 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "uchar[]", { { kDLUInt, 8, 1 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "int[]", { { kDLInt, 32, 1 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "uint[]", { { kDLUInt, 32, 1 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "int64[]", { { kDLInt, 64, 1 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "uint64[]", { { kDLUInt, 64, 1 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "half[]", { { kDLFloat, 16, 1 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "float[]", { { kDLFloat, 32, 1 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "double[]", { { kDLFloat, 64, 1 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "string[]", { { kDLUInt, 8, 1 }, OVSTAGE_SEMANTIC_STRING, true } },
        { "token[]", { { kDLUInt, 64, 1 }, OVSTAGE_SEMANTIC_TOKEN_ID, true } },
        { "asset[]", { { kDLUInt, 8, 1 }, OVSTAGE_SEMANTIC_ASSET_STRING, true } },
        { "int2[]", { { kDLInt, 32, 2 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "int3[]", { { kDLInt, 32, 3 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "int4[]", { { kDLInt, 32, 4 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "float2[]", { { kDLFloat, 32, 2 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "float3[]", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "float4[]", { { kDLFloat, 32, 4 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "double2[]", { { kDLFloat, 64, 2 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "double3[]", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "double4[]", { { kDLFloat, 64, 4 }, OVSTAGE_SEMANTIC_NONE, true } },
        { "point2f[]", { { kDLFloat, 32, 2 }, OVSTAGE_SEMANTIC_POINT, true } },
        { "point3f[]", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_POINT, true } },
        { "point2d[]", { { kDLFloat, 64, 2 }, OVSTAGE_SEMANTIC_POINT, true } },
        { "point3d[]", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_POINT, true } },
        { "vector2f[]", { { kDLFloat, 32, 2 }, OVSTAGE_SEMANTIC_VECTOR, true } },
        { "vector3f[]", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_VECTOR, true } },
        { "vector2d[]", { { kDLFloat, 64, 2 }, OVSTAGE_SEMANTIC_VECTOR, true } },
        { "vector3d[]", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_VECTOR, true } },
        { "normal3f[]", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_NORMAL, true } },
        { "normal3d[]", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_NORMAL, true } },
        { "color3f[]", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_COLOR, true } },
        { "color3d[]", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_COLOR, true } },
        { "color4f[]", { { kDLFloat, 32, 4 }, OVSTAGE_SEMANTIC_COLOR, true } },
        { "color4d[]", { { kDLFloat, 64, 4 }, OVSTAGE_SEMANTIC_COLOR, true } },
        { "quatf[]", { { kDLFloat, 32, 4 }, OVSTAGE_SEMANTIC_QUATERNION, true } },
        { "quatd[]", { { kDLFloat, 64, 4 }, OVSTAGE_SEMANTIC_QUATERNION, true } },
        { "quath[]", { { kDLFloat, 16, 4 }, OVSTAGE_SEMANTIC_QUATERNION, true } },
        { "texCoord2f[]", { { kDLFloat, 32, 2 }, OVSTAGE_SEMANTIC_TEXTURE_COORDINATE, true } },
        { "texCoord3f[]", { { kDLFloat, 32, 3 }, OVSTAGE_SEMANTIC_TEXTURE_COORDINATE, true } },
        { "texCoord2d[]", { { kDLFloat, 64, 2 }, OVSTAGE_SEMANTIC_TEXTURE_COORDINATE, true } },
        { "texCoord3d[]", { { kDLFloat, 64, 3 }, OVSTAGE_SEMANTIC_TEXTURE_COORDINATE, true } },
        { "matrix2d[]", { { kDLFloat, 64, 4 }, OVSTAGE_SEMANTIC_MATRIX, true } },
        { "matrix3d[]", { { kDLFloat, 64, 9 }, OVSTAGE_SEMANTIC_MATRIX, true } },
        { "matrix4d[]", { { kDLFloat, 64, 16 }, OVSTAGE_SEMANTIC_MATRIX, true } },
    };
    const auto it = s_kMap.find(name);
    return it == s_kMap.end() ? std::nullopt : std::optional<UsdTypeDescriptor>(it->second);
}

struct AttributeTypeInformation
{
    DLDataType dtype{};
    ovstage_attribute_semantic_t semantic{ OVSTAGE_SEMANTIC_NONE };
    bool is_array{ false };
    bool found{ false };
};

static AttributeTypeInformation readAttributeTypeInformation(ovstage_instance_t* instance,
                                                             path_dictionary_instance_t* dictionary,
                                                             const std::string& path,
                                                             const std::string& attributeName)
{
    AttributeTypeInformation information{};

    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList).status != OVX_API_SUCCESS)
        return information;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return information;
    }

    const ovx_string_t attributeString{ attributeName.c_str(), attributeName.size() };
    ovx_token_t attributeToken = OVX_INVALID_TOKEN;
    path_dictionary_create_tokens_from_strings(dictionary, &attributeString, 1, &attributeToken);

    if (attributeToken != OVX_INVALID_TOKEN)
    {
        ovstage_ordinal_range_t range{};
        range.has_start_ordinal = false;
        range.end_ordinal = UINT64_MAX;

        ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
        ovstage_enqueue_result_t enqueueResult =
            ovstage_read_attributes(instance, query, &attributeToken, 1, range, &read);
        if (enqueueResult.status == OVSTAGE_OK)
        {
            details::waitAndRelease(instance, enqueueResult);
            for (;;)
            {
                ovstage_read_group_t group{};
                const ovstage_api_status_t fetched =
                    ovstage_fetch_read_next(instance, read, OVSTAGE_TIMEOUT_INFINITE, &group);
                if (fetched == OVSTAGE_ERROR_END_OF_ITERATION)
                    break;
                if (fetched != OVSTAGE_OK)
                    break;
                if (!group.is_delete && group.data.tensors && group.data.tensor_count > 0)
                {
                    information.dtype = group.data.tensors[0].dtype;
                    information.semantic = group.semantic;
                    information.is_array = group.is_array;
                    information.found = true;
                }
                ovstage_release_group(instance, &group);
            }
            details::waitAndRelease(instance, ovstage_release_read(instance, read));
        }
    }

    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);
    return information;
}

static array::Dtype dlTypeToDtype(DLDataType dt)
{
    switch (dt.code)
    {
    case kDLInt:
        switch (dt.bits)
        {
        case 8:
            return array::Dtype::Int8();
        case 16:
            return array::Dtype::Int16();
        case 32:
            return array::Dtype::Int32();
        case 64:
            return array::Dtype::Int64();
        }
        break;
    case kDLUInt:
        switch (dt.bits)
        {
        case 8:
            return array::Dtype::UInt8();
        case 16:
            return array::Dtype::UInt16();
        case 32:
            return array::Dtype::UInt32();
        case 64:
            return array::Dtype::UInt64();
        }
        break;
    case kDLFloat:
        switch (dt.bits)
        {
        case 32:
            return array::Dtype::Float32();
        case 64:
            return array::Dtype::Float64();
        }
        break;
    }
    throw std::runtime_error("Unsupported DLDataType");
}

// ---- Xform helpers ----

static const char g_kLocalMatrixAttr[] = "omni:xform";
static constexpr DLDataType g_kMatrix4dDType{ kDLFloat, 64, 16 };

static const std::unordered_set<std::string>& xformableTypes()
{
    static const std::unordered_set<std::string> s_kXformableTypes = {
        "Xform",     "Mesh",          "BasisCurves",  "NurbsCurves", "NurbsPatch",  "Points",        "PointInstancer",
        "Camera",    "Sphere",        "Cube",         "Cylinder",    "Cone",        "Capsule",       "Plane",
        "SkelRoot",  "Skeleton",      "DistantLight", "DomeLight",   "SphereLight", "CylinderLight", "DiskLight",
        "RectLight", "GeometryLight", "PortalLight",  "MeshLight",   "PluginLight", "Volume",
    };
    return s_kXformableTypes;
}

static void identityMat(double m[16])
{
    for (int i = 0; i < 16; ++i)
        m[i] = 0.0;
    m[0] = m[5] = m[10] = m[15] = 1.0;
}

// out = a * b  (row-major 4×4)
static void mulMat(const double a[16], const double b[16], double out[16])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
        {
            double s = 0.0;
            for (int k = 0; k < 4; ++k)
                s += a[i * 4 + k] * b[k * 4 + j];
            out[i * 4 + j] = s;
        }
}

// Invert a TRS matrix (row-vector convention, translation in last row).
static bool invertTRS(const double m[16], double inv[16])
{
    double sx2 = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
    double sy2 = m[4] * m[4] + m[5] * m[5] + m[6] * m[6];
    double sz2 = m[8] * m[8] + m[9] * m[9] + m[10] * m[10];
    if (sx2 < 1e-30 || sy2 < 1e-30 || sz2 < 1e-30)
        return false;
    double isx2 = 1.0 / sx2, isy2 = 1.0 / sy2, isz2 = 1.0 / sz2;

    // (S*R)^-1 = R^T * S^-1; in row-vector: (R^T*S^-1)[i][j] = matrix[j][i]/sj^2
    double i00 = m[0] * isx2, i01 = m[4] * isy2, i02 = m[8] * isz2;
    double i10 = m[1] * isx2, i11 = m[5] * isy2, i12 = m[9] * isz2;
    double i20 = m[2] * isx2, i21 = m[6] * isy2, i22 = m[10] * isz2;
    double tx = m[12], ty = m[13], tz = m[14];

    inv[0] = i00;
    inv[1] = i01;
    inv[2] = i02;
    inv[3] = 0.0;
    inv[4] = i10;
    inv[5] = i11;
    inv[6] = i12;
    inv[7] = 0.0;
    inv[8] = i20;
    inv[9] = i21;
    inv[10] = i22;
    inv[11] = 0.0;
    inv[12] = -(tx * i00 + ty * i10 + tz * i20);
    inv[13] = -(tx * i01 + ty * i11 + tz * i21);
    inv[14] = -(tx * i02 + ty * i12 + tz * i22);
    inv[15] = 1.0;
    return true;
}

static void extractScale(const double m[16], double s[3])
{
    s[0] = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    s[1] = std::sqrt(m[4] * m[4] + m[5] * m[5] + m[6] * m[6]);
    s[2] = std::sqrt(m[8] * m[8] + m[9] * m[9] + m[10] * m[10]);
}

// Rotation matrix (row-vector convention) → quaternion xyzw.
// r{i}{j} = R_row[i][j] = R_col[j][i].
static void convertRotationToQuaternion(
    double r00, double r01, double r02, double r10, double r11, double r12, double r20, double r21, double r22, double q[4])
{
    double trace = r00 + r11 + r22;
    if (trace > 0.0)
    {
        double s = 0.5 / std::sqrt(trace + 1.0);
        q[0] = (r12 - r21) * s;
        q[1] = (r20 - r02) * s;
        q[2] = (r01 - r10) * s;
        q[3] = 0.25 / s;
    }
    else if (r00 > r11 && r00 > r22)
    {
        double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
        q[0] = 0.25 * s;
        q[1] = (r10 + r01) / s;
        q[2] = (r20 + r02) / s;
        q[3] = (r12 - r21) / s;
    }
    else if (r11 > r22)
    {
        double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
        q[0] = (r10 + r01) / s;
        q[1] = 0.25 * s;
        q[2] = (r21 + r12) / s;
        q[3] = (r20 - r02) / s;
    }
    else
    {
        double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
        q[0] = (r20 + r02) / s;
        q[1] = (r21 + r12) / s;
        q[2] = 0.25 * s;
        q[3] = (r01 - r10) / s;
    }
    double len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (len > 1e-15)
    {
        q[0] /= len;
        q[1] /= len;
        q[2] /= len;
        q[3] /= len;
    }
}

static void extractPose(const double m[16], const double s[3], double t[3], double q[4])
{
    t[0] = m[12];
    t[1] = m[13];
    t[2] = m[14];
    double r00 = (s[0] > 1e-15) ? m[0] / s[0] : 1.0;
    double r01 = (s[0] > 1e-15) ? m[1] / s[0] : 0.0;
    double r02 = (s[0] > 1e-15) ? m[2] / s[0] : 0.0;
    double r10 = (s[1] > 1e-15) ? m[4] / s[1] : 0.0;
    double r11 = (s[1] > 1e-15) ? m[5] / s[1] : 1.0;
    double r12 = (s[1] > 1e-15) ? m[6] / s[1] : 0.0;
    double r20 = (s[2] > 1e-15) ? m[8] / s[2] : 0.0;
    double r21 = (s[2] > 1e-15) ? m[9] / s[2] : 0.0;
    double r22 = (s[2] > 1e-15) ? m[10] / s[2] : 1.0;
    convertRotationToQuaternion(r00, r01, r02, r10, r11, r12, r20, r21, r22, q);
}

// Build TRS matrix from translation, quaternion xyzw, and scale.
static void buildMatrix(const double t[3], const double q[4], const double s[3], double m[16])
{
    double x = q[0], y = q[1], z = q[2], w = q[3];
    // Row-vector convention: R_row = R_col^T
    double r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y + w * z), r02 = 2 * (x * z - w * y);
    double r10 = 2 * (x * y - w * z), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z + w * x);
    double r20 = 2 * (x * z + w * y), r21 = 2 * (y * z - w * x), r22 = 1 - 2 * (x * x + y * y);
    m[0] = s[0] * r00;
    m[1] = s[0] * r01;
    m[2] = s[0] * r02;
    m[3] = 0.0;
    m[4] = s[1] * r10;
    m[5] = s[1] * r11;
    m[6] = s[1] * r12;
    m[7] = 0.0;
    m[8] = s[2] * r20;
    m[9] = s[2] * r21;
    m[10] = s[2] * r22;
    m[11] = 0.0;
    m[12] = t[0];
    m[13] = t[1];
    m[14] = t[2];
    m[15] = 1.0;
}

// Read omni:xform for one path; fills mat with identity if not set.
static void readLocalMatrix(ovstage_instance_t* instance,
                            path_dictionary_instance_t* dictionary,
                            const std::string& path,
                            double mat[16])
{
    identityMat(mat);
    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList).status != OVX_API_SUCCESS)
        return;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return;
    }
    const ovx_string_t attributeString{ g_kLocalMatrixAttr, sizeof(g_kLocalMatrixAttr) - 1 };
    ovx_token_t token = OVX_INVALID_TOKEN;
    path_dictionary_create_tokens_from_strings(dictionary, &attributeString, 1, &token);
    if (token != OVX_INVALID_TOKEN)
    {
        ovstage_ordinal_range_t range{};
        range.has_start_ordinal = false;
        range.end_ordinal = UINT64_MAX;
        ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
        ovstage_enqueue_result_t enqueueResult = ovstage_read_attributes(instance, query, &token, 1, range, &read);
        if (enqueueResult.status == OVSTAGE_OK)
        {
            details::waitAndRelease(instance, enqueueResult);
            for (;;)
            {
                ovstage_read_group_t group{};
                if (ovstage_fetch_read_next(instance, read, OVSTAGE_TIMEOUT_INFINITE, &group) != OVSTAGE_OK)
                    break;
                if (!group.is_delete && group.data.tensors && group.data.tensor_count >= 1)
                {
                    const DLTensor& t = group.data.tensors[0];
                    if (t.data && t.ndim == 1 && t.shape && t.shape[0] >= 1 && t.dtype.code == kDLFloat &&
                        t.dtype.bits == 64 && t.dtype.lanes == 16)
                        std::memcpy(mat, t.data, 16 * sizeof(double));
                }
                ovstage_release_group(instance, &group);
            }
            details::waitAndRelease(instance, ovstage_release_read(instance, read));
        }
    }
    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);
}

// Read omni:xform for pathCount paths; returns pathCount matrices (identity for missing ones).
static std::vector<std::array<double, 16>> readLocalMatrices(ovstage_instance_t* instance,
                                                             path_dictionary_instance_t* dictionary,
                                                             const std::vector<std::string>& paths)
{
    const size_t pathCount = paths.size();
    std::vector<std::array<double, 16>> result(pathCount);
    for (auto& m : result)
        identityMat(m.data());

    std::vector<ovx_string_t> pathStrings(pathCount);
    for (size_t i = 0; i < pathCount; ++i)
        pathStrings[i] = { paths[i].c_str(), paths[i].size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, pathStrings.data(), pathCount, &pathList).status !=
        OVX_API_SUCCESS)
        return result;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return result;
    }
    const ovx_string_t attributeString{ g_kLocalMatrixAttr, sizeof(g_kLocalMatrixAttr) - 1 };
    ovx_token_t token = OVX_INVALID_TOKEN;
    path_dictionary_create_tokens_from_strings(dictionary, &attributeString, 1, &token);
    if (token != OVX_INVALID_TOKEN)
    {
        ovstage_ordinal_range_t range{};
        range.has_start_ordinal = false;
        range.end_ordinal = UINT64_MAX;
        ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
        ovstage_enqueue_result_t enqueueResult = ovstage_read_attributes(instance, query, &token, 1, range, &read);
        if (enqueueResult.status == OVSTAGE_OK)
        {
            details::waitAndRelease(instance, enqueueResult);
            for (;;)
            {
                ovstage_read_group_t group{};
                if (ovstage_fetch_read_next(instance, read, OVSTAGE_TIMEOUT_INFINITE, &group) != OVSTAGE_OK)
                    break;
                if (!group.is_delete && !group.is_array && !group.data.mask && group.data.tensors &&
                    group.data.tensor_count >= 1)
                {
                    const DLTensor& t = group.data.tensors[0];
                    if (t.data && t.shape && t.ndim == 1 && t.dtype.code == kDLFloat && t.dtype.bits == 64 &&
                        t.dtype.lanes == 16)
                    {
                        const double* source = static_cast<const double*>(t.data);
                        for (size_t k = 0; k < group.prims.count; ++k)
                        {
                            size_t outputIndex = group.prims.index_map ? static_cast<size_t>(group.prims.index_map[k]) :
                                                                         (group.prims.offset + k);
                            size_t dataIndex = group.data.index_map ? static_cast<size_t>(group.data.index_map[k]) : k;
                            if (outputIndex < pathCount && static_cast<int64_t>(dataIndex) < t.shape[0])
                                std::memcpy(result[outputIndex].data(), source + dataIndex * 16, 16 * sizeof(double));
                        }
                    }
                }
                ovstage_release_group(instance, &group);
            }
            details::waitAndRelease(instance, ovstage_release_read(instance, read));
        }
    }
    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);
    return result;
}

// Write omni:xform matrices for pathCount paths at ordinal (does NOT advance write floor).
static void writeLocalMatrices(ovstage_instance_t* instance,
                               path_dictionary_instance_t* dictionary,
                               ovstage_ordinal_t ordinal,
                               const std::vector<std::string>& paths,
                               const std::vector<std::array<double, 16>>& matrices)
{
    const size_t pathCount = paths.size();
    std::vector<ovx_string_t> pathStrings(pathCount);
    for (size_t i = 0; i < pathCount; ++i)
        pathStrings[i] = { paths[i].c_str(), paths[i].size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, pathStrings.data(), pathCount, &pathList).status !=
        OVX_API_SUCCESS)
        return;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return;
    }
    std::vector<double> flat(pathCount * 16);
    for (size_t i = 0; i < pathCount; ++i)
        std::memcpy(flat.data() + i * 16, matrices[i].data(), 16 * sizeof(double));
    int64_t shape = static_cast<int64_t>(pathCount);
    DLTensor tensor{};
    tensor.data = flat.data();
    tensor.device = { kDLCPU, 0 };
    tensor.ndim = 1;
    tensor.dtype = g_kMatrix4dDType;
    tensor.shape = &shape;
    ovstage_write_data_t write{};
    write.tensors = &tensor;
    write.tensor_count = 1;
    write.semantic = OVSTAGE_SEMANTIC_MATRIX;
    write.is_array = false;
    const ovx_string_or_token_t attributeArgument{ OVX_INVALID_TOKEN,
                                                   { g_kLocalMatrixAttr, sizeof(g_kLocalMatrixAttr) - 1 } };
    details::waitAndRelease(
        instance, ovstage_write_attribute(instance, query, attributeArgument, ordinal, write, OVSTAGE_PRIM_MODE_UPSERT));
    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);
}

// Build ancestor chain [root_child, ..., parent, path], excluding "/".
static std::vector<std::string> ancestorChain(const std::string& path)
{
    std::vector<std::string> chain;
    std::string cur = path;
    while (!cur.empty() && cur != "/")
    {
        chain.push_back(cur);
        size_t pos = cur.rfind('/');
        if (pos == 0)
            break;
        cur = cur.substr(0, pos);
    }
    std::reverse(chain.begin(), chain.end());
    return chain;
}

// W = L_self * L_parent * ...  (row-vector convention; accumulate by prepending each level).
static std::array<double, 16> computeWorldMatrix(ovstage_instance_t* instance,
                                                 path_dictionary_instance_t* dictionary,
                                                 const std::string& path)
{
    const auto chain = ancestorChain(path);
    std::array<double, 16> world;
    identityMat(world.data());
    for (const auto& p : chain)
    {
        double local[16];
        readLocalMatrix(instance, dictionary, p, local);
        double temp[16];
        mulMat(local, world.data(), temp);
        std::memcpy(world.data(), temp, 16 * sizeof(double));
    }
    return world;
}

static std::string parentPath(const std::string& path)
{
    if (path.empty() || path == "/")
        return "";
    size_t pos = path.rfind('/');
    if (pos == 0)
        return "/";
    return path.substr(0, pos);
}

static void validateXformable(int64_t stageId, const std::string& path)
{
    details::validatePrimAtPath(stageId, path, true);
    const std::string type = getTypeName(stageId, path);
    if (!xformableTypes().count(type))
        throw std::invalid_argument("Prim '" + path + "' (type '" + type + "') is not Xformable");
}

static void sealOrdinal(ovstage_instance_t* instance, ovstage_ordinal_t ordinal)
{
    ovstage_write_floor_desc_t floorDescription{};
    floorDescription.ordinal = ordinal;
    floorDescription.scope = OVSTAGE_SCOPE_ALL;
    details::waitAndRelease(instance, ovstage_advance_write_floor(instance, &floorDescription));
}

} // namespace

int64_t createStage()
{
    ovstage_instance_desc_t desc{};
    ovstage_instance_t* instance = nullptr;
    if (ovstage_create_instance(&desc, &instance) != OVSTAGE_OK)
    {
        return -1;
    }
    return details::registerInstance(instance);
}

int64_t openStage(const std::string& usdPath)
{
    ovstage_instance_desc_t desc{};
    ovstage_instance_t* instance = nullptr;
    if (ovstage_create_instance(&desc, &instance) != OVSTAGE_OK)
    {
        return -1;
    }

    const ovx_string_t path{ usdPath.c_str(), usdPath.size() };
    const ovstage_population_enqueue_result_t populationEnqueue = ovstage_population_open_usd_from_file(
        instance, path, /*ordinal=*/1, /*time=*/0.0, OVSTAGE_POPULATION_DOMAIN_ALL);
    if (populationEnqueue.status != OVSTAGE_OK ||
        ovstage_population_wait_op(instance, populationEnqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr) != OVSTAGE_OK)
    {
        ovstage_destroy_instance(instance);
        return -1;
    }

    sealOrdinal(instance, /*ordinal=*/1);
    return details::registerInstance(instance, /*nextOrdinal=*/2);
}

bool saveStage(int64_t stageId, const std::string& usdPath)
{
    (void)stageId;
    (void)usdPath;
    throw std::logic_error("The ovstage library does not support saving a stage to persistent storage");
}

bool closeStage(int64_t stageId)
{
    ovstage_instance_t* instance = details::unregisterInstance(stageId);
    if (!instance)
    {
        return false;
    }
    ovstage_destroy_instance(instance);
    return true;
}

bool isStageValid(int64_t stageId)
{
    return details::getInstance(stageId, false) != nullptr;
}

std::string exportStageToString(int64_t stageId)
{
    (void)stageId;
    throw std::logic_error("The ovstage library does not support serializing a stage to a string");
}

int64_t importStageFromString(const std::string& usdString)
{
    ovstage_instance_desc_t desc{};
    ovstage_instance_t* instance = nullptr;
    if (ovstage_create_instance(&desc, &instance) != OVSTAGE_OK)
    {
        return -1;
    }

    const ovx_string_t usda{ usdString.c_str(), usdString.size() };
    const ovstage_population_enqueue_result_t populationEnqueue = ovstage_population_open_usd_from_string(
        instance, usda, /*ordinal=*/1, /*time=*/0.0, OVSTAGE_POPULATION_DOMAIN_ALL);
    if (populationEnqueue.status != OVSTAGE_OK ||
        ovstage_population_wait_op(instance, populationEnqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr) != OVSTAGE_OK)
    {
        ovstage_destroy_instance(instance);
        return -1;
    }

    sealOrdinal(instance, /*ordinal=*/1);
    return details::registerInstance(instance, /*nextOrdinal=*/2);
}

void* getStagePtr(int64_t stageId)
{
    return details::getInstance(stageId, false);
}

void definePrim(int64_t stageId, const std::string& path, const std::string& typeName)
{
    if (!isValidPathString(path) || path[0] != '/')
        throw isaacsim::common::exceptions::PrimPathStringError(path);

    ovstage_instance_t* instance = details::getInstance(stageId, false);
    if (!instance)
        return;

    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);
    if (!dictionary)
        return;

    // Throw if prim already exists with a different type
    const std::optional<std::string> existing = readPrimType(instance, dictionary, path);
    if (existing.has_value() && *existing != typeName)
        throw std::runtime_error("Prim at path (" + path + ") already exists with type '" + *existing + "'");

    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);
    if (ordinal == 0)
        return;

    // Create any ancestor prims that don't already exist (with untyped sentinel).
    // All ancestors and the target share the same ordinal; one floor advance seals them all.
    for (size_t pos = 1; (pos = path.find('/', pos)) != std::string::npos; ++pos)
    {
        const std::string ancestor = path.substr(0, pos);
        if (!readPrimType(instance, dictionary, ancestor).has_value())
            writePrimTypeToPath(instance, dictionary, ancestor, g_kUntypedSentinel, ordinal);
    }

    // Write the target prim.
    const std::string effectiveTypeName = typeName.empty() ? g_kUntypedSentinel : typeName;
    writePrimTypeToPath(instance, dictionary, path, effectiveTypeName, ordinal);

    // Seal the ordinal so all new prims are immediately visible to readers.
    ovstage_write_floor_desc_t floorDescription{};
    floorDescription.ordinal = ordinal;
    floorDescription.scope = OVSTAGE_SCOPE_ALL;
    details::waitAndRelease(instance, ovstage_advance_write_floor(instance, &floorDescription));
}

std::tuple<bool, std::string> movePrim(int64_t stageId, const std::string& targetPath, const std::string& destinationPath)
{
    (void)stageId;
    (void)targetPath;
    (void)destinationPath;
    throw std::logic_error("The ovstage library does not support authoring (move) prims");
}

bool removePrim(int64_t stageId, const std::string& path)
{
    details::validatePrimAtPath(stageId, path, true);

    ovstage_instance_t* instance = details::getInstance(stageId, false);
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);
    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);

    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList);

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_query_from_path_list(instance, pathList, &query);

    // Passing nullptr + count 0 tombstones the entire prim (all its attributes).
    const ovstage_enqueue_result_t deleteEnqueueResult = ovstage_delete_attributes(instance, query, nullptr, 0, ordinal);
    const bool ok = deleteEnqueueResult.status == OVSTAGE_OK;
    details::waitAndRelease(instance, deleteEnqueueResult);

    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);

    if (!ok)
        return false;

    ovstage_write_floor_desc_t floorDescription{};
    floorDescription.ordinal = ordinal;
    floorDescription.scope = OVSTAGE_SCOPE_ALL;
    details::waitAndRelease(instance, ovstage_advance_write_floor(instance, &floorDescription));
    return true;
}

void traversePrim(int64_t stageId, const std::string& path, std::function<bool(const std::string&)> callback)
{
    if (!callback(path))
    {
        return;
    }
    for (const std::string& child : getChildren(stageId, path))
    {
        traversePrim(stageId, child, callback);
    }
}

std::tuple<float, float> getStageUnits(int64_t stageId)
{
    (void)stageId;
    throw std::logic_error("The ovstage library does not support authoring (get) stage units");
}

void setStageUnits(int64_t stageId, std::optional<float> metersPerUnit, std::optional<float> kilogramsPerUnit)
{
    (void)stageId;
    (void)metersPerUnit;
    (void)kilogramsPerUnit;
    throw std::logic_error("The ovstage library does not support authoring (set) stage units");
}

std::string getStageUpAxis(int64_t stageId)
{
    (void)stageId;
    throw std::logic_error("The ovstage library does not support authoring (get) stage up-axis");
}

void setStageUpAxis(int64_t stageId, const std::string& upAxis)
{
    (void)stageId;
    (void)upAxis;
    throw std::logic_error("The ovstage library does not support authoring (set) stage up-axis");
}

std::tuple<float, float, float> getStageTimeCode(int64_t stageId)
{
    (void)stageId;
    throw std::logic_error("The ovstage library does not support authoring (get) stage time code");
}

void setStageTimeCode(int64_t stageId,
                      std::optional<float> startTimeCode,
                      std::optional<float> endTimeCode,
                      std::optional<float> timeCodesPerSecond)
{
    (void)stageId;
    (void)startTimeCode;
    (void)endTimeCode;
    (void)timeCodesPerSecond;
    throw std::logic_error("The ovstage library does not support authoring (set) stage time code");
}

void addReferenceToStage(int64_t stageId, const std::string& path, const std::string& usdPath, const std::string& typeName)
{
    (void)stageId;
    (void)path;
    (void)usdPath;
    (void)typeName;
    throw std::logic_error("The ovstage library does not support authoring (add) references to stage");
}

std::string generateStageRepresentation(int64_t stageId, const std::string& mode)
{
    std::ostringstream oss;
    bool first = true;

    if (mode == "list")
    {
        traversePrim(stageId, "/",
                     [&](const std::string& path) -> bool
                     {
                         if (path == "/")
                             return true;
                         if (!first)
                             oss << "\n";
                         oss << path << " (" << getTypeName(stageId, path) << ")";
                         first = false;
                         return true;
                     });
    }
    else if (mode == "tree")
    {
        std::function<void(const std::string&, int)> generateTree;
        generateTree = [&](const std::string& path, int indent)
        {
            std::string prefix;
            if (indent > 0)
            {
                for (int i = 0; i < indent - 1; ++i)
                    prefix += "│  ";
                prefix += "├─ ";
            }
            const std::string name = (path == "/") ? "/" : path.substr(path.rfind('/') + 1);
            if (!first)
                oss << "\n";
            oss << prefix << name << " (" << getTypeName(stageId, path) << ")";
            first = false;
            for (const std::string& child : getChildren(stageId, path))
                generateTree(child, indent + 1);
        };
        generateTree("/", 0);
    }
    else
    {
        throw std::invalid_argument("Invalid mode: '" + mode + "'. Valid modes are: 'list', 'tree'");
    }

    return oss.str();
}

bool isValidPathString(const std::string& path)
{
    return details::isValidPathString(path);
}

bool isPrimValid(int64_t stageId, const std::string& path)
{
    auto instance = details::getInstance(stageId, true);
    if (!details::isValidPathString(path))
    {
        throw isaacsim::common::exceptions::PrimPathStringError(path);
    }
    return details::validatePrimAtPath(instance, path);
}

std::string getName(int64_t stageId, const std::string& path)
{
    details::validatePrimAtPath(stageId, path, true);
    if (path == "/")
    {
        return "";
    }
    const size_t pos = path.rfind('/');
    return path.substr(pos + 1);
}

std::string getTypeName(int64_t stageId, const std::string& path)
{
    auto instance = details::getInstance(stageId, true);
    details::validatePrimAtPath(instance, path, true);
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);
    if (!dictionary)
    {
        return {};
    }
    return readPrimType(instance, dictionary, path).value_or("");
}

std::string getParent(int64_t stageId, const std::string& path)
{
    details::validatePrimAtPath(stageId, path, true);
    if (path == "/")
    {
        return "";
    }
    const size_t pos = path.rfind('/');
    return (pos == 0) ? "/" : path.substr(0, pos);
}

std::vector<std::string> getChildren(int64_t stageId, const std::string& path)
{
    ovstage_instance_t* instance = details::getInstance(stageId, true);
    details::validatePrimAtPath(instance, path, true);

    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);
    if (!dictionary)
        return {};

    if (path == "/")
    {
        static constexpr char s_kUsdPathAttr[] = "usd-path";
        const ovx_string_t prefixValue{ "/", 1 };
        ovstage_predicate_t pred{};
        pred.attribute = { OVX_INVALID_TOKEN, ovx_string_t{ s_kUsdPathAttr, sizeof(s_kUsdPathAttr) - 1 } };
        pred.op = OVSTAGE_FILTER_OP_PREFIX;
        pred.values = &prefixValue;
        pred.value_count = 1;
        ovstage_filter_t filter{};
        filter.predicates = &pred;
        filter.count = 1;

        const ovx_string_t primTypeString{ g_kPrimTypeAttr, sizeof(g_kPrimTypeAttr) - 1 };
        ovx_token_t primTypeToken = OVX_INVALID_TOKEN;
        path_dictionary_create_tokens_from_strings(dictionary, &primTypeString, 1, &primTypeToken);

        ovstage_query_handle_t filterQuery = OVSTAGE_INVALID_QUERY_HANDLE;
        ovstage_enqueue_result_t enqueueResult = ovstage_query(instance, &filter, nullptr, 0, &filterQuery);
        if (enqueueResult.status != OVSTAGE_OK || primTypeToken == OVX_INVALID_TOKEN)
            return {};
        details::waitAndRelease(instance, enqueueResult);

        ovstage_ordinal_range_t range{};
        range.end_ordinal = UINT64_MAX;
        range.has_start_ordinal = false;
        ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
        enqueueResult = ovstage_read_attributes(instance, filterQuery, &primTypeToken, 1, range, &read);
        if (enqueueResult.status != OVSTAGE_OK)
        {
            details::waitAndRelease(instance, ovstage_release_query(instance, filterQuery));
            return {};
        }
        details::waitAndRelease(instance, enqueueResult);

        std::vector<std::string> children;
        for (;;)
        {
            ovstage_read_group_t group{};
            const ovstage_api_status_t fetched =
                ovstage_fetch_read_next(instance, read, OVSTAGE_TIMEOUT_INFINITE, &group);
            if (fetched == OVSTAGE_ERROR_END_OF_ITERATION)
                break;
            if (fetched != OVSTAGE_OK)
                break;
            if (!group.is_delete && group.prims.index_map == nullptr)
            {
                std::vector<ovx_primpath_t> primPaths(group.prims.count);
                size_t retrievedCount = 0;
                if (path_dictionary_get_paths_from_path_list(dictionary, group.prims.list, group.prims.offset,
                                                             group.prims.count, primPaths.data(), &retrievedCount)
                        .status == OVX_API_SUCCESS)
                {
                    for (size_t i = 0; i < retrievedCount; ++i)
                    {
                        std::string pathString = details::primPathToString(dictionary, primPaths[i]);
                        // Keep only depth-1 paths (no "/" after position 0).
                        if (!pathString.empty() && pathString.find('/', 1) == std::string::npos)
                            children.push_back(std::move(pathString));
                    }
                }
            }
            ovstage_release_group(instance, &group);
        }
        details::waitAndRelease(instance, ovstage_release_read(instance, read));
        details::waitAndRelease(instance, ovstage_release_query(instance, filterQuery));
        return children;
    }

    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList).status != OVX_API_SUCCESS)
    {
        return {};
    }

    ovstage_hierarchy_handle_t handle = OVSTAGE_INVALID_HIERARCHY_HANDLE;
    ovstage_enqueue_result_t enqueueResult =
        ovstage_get_hierarchy(instance, pathList, UINT64_MAX, OVSTAGE_HIERARCHY_CHILDREN, &handle);
    details::waitAndRelease(instance, enqueueResult);

    std::vector<std::string> children;
    if (enqueueResult.status == OVSTAGE_OK)
    {
        ovstage_hierarchy_result_t result{};
        if (ovstage_fetch_hierarchy_result(instance, handle, &result) == OVSTAGE_OK)
        {
            if (result.input_count >= 1 && result.items && result.items[0].status == OVSTAGE_OK)
            {
                const size_t pathCount = result.items[0].path_count;
                if (pathCount > 0 && result.paths)
                {
                    children.reserve(pathCount);
                    for (size_t i = 0; i < pathCount; ++i)
                    {
                        const ovx_string_t& p = result.paths[result.items[0].path_offset + i].string;
                        children.emplace_back(p.ptr ? std::string(p.ptr, p.length) : std::string{});
                    }
                }
            }
            ovstage_release_hierarchy_result(instance, &result);
        }
        details::waitAndRelease(instance, ovstage_release_hierarchy(instance, handle));
    }

    path_dictionary_release_path_list_reference(dictionary, pathList);
    return children;
}

bool isA(int64_t stageId, const std::string& path, const std::string& schemaType)
{
    return getTypeName(stageId, path) == schemaType;
}

bool hasApi(int64_t stageId,
            const std::string& path,
            const std::string& schemaType,
            const std::optional<std::string>& instanceName)
{
    details::validatePrimAtPath(stageId, path, true);
    const std::string schemaId = instanceName ? schemaType + ":" + *instanceName : schemaType;
    const std::vector<std::string> current = getAppliedSchemas(stageId, path);
    return std::find(current.begin(), current.end(), schemaId) != current.end();
}

bool applyApi(int64_t stageId,
              const std::string& path,
              const std::string& schemaType,
              const std::optional<std::string>& instanceName)
{
    ovstage_instance_t* instance = details::getInstance(stageId, true);
    details::validatePrimAtPath(instance, path, true);

    const std::string schemaId = instanceName ? schemaType + ":" + *instanceName : schemaType;
    std::vector<std::string> current = getAppliedSchemas(stageId, path);
    if (std::find(current.begin(), current.end(), schemaId) != current.end())
        return true;
    current.push_back(schemaId);

    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);
    if (!dictionary)
        return false;

    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);
    if (ordinal == 0)
        return false;

    writeSchemasToPath(instance, dictionary, path, current, ordinal);

    ovstage_write_floor_desc_t floorDescription{};
    floorDescription.ordinal = ordinal;
    floorDescription.scope = OVSTAGE_SCOPE_ALL;
    details::waitAndRelease(instance, ovstage_advance_write_floor(instance, &floorDescription));
    return true;
}

bool removeApi(int64_t stageId,
               const std::string& path,
               const std::string& schemaType,
               const std::optional<std::string>& instanceName)
{
    ovstage_instance_t* instance = details::getInstance(stageId, true);
    details::validatePrimAtPath(instance, path, true);

    const std::string schemaId = instanceName ? schemaType + ":" + *instanceName : schemaType;
    std::vector<std::string> current = getAppliedSchemas(stageId, path);
    const auto it = std::find(current.begin(), current.end(), schemaId);
    if (it == current.end())
        return false;
    current.erase(it);

    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);
    if (!dictionary)
        return false;

    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);
    if (ordinal == 0)
        return false;

    writeSchemasToPath(instance, dictionary, path, current, ordinal);

    ovstage_write_floor_desc_t floorDescription{};
    floorDescription.ordinal = ordinal;
    floorDescription.scope = OVSTAGE_SCOPE_ALL;
    details::waitAndRelease(instance, ovstage_advance_write_floor(instance, &floorDescription));
    return true;
}

std::vector<std::string> getAppliedSchemas(int64_t stageId, const std::string& path)
{
    ovstage_instance_t* instance = details::getInstance(stageId, true);
    details::validatePrimAtPath(instance, path, true);

    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);
    if (!dictionary)
        return {};

    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList).status != OVX_API_SUCCESS)
        return {};

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return {};
    }

    const ovx_string_t attributeString{ g_kSchemasAttr, sizeof(g_kSchemasAttr) - 1 };
    ovx_token_t attributeToken = OVX_INVALID_TOKEN;
    path_dictionary_create_tokens_from_strings(dictionary, &attributeString, 1, &attributeToken);

    // Each write of usd-schemas is a whole-set replacement; keep only the last
    // non-delete group so successive ordinal writes don't accumulate duplicates.
    std::vector<std::string> schemas;
    if (attributeToken != OVX_INVALID_TOKEN)
    {
        ovstage_ordinal_range_t range{};
        range.end_ordinal = UINT64_MAX;
        range.has_start_ordinal = false;

        ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
        ovstage_enqueue_result_t enqueueResult =
            ovstage_read_attributes(instance, query, &attributeToken, 1, range, &read);
        if (enqueueResult.status == OVSTAGE_OK)
        {
            details::waitAndRelease(instance, enqueueResult);
            for (;;)
            {
                ovstage_read_group_t group{};
                const ovstage_api_status_t fetched =
                    ovstage_fetch_read_next(instance, read, OVSTAGE_TIMEOUT_INFINITE, &group);
                if (fetched == OVSTAGE_ERROR_END_OF_ITERATION)
                    break;
                if (fetched != OVSTAGE_OK)
                    break;
                if (!group.is_delete && group.is_array && group.prims.count >= 1 && group.data.tensors && !group.data.mask)
                {
                    const uint32_t row = group.data.index_map ? group.data.index_map[0] : 0;
                    if (row < group.data.tensor_count)
                    {
                        const DLTensor& tensor = group.data.tensors[row];
                        if (tensor.data && tensor.ndim == 1 && tensor.shape && tensor.dtype.code == kDLUInt &&
                            tensor.dtype.bits == 64 && tensor.dtype.lanes == 1)
                        {
                            // Overwrite on each valid group: last write wins (whole-set semantics).
                            schemas.clear();
                            schemas.reserve(static_cast<size_t>(tensor.shape[0]));
                            const auto* ids = static_cast<const uint64_t*>(tensor.data);
                            for (int64_t i = 0; i < tensor.shape[0]; ++i)
                            {
                                const ovx_token_t token = static_cast<ovx_token_t>(ids[i]);
                                ovx_string_t name{};
                                if (path_dictionary_get_strings_from_tokens(dictionary, &token, 1, &name).status ==
                                        OVX_API_SUCCESS &&
                                    name.ptr)
                                {
                                    schemas.emplace_back(name.ptr, name.length);
                                }
                            }
                        }
                    }
                }
                else if (group.is_delete)
                {
                    schemas.clear();
                }
                ovstage_release_group(instance, &group);
            }
            details::waitAndRelease(instance, ovstage_release_read(instance, read));
        }
    }

    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);
    return schemas;
}

std::unordered_map<std::string, std::vector<std::string>> getVariants(int64_t stageId,
                                                                      const std::string& path,
                                                                      bool selection)
{
    (void)stageId;
    (void)path;
    (void)selection;
    throw std::logic_error("The ovstage library does not support authoring (get) variants");
}

void setVariants(int64_t stageId,
                 const std::string& path,
                 const std::unordered_map<std::string, std::vector<std::string>>& variants)
{
    (void)stageId;
    (void)path;
    (void)variants;
    throw std::logic_error("The ovstage library does not support authoring (set) variants");
}

bool createPrimAttribute(int64_t stageId,
                         const std::string& path,
                         const std::string& attributeName,
                         const std::string& typeName)
{
    details::validatePrimAtPath(stageId, path, true);
    const auto desc = parseUsdTypeName(typeName);
    if (!desc)
        throw std::invalid_argument("Unknown USD type name: '" + typeName + "'");

    ovstage_instance_t* instance = details::getInstance(stageId, false);
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);

    const AttributeTypeInformation existing = readAttributeTypeInformation(instance, dictionary, path, attributeName);
    if (existing.found)
    {
        if (existing.dtype.code != desc->dtype.code || existing.dtype.bits != desc->dtype.bits ||
            existing.dtype.lanes != desc->dtype.lanes || existing.is_array != desc->is_array)
            throw std::invalid_argument("Attribute '" + attributeName +
                                        "' already exists with a different type on prim '" + path + "'");
        return true;
    }

    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);
    if (ordinal == 0)
        return false;

    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList).status != OVX_API_SUCCESS)
        return false;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return false;
    }

    // Write a zero-initialized element to create the column with the correct dtype/semantic.
    const size_t elementByteCount = static_cast<size_t>(desc->dtype.bits / 8) * desc->dtype.lanes;
    std::vector<std::byte> zeroData(elementByteCount, std::byte{ 0 });
    int64_t oneShape = 1;
    int64_t zeroShape = 0;

    DLTensor tensor{};
    tensor.device = { kDLCPU, 0 };
    tensor.ndim = 1;
    tensor.dtype = desc->dtype;
    if (!desc->is_array)
    {
        tensor.data = zeroData.data();
        tensor.shape = &oneShape;
    }
    else
    {
        tensor.data = nullptr;
        tensor.shape = &zeroShape;
    }

    ovstage_write_data_t write{};
    write.tensors = &tensor;
    write.tensor_count = 1;
    write.semantic = desc->semantic;
    write.is_array = desc->is_array;

    const ovx_string_or_token_t attributeArgument{ OVX_INVALID_TOKEN, { attributeName.c_str(), attributeName.size() } };
    const ovstage_enqueue_result_t writeEnqueueResult =
        ovstage_write_attribute(instance, query, attributeArgument, ordinal, write, OVSTAGE_PRIM_MODE_UPSERT);
    const bool ok = writeEnqueueResult.status == OVSTAGE_OK;
    details::waitAndRelease(instance, writeEnqueueResult);

    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);

    ovstage_write_floor_desc_t floorDescription{};
    floorDescription.ordinal = ordinal;
    floorDescription.scope = OVSTAGE_SCOPE_ALL;
    details::waitAndRelease(instance, ovstage_advance_write_floor(instance, &floorDescription));
    return ok;
}

bool removePrimAttribute(int64_t stageId, const std::string& path, const std::string& attributeName)
{
    details::validatePrimAtPath(stageId, path, true);

    ovstage_instance_t* instance = details::getInstance(stageId, false);
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);

    const AttributeTypeInformation existing = readAttributeTypeInformation(instance, dictionary, path, attributeName);
    if (!existing.found)
        throw isaacsim::common::exceptions::AttributeNameError(attributeName);

    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);
    if (ordinal == 0)
        return false;

    const ovx_string_t pathString{ path.c_str(), path.size() };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList).status != OVX_API_SUCCESS)
        return false;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(instance, pathList, &query) != OVSTAGE_OK)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return false;
    }

    const ovx_string_or_token_t attributeArgument{ OVX_INVALID_TOKEN, { attributeName.c_str(), attributeName.size() } };
    const ovstage_enqueue_result_t deleteEnqueueResult =
        ovstage_delete_attributes(instance, query, &attributeArgument, 1, ordinal);
    const bool ok = deleteEnqueueResult.status == OVSTAGE_OK;
    details::waitAndRelease(instance, deleteEnqueueResult);

    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);

    ovstage_write_floor_desc_t floorDescription{};
    floorDescription.ordinal = ordinal;
    floorDescription.scope = OVSTAGE_SCOPE_ALL;
    details::waitAndRelease(instance, ovstage_advance_write_floor(instance, &floorDescription));
    return ok;
}

std::string getPrimAttributeTypeName(int64_t stageId, const std::string& path, const std::string& attributeName)
{
    // TODO: Implement this
    (void)stageId;
    (void)path;
    (void)attributeName;
    return {};
}

std::vector<std::string> getPrimAttributeNames(int64_t stageId, const std::string& path)
{
    // TODO: Implement this
    (void)stageId;
    (void)path;
    return {};
}

AttributeValues getPrimAttributeValues(int64_t stageId,
                                       const std::vector<std::string>& paths,
                                       const std::string& attributeName)
{
    if (paths.empty())
        throw std::invalid_argument("The `paths` parameter must not be empty");

    const size_t pathCount = paths.size();
    ovstage_instance_t* instance = details::getInstance(stageId, true);
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);

    std::vector<ovx_string_t> pathStrings(pathCount);
    for (size_t i = 0; i < pathCount; ++i)
        pathStrings[i] = { paths[i].c_str(), paths[i].size() };

    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    path_dictionary_create_path_list_from_strings(dictionary, pathStrings.data(), pathCount, &pathList);
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_query_from_path_list(instance, pathList, &query);

    const ovx_string_t attributeString{ attributeName.c_str(), attributeName.size() };
    ovx_token_t attributeToken = OVX_INVALID_TOKEN;
    path_dictionary_create_tokens_from_strings(dictionary, &attributeString, 1, &attributeToken);

    ovstage_ordinal_range_t range{};
    range.has_start_ordinal = false;
    range.end_ordinal = UINT64_MAX;

    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
    ovstage_enqueue_result_t readEnqueueResult =
        ovstage_read_attributes(instance, query, &attributeToken, 1, range, &read);
    if (readEnqueueResult.status != OVSTAGE_OK)
    {
        details::waitAndRelease(instance, ovstage_release_query(instance, query));
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return {};
    }
    details::waitAndRelease(instance, readEnqueueResult);

    std::vector<std::string> stringOutput(pathCount);
    std::vector<std::byte> fixedBuffer;
    std::vector<std::vector<std::byte>> raggedBuffers(pathCount);
    DLDataType columnDataType{};
    bool columnIsArray = false;
    bool columnIsString = false;
    bool receivedMetadata = false;

    for (;;)
    {
        ovstage_read_group_t group{};
        const ovstage_api_status_t fetched = ovstage_fetch_read_next(instance, read, OVSTAGE_TIMEOUT_INFINITE, &group);
        if (fetched == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        if (fetched != OVSTAGE_OK)
            break;

        if (!group.is_delete && group.data.tensors && group.data.tensor_count > 0)
        {
            if (!receivedMetadata)
            {
                columnDataType = group.data.tensors[0].dtype;
                columnIsArray = group.is_array;
                columnIsString =
                    (group.semantic == OVSTAGE_SEMANTIC_STRING || group.semantic == OVSTAGE_SEMANTIC_ASSET_STRING);
                if (!columnIsArray)
                {
                    const size_t elementByteCount = static_cast<size_t>(columnDataType.bits / 8) * columnDataType.lanes;
                    fixedBuffer.assign(pathCount * elementByteCount, std::byte{ 0 });
                }
                receivedMetadata = true;
            }

            const size_t elementByteCount = static_cast<size_t>(columnDataType.bits / 8) * columnDataType.lanes;

            for (size_t k = 0; k < group.prims.count; ++k)
            {
                const size_t outputIndex =
                    group.prims.index_map ? static_cast<size_t>(group.prims.index_map[k]) : (group.prims.offset + k);
                const size_t dataIndex = group.data.index_map ? static_cast<size_t>(group.data.index_map[k]) : k;

                if (outputIndex >= pathCount)
                    continue;

                const DLTensor& t = group.data.tensors[columnIsArray ? dataIndex : 0];

                if (columnIsString)
                {
                    if (t.data && t.shape && t.shape[0] > 0)
                        stringOutput[outputIndex] =
                            std::string(reinterpret_cast<const char*>(t.data), static_cast<size_t>(t.shape[0]));
                }
                else if (columnIsArray)
                {
                    if (t.data && t.shape && t.shape[0] > 0)
                    {
                        const size_t byteCount = static_cast<size_t>(t.shape[0]) * elementByteCount;
                        raggedBuffers[outputIndex].resize(byteCount);
                        std::memcpy(raggedBuffers[outputIndex].data(), t.data, byteCount);
                    }
                }
                else
                {
                    if (t.data)
                        std::memcpy(fixedBuffer.data() + outputIndex * elementByteCount,
                                    static_cast<const std::byte*>(t.data) + dataIndex * elementByteCount,
                                    elementByteCount);
                }
            }
        }
        ovstage_release_group(instance, &group);
    }

    details::waitAndRelease(instance, ovstage_release_read(instance, read));
    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);

    if (!receivedMetadata)
        return {};

    if (columnIsString)
        return stringOutput;

    const array::Dtype dtype = dlTypeToDtype(columnDataType);
    const int64_t lanes = static_cast<int64_t>(columnDataType.lanes);

    if (!columnIsArray)
    {
        auto buffer = std::shared_ptr<std::byte[]>(new std::byte[fixedBuffer.size()]());
        std::memcpy(buffer.get(), fixedBuffer.data(), fixedBuffer.size());
        return array::Array::fromBuffer(buffer, array::Shape{ static_cast<int64_t>(pathCount), lanes }, dtype);
    }

    // Ragged: pack into {pathCount, maximumElementCount, lanes}
    const size_t elementByteCount = static_cast<size_t>(columnDataType.bits / 8) * columnDataType.lanes;
    size_t maximumElementCount = 0;
    for (size_t i = 0; i < pathCount; ++i)
        maximumElementCount = std::max(
            maximumElementCount, raggedBuffers[i].empty() ? size_t{ 0 } : raggedBuffers[i].size() / elementByteCount);

    const size_t totalBytes = pathCount * maximumElementCount * elementByteCount;
    auto buffer = std::shared_ptr<std::byte[]>(new std::byte[totalBytes + 1]());
    std::memset(buffer.get(), 0, totalBytes);
    for (size_t i = 0; i < pathCount; ++i)
    {
        if (!raggedBuffers[i].empty())
            std::memcpy(buffer.get() + i * maximumElementCount * elementByteCount, raggedBuffers[i].data(),
                        raggedBuffers[i].size());
    }
    return array::Array::fromBuffer(
        buffer, array::Shape{ static_cast<int64_t>(pathCount), static_cast<int64_t>(maximumElementCount), lanes }, dtype);
}

std::vector<bool> setPrimAttributeValues(int64_t stageId,
                                         const std::vector<std::string>& paths,
                                         const std::string& attributeName,
                                         const AttributeValues& values)
{
    if (paths.empty())
        throw std::invalid_argument("The `paths` parameter must not be empty");

    const size_t pathCount = paths.size();
    ovstage_instance_t* instance = details::getInstance(stageId, true);
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(instance);

    std::vector<ovx_string_t> pathStrings(pathCount);
    for (size_t i = 0; i < pathCount; ++i)
        pathStrings[i] = { paths[i].c_str(), paths[i].size() };

    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    path_dictionary_create_path_list_from_strings(dictionary, pathStrings.data(), pathCount, &pathList);
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_query_from_path_list(instance, pathList, &query);

    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);
    const ovx_string_or_token_t attributeArgument{ OVX_INVALID_TOKEN, { attributeName.c_str(), attributeName.size() } };

    bool ok = false;

    if (std::holds_alternative<std::vector<std::string>>(values))
    {
        const auto& strings = std::get<std::vector<std::string>>(values);
        if (strings.size() != pathCount)
            throw std::invalid_argument("String values count (" + std::to_string(strings.size()) +
                                        ") does not match paths count (" + std::to_string(pathCount) + ")");

        std::vector<DLTensor> tensors(pathCount);
        std::vector<int64_t> shapes(pathCount);
        for (size_t i = 0; i < pathCount; ++i)
        {
            shapes[i] = static_cast<int64_t>(strings[i].size());
            tensors[i].data = const_cast<char*>(strings[i].data());
            tensors[i].device = { kDLCPU, 0 };
            tensors[i].ndim = 1;
            tensors[i].dtype = { kDLUInt, 8, 1 };
            tensors[i].shape = &shapes[i];
        }
        ovstage_write_data_t write{};
        write.tensors = tensors.data();
        write.tensor_count = static_cast<uint32_t>(pathCount);
        write.is_array = true;
        write.semantic = OVSTAGE_SEMANTIC_STRING;
        const ovstage_enqueue_result_t enqueueResult =
            ovstage_write_attribute(instance, query, attributeArgument, ordinal, write, OVSTAGE_PRIM_MODE_UPSERT);
        ok = enqueueResult.status == OVSTAGE_OK;
        details::waitAndRelease(instance, enqueueResult);
    }
    else if (std::holds_alternative<array::Array>(values))
    {
        const auto& arrayValue = std::get<array::Array>(values);
        const AttributeTypeInformation columnInformation =
            readAttributeTypeInformation(instance, dictionary, paths[0], attributeName);
        if (!columnInformation.found)
            throw isaacsim::common::exceptions::AttributeNameError(attributeName);

        const DLDataType columnDataType = columnInformation.dtype;
        const size_t elementByteCount = static_cast<size_t>(columnDataType.bits / 8) * columnDataType.lanes;

        if (!columnInformation.is_array)
        {
            // Fixed scalar: one element per prim, single tensor of shape {pathCount}
            int64_t fixedShape = static_cast<int64_t>(pathCount);
            DLTensor tensor{};
            tensor.data = const_cast<void*>(arrayValue.data());
            tensor.device = { kDLCPU, 0 };
            tensor.ndim = 1;
            tensor.dtype = columnDataType;
            tensor.shape = &fixedShape;

            ovstage_write_data_t write{};
            write.tensors = &tensor;
            write.tensor_count = 1;
            write.is_array = false;
            write.semantic = columnInformation.semantic;
            const ovstage_enqueue_result_t enqueueResult =
                ovstage_write_attribute(instance, query, attributeArgument, ordinal, write, OVSTAGE_PRIM_MODE_UPSERT);
            ok = enqueueResult.status == OVSTAGE_OK;
            details::waitAndRelease(instance, enqueueResult);
        }
        else
        {
            // Ragged: one tensor per prim, each of shape {maximumElementCount}
            const size_t totalBytes = arrayValue.size() * arrayValue.dtype().size();
            const size_t maximumElementCount =
                (pathCount > 0 && elementByteCount > 0) ? totalBytes / (pathCount * elementByteCount) : 0;
            const auto* source = static_cast<const std::byte*>(arrayValue.data());

            std::vector<DLTensor> tensors(pathCount);
            std::vector<int64_t> shapes(pathCount, static_cast<int64_t>(maximumElementCount));
            for (size_t i = 0; i < pathCount; ++i)
            {
                tensors[i].data = const_cast<std::byte*>(source + i * maximumElementCount * elementByteCount);
                tensors[i].device = { kDLCPU, 0 };
                tensors[i].ndim = 1;
                tensors[i].dtype = columnDataType;
                tensors[i].shape = &shapes[i];
            }
            ovstage_write_data_t write{};
            write.tensors = tensors.data();
            write.tensor_count = static_cast<uint32_t>(pathCount);
            write.is_array = true;
            write.semantic = columnInformation.semantic;
            const ovstage_enqueue_result_t enqueueResult =
                ovstage_write_attribute(instance, query, attributeArgument, ordinal, write, OVSTAGE_PRIM_MODE_UPSERT);
            ok = enqueueResult.status == OVSTAGE_OK;
            details::waitAndRelease(instance, enqueueResult);
        }
    }

    details::waitAndRelease(instance, ovstage_release_query(instance, query));
    path_dictionary_release_path_list_reference(dictionary, pathList);

    ovstage_write_floor_desc_t floorDescription{};
    floorDescription.ordinal = ordinal;
    floorDescription.scope = OVSTAGE_SCOPE_ALL;
    details::waitAndRelease(instance, ovstage_advance_write_floor(instance, &floorDescription));

    return std::vector<bool>(pathCount, ok);
}

std::vector<std::string> findMatchingPrimPaths(int64_t stageId, const std::string& path, bool traverse)
{
    std::vector<std::string> result;

    // Fast path: exact prim match — isValidPathString + isPrimValid replaces SdfPath + getPrimAtPath
    if (isValidPathString(path) && path[0] == '/' && isPrimValid(stageId, path))
    {
        if (traverse)
        {
            traversePrim(stageId, path,
                         [&result](const std::string& p)
                         {
                             result.push_back(p);
                             return true;
                         });
        }
        else
        {
            result.push_back(path);
        }
        return result;
    }

    // Regex search over all prims
    if (traverse)
    {
        std::regex pattern;
        try
        {
            pattern = std::regex(path);
        }
        catch (const std::regex_error&)
        {
            return result;
        }

        traversePrim(stageId, "/",
                     [&result, &pattern](const std::string& p)
                     {
                         if (std::regex_search(p, pattern, std::regex_constants::match_continuous))
                             result.push_back(p);
                         return true;
                     });
    }
    else
    {
        // Segment-wise search: split by "/" and match level by level with getChildren
        const auto start = path.find_first_not_of('/');
        const auto end = path.find_last_not_of('/');
        const std::string trimmed = (start == std::string::npos) ? "" : path.substr(start, end - start + 1);

        std::vector<std::regex> patterns;
        std::istringstream ss(trimmed);
        std::string segment;
        try
        {
            while (std::getline(ss, segment, '/'))
                patterns.emplace_back("^" + segment + "$");
        }
        catch (const std::regex_error&)
        {
            return result;
        }

        std::vector<std::string> roots = { "/" };
        std::vector<std::string> matches;
        for (size_t i = 0; i < patterns.size(); ++i)
        {
            for (const std::string& root : roots)
            {
                for (const std::string& child : getChildren(stageId, root))
                {
                    const std::string name = child.substr(child.rfind('/') + 1);
                    if (std::regex_match(name, patterns[i]))
                        matches.push_back(child);
                }
            }
            if (i < patterns.size() - 1)
            {
                roots = std::move(matches);
                matches.clear();
            }
        }
        result = std::move(matches);
    }

    return result;
}

void resetXformOpProperties(int64_t stageId, const std::string& path)
{
    (void)stageId;
    (void)path;
    throw std::logic_error("The ovstage library does not support authoring (reset) xform op properties");
}

array::Array getXformLocalScales(int64_t stageId, const std::vector<std::string>& paths)
{
    for (const auto& p : paths)
        validateXformable(stageId, p);
    auto* instance = details::getInstance(stageId, true);
    auto* dictionary = ovstage_get_path_dictionary(instance);
    auto matrices = readLocalMatrices(instance, dictionary, paths);

    std::vector<std::vector<double>> result(paths.size(), std::vector<double>(3));
    for (size_t i = 0; i < paths.size(); ++i)
    {
        double s[3];
        extractScale(matrices[i].data(), s);
        result[i] = { s[0], s[1], s[2] };
    }
    return array::Array(result);
}

void setXformLocalScales(int64_t stageId, const std::vector<std::string>& paths, const array::Array& scales)
{
    const size_t pathCount = paths.size();
    for (const auto& p : paths)
        validateXformable(stageId, p);

    auto scaleRows = scales.get<std::vector<std::vector<double>>>();
    if (scaleRows.size() != pathCount)
        throw std::invalid_argument("scales row count (" + std::to_string(scaleRows.size()) +
                                    ") does not match paths count (" + std::to_string(pathCount) + ")");
    for (const auto& row : scaleRows)
        if (row.size() != 3)
            throw std::invalid_argument("each scale must have 3 components");

    auto* instance = details::getInstance(stageId, true);
    auto* dictionary = ovstage_get_path_dictionary(instance);
    auto matrices = readLocalMatrices(instance, dictionary, paths);

    for (size_t i = 0; i < pathCount; ++i)
    {
        double oldS[3];
        extractScale(matrices[i].data(), oldS);
        double t[3], q[4];
        extractPose(matrices[i].data(), oldS, t, q);
        const double newS[3] = { scaleRows[i][0], scaleRows[i][1], scaleRows[i][2] };
        buildMatrix(t, q, newS, matrices[i].data());
    }

    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);
    if (ordinal == 0)
        return;
    writeLocalMatrices(instance, dictionary, ordinal, paths, matrices);
    sealOrdinal(instance, ordinal);
}

std::tuple<array::Array, array::Array> getXformLocalPoses(int64_t stageId, const std::vector<std::string>& paths)
{
    for (const auto& p : paths)
        validateXformable(stageId, p);
    auto* instance = details::getInstance(stageId, true);
    auto* dictionary = ovstage_get_path_dictionary(instance);
    auto matrices = readLocalMatrices(instance, dictionary, paths);

    std::vector<std::vector<double>> translations(paths.size(), std::vector<double>(3));
    std::vector<std::vector<double>> orientations(paths.size(), std::vector<double>(4));
    for (size_t i = 0; i < paths.size(); ++i)
    {
        double s[3];
        extractScale(matrices[i].data(), s);
        double t[3], q[4];
        extractPose(matrices[i].data(), s, t, q);
        translations[i] = { t[0], t[1], t[2] };
        orientations[i] = { q[0], q[1], q[2], q[3] };
    }
    return { array::Array(translations), array::Array(orientations) };
}

void setXformLocalPoses(int64_t stageId,
                        const std::vector<std::string>& paths,
                        const std::optional<array::Array>& translations,
                        const std::optional<array::Array>& orientations)
{
    if (!translations && !orientations)
        return;
    const size_t pathCount = paths.size();
    for (const auto& p : paths)
        validateXformable(stageId, p);

    std::vector<std::vector<double>> translationRows, orientationRows;
    if (translations)
    {
        translationRows = translations->get<std::vector<std::vector<double>>>();
        if (translationRows.size() != pathCount)
            throw std::invalid_argument("translations row count (" + std::to_string(translationRows.size()) +
                                        ") does not match paths count (" + std::to_string(pathCount) + ")");
        for (const auto& r : translationRows)
            if (r.size() != 3)
                throw std::invalid_argument("each translation must have 3 components");
    }
    if (orientations)
    {
        orientationRows = orientations->get<std::vector<std::vector<double>>>();
        if (orientationRows.size() != pathCount)
            throw std::invalid_argument("orientations row count (" + std::to_string(orientationRows.size()) +
                                        ") does not match paths count (" + std::to_string(pathCount) + ")");
        for (const auto& r : orientationRows)
            if (r.size() != 4)
                throw std::invalid_argument("each orientation must have 4 components");
    }

    auto* instance = details::getInstance(stageId, true);
    auto* dictionary = ovstage_get_path_dictionary(instance);
    auto matrices = readLocalMatrices(instance, dictionary, paths);

    for (size_t i = 0; i < pathCount; ++i)
    {
        double s[3];
        extractScale(matrices[i].data(), s);
        double t[3], q[4];
        extractPose(matrices[i].data(), s, t, q);
        if (translations)
        {
            t[0] = translationRows[i][0];
            t[1] = translationRows[i][1];
            t[2] = translationRows[i][2];
        }
        if (orientations)
        {
            q[0] = orientationRows[i][0];
            q[1] = orientationRows[i][1];
            q[2] = orientationRows[i][2];
            q[3] = orientationRows[i][3];
        }
        buildMatrix(t, q, s, matrices[i].data());
    }

    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);
    if (ordinal == 0)
        return;
    writeLocalMatrices(instance, dictionary, ordinal, paths, matrices);
    sealOrdinal(instance, ordinal);
}

std::tuple<array::Array, array::Array> getXformWorldPoses(int64_t stageId, const std::vector<std::string>& paths)
{
    for (const auto& p : paths)
        validateXformable(stageId, p);
    auto* instance = details::getInstance(stageId, true);
    auto* dictionary = ovstage_get_path_dictionary(instance);

    std::vector<std::vector<double>> positions(paths.size(), std::vector<double>(3));
    std::vector<std::vector<double>> orientations(paths.size(), std::vector<double>(4));
    for (size_t i = 0; i < paths.size(); ++i)
    {
        auto worldMatrix = computeWorldMatrix(instance, dictionary, paths[i]);
        double s[3];
        extractScale(worldMatrix.data(), s);
        double t[3], q[4];
        extractPose(worldMatrix.data(), s, t, q);
        positions[i] = { t[0], t[1], t[2] };
        orientations[i] = { q[0], q[1], q[2], q[3] };
    }
    return { array::Array(positions), array::Array(orientations) };
}

void setXformWorldPoses(int64_t stageId,
                        const std::vector<std::string>& paths,
                        const std::optional<array::Array>& positions,
                        const std::optional<array::Array>& orientations)
{
    if (!positions && !orientations)
        return;
    const size_t pathCount = paths.size();
    for (const auto& p : paths)
        validateXformable(stageId, p);

    std::vector<std::vector<double>> positionRows, orientationRows;
    if (positions)
    {
        positionRows = positions->get<std::vector<std::vector<double>>>();
        if (positionRows.size() != pathCount)
            throw std::invalid_argument("positions row count (" + std::to_string(positionRows.size()) +
                                        ") does not match paths count (" + std::to_string(pathCount) + ")");
        for (const auto& r : positionRows)
            if (r.size() != 3)
                throw std::invalid_argument("each position must have 3 components");
    }
    if (orientations)
    {
        orientationRows = orientations->get<std::vector<std::vector<double>>>();
        if (orientationRows.size() != pathCount)
            throw std::invalid_argument("orientations row count (" + std::to_string(orientationRows.size()) +
                                        ") does not match paths count (" + std::to_string(pathCount) + ")");
        for (const auto& r : orientationRows)
            if (r.size() != 4)
                throw std::invalid_argument("each orientation must have 4 components");
    }

    auto* instance = details::getInstance(stageId, true);
    auto* dictionary = ovstage_get_path_dictionary(instance);
    auto matrices = readLocalMatrices(instance, dictionary, paths);

    for (size_t i = 0; i < pathCount; ++i)
    {
        // Current world TRS
        auto worldMatrix = computeWorldMatrix(instance, dictionary, paths[i]);
        double ws[3];
        extractScale(worldMatrix.data(), ws);
        double wt[3], wq[4];
        extractPose(worldMatrix.data(), ws, wt, wq);

        // Apply requested overrides
        if (positions)
        {
            wt[0] = positionRows[i][0];
            wt[1] = positionRows[i][1];
            wt[2] = positionRows[i][2];
        }
        if (orientations)
        {
            wq[0] = orientationRows[i][0];
            wq[1] = orientationRows[i][1];
            wq[2] = orientationRows[i][2];
            wq[3] = orientationRows[i][3];
        }

        // Build desired world matrix (use world scale as-is)
        double wDesired[16];
        buildMatrix(wt, wq, ws, wDesired);

        // Parent world matrix
        const std::string parent = parentPath(paths[i]);
        std::array<double, 16> parentWorldMatrix;
        identityMat(parentWorldMatrix.data());
        if (!parent.empty() && parent != "/")
            parentWorldMatrix = computeWorldMatrix(instance, dictionary, parent);

        // L_new = W_desired * inv(W_parent)
        double invParent[16];
        if (!invertTRS(parentWorldMatrix.data(), invParent))
            identityMat(invParent);
        double lNew[16];
        mulMat(wDesired, invParent, lNew);

        // Preserve the original local scale
        double ls[3];
        extractScale(matrices[i].data(), ls);
        double lt[3], lq[4];
        double lNewS[3];
        extractScale(lNew, lNewS);
        extractPose(lNew, lNewS, lt, lq);
        buildMatrix(lt, lq, ls, matrices[i].data());
    }

    const ovstage_ordinal_t ordinal = details::consumeOrdinal(stageId);
    if (ordinal == 0)
        return;
    writeLocalMatrices(instance, dictionary, ordinal, paths, matrices);
    sealOrdinal(instance, ordinal);
}

} // namespace ovstage
} // namespace usd
} // namespace foundation
} // namespace isaacsim

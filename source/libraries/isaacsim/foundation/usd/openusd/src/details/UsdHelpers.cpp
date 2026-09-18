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

#include "UsdHelpers.hpp"

#include <isaacsim/common/exceptions/Exceptions.hpp>
#include <pxr/base/gf/half.h>
#include <pxr/base/gf/matrix2d.h>
#include <pxr/base/gf/matrix3d.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/quath.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec2h.h>
#include <pxr/base/gf/vec2i.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec3h.h>
#include <pxr/base/gf/vec3i.h>
#include <pxr/base/gf/vec4d.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/gf/vec4h.h>
#include <pxr/base/gf/vec4i.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/timeCode.h>
#include <pxr/usd/usd/timeCode.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include <functional>
#include <stdexcept>
#include <string_view>
#include <unordered_map>

namespace isaacsim
{
namespace foundation
{
namespace usd
{
namespace openusd
{
namespace details
{

namespace
{

// ---------------------------------------------------------------------------
// asset
// ---------------------------------------------------------------------------

AttributeValue readAsset(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::SdfAssetPath assetPath;
    attribute.Get(&assetPath);
    return assetPath.GetAssetPath();
}

AttributeValue readAssetArray(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtArray<PXR_NS::SdfAssetPath> array;
    attribute.Get(&array);
    std::vector<std::string> values;
    values.reserve(array.size());
    for (const auto& v : array)
    {
        values.push_back(v.GetAssetPath());
    }
    return values;
}

// ---------------------------------------------------------------------------
// half  (GfHalf -> float)
// ---------------------------------------------------------------------------

AttributeValue readGfHalf(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::GfHalf value{};
    attribute.Get(&value);
    return array::Array(static_cast<float>(value));
}

AttributeValue readGfHalfArray(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtArray<PXR_NS::GfHalf> vtArray;
    attribute.Get(&vtArray);
    std::vector<float> values;
    values.reserve(vtArray.size());
    for (const auto& v : vtArray)
    {
        values.push_back(static_cast<float>(v));
    }
    return array::Array(values);
}

// ---------------------------------------------------------------------------
// string
// ---------------------------------------------------------------------------

AttributeValue readString(const PXR_NS::UsdAttribute& attribute)
{
    std::string value;
    attribute.Get(&value);
    return value;
}

AttributeValue readStringArray(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtArray<std::string> array;
    attribute.Get(&array);
    return std::vector<std::string>(array.begin(), array.end());
}

// ---------------------------------------------------------------------------
// token  (TfToken -> std::string)
// ---------------------------------------------------------------------------

AttributeValue readToken(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::TfToken value;
    attribute.Get(&value);
    return value.GetString();
}

AttributeValue readTokenArray(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtArray<PXR_NS::TfToken> array;
    attribute.Get(&array);
    std::vector<std::string> values;
    values.reserve(array.size());
    for (const auto& v : array)
    {
        values.push_back(v.GetString());
    }
    return values;
}

// ---------------------------------------------------------------------------
// timecode  (SdfTimeCode -> double)
// ---------------------------------------------------------------------------

AttributeValue readTimeCode(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::SdfTimeCode value{};
    attribute.Get(&value);
    return array::Array(value.GetValue());
}

AttributeValue readTimeCodeArray(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtArray<PXR_NS::SdfTimeCode> vtArray;
    attribute.Get(&vtArray);
    std::vector<double> values;
    values.reserve(vtArray.size());
    for (const auto& v : vtArray)
    {
        values.push_back(v.GetValue());
    }
    return array::Array(values);
}

// ---------------------------------------------------------------------------
// Generic read templates
// ---------------------------------------------------------------------------

template <typename T>
AttributeValue readScalar(const PXR_NS::UsdAttribute& attribute)
{
    T value{};
    attribute.Get(&value);
    return array::Array(value);
}

template <typename T>
AttributeValue readScalarArray(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtArray<T> vtArray;
    attribute.Get(&vtArray);
    return array::Array(std::vector<T>(vtArray.begin(), vtArray.end()));
}

// GfVecNT -> std::vector<ScalarT>; static_cast handles GfHalf -> float
template <typename GfVecT, typename ScalarT, int N>
AttributeValue readGfVector(const PXR_NS::UsdAttribute& attribute)
{
    GfVecT v{};
    attribute.Get(&v);
    std::vector<ScalarT> result;
    result.reserve(N);
    for (int i = 0; i < N; ++i)
        result.push_back(static_cast<ScalarT>(v[i]));
    return array::Array(result);
}

template <typename GfVecT, typename ScalarT, int N>
AttributeValue readGfVecArray(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtArray<GfVecT> vtArray;
    attribute.Get(&vtArray);
    std::vector<std::vector<ScalarT>> values;
    values.reserve(vtArray.size());
    for (const auto& v : vtArray)
    {
        std::vector<ScalarT> row;
        row.reserve(N);
        for (int i = 0; i < N; ++i)
            row.push_back(static_cast<ScalarT>(v[i]));
        values.push_back(std::move(row));
    }
    return array::Array(values);
}

// GfMatrixNd -> std::vector<double> (row-major); all USD matrices are double
template <typename GfMatrixT, int N>
AttributeValue readGfMatrix(const PXR_NS::UsdAttribute& attribute)
{
    GfMatrixT m{};
    attribute.Get(&m);
    const double* data = m.GetArray();
    return array::Array(std::vector<double>(data, data + N * N));
}

template <typename GfMatrixT, int N>
AttributeValue readGfMatrixArray(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtArray<GfMatrixT> vtArray;
    attribute.Get(&vtArray);
    std::vector<std::vector<double>> values;
    values.reserve(vtArray.size());
    for (const auto& m : vtArray)
    {
        const double* data = m.GetArray();
        values.push_back(std::vector<double>(data, data + N * N));
    }
    return array::Array(values);
}

// GfQuat* -> std::vector<ScalarT> [ix, iy, iz, w]; static_cast handles GfHalf -> float
template <typename GfQuatT, typename ScalarT>
AttributeValue readGfQuaternion(const PXR_NS::UsdAttribute& attribute)
{
    GfQuatT q{};
    attribute.Get(&q);
    const auto& img = q.GetImaginary();
    return array::Array(std::vector<ScalarT>{ static_cast<ScalarT>(img[0]), static_cast<ScalarT>(img[1]),
                                              static_cast<ScalarT>(img[2]), static_cast<ScalarT>(q.GetReal()) });
}

template <typename GfQuatT, typename ScalarT>
AttributeValue readQuatArray(const PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtArray<GfQuatT> vtArray;
    attribute.Get(&vtArray);
    std::vector<std::vector<ScalarT>> values;
    values.reserve(vtArray.size());
    for (const auto& q : vtArray)
    {
        const auto& img = q.GetImaginary();
        values.push_back({ static_cast<ScalarT>(img[0]), static_cast<ScalarT>(img[1]), static_cast<ScalarT>(img[2]),
                           static_cast<ScalarT>(q.GetReal()) });
    }
    return array::Array(values);
}

// clang-format off
std::unordered_map<std::string, std::function<AttributeValue(const PXR_NS::UsdAttribute&)>> attributeReaders = {
    { "asset",          readAsset },
    { "asset[]",        readAssetArray },

    { "bool",           readScalar<bool> },
    { "bool[]",         readScalarArray<bool> },

    { "double",         readScalar<double> },
    { "double[]",       readScalarArray<double> },

    { "float",          readScalar<float> },
    { "float[]",        readScalarArray<float> },

    { "half",           readGfHalf },
    { "half[]",         readGfHalfArray },

    { "int",            readScalar<int> },
    { "int[]",          readScalarArray<int> },

    { "int64",          readScalar<int64_t> },
    { "int64[]",        readScalarArray<int64_t> },

    { "uint",           readScalar<unsigned int> },
    { "uint[]",         readScalarArray<unsigned int> },

    { "uint64",         readScalar<uint64_t> },
    { "uint64[]",       readScalarArray<uint64_t> },

    { "uchar",          readScalar<unsigned char> },
    { "uchar[]",        readScalarArray<unsigned char> },

    { "string",         readString },
    { "string[]",       readStringArray },

    { "token",          readToken },
    { "token[]",        readTokenArray },

    { "timecode",       readTimeCode },
    { "timecode[]",     readTimeCodeArray },

    // GfVec2d
    { "double2",        readGfVector<PXR_NS::GfVec2d, double, 2> },
    { "texCoord2d",     readGfVector<PXR_NS::GfVec2d, double, 2> },
    { "double2[]",      readGfVecArray<PXR_NS::GfVec2d, double, 2> },
    { "texCoord2d[]",   readGfVecArray<PXR_NS::GfVec2d, double, 2> },

    // GfVec3d
    { "double3",        readGfVector<PXR_NS::GfVec3d, double, 3> },
    { "color3d",        readGfVector<PXR_NS::GfVec3d, double, 3> },
    { "normal3d",       readGfVector<PXR_NS::GfVec3d, double, 3> },
    { "point3d",        readGfVector<PXR_NS::GfVec3d, double, 3> },
    { "vector3d",       readGfVector<PXR_NS::GfVec3d, double, 3> },
    { "texCoord3d",     readGfVector<PXR_NS::GfVec3d, double, 3> },
    { "double3[]",      readGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "color3d[]",      readGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "normal3d[]",     readGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "point3d[]",      readGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "vector3d[]",     readGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "texCoord3d[]",   readGfVecArray<PXR_NS::GfVec3d, double, 3> },

    // GfVec4d
    { "double4",        readGfVector<PXR_NS::GfVec4d, double, 4> },
    { "color4d",        readGfVector<PXR_NS::GfVec4d, double, 4> },
    { "double4[]",      readGfVecArray<PXR_NS::GfVec4d, double, 4> },
    { "color4d[]",      readGfVecArray<PXR_NS::GfVec4d, double, 4> },

    // GfVec2f
    { "float2",         readGfVector<PXR_NS::GfVec2f, float, 2> },
    { "texCoord2f",     readGfVector<PXR_NS::GfVec2f, float, 2> },
    { "float2[]",       readGfVecArray<PXR_NS::GfVec2f, float, 2> },
    { "texCoord2f[]",   readGfVecArray<PXR_NS::GfVec2f, float, 2> },

    // GfVec3f
    { "float3",         readGfVector<PXR_NS::GfVec3f, float, 3> },
    { "color3f",        readGfVector<PXR_NS::GfVec3f, float, 3> },
    { "normal3f",       readGfVector<PXR_NS::GfVec3f, float, 3> },
    { "point3f",        readGfVector<PXR_NS::GfVec3f, float, 3> },
    { "vector3f",       readGfVector<PXR_NS::GfVec3f, float, 3> },
    { "texCoord3f",     readGfVector<PXR_NS::GfVec3f, float, 3> },
    { "float3[]",       readGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "color3f[]",      readGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "normal3f[]",     readGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "point3f[]",      readGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "vector3f[]",     readGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "texCoord3f[]",   readGfVecArray<PXR_NS::GfVec3f, float, 3> },

    // GfVec4f
    { "float4",         readGfVector<PXR_NS::GfVec4f, float, 4> },
    { "color4f",        readGfVector<PXR_NS::GfVec4f, float, 4> },
    { "float4[]",       readGfVecArray<PXR_NS::GfVec4f, float, 4> },
    { "color4f[]",      readGfVecArray<PXR_NS::GfVec4f, float, 4> },

    // GfVec2h
    { "half2",          readGfVector<PXR_NS::GfVec2h, float, 2> },
    { "texCoord2h",     readGfVector<PXR_NS::GfVec2h, float, 2> },
    { "half2[]",        readGfVecArray<PXR_NS::GfVec2h, float, 2> },
    { "texCoord2h[]",   readGfVecArray<PXR_NS::GfVec2h, float, 2> },

    // GfVec3h
    { "half3",          readGfVector<PXR_NS::GfVec3h, float, 3> },
    { "color3h",        readGfVector<PXR_NS::GfVec3h, float, 3> },
    { "normal3h",       readGfVector<PXR_NS::GfVec3h, float, 3> },
    { "point3h",        readGfVector<PXR_NS::GfVec3h, float, 3> },
    { "vector3h",       readGfVector<PXR_NS::GfVec3h, float, 3> },
    { "texCoord3h",     readGfVector<PXR_NS::GfVec3h, float, 3> },
    { "half3[]",        readGfVecArray<PXR_NS::GfVec3h, float, 3> },
    { "color3h[]",      readGfVecArray<PXR_NS::GfVec3h, float, 3> },
    { "normal3h[]",     readGfVecArray<PXR_NS::GfVec3h, float, 3> },
    { "point3h[]",      readGfVecArray<PXR_NS::GfVec3h, float, 3> },
    { "vector3h[]",     readGfVecArray<PXR_NS::GfVec3h, float, 3> },
    { "texCoord3h[]",   readGfVecArray<PXR_NS::GfVec3h, float, 3> },

    // GfVec4h
    { "half4",          readGfVector<PXR_NS::GfVec4h, float, 4> },
    { "color4h",        readGfVector<PXR_NS::GfVec4h, float, 4> },
    { "half4[]",        readGfVecArray<PXR_NS::GfVec4h, float, 4> },
    { "color4h[]",      readGfVecArray<PXR_NS::GfVec4h, float, 4> },

    // GfVec2i
    { "int2",           readGfVector<PXR_NS::GfVec2i, int, 2> },
    { "int2[]",         readGfVecArray<PXR_NS::GfVec2i, int, 2> },

    // GfVec3i
    { "int3",           readGfVector<PXR_NS::GfVec3i, int, 3> },
    { "int3[]",         readGfVecArray<PXR_NS::GfVec3i, int, 3> },

    // GfVec4i
    { "int4",           readGfVector<PXR_NS::GfVec4i, int, 4> },
    { "int4[]",         readGfVecArray<PXR_NS::GfVec4i, int, 4> },

    // GfMatrix2d
    { "matrix2d",       readGfMatrix<PXR_NS::GfMatrix2d, 2> },
    { "matrix2d[]",     readGfMatrixArray<PXR_NS::GfMatrix2d, 2> },

    // GfMatrix3d
    { "matrix3d",       readGfMatrix<PXR_NS::GfMatrix3d, 3> },
    { "matrix3d[]",     readGfMatrixArray<PXR_NS::GfMatrix3d, 3> },

    // GfMatrix4d
    { "matrix4d",       readGfMatrix<PXR_NS::GfMatrix4d, 4> },
    { "frame4d",        readGfMatrix<PXR_NS::GfMatrix4d, 4> },
    { "matrix4d[]",     readGfMatrixArray<PXR_NS::GfMatrix4d, 4> },
    { "frame4d[]",      readGfMatrixArray<PXR_NS::GfMatrix4d, 4> },

    // GfQuatd
    { "quatd",          readGfQuaternion<PXR_NS::GfQuatd, double> },
    { "quatd[]",        readQuatArray<PXR_NS::GfQuatd, double> },

    // GfQuatf
    { "quatf",          readGfQuaternion<PXR_NS::GfQuatf, float> },
    { "quatf[]",        readQuatArray<PXR_NS::GfQuatf, float> },

    // GfQuath
    { "quath",          readGfQuaternion<PXR_NS::GfQuath, float> },
    { "quath[]",        readQuatArray<PXR_NS::GfQuath, float> },
};
// clang-format on

// ---------------------------------------------------------------------------
// Validation / coercion helpers
// ---------------------------------------------------------------------------

// Returns the value as T, coercing from any other numeric scalar in the variant.
// Excludes bool and string/vector types - those must match exactly.
template <typename T>
T parseNumericScalar(const AttributeValue& value, std::string_view typeName)
{
    if (std::holds_alternative<array::Array>(value))
    {
        return std::get<array::Array>(value).item<T>();
    }
    throw std::invalid_argument("Value type is not compatible with the required numeric scalar for " +
                                std::string(typeName));
}

// Returns vector<T>, coercing element-wise from any compatible numeric vector alternative.
template <typename T>
std::vector<T> parseNumericVector(const AttributeValue& value, std::string_view typeName)
{
    if (std::holds_alternative<array::Array>(value))
    {
        return std::get<array::Array>(value).get<std::vector<T>>();
    }
    throw std::invalid_argument("Value type is not compatible with the required numeric vector for " +
                                std::string(typeName));
}

// Returns vector<vector<T>>, coercing element-wise from any compatible nested numeric vector alternative.
template <typename T>
std::vector<std::vector<T>> parseNestedNumericVector(const AttributeValue& value, std::string_view typeName)
{
    if (std::holds_alternative<array::Array>(value))
    {
        return std::get<array::Array>(value).get<std::vector<std::vector<T>>>();
    }
    throw std::invalid_argument("Value type is not compatible with the required numeric nested vector for " +
                                std::string(typeName));
}

// Returns the value as the exact type T; throws std::invalid_argument on mismatch.
template <typename T>
const T& parseExactValue(const AttributeValue& value, std::string_view typeName)
{
    if (const auto* p = std::get_if<T>(&value))
    {
        return *p;
    }
    throw std::invalid_argument("Value type does not match the required attribute type for " + std::string(typeName));
}

// ---------------------------------------------------------------------------
// Write helpers
// ---------------------------------------------------------------------------

bool writeAsset(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    return attribute.Set(PXR_NS::SdfAssetPath(parseExactValue<std::string>(value, typeName)));
}

bool writeAssetArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto& strings = parseExactValue<std::vector<std::string>>(value, typeName);
    PXR_NS::VtArray<PXR_NS::SdfAssetPath> array;
    array.reserve(strings.size());
    for (const auto& s : strings)
        array.push_back(PXR_NS::SdfAssetPath(s));
    return attribute.Set(array);
}

bool writeBool(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    return attribute.Set(parseNumericScalar<bool>(value, typeName));
}

bool writeBoolArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto vec = parseNumericVector<bool>(value, typeName);
    PXR_NS::VtArray<bool> array;
    array.reserve(vec.size());
    for (bool b : vec)
        array.push_back(b);
    return attribute.Set(array);
}

bool writeGfHalf(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    return attribute.Set(PXR_NS::GfHalf(parseNumericScalar<float>(value, typeName)));
}

bool writeGfHalfArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto vec = parseNumericVector<float>(value, typeName);
    PXR_NS::VtArray<PXR_NS::GfHalf> array;
    array.reserve(vec.size());
    for (float f : vec)
        array.push_back(PXR_NS::GfHalf(f));
    return attribute.Set(array);
}

bool writeString(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    return attribute.Set(parseExactValue<std::string>(value, typeName));
}

bool writeStringArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto& vec = parseExactValue<std::vector<std::string>>(value, typeName);
    return attribute.Set(PXR_NS::VtArray<std::string>(vec.begin(), vec.end()));
}

bool writeToken(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    return attribute.Set(PXR_NS::TfToken(parseExactValue<std::string>(value, typeName)));
}

bool writeTokenArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto& vec = parseExactValue<std::vector<std::string>>(value, typeName);
    PXR_NS::VtArray<PXR_NS::TfToken> array;
    array.reserve(vec.size());
    for (const auto& s : vec)
        array.push_back(PXR_NS::TfToken(s));
    return attribute.Set(array);
}

bool writeTimeCode(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    return attribute.Set(PXR_NS::SdfTimeCode(parseNumericScalar<double>(value, typeName)));
}

bool writeTimeCodeArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto vec = parseNumericVector<double>(value, typeName);
    PXR_NS::VtArray<PXR_NS::SdfTimeCode> array;
    array.reserve(vec.size());
    for (double d : vec)
        array.push_back(PXR_NS::SdfTimeCode(d));
    return attribute.Set(array);
}

template <typename T>
bool writeNumericScalar(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    return attribute.Set(parseNumericScalar<T>(value, typeName));
}

template <typename T>
bool writeNumericScalarArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto vec = parseNumericVector<T>(value, typeName);
    return attribute.Set(PXR_NS::VtArray<T>(vec.begin(), vec.end()));
}

// GfVecNT scalar; ComponentT defaults to ScalarT (pass GfHalf for Vec*h)
template <typename GfVecT, typename ScalarT, int N, typename ComponentT = ScalarT>
bool writeGfVector(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto v = parseNumericVector<ScalarT>(value, typeName);
    if (static_cast<int>(v.size()) != N)
        throw std::invalid_argument("Expected " + std::to_string(N) + " elements for " + std::string(typeName));
    if constexpr (N == 2)
        return attribute.Set(GfVecT(static_cast<ComponentT>(v[0]), static_cast<ComponentT>(v[1])));
    else if constexpr (N == 3)
        return attribute.Set(
            GfVecT(static_cast<ComponentT>(v[0]), static_cast<ComponentT>(v[1]), static_cast<ComponentT>(v[2])));
    else
        return attribute.Set(GfVecT(static_cast<ComponentT>(v[0]), static_cast<ComponentT>(v[1]),
                                    static_cast<ComponentT>(v[2]), static_cast<ComponentT>(v[3])));
}

template <typename GfVecT, typename ScalarT, int N, typename ComponentT = ScalarT>
bool writeGfVecArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto vecs = parseNestedNumericVector<ScalarT>(value, typeName);
    PXR_NS::VtArray<GfVecT> array;
    array.reserve(vecs.size());
    for (const auto& v : vecs)
    {
        if (static_cast<int>(v.size()) != N)
            throw std::invalid_argument("Expected " + std::to_string(N) + " elements per entry for " +
                                        std::string(typeName));
        if constexpr (N == 2)
            array.push_back(GfVecT(static_cast<ComponentT>(v[0]), static_cast<ComponentT>(v[1])));
        else if constexpr (N == 3)
            array.push_back(
                GfVecT(static_cast<ComponentT>(v[0]), static_cast<ComponentT>(v[1]), static_cast<ComponentT>(v[2])));
        else
            array.push_back(GfVecT(static_cast<ComponentT>(v[0]), static_cast<ComponentT>(v[1]),
                                   static_cast<ComponentT>(v[2]), static_cast<ComponentT>(v[3])));
    }
    return attribute.Set(array);
}

// GfMatrixNd scalar; all USD matrices are double
template <typename GfMatrixT, int N>
bool writeGfMatrix(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto v = parseNumericVector<double>(value, typeName);
    if (static_cast<int>(v.size()) != N * N)
        throw std::invalid_argument("Expected " + std::to_string(N * N) + " elements (row-major) for " +
                                    std::string(typeName));
    GfMatrixT m;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            m[i][j] = v[i * N + j];
    return attribute.Set(m);
}

template <typename GfMatrixT, int N>
bool writeGfMatrixArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto mats = parseNestedNumericVector<double>(value, typeName);
    PXR_NS::VtArray<GfMatrixT> array;
    array.reserve(mats.size());
    for (const auto& v : mats)
    {
        if (static_cast<int>(v.size()) != N * N)
            throw std::invalid_argument("Expected " + std::to_string(N * N) + " elements (row-major) per " +
                                        std::string(typeName) + " entry");
        GfMatrixT m;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                m[i][j] = v[i * N + j];
        array.push_back(m);
    }
    return attribute.Set(array);
}

// GfQuat scalar; input [ix, iy, iz, w], PXR ctor (real, imaginary)
template <typename GfQuatT, typename GfVec3T, typename ScalarT, typename ComponentT = ScalarT>
bool writeGfQuaternion(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto v = parseNumericVector<ScalarT>(value, typeName);
    if (v.size() != 4)
        throw std::invalid_argument("Expected 4 elements [ix, iy, iz, w] for " + std::string(typeName));
    return attribute.Set(
        GfQuatT(static_cast<ComponentT>(v[3]),
                GfVec3T(static_cast<ComponentT>(v[0]), static_cast<ComponentT>(v[1]), static_cast<ComponentT>(v[2]))));
}

template <typename GfQuatT, typename GfVec3T, typename ScalarT, typename ComponentT = ScalarT>
bool writeGfQuatArray(PXR_NS::UsdAttribute& attribute, const AttributeValue& value, std::string_view typeName)
{
    const auto quats = parseNestedNumericVector<ScalarT>(value, typeName);
    PXR_NS::VtArray<GfQuatT> array;
    array.reserve(quats.size());
    for (const auto& v : quats)
    {
        if (v.size() != 4)
            throw std::invalid_argument("Expected 4 elements [ix, iy, iz, w] per " + std::string(typeName) + " entry");
        array.push_back(GfQuatT(
            static_cast<ComponentT>(v[3]),
            GfVec3T(static_cast<ComponentT>(v[0]), static_cast<ComponentT>(v[1]), static_cast<ComponentT>(v[2]))));
    }
    return attribute.Set(array);
}

// clang-format off
std::unordered_map<
    std::string,
    std::function<bool(PXR_NS::UsdAttribute&, const AttributeValue&, std::string_view)>> attributeWriters = {
    { "asset",          writeAsset },
    { "asset[]",        writeAssetArray },

    { "bool",           writeBool },
    { "bool[]",         writeBoolArray },

    { "double",         writeNumericScalar<double> },
    { "double[]",       writeNumericScalarArray<double> },

    { "float",          writeNumericScalar<float> },
    { "float[]",        writeNumericScalarArray<float> },

    { "half",           writeGfHalf },
    { "half[]",         writeGfHalfArray },

    { "int",            writeNumericScalar<int> },
    { "int[]",          writeNumericScalarArray<int> },

    { "int64",          writeNumericScalar<int64_t> },
    { "int64[]",        writeNumericScalarArray<int64_t> },

    { "uint",           writeNumericScalar<unsigned int> },
    { "uint[]",         writeNumericScalarArray<unsigned int> },

    { "uint64",         writeNumericScalar<uint64_t> },
    { "uint64[]",       writeNumericScalarArray<uint64_t> },

    { "uchar",          writeNumericScalar<unsigned char> },
    { "uchar[]",        writeNumericScalarArray<unsigned char> },

    { "string",         writeString },
    { "string[]",       writeStringArray },

    { "token",          writeToken },
    { "token[]",        writeTokenArray },

    { "timecode",       writeTimeCode },
    { "timecode[]",     writeTimeCodeArray },

    // GfVec2d
    { "double2",        writeGfVector<PXR_NS::GfVec2d, double, 2> },
    { "texCoord2d",     writeGfVector<PXR_NS::GfVec2d, double, 2> },
    { "double2[]",      writeGfVecArray<PXR_NS::GfVec2d, double, 2> },
    { "texCoord2d[]",   writeGfVecArray<PXR_NS::GfVec2d, double, 2> },

    // GfVec3d
    { "double3",        writeGfVector<PXR_NS::GfVec3d, double, 3> },
    { "color3d",        writeGfVector<PXR_NS::GfVec3d, double, 3> },
    { "normal3d",       writeGfVector<PXR_NS::GfVec3d, double, 3> },
    { "point3d",        writeGfVector<PXR_NS::GfVec3d, double, 3> },
    { "vector3d",       writeGfVector<PXR_NS::GfVec3d, double, 3> },
    { "texCoord3d",     writeGfVector<PXR_NS::GfVec3d, double, 3> },
    { "double3[]",      writeGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "color3d[]",      writeGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "normal3d[]",     writeGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "point3d[]",      writeGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "vector3d[]",     writeGfVecArray<PXR_NS::GfVec3d, double, 3> },
    { "texCoord3d[]",   writeGfVecArray<PXR_NS::GfVec3d, double, 3> },

    // GfVec4d
    { "double4",        writeGfVector<PXR_NS::GfVec4d, double, 4> },
    { "color4d",        writeGfVector<PXR_NS::GfVec4d, double, 4> },
    { "double4[]",      writeGfVecArray<PXR_NS::GfVec4d, double, 4> },
    { "color4d[]",      writeGfVecArray<PXR_NS::GfVec4d, double, 4> },

    // GfVec2f
    { "float2",         writeGfVector<PXR_NS::GfVec2f, float, 2> },
    { "texCoord2f",     writeGfVector<PXR_NS::GfVec2f, float, 2> },
    { "float2[]",       writeGfVecArray<PXR_NS::GfVec2f, float, 2> },
    { "texCoord2f[]",   writeGfVecArray<PXR_NS::GfVec2f, float, 2> },

    // GfVec3f
    { "float3",         writeGfVector<PXR_NS::GfVec3f, float, 3> },
    { "color3f",        writeGfVector<PXR_NS::GfVec3f, float, 3> },
    { "normal3f",       writeGfVector<PXR_NS::GfVec3f, float, 3> },
    { "point3f",        writeGfVector<PXR_NS::GfVec3f, float, 3> },
    { "vector3f",       writeGfVector<PXR_NS::GfVec3f, float, 3> },
    { "texCoord3f",     writeGfVector<PXR_NS::GfVec3f, float, 3> },
    { "float3[]",       writeGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "color3f[]",      writeGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "normal3f[]",     writeGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "point3f[]",      writeGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "vector3f[]",     writeGfVecArray<PXR_NS::GfVec3f, float, 3> },
    { "texCoord3f[]",   writeGfVecArray<PXR_NS::GfVec3f, float, 3> },

    // GfVec4f
    { "float4",         writeGfVector<PXR_NS::GfVec4f, float, 4> },
    { "color4f",        writeGfVector<PXR_NS::GfVec4f, float, 4> },
    { "float4[]",       writeGfVecArray<PXR_NS::GfVec4f, float, 4> },
    { "color4f[]",      writeGfVecArray<PXR_NS::GfVec4f, float, 4> },

    // GfVec2h
    { "half2",          writeGfVector<PXR_NS::GfVec2h, float, 2, PXR_NS::GfHalf> },
    { "texCoord2h",     writeGfVector<PXR_NS::GfVec2h, float, 2, PXR_NS::GfHalf> },
    { "half2[]",        writeGfVecArray<PXR_NS::GfVec2h, float, 2, PXR_NS::GfHalf> },
    { "texCoord2h[]",   writeGfVecArray<PXR_NS::GfVec2h, float, 2, PXR_NS::GfHalf> },

    // GfVec3h
    { "half3",          writeGfVector<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "color3h",        writeGfVector<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "normal3h",       writeGfVector<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "point3h",        writeGfVector<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "vector3h",       writeGfVector<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "texCoord3h",     writeGfVector<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "half3[]",        writeGfVecArray<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "color3h[]",      writeGfVecArray<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "normal3h[]",     writeGfVecArray<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "point3h[]",      writeGfVecArray<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "vector3h[]",     writeGfVecArray<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },
    { "texCoord3h[]",   writeGfVecArray<PXR_NS::GfVec3h, float, 3, PXR_NS::GfHalf> },

    // GfVec4h
    { "half4",          writeGfVector<PXR_NS::GfVec4h, float, 4, PXR_NS::GfHalf> },
    { "color4h",        writeGfVector<PXR_NS::GfVec4h, float, 4, PXR_NS::GfHalf> },
    { "half4[]",        writeGfVecArray<PXR_NS::GfVec4h, float, 4, PXR_NS::GfHalf> },
    { "color4h[]",      writeGfVecArray<PXR_NS::GfVec4h, float, 4, PXR_NS::GfHalf> },

    // GfVec2i
    { "int2",           writeGfVector<PXR_NS::GfVec2i, int, 2> },
    { "int2[]",         writeGfVecArray<PXR_NS::GfVec2i, int, 2> },

    // GfVec3i
    { "int3",           writeGfVector<PXR_NS::GfVec3i, int, 3> },
    { "int3[]",         writeGfVecArray<PXR_NS::GfVec3i, int, 3> },

    // GfVec4i
    { "int4",           writeGfVector<PXR_NS::GfVec4i, int, 4> },
    { "int4[]",         writeGfVecArray<PXR_NS::GfVec4i, int, 4> },

    // GfMatrix2d
    { "matrix2d",       writeGfMatrix<PXR_NS::GfMatrix2d, 2> },
    { "matrix2d[]",     writeGfMatrixArray<PXR_NS::GfMatrix2d, 2> },

    // GfMatrix3d
    { "matrix3d",       writeGfMatrix<PXR_NS::GfMatrix3d, 3> },
    { "matrix3d[]",     writeGfMatrixArray<PXR_NS::GfMatrix3d, 3> },

    // GfMatrix4d
    { "matrix4d",       writeGfMatrix<PXR_NS::GfMatrix4d, 4> },
    { "frame4d",        writeGfMatrix<PXR_NS::GfMatrix4d, 4> },
    { "matrix4d[]",     writeGfMatrixArray<PXR_NS::GfMatrix4d, 4> },
    { "frame4d[]",      writeGfMatrixArray<PXR_NS::GfMatrix4d, 4> },

    // GfQuatd
    { "quatd",          writeGfQuaternion<PXR_NS::GfQuatd, PXR_NS::GfVec3d, double> },
    { "quatd[]",        writeGfQuatArray<PXR_NS::GfQuatd, PXR_NS::GfVec3d, double> },

    // GfQuatf
    { "quatf",          writeGfQuaternion<PXR_NS::GfQuatf, PXR_NS::GfVec3f, float> },
    { "quatf[]",        writeGfQuatArray<PXR_NS::GfQuatf, PXR_NS::GfVec3f, float> },

    // GfQuath
    { "quath",          writeGfQuaternion<PXR_NS::GfQuath, PXR_NS::GfVec3h, float, PXR_NS::GfHalf> },
    { "quath[]",        writeGfQuatArray<PXR_NS::GfQuath, PXR_NS::GfVec3h, float, PXR_NS::GfHalf> },
};
// clang-format on

PXR_NS::GfMatrix4d getLocalTransform(const PXR_NS::UsdGeomXformable& xformable)
{
    PXR_NS::GfMatrix4d matrix(1.0);
    bool resetsXformStack = false;
    xformable.GetLocalTransformation(&matrix, &resetsXformStack, PXR_NS::UsdTimeCode::Default());
    return matrix;
}

} // namespace

int64_t getStageId(const PXR_NS::UsdStageRefPtr& stage)
{
    return static_cast<int64_t>(PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt());
}

PXR_NS::UsdStageRefPtr getStage(int64_t stageId, bool throwIfInvalid)
{
    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    PXR_NS::UsdStageCache::Id id = PXR_NS::UsdStageCache::Id::FromLongInt(stageId);
    PXR_NS::UsdStageRefPtr stage = cache.Find(id);
    if (!stage && throwIfInvalid)
    {
        throw std::invalid_argument("Invalid stage ID (" + std::to_string(stageId) + ")");
    }
    return stage;
}

PXR_NS::UsdPrim getPrimAtPath(const PXR_NS::UsdStageRefPtr& stage, const std::string& path, bool throwIfInvalid)
{
    if (!stage)
    {
        if (throwIfInvalid)
        {
            throw std::invalid_argument("Invalid stage");
        }
        return PXR_NS::UsdPrim();
    }
    if (!PXR_NS::SdfPath::IsValidPathString(path))
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::PrimPathStringError(path);
        }
        return PXR_NS::UsdPrim();
    }
    PXR_NS::UsdPrim prim = stage->GetPrimAtPath(PXR_NS::SdfPath(path));
    if (throwIfInvalid && !prim.IsValid())
    {
        throw isaacsim::common::exceptions::PrimPathError(path);
    }
    return prim;
}

PXR_NS::UsdPrim getPrimAtPath(int64_t stageId, const std::string& path, bool throwIfInvalid)
{
    return getPrimAtPath(getStage(stageId, throwIfInvalid), path, throwIfInvalid);
}

AttributeValue getAttributeValue(const PXR_NS::UsdAttribute& attribute)
{
    std::string typeName = attribute.GetTypeName().GetAsToken().GetString();
    if (attributeReaders.find(typeName) != attributeReaders.end())
    {
        return attributeReaders[typeName](attribute);
    }
    throw std::invalid_argument("Unsupported attribute type: '" + typeName + "'");
}

bool setAttributeValue(PXR_NS::UsdAttribute& attribute, const AttributeValue& value)
{
    std::string typeName = attribute.GetTypeName().GetAsToken().GetString();
    if (attributeWriters.find(typeName) != attributeWriters.end())
    {
        return attributeWriters[typeName](attribute, value, typeName);
    }
    throw std::invalid_argument("Unsupported attribute type: '" + typeName + "'");
}

PXR_NS::UsdGeomXformable getXformableAtPath(int64_t stageId,
                                            const std::string& path,
                                            bool throwIfInvalid,
                                            bool throwIfXformOpOrderInvalid)
{
    PXR_NS::UsdGeomXformable xformable(getPrimAtPath(stageId, path, throwIfInvalid));
    if (!xformable)
    {
        if (throwIfInvalid)
        {
            throw std::invalid_argument("Prim at path '" + path + "' is not Xformable");
        }
    }
    if (throwIfXformOpOrderInvalid)
    {
        bool resetsXformStack = false;
        const std::vector<PXR_NS::UsdGeomXformOp> ops = xformable.GetOrderedXformOps(&resetsXformStack);
        if (ops.size() != 3 || ops[0].GetOpType() != PXR_NS::UsdGeomXformOp::TypeTranslate ||
            ops[1].GetOpType() != PXR_NS::UsdGeomXformOp::TypeOrient ||
            ops[2].GetOpType() != PXR_NS::UsdGeomXformOp::TypeScale)
        {
            std::string currentOps = "[";
            for (size_t i = 0; i < ops.size(); ++i)
            {
                if (i > 0)
                {
                    currentOps += ", ";
                }
                currentOps += ops[i].GetOpName().GetString();
            }
            currentOps += "]";
            throw std::invalid_argument(
                "Prim at path '" + path +
                "' does not have the canonical transform ops order [xformOp:translate, xformOp:orient, xformOp:scale] "
                "but instead has the following transform ops order: " +
                currentOps);
        }
    }
    return xformable;
}

PXR_NS::GfVec3d getXformLocalScale(const PXR_NS::UsdGeomXformable& xformable)
{
    PXR_NS::GfTransform transform(getLocalTransform(xformable));
    return transform.GetScale();
}

void setXformLocalScale(PXR_NS::UsdGeomXformable xformable, const PXR_NS::GfVec3d& scale)
{
    bool resetsXformStack = false;
    const std::vector<PXR_NS::UsdGeomXformOp> ops = xformable.GetOrderedXformOps(&resetsXformStack);
    ops[2].Set(scale);
}

std::pair<PXR_NS::GfVec3d, PXR_NS::GfQuatd> getXformLocalPose(const PXR_NS::UsdGeomXformable& xformable)
{
    PXR_NS::GfMatrix4d matrix = getLocalTransform(xformable);
    matrix.Orthonormalize();
    return { matrix.ExtractTranslation(), matrix.ExtractRotationQuat() };
}

void setXformLocalPose(PXR_NS::UsdGeomXformable xformable,
                       const std::optional<PXR_NS::GfVec3d>& translation,
                       const std::optional<PXR_NS::GfQuatd>& orientation)
{
    bool resetsXformStack = false;
    const std::vector<PXR_NS::UsdGeomXformOp> ops = xformable.GetOrderedXformOps(&resetsXformStack);
    if (translation.has_value())
    {
        ops[0].Set(*translation);
    }
    if (orientation.has_value())
    {
        ops[1].Set(orientation->GetNormalized());
    }
}

std::pair<PXR_NS::GfVec3d, PXR_NS::GfQuatd> getXformWorldPose(const PXR_NS::UsdGeomXformable& xformable)
{
    PXR_NS::GfMatrix4d matrix = xformable.ComputeLocalToWorldTransform(PXR_NS::UsdTimeCode::Default());
    matrix.Orthonormalize();
    return { matrix.ExtractTranslation(), matrix.ExtractRotationQuat() };
}

void setXformWorldPose(PXR_NS::UsdGeomXformable xformable,
                       const std::optional<PXR_NS::GfVec3d>& position,
                       const std::optional<PXR_NS::GfQuatd>& orientation)
{
    PXR_NS::GfMatrix4d worldMatrix = xformable.ComputeLocalToWorldTransform(PXR_NS::UsdTimeCode::Default());
    PXR_NS::GfTransform worldTransform(worldMatrix);
    if (position.has_value())
    {
        worldTransform.SetTranslation(*position);
    }
    if (orientation.has_value())
    {
        worldTransform.SetRotation(PXR_NS::GfRotation(orientation->GetNormalized()));
    }
    // Walk the full ancestor chain so that non-Xformable prims (e.g. Scope) that
    // inherit their world transform from above are handled correctly.
    PXR_NS::GfMatrix4d parentWorldMatrix(1.0);
    const PXR_NS::UsdPrim parent = xformable.GetPrim().GetParent();
    if (parent.IsValid() && !parent.IsPseudoRoot())
    {
        PXR_NS::UsdGeomXformCache xformCache(PXR_NS::UsdTimeCode::Default());
        parentWorldMatrix = xformCache.GetLocalToWorldTransform(parent);
    }
    bool resetsXformStack = false;
    const std::vector<PXR_NS::UsdGeomXformOp> ops = xformable.GetOrderedXformOps(&resetsXformStack);
    const PXR_NS::GfTransform localTransform(worldTransform.GetMatrix() * parentWorldMatrix.GetInverse());
    if (position.has_value())
    {
        ops[0].Set(localTransform.GetTranslation());
    }
    if (orientation.has_value())
    {
        ops[1].Set(localTransform.GetRotation().GetQuat());
    }
}

} // namespace details
} // namespace openusd
} // namespace usd
} // namespace foundation
} // namespace isaacsim

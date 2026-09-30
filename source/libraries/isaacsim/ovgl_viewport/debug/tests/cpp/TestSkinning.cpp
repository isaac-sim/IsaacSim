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

#include "Skinning.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

using Bytes = std::vector<uint8_t>;
using Attributes = std::unordered_map<std::string, Bytes>;

template <class T>
Bytes bytesOf(const std::vector<T>& values)
{
    Bytes output(values.size() * sizeof(T));
    if (!output.empty())
        std::memcpy(output.data(), values.data(), output.size());
    return output;
}

Bytes strings(std::initializer_list<const char*> values)
{
    Bytes output;
    for (const char* value : values)
    {
        const size_t size = std::strlen(value);
        output.insert(output.end(), value, value + size);
        output.push_back(0);
    }
    return output;
}

std::string key(const std::string& path, const std::string& attribute)
{
    return path + "\n" + attribute;
}

bool near(float actual, float expected)
{
    return std::fabs(actual - expected) <= 2e-5f;
}

bool expect(const char* label, float actual, float expected)
{
    if (near(actual, expected))
        return true;
    // logging: allow printf - standalone Carb/Omni-free native test failure output.
    std::fprintf(stderr, "%s: got %.9g, expected %.9g\n", label, actual, expected);
    return false;
}

} // namespace

int main()
{
    using Matrix = std::array<double, 16>;
    const Matrix identity = {
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    };
    Attributes attributes;
    attributes[key("/World/Mesh", "skel:skeleton")] = strings({ "/World/Skel" });
    attributes[key("/World/Mesh", "skel:joints")] = strings({ "root" });
    attributes[key("/World/Mesh", "primvars:skel:jointIndices")] = bytesOf(std::vector<int>{ 0, 0, 0 });
    attributes[key("/World/Mesh", "primvars:skel:jointWeights")] = bytesOf(std::vector<float>{ 1, 1, 1 });
    attributes[key("/World/Mesh", "primvars:skel:geomBindTransform")] = bytesOf(std::vector<Matrix>{ identity });
    attributes[key("/World/Mesh", "worldMatrix")] = bytesOf(std::vector<Matrix>{ identity });

    attributes[key("/World/Skel", "joints")] = strings({ "root" });
    attributes[key("/World/Skel", "bindTransforms")] = bytesOf(std::vector<Matrix>{ identity });
    attributes[key("/World/Skel", "restTransforms")] = bytesOf(std::vector<Matrix>{ identity });
    attributes[key("/World/Skel", "skel:animationSource")] = strings({ "/World/Anim" });
    attributes[key("/World/Skel", "worldMatrix")] = bytesOf(std::vector<Matrix>{ identity });

    constexpr float halfSqrtTwo = 0.7071067811865476f;
    attributes[key("/World/Anim", "joints")] = strings({ "root" });
    attributes[key("/World/Anim", "translations")] = bytesOf(std::vector<float>{ 1, 2, 3 });
    attributes[key("/World/Anim", "rotations")] = bytesOf(std::vector<float>{ 0, 0, halfSqrtTwo, halfSqrtTwo });
    attributes[key("/World/Anim", "scales")] = bytesOf(std::vector<float>{ 1, 1, 1 });

    isaacsim::ovgl_viewport::debug::details::ovgl::UsdSkelDeformer deformer(
        [&](const std::string& path, const std::string& attribute, Bytes& output)
        {
            const auto found = attributes.find(key(path, attribute));
            if (found == attributes.end())
                return false;
            output = found->second;
            return true;
        });

    std::vector<float> points = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };
    std::vector<float> normals = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    const int corners[] = { 0, 1, 2 };
    if (!deformer.deform("/World/Mesh", corners, 3, points, normals))
    {
        // logging: allow printf - standalone Carb/Omni-free native test failure output.
        std::fprintf(stderr, "deform unexpectedly rejected complete binding\n");
        return 1;
    }

    bool ok = true;
    const float expected_points[] = { 1, 3, 3, 0, 2, 3, 1, 2, 3 };
    const float expected_normals[] = { 0, 1, 0, -1, 0, 0, 0, 0, 1 };
    for (size_t index = 0; index < points.size(); ++index)
        ok &= expect("point", points[index], expected_points[index]);
    for (size_t index = 0; index < normals.size(); ++index)
        ok &= expect("face-varying normal", normals[index], expected_normals[index]);
    return ok ? 0 : 1;
}

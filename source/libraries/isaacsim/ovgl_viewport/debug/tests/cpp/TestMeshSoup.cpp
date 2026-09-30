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

#include "MeshSoup.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

bool expectVector(const char* label, const std::vector<float>& actual, const std::vector<float>& expected)
{
    if (actual == expected)
        return true;
    // logging: allow printf - standalone Carb/Omni-free native test failure output.
    std::fprintf(stderr, "%s mismatch: got %zu floats, expected %zu\n", label, actual.size(), expected.size());
    return false;
}

bool testQuadFanWithAuthoredStreams()
{
    const float points[] = { 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0 };
    const int counts[] = { 4 };
    const int indices[] = { 0, 1, 2, 3 };
    const float normals[] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, -1, 0, 0 };
    const float texcoords[] = { 0, 0, 1, 0, 1, 1, 0, 1 };
    const int texcoord_indices[] = { 3, 2, 1, 0 };
    std::vector<float> output_points, output_normals, output_texcoords;
    isaacsim::ovgl_viewport::debug::details::ovgl::buildMeshSoup(points, 4, counts, 1, indices, 4, normals, 4,
                                                                 texcoords, 4, texcoord_indices, 4, output_points,
                                                                 output_normals, output_texcoords);

    bool ok = true;
    ok &= expectVector("points", output_points,
                       {
                           0,
                           0,
                           0,
                           1,
                           0,
                           0,
                           1,
                           1,
                           0,
                           0,
                           0,
                           0,
                           1,
                           1,
                           0,
                           0,
                           1,
                           0,
                       });
    ok &= expectVector("face-varying normals", output_normals,
                       {
                           1,
                           0,
                           0,
                           0,
                           1,
                           0,
                           0,
                           0,
                           1,
                           1,
                           0,
                           0,
                           0,
                           0,
                           1,
                           -1,
                           0,
                           0,
                       });
    ok &= expectVector("indexed face-varying UVs", output_texcoords,
                       {
                           0,
                           1,
                           1,
                           1,
                           1,
                           0,
                           0,
                           1,
                           1,
                           0,
                           0,
                           0,
                       });
    return ok;
}

// Stock HdMeshUtil::ComputeTriangleIndices skips degenerate faces (count < 3)
// and keeps triangulating the faces after them (pxr/imaging/hd/meshUtil.cpp).
bool testDegenerateFaceSkipped()
{
    const float points[] = {
        0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 2, 0, 0, 3, 0, 0, 3, 1, 0, 2, 1, 0,
    };
    const int counts[] = { 4, 2, 4 };
    const int indices[] = { 0, 1, 2, 3, 0, 1, 4, 5, 6, 7 };
    std::vector<float> output_points, output_normals, output_texcoords;
    isaacsim::ovgl_viewport::debug::details::ovgl::buildMeshSoup(points, 8, counts, 3, indices, 10, nullptr, 0, nullptr,
                                                                 0, nullptr, 0, output_points, output_normals,
                                                                 output_texcoords);

    return expectVector(
        "points around degenerate face", output_points,
        {
            0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 2, 0, 0, 3, 0, 0, 3, 1, 0, 2, 0, 0, 3, 1, 0, 2, 1, 0,
        });
}

// The out-of-range corner guard is a malformed-buffer stop, not a skip: a
// face whose corners run past faceVertexIndices must end triangulation.
bool testTruncatedIndexBufferStops()
{
    const float points[] = { 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0 };
    const int counts[] = { 4, 4 };
    const int indices[] = { 0, 1, 2, 3, 0, 1 };
    std::vector<float> output_points, output_normals, output_texcoords;
    isaacsim::ovgl_viewport::debug::details::ovgl::buildMeshSoup(points, 4, counts, 2, indices, 6, nullptr, 0, nullptr,
                                                                 0, nullptr, 0, output_points, output_normals,
                                                                 output_texcoords);

    return expectVector("points after truncated buffer", output_points,
                        {
                            0,
                            0,
                            0,
                            1,
                            0,
                            0,
                            1,
                            1,
                            0,
                            0,
                            0,
                            0,
                            1,
                            1,
                            0,
                            0,
                            1,
                            0,
                        });
}

// A mesh that authors NO normals is smooth-shaded, not faceted: official
// ovrtx 0.4 smooth-shades one, and ovgl used to emit a flat per-face normal.
// Two unit quads folded 90 degrees about the y=1 edge, so the answer is exact:
// the two corners shared by both faces average to normalize(0,-1,1), and the
// corners belonging to one face keep that face's own normal.
bool testUnauthoredNormalsAreSmooth()
{
    const float points[] = {
        0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, // quad A, in z=0  -> +Z
        1, 1, 1, 0, 1, 1, // quad B rises in y=1 -> -Y
    };
    const int counts[] = { 4, 4 };
    const int indices[] = { 0, 1, 2, 3, 3, 2, 4, 5 };
    std::vector<float> output_points, output_normals, output_texcoords;
    isaacsim::ovgl_viewport::debug::details::ovgl::buildMeshSoup(points, 6, counts, 2, indices, 8, nullptr, 0, nullptr,
                                                                 0, nullptr, 0, output_points, output_normals,
                                                                 output_texcoords);

    const float s = 0.70710678f; // normalize(0,-1,1)
    const float expected[] = {
        0, 0,  1, 0, 0,  1, 0, -s, s, // A tri 0: v0 v1 v2
        0, 0,  1, 0, -s, s, 0, -s, s, // A tri 1: v0 v2 v3
        0, -s, s, 0, -s, s, 0, -1, 0, // B tri 0: v3 v2 v4
        0, -s, s, 0, -1, 0, 0, -1, 0, // B tri 1: v3 v4 v5
    };
    const size_t expected_count = sizeof(expected) / sizeof(expected[0]);
    if (output_normals.size() != expected_count)
    {
        std::fprintf(stderr, "smooth normals: got %zu floats, expected %zu\n", output_normals.size(), expected_count);
        return false;
    }
    for (size_t i = 0; i < expected_count; ++i)
    {
        if (std::fabs(output_normals[i] - expected[i]) > 1e-5f)
        {
            std::fprintf(stderr, "smooth normals[%zu]: got %f, expected %f\n", i, output_normals[i], expected[i]);
            return false;
        }
    }
    return true;
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testQuadFanWithAuthoredStreams();
    ok &= testDegenerateFaceSkipped();
    ok &= testTruncatedIndexBufferStops();
    ok &= testUnauthoredNormalsAreSmooth();
    return ok ? 0 : 1;
}

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

#include <algorithm>
#include <cmath>

namespace isaacsim
{
namespace ovgl_viewport
{
namespace debug
{
namespace details
{
namespace ovgl
{
namespace
{

void pushFlatTriangle(
    std::vector<float>& points, std::vector<float>& normals, const float a[3], const float b[3], const float c[3])
{
    points.insert(points.end(), a, a + 3);
    points.insert(points.end(), b, b + 3);
    points.insert(points.end(), c, c + 3);
    const float edge1[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
    const float edge2[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
    float normal[3] = {
        edge1[1] * edge2[2] - edge1[2] * edge2[1],
        edge1[2] * edge2[0] - edge1[0] * edge2[2],
        edge1[0] * edge2[1] - edge1[1] * edge2[0],
    };
    float length = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    if (length < 1e-12f)
        length = 1.0f;
    for (float& component : normal)
        component /= length;
    for (int vertex = 0; vertex < 3; ++vertex)
        normals.insert(normals.end(), normal, normal + 3);
}

} // namespace

void buildMeshSoup(const float* points,
                   size_t point_count,
                   const int* face_counts,
                   size_t face_count,
                   const int* face_indices,
                   size_t face_index_count,
                   const float* normals,
                   size_t normal_count,
                   const float* texcoords,
                   size_t texcoord_count,
                   const int* texcoord_indices,
                   size_t texcoord_index_count,
                   std::vector<float>& output_points,
                   std::vector<float>& output_normals,
                   std::vector<float>& output_texcoords)
{
    const bool uv_indexed = texcoords && texcoord_indices && texcoord_index_count > 0;
    const bool uv_per_point = texcoords && !uv_indexed && texcoord_count == point_count && point_count > 0;
    const bool uv_per_corner = texcoords && !uv_indexed && texcoord_count == face_index_count && face_index_count > 0;
    const bool uv_index_per_corner = uv_indexed && texcoord_index_count == face_index_count;

    auto lookup_uv = [&](size_t element) -> const float*
    { return element < texcoord_count ? texcoords + 2 * element : nullptr; };
    auto push_uv = [&](int point_index, size_t corner)
    {
        const float* st = nullptr;
        if (uv_indexed)
        {
            const size_t slot = uv_index_per_corner ? corner : static_cast<size_t>(std::max(point_index, 0));
            if (slot < texcoord_index_count)
            {
                const int element = texcoord_indices[slot];
                if (element >= 0)
                    st = lookup_uv(static_cast<size_t>(element));
            }
        }
        else if (uv_per_corner && corner < texcoord_count)
        {
            st = texcoords + 2 * corner;
        }
        else if (uv_per_point && point_index >= 0)
        {
            st = lookup_uv(static_cast<size_t>(point_index));
        }
        output_texcoords.push_back(st ? st[0] : 0.0f);
        output_texcoords.push_back(st ? st[1] : 0.0f);
    };
    auto point = [&](int index, float output[3])
    {
        if (index < 0 || static_cast<size_t>(index) >= point_count)
        {
            output[0] = output[1] = output[2] = 0.0f;
            return;
        }
        output[0] = points[3 * index + 0];
        output[1] = points[3 * index + 1];
        output[2] = points[3 * index + 2];
    };

    /* USD normals may be vertex/varying (#points) or faceVarying
     * (#faceVertices). Preserve either authored stream while expanding the
     * polygon fan. Apple's meshes use indexed face-varying UVs and
     * face-varying normals; discarding the latter produces visible faceting. */
    const bool normals_per_corner = normals && normal_count == face_index_count && face_index_count > 0;
    const bool normals_per_point = normals && normal_count == point_count && point_count > 0;
    const bool authored_normals = normals_per_corner || normals_per_point;
    auto normal = [&](int point_index, size_t corner, float output[3])
    {
        const float* source = nullptr;
        if (normals_per_corner && corner < normal_count)
            source = normals + 3 * corner;
        else if (normals_per_point && point_index >= 0 && static_cast<size_t>(point_index) < normal_count)
            source = normals + 3 * static_cast<size_t>(point_index);
        if (!source)
        {
            output[0] = output[1] = 0.0f;
            output[2] = 1.0f;
            return;
        }
        output[0] = source[0];
        output[1] = source[1];
        output[2] = source[2];
    };

    /* No authored normals: SMOOTH-shade rather than facet.
     *
     * Official ovrtx 0.4 smooth-shades a polygonal mesh that authors no
     * normals; ovgl faceted it, which on dense tessellation reads as a blocky,
     * quantised surface. Measured on a 48x96 sphere probe under the g1_view
     * rig: with normals authored both stacks render a clean highlight, with
     * them absent official was unchanged and ovgl broke into visible ~4 px
     * facets. Nothing in the G1 hits this (all 98 of its meshes author
     * faceVarying normals, which the branch above already honours) -- it is
     * every OTHER unnormaled asset that was faceted.
     *
     * Area-weighted vertex normals, the standard construction: accumulate each
     * face's UNNORMALISED Newell normal into its corners and normalize once at
     * the end. Newell rather than a single edge cross product because it is
     * correct for non-planar n-gons, and its magnitude is twice the polygon
     * area -- so the area weighting falls out for free, with no per-face sqrt.
     * O(faceIndices + points), one sqrt per point. */
    std::vector<float> computed_normals;
    if (!authored_normals && point_count > 0 && face_count > 0)
    {
        computed_normals.assign(point_count * 3, 0.0f);
        size_t scan = 0;
        for (size_t face = 0; face < face_count; ++face)
        {
            const int count = face_counts[face];
            if (count < 3)
            {
                if (count > 0)
                    scan += static_cast<size_t>(count);
                continue;
            }
            if (scan + static_cast<size_t>(count) > face_index_count)
                break;
            float faceNormal[3] = { 0.0f, 0.0f, 0.0f };
            for (int k = 0; k < count; ++k)
            {
                float current[3], next[3];
                point(face_indices[scan + static_cast<size_t>(k)], current);
                point(face_indices[scan + static_cast<size_t>((k + 1) % count)], next);
                faceNormal[0] += (current[1] - next[1]) * (current[2] + next[2]);
                faceNormal[1] += (current[2] - next[2]) * (current[0] + next[0]);
                faceNormal[2] += (current[0] - next[0]) * (current[1] + next[1]);
            }
            for (int k = 0; k < count; ++k)
            {
                const int index = face_indices[scan + static_cast<size_t>(k)];
                if (index < 0 || static_cast<size_t>(index) >= point_count)
                    continue;
                computed_normals[3 * static_cast<size_t>(index) + 0] += faceNormal[0];
                computed_normals[3 * static_cast<size_t>(index) + 1] += faceNormal[1];
                computed_normals[3 * static_cast<size_t>(index) + 2] += faceNormal[2];
            }
            scan += static_cast<size_t>(count);
        }
        for (size_t vertex = 0; vertex < point_count; ++vertex)
        {
            float* n = &computed_normals[3 * vertex];
            const float length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            /* An isolated or fully degenerate vertex contributes no area; +Z
             * matches the fallback the authored-normal path already uses. */
            if (length < 1e-12f)
            {
                n[0] = n[1] = 0.0f;
                n[2] = 1.0f;
            }
            else
            {
                n[0] /= length;
                n[1] /= length;
                n[2] /= length;
            }
        }
    }
    auto push_computed_normal = [&](int point_index)
    {
        if (point_index >= 0 && static_cast<size_t>(point_index) < point_count)
        {
            const float* n = &computed_normals[3 * static_cast<size_t>(point_index)];
            output_normals.insert(output_normals.end(), n, n + 3);
        }
        else
        {
            output_normals.push_back(0.0f);
            output_normals.push_back(0.0f);
            output_normals.push_back(1.0f);
        }
    };

    size_t corner = 0;
    for (size_t face = 0; face < face_count; ++face)
    {
        const int count = face_counts[face];
        if (count < 3)
        {
            // Stock HdMeshUtil skips degenerate faces but still walks past
            // their corners (pxr/imaging/hd/meshUtil.cpp, ComputeTriangleIndices).
            if (count > 0)
                corner += static_cast<size_t>(count);
            continue;
        }
        if (corner + static_cast<size_t>(count) > face_index_count)
            break;
        const int first_index = face_indices[corner];
        float first[3];
        point(first_index, first);
        for (int offset = 1; offset < count - 1; ++offset)
        {
            const int second_index = face_indices[corner + offset];
            const int third_index = face_indices[corner + offset + 1];
            float second[3], third[3];
            point(second_index, second);
            point(third_index, third);
            if (authored_normals)
            {
                float first_normal[3], second_normal[3], third_normal[3];
                normal(first_index, corner, first_normal);
                normal(second_index, corner + static_cast<size_t>(offset), second_normal);
                normal(third_index, corner + static_cast<size_t>(offset) + 1, third_normal);
                output_points.insert(output_points.end(), first, first + 3);
                output_points.insert(output_points.end(), second, second + 3);
                output_points.insert(output_points.end(), third, third + 3);
                output_normals.insert(output_normals.end(), first_normal, first_normal + 3);
                output_normals.insert(output_normals.end(), second_normal, second_normal + 3);
                output_normals.insert(output_normals.end(), third_normal, third_normal + 3);
            }
            else if (!computed_normals.empty())
            {
                output_points.insert(output_points.end(), first, first + 3);
                output_points.insert(output_points.end(), second, second + 3);
                output_points.insert(output_points.end(), third, third + 3);
                push_computed_normal(first_index);
                push_computed_normal(second_index);
                push_computed_normal(third_index);
            }
            else
            {
                pushFlatTriangle(output_points, output_normals, first, second, third);
            }
            push_uv(first_index, corner);
            push_uv(second_index, corner + static_cast<size_t>(offset));
            push_uv(third_index, corner + static_cast<size_t>(offset) + 1);
        }
        corner += static_cast<size_t>(count);
    }
}

} // namespace ovgl
} // namespace details
} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim

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

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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

/* Portable UsdSkel evaluator for the ovstage -> OVGL bridge.
 *
 * OVGL deliberately has no OpenUSD dependency. OVPopulation mirrors the
 * authored UsdSkel arrays and relationships into ovstage, and this class
 * consumes only those portable columns. The implementation follows OpenUSD's
 * row-vector linear-blend convention.
 */
class UsdSkelDeformer
{
public:
    using AttributeReader =
        std::function<bool(const std::string& prim_path, const std::string& attribute_name, std::vector<uint8_t>& output)>;

    explicit UsdSkelDeformer(AttributeReader reader);

    /* Deform points and authored normals in mesh-local space. Normals may be
     * vertex-rate (#points) or face-varying (#faceVertexIndices). Returns true
     * only when a complete inherited UsdSkel binding was evaluated. */
    bool deform(const std::string& mesh_path,
                const int* face_vertex_indices,
                size_t face_vertex_index_count,
                std::vector<float>& points,
                std::vector<float>& normals);

private:
    struct SkeletonPose;

    bool _readAttribute(const std::string& path, const std::string& name, std::vector<uint8_t>& output) const;
    bool _readInheritedAttribute(const std::string& path, const std::string& name, std::vector<uint8_t>& output) const;
    std::string _readInheritedTarget(const std::string& path, const std::string& relationship) const;
    const SkeletonPose* _getPose(const std::string& skeletonPath);

    AttributeReader m_reader;
    std::unordered_map<std::string, std::shared_ptr<SkeletonPose>> m_poseCache;
};

} // namespace ovgl
} // namespace details
} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim

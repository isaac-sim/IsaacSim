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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <unordered_map>

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

using Matrix4d = std::array<double, 16>;

constexpr Matrix4d g_kIdentity = {
    1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
};

void skinningDiagnostic(const std::string& mesh_path, const char* phase, size_t first = 0, size_t second = 0)
{
    if (!std::getenv("OVGL_DEBUG_SKEL"))
        return;
    // logging: allow printf - OVGL is a Carb/Omni-free portable library and
    // this developer-only trace is disabled unless explicitly requested.
    std::fprintf(stderr, "[ovgl-skel] %s: %s (%zu, %zu)\n", mesh_path.c_str(), phase, first, second);
}

std::string firstNulString(const std::vector<uint8_t>& bytes)
{
    if (bytes.empty())
        return {};
    const char* begin = reinterpret_cast<const char*>(bytes.data());
    const void* end = std::memchr(begin, '\0', bytes.size());
    const size_t length = end ? static_cast<const char*>(end) - begin : bytes.size();
    return std::string(begin, length);
}

std::vector<std::string> splitNulStrings(const std::vector<uint8_t>& bytes)
{
    std::vector<std::string> output;
    size_t begin = 0;
    while (begin < bytes.size())
    {
        size_t end = begin;
        while (end < bytes.size() && bytes[end] != 0)
            ++end;
        output.emplace_back(reinterpret_cast<const char*>(bytes.data() + begin), end - begin);
        begin = end + 1;
    }
    return output;
}

template <class T>
bool copyTrivialArray(const std::vector<uint8_t>& bytes, std::vector<T>& output)
{
    if (bytes.empty() || bytes.size() % sizeof(T) != 0)
        return false;
    output.resize(bytes.size() / sizeof(T));
    std::memcpy(output.data(), bytes.data(), bytes.size());
    return true;
}

bool copyMatrixArray(const std::vector<uint8_t>& bytes, std::vector<Matrix4d>& output)
{
    return copyTrivialArray(bytes, output);
}

float halfToFloat(uint16_t half)
{
    const uint32_t sign = static_cast<uint32_t>(half & 0x8000u) << 16;
    const uint32_t exponent = (half >> 10) & 0x1fu;
    uint32_t mantissa = half & 0x03ffu;
    uint32_t bits = 0;
    if (exponent == 0)
    {
        if (mantissa == 0)
        {
            bits = sign;
        }
        else
        {
            uint32_t normalized_exponent = 127u - 15u + 1u;
            while ((mantissa & 0x0400u) == 0)
            {
                mantissa <<= 1;
                --normalized_exponent;
            }
            mantissa &= 0x03ffu;
            bits = sign | (normalized_exponent << 23) | (mantissa << 13);
        }
    }
    else if (exponent == 0x1fu)
    {
        bits = sign | 0x7f800000u | (mantissa << 13);
    }
    else
    {
        bits = sign | ((exponent + (127u - 15u)) << 23) | (mantissa << 13);
    }
    float output;
    std::memcpy(&output, &bits, sizeof(output));
    return output;
}

bool readNumericValues(const std::vector<uint8_t>& bytes, size_t count, std::vector<float>& output)
{
    output.resize(count);
    if (bytes.size() == count * sizeof(float))
    {
        std::memcpy(output.data(), bytes.data(), bytes.size());
        return true;
    }
    if (bytes.size() == count * sizeof(double))
    {
        for (size_t i = 0; i < count; ++i)
        {
            double value;
            std::memcpy(&value, bytes.data() + i * sizeof(double), sizeof(value));
            output[i] = static_cast<float>(value);
        }
        return true;
    }
    if (bytes.size() == count * sizeof(uint16_t))
    {
        for (size_t i = 0; i < count; ++i)
        {
            uint16_t value;
            std::memcpy(&value, bytes.data() + i * sizeof(value), sizeof(value));
            output[i] = halfToFloat(value);
        }
        return true;
    }
    output.clear();
    return false;
}

void multiply(const Matrix4d& a, const Matrix4d& b, Matrix4d& output)
{
    Matrix4d result{};
    for (int row = 0; row < 4; ++row)
    {
        for (int column = 0; column < 4; ++column)
        {
            for (int k = 0; k < 4; ++k)
                result[row * 4 + column] += a[row * 4 + k] * b[k * 4 + column];
        }
    }
    output = result;
}

bool invertAffine(const Matrix4d& matrix, Matrix4d& output)
{
    const double a00 = matrix[0], a01 = matrix[1], a02 = matrix[2];
    const double a10 = matrix[4], a11 = matrix[5], a12 = matrix[6];
    const double a20 = matrix[8], a21 = matrix[9], a22 = matrix[10];

    const double c00 = a11 * a22 - a12 * a21;
    const double c01 = -(a10 * a22 - a12 * a20);
    const double c02 = a10 * a21 - a11 * a20;
    const double c10 = -(a01 * a22 - a02 * a21);
    const double c11 = a00 * a22 - a02 * a20;
    const double c12 = -(a00 * a21 - a01 * a20);
    const double c20 = a01 * a12 - a02 * a11;
    const double c21 = -(a00 * a12 - a02 * a10);
    const double c22 = a00 * a11 - a01 * a10;
    const double determinant = a00 * c00 + a01 * c01 + a02 * c02;
    if (std::fabs(determinant) < 1e-12)
        return false;

    const double inverse_determinant = 1.0 / determinant;
    output = {
        c00 * inverse_determinant,
        c10 * inverse_determinant,
        c20 * inverse_determinant,
        0,
        c01 * inverse_determinant,
        c11 * inverse_determinant,
        c21 * inverse_determinant,
        0,
        c02 * inverse_determinant,
        c12 * inverse_determinant,
        c22 * inverse_determinant,
        0,
        0,
        0,
        0,
        1,
    };
    const double tx = matrix[12], ty = matrix[13], tz = matrix[14];
    output[12] = -(tx * output[0] + ty * output[4] + tz * output[8]);
    output[13] = -(tx * output[1] + ty * output[5] + tz * output[9]);
    output[14] = -(tx * output[2] + ty * output[6] + tz * output[10]);
    return true;
}

Matrix4d normalMatrix(const Matrix4d& matrix)
{
    Matrix4d inverse;
    if (!invertAffine(matrix, inverse))
        return matrix;
    Matrix4d output{};
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            output[row * 4 + column] = inverse[column * 4 + row];
    return output;
}

void transformPoint(const Matrix4d& matrix, const float point[3], float output[3])
{
    const double x = point[0], y = point[1], z = point[2];
    output[0] = static_cast<float>(matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12]);
    output[1] = static_cast<float>(matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13]);
    output[2] = static_cast<float>(matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14]);
}

void transformDirection(const Matrix4d& matrix, const float direction[3], double output[3])
{
    const double x = direction[0], y = direction[1], z = direction[2];
    output[0] = matrix[0] * x + matrix[4] * y + matrix[8] * z;
    output[1] = matrix[1] * x + matrix[5] * y + matrix[9] * z;
    output[2] = matrix[2] * x + matrix[6] * y + matrix[10] * z;
}

void normalize(double direction[3])
{
    const double length =
        std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    if (length <= 1e-12)
        return;
    direction[0] /= length;
    direction[1] /= length;
    direction[2] /= length;
}

void quaternionMatrix(const float quaternion_xyzw[4], Matrix4d& output)
{
    double x = quaternion_xyzw[0], y = quaternion_xyzw[1];
    double z = quaternion_xyzw[2], w = quaternion_xyzw[3];
    const double length = std::sqrt(x * x + y * y + z * z + w * w);
    if (length > 1e-12)
    {
        x /= length;
        y /= length;
        z /= length;
        w /= length;
    }
    else
    {
        x = y = z = 0.0;
        w = 1.0;
    }
    /* Row-vector rotation matrix, matching GfMatrix4d/UsdSkel. */
    output = {
        1 - 2 * y * y - 2 * z * z,
        2 * x * y + 2 * w * z,
        2 * x * z - 2 * w * y,
        0,
        2 * x * y - 2 * w * z,
        1 - 2 * x * x - 2 * z * z,
        2 * y * z + 2 * w * x,
        0,
        2 * x * z + 2 * w * y,
        2 * y * z - 2 * w * x,
        1 - 2 * x * x - 2 * y * y,
        0,
        0,
        0,
        0,
        1,
    };
}

Matrix4d makeTrs(const float translation[3], const float rotation_xyzw[4], const float scale[3])
{
    Matrix4d output;
    quaternionMatrix(rotation_xyzw, output);
    for (int column = 0; column < 3; ++column)
    {
        output[column] *= scale[0];
        output[4 + column] *= scale[1];
        output[8 + column] *= scale[2];
    }
    output[12] = translation[0];
    output[13] = translation[1];
    output[14] = translation[2];
    return output;
}

int jointParent(const std::vector<std::string>& joints,
                size_t joint_index,
                const std::unordered_map<std::string, int>& indices)
{
    const std::string& joint = joints[joint_index];
    const size_t slash = joint.rfind('/');
    if (slash == std::string::npos || slash == 0)
        return -1;
    const auto found = indices.find(joint.substr(0, slash));
    return found == indices.end() ? -1 : found->second;
}

} // namespace

struct UsdSkelDeformer::SkeletonPose
{
    bool valid = false;
    std::vector<std::string> joints;
    std::unordered_map<std::string, int> joint_indices;
    std::vector<Matrix4d> skin_matrices;
    std::vector<Matrix4d> normal_matrices;
    Matrix4d world = g_kIdentity;
};

UsdSkelDeformer::UsdSkelDeformer(AttributeReader reader) : m_reader(std::move(reader))
{
}

bool UsdSkelDeformer::_readAttribute(const std::string& path, const std::string& name, std::vector<uint8_t>& output) const
{
    output.clear();
    return m_reader && m_reader(path, name, output) && !output.empty();
}

bool UsdSkelDeformer::_readInheritedAttribute(const std::string& path,
                                              const std::string& name,
                                              std::vector<uint8_t>& output) const
{
    for (std::string current = path; !current.empty();)
    {
        if (_readAttribute(current, name, output))
            return true;
        const size_t slash = current.rfind('/');
        if (slash == std::string::npos || slash == 0)
            break;
        current.resize(slash);
    }
    output.clear();
    return false;
}

std::string UsdSkelDeformer::_readInheritedTarget(const std::string& path, const std::string& relationship) const
{
    std::vector<uint8_t> bytes;
    if (!_readInheritedAttribute(path, relationship, bytes))
        return {};
    std::string target = firstNulString(bytes);
    const size_t dot = target.rfind('.');
    if (dot != std::string::npos)
        target.resize(dot);
    return !target.empty() && target.front() == '/' ? target : std::string();
}

const UsdSkelDeformer::SkeletonPose* UsdSkelDeformer::_getPose(const std::string& skeletonPath)
{
    const auto existing = m_poseCache.find(skeletonPath);
    if (existing != m_poseCache.end())
        return existing->second->valid ? existing->second.get() : nullptr;

    std::shared_ptr<SkeletonPose>& cached = m_poseCache[skeletonPath];
    cached = std::make_shared<SkeletonPose>();
    SkeletonPose& pose = *cached;
    std::vector<uint8_t> bytes;
    if (!_readAttribute(skeletonPath, "joints", bytes))
        return nullptr;
    pose.joints = splitNulStrings(bytes);
    if (pose.joints.empty())
        return nullptr;
    for (size_t index = 0; index < pose.joints.size(); ++index)
        pose.joint_indices.emplace(pose.joints[index], static_cast<int>(index));

    std::vector<Matrix4d> bind_matrices;
    std::vector<Matrix4d> local_matrices;
    if (!_readAttribute(skeletonPath, "bindTransforms", bytes) || !copyMatrixArray(bytes, bind_matrices) ||
        bind_matrices.size() != pose.joints.size())
        return nullptr;
    if (!_readAttribute(skeletonPath, "restTransforms", bytes) || !copyMatrixArray(bytes, local_matrices) ||
        local_matrices.size() != pose.joints.size())
        return nullptr;

    const std::string animation_path = _readInheritedTarget(skeletonPath, "skel:animationSource");
    if (!animation_path.empty() && _readAttribute(animation_path, "joints", bytes))
    {
        const std::vector<std::string> animation_joints = splitNulStrings(bytes);
        const size_t count = animation_joints.size();
        std::vector<float> translations, rotations, scales(count * 3, 1.0f);
        std::vector<uint8_t> translations_bytes, rotations_bytes, scales_bytes;
        const bool have_translation = _readAttribute(animation_path, "translations", translations_bytes) &&
                                      readNumericValues(translations_bytes, count * 3, translations);
        const bool have_rotation = _readAttribute(animation_path, "rotations", rotations_bytes) &&
                                   readNumericValues(rotations_bytes, count * 4, rotations);
        /* UsdSkel scales default to one. OVPopulation versions predating
         * half-vector support omit Apple's half3[] array, so retain that
         * schema default when the portable column is absent. */
        if (_readAttribute(animation_path, "scales", scales_bytes))
            readNumericValues(scales_bytes, count * 3, scales);
        if (have_translation && have_rotation)
        {
            for (size_t index = 0; index < count; ++index)
            {
                const auto found = pose.joint_indices.find(animation_joints[index]);
                if (found == pose.joint_indices.end())
                    continue;
                local_matrices[static_cast<size_t>(found->second)] =
                    makeTrs(&translations[index * 3], &rotations[index * 4], &scales[index * 3]);
            }
        }
    }

    std::vector<Matrix4d> skeleton_matrices(pose.joints.size());
    std::vector<uint8_t> state(pose.joints.size(), 0);
    std::function<bool(size_t)> compose = [&](size_t joint)
    {
        if (state[joint] == 2)
            return true;
        if (state[joint] == 1)
            return false;
        state[joint] = 1;
        const int parent = jointParent(pose.joints, joint, pose.joint_indices);
        if (parent >= 0)
        {
            if (!compose(static_cast<size_t>(parent)))
                return false;
            multiply(local_matrices[joint], skeleton_matrices[static_cast<size_t>(parent)], skeleton_matrices[joint]);
        }
        else
        {
            skeleton_matrices[joint] = local_matrices[joint];
        }
        state[joint] = 2;
        return true;
    };

    pose.skin_matrices.resize(pose.joints.size());
    pose.normal_matrices.resize(pose.joints.size());
    for (size_t joint = 0; joint < pose.joints.size(); ++joint)
    {
        if (!compose(joint))
            return nullptr;
        Matrix4d inverse_bind;
        if (!invertAffine(bind_matrices[joint], inverse_bind))
            return nullptr;
        multiply(inverse_bind, skeleton_matrices[joint], pose.skin_matrices[joint]);
        pose.normal_matrices[joint] = normalMatrix(pose.skin_matrices[joint]);
    }

    if (_readAttribute(skeletonPath, "worldMatrix", bytes) && bytes.size() >= sizeof(Matrix4d))
        std::memcpy(pose.world.data(), bytes.data(), sizeof(Matrix4d));
    pose.valid = true;
    return &pose;
}

bool UsdSkelDeformer::deform(const std::string& mesh_path,
                             const int* face_vertex_indices,
                             size_t face_vertex_index_count,
                             std::vector<float>& points,
                             std::vector<float>& normals)
{
    if (points.empty() || points.size() % 3 != 0)
        return false;
    const size_t point_count = points.size() / 3;
    const std::string skeleton_path = _readInheritedTarget(mesh_path, "skel:skeleton");
    if (skeleton_path.empty())
    {
        skinningDiagnostic(mesh_path, "no inherited skeleton target");
        return false;
    }
    const SkeletonPose* pose = _getPose(skeleton_path);
    if (!pose)
    {
        skinningDiagnostic(mesh_path, "skeleton pose rejected");
        return false;
    }

    std::vector<uint8_t> index_bytes, weight_bytes;
    if (!_readInheritedAttribute(mesh_path, "primvars:skel:jointIndices", index_bytes) ||
        !_readInheritedAttribute(mesh_path, "primvars:skel:jointWeights", weight_bytes))
    {
        skinningDiagnostic(mesh_path, "joint influence columns absent");
        return false;
    }
    std::vector<int> joint_indices;
    std::vector<float> joint_weights;
    if (!copyTrivialArray(index_bytes, joint_indices) || !copyTrivialArray(weight_bytes, joint_weights) ||
        joint_indices.empty() || joint_indices.size() != joint_weights.size())
    {
        skinningDiagnostic(mesh_path, "joint influence arrays invalid", joint_indices.size(), joint_weights.size());
        return false;
    }

    bool constant_influences = false;
    size_t influence_count = 0;
    if (joint_indices.size() % point_count == 0)
    {
        influence_count = joint_indices.size() / point_count;
    }
    else
    {
        /* Constant interpolation stores one influence tuple for the mesh. */
        constant_influences = true;
        influence_count = joint_indices.size();
    }
    if (influence_count == 0)
        return false;

    std::vector<int> mesh_to_skeleton;
    std::vector<uint8_t> mesh_joint_bytes;
    if (_readInheritedAttribute(mesh_path, "skel:joints", mesh_joint_bytes))
    {
        const std::vector<std::string> mesh_joints = splitNulStrings(mesh_joint_bytes);
        mesh_to_skeleton.resize(mesh_joints.size(), -1);
        for (size_t index = 0; index < mesh_joints.size(); ++index)
        {
            const auto found = pose->joint_indices.find(mesh_joints[index]);
            if (found != pose->joint_indices.end())
                mesh_to_skeleton[index] = found->second;
        }
    }

    Matrix4d geometry_bind = g_kIdentity;
    std::vector<uint8_t> bytes;
    if (_readInheritedAttribute(mesh_path, "primvars:skel:geomBindTransform", bytes) && bytes.size() >= sizeof(Matrix4d))
        std::memcpy(geometry_bind.data(), bytes.data(), sizeof(Matrix4d));

    Matrix4d mesh_world = g_kIdentity;
    if (_readAttribute(mesh_path, "worldMatrix", bytes) && bytes.size() >= sizeof(Matrix4d))
        std::memcpy(mesh_world.data(), bytes.data(), sizeof(Matrix4d));
    Matrix4d inverse_mesh_world;
    if (!invertAffine(mesh_world, inverse_mesh_world))
        return false;
    Matrix4d skeleton_to_mesh;
    multiply(pose->world, inverse_mesh_world, skeleton_to_mesh);
    const Matrix4d geometry_bind_normal = normalMatrix(geometry_bind);
    const Matrix4d skeleton_to_mesh_normal = normalMatrix(skeleton_to_mesh);

    const auto skeleton_joint = [&](size_t vertex, size_t influence)
    {
        const size_t offset = constant_influences ? influence : vertex * influence_count + influence;
        if (offset >= joint_indices.size())
            return -1;
        const int local = joint_indices[offset];
        if (mesh_to_skeleton.empty())
            return local;
        return local >= 0 && static_cast<size_t>(local) < mesh_to_skeleton.size() ?
                   mesh_to_skeleton[static_cast<size_t>(local)] :
                   -1;
    };
    const auto influence_weight = [&](size_t vertex, size_t influence)
    {
        const size_t offset = constant_influences ? influence : vertex * influence_count + influence;
        return offset < joint_weights.size() ? joint_weights[offset] : 0.0f;
    };

    for (size_t vertex = 0; vertex < point_count; ++vertex)
    {
        float skeleton_point[3];
        transformPoint(geometry_bind, &points[vertex * 3], skeleton_point);
        double accumulated[3] = { 0, 0, 0 };
        double weight_sum = 0;
        for (size_t influence = 0; influence < influence_count; ++influence)
        {
            const int joint = skeleton_joint(vertex, influence);
            const float weight = influence_weight(vertex, influence);
            if (joint < 0 || static_cast<size_t>(joint) >= pose->skin_matrices.size() || weight == 0.0f)
                continue;
            float transformed[3];
            transformPoint(pose->skin_matrices[static_cast<size_t>(joint)], skeleton_point, transformed);
            for (int component = 0; component < 3; ++component)
                accumulated[component] += weight * transformed[component];
            weight_sum += weight;
        }
        if (weight_sum > 0.0)
        {
            const float accumulated_point[3] = {
                static_cast<float>(accumulated[0]),
                static_cast<float>(accumulated[1]),
                static_cast<float>(accumulated[2]),
            };
            transformPoint(skeleton_to_mesh, accumulated_point, &points[vertex * 3]);
        }
    }

    const bool vertex_normals = normals.size() == point_count * 3;
    const bool face_varying_normals = face_vertex_indices && normals.size() == face_vertex_index_count * 3;
    if (!vertex_normals && !face_varying_normals)
        return true;
    const size_t normal_count = normals.size() / 3;
    for (size_t normal_index = 0; normal_index < normal_count; ++normal_index)
    {
        const int vertex = vertex_normals ? static_cast<int>(normal_index) : face_vertex_indices[normal_index];
        if (vertex < 0 || static_cast<size_t>(vertex) >= point_count)
            continue;
        double skeleton_normal[3];
        transformDirection(geometry_bind_normal, &normals[normal_index * 3], skeleton_normal);
        const float source[3] = {
            static_cast<float>(skeleton_normal[0]),
            static_cast<float>(skeleton_normal[1]),
            static_cast<float>(skeleton_normal[2]),
        };
        double accumulated[3] = { 0, 0, 0 };
        for (size_t influence = 0; influence < influence_count; ++influence)
        {
            const int joint = skeleton_joint(static_cast<size_t>(vertex), influence);
            const float weight = influence_weight(static_cast<size_t>(vertex), influence);
            if (joint < 0 || static_cast<size_t>(joint) >= pose->normal_matrices.size() || weight == 0.0f)
                continue;
            double transformed[3];
            transformDirection(pose->normal_matrices[static_cast<size_t>(joint)], source, transformed);
            for (int component = 0; component < 3; ++component)
                accumulated[component] += weight * transformed[component];
        }
        const float accumulated_normal[3] = {
            static_cast<float>(accumulated[0]),
            static_cast<float>(accumulated[1]),
            static_cast<float>(accumulated[2]),
        };
        double mesh_normal[3];
        transformDirection(skeleton_to_mesh_normal, accumulated_normal, mesh_normal);
        normalize(mesh_normal);
        for (int component = 0; component < 3; ++component)
            normals[normal_index * 3 + component] = static_cast<float>(mesh_normal[component]);
    }
    skinningDiagnostic(mesh_path, "deformed", point_count, normal_count);
    return true;
}

} // namespace ovgl
} // namespace details
} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim

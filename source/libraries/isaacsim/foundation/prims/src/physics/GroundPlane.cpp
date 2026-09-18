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

#include <isaacsim/foundation/prims/physics/GroundPlane.hpp>

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace isaacsim
{
namespace foundation
{
namespace prims
{
namespace physics
{

namespace objects = isaacsim::foundation::objects;

namespace
{


std::tuple<std::string, std::string> findCompositePaths(const std::string& path, const std::vector<std::string>& children)
{
    if (children.size() != 2 && children.size() != 3)
    {
        throw std::invalid_argument("Ground plane (at path '" + path +
                                    "') must have 2 or 3 child prims: plane, mesh and visual material (optional). Got " +
                                    std::to_string(children.size()) + " child prims");
    }
    std::string planePath;
    std::string meshPath;
    const std::vector<std::string> typeNames = objects::Prim(children, /*resolvePaths=*/false).getTypeName();
    for (size_t i = 0; i < children.size(); ++i)
    {
        if (typeNames[i] == "Plane")
        {
            planePath = children[i];
        }
        else if (typeNames[i] == "Mesh")
        {
            meshPath = children[i];
        }
    }
    if (planePath.empty())
    {
        throw std::invalid_argument("No Plane child prim found for ground plane prim at path '" + path + "'");
    }
    if (meshPath.empty())
    {
        throw std::invalid_argument("No Mesh child prim found for ground plane prim at path '" + path + "'");
    }
    return { planePath, meshPath };
}

std::array<float, 4> resolveAxisOrientation(const std::string& axis)
{
    // sqrt(2) / 2, the sine and cosine of a 45 degree half-angle.
    constexpr float halfTurnComponent = 0.70710678f;
    if (axis == "Z" || axis == "z")
    {
        return { 0.0f, 0.0f, 0.0f, 1.0f };
    }
    if (axis == "X" || axis == "x")
    {
        // Rotate +90 degrees about Y, which maps +Z onto +X.
        return { 0.0f, halfTurnComponent, 0.0f, halfTurnComponent };
    }
    if (axis == "Y" || axis == "y")
    {
        // Rotate -90 degrees about X, which maps +Z onto +Y.
        return { -halfTurnComponent, 0.0f, 0.0f, halfTurnComponent };
    }
    throw std::invalid_argument("Invalid axis: '" + axis + "'. Supported axes are 'X', 'Y' and 'Z'");
}

std::vector<float> resolveSizes(const std::optional<array::Array>& sizes, size_t count)
{
    const array::Array values = sizes.has_value() ? *sizes : array::Array(100.0f);
    return values.toDtype(array::Dtype::Float32())
        .flatten()
        .broadcastTo(array::Shape({ static_cast<int64_t>(count) }))
        .get<std::vector<float>>();
}

std::tuple<std::vector<float>, std::vector<float>> computeMeshTransforms(const std::vector<std::string>& axes,
                                                                         const std::vector<float>& sizes)
{
    std::vector<float> orientations;
    std::vector<float> scales;
    orientations.reserve(axes.size() * 4);
    scales.reserve(axes.size() * 3);
    for (size_t i = 0; i < axes.size(); ++i)
    {
        const std::array<float, 4> orientation = resolveAxisOrientation(axes[i]);
        orientations.insert(orientations.end(), orientation.begin(), orientation.end());
        scales.insert(scales.end(), 3, sizes[i]);
    }
    return { orientations, scales };
}

} // namespace

GroundPlane::GroundPlane(const std::variant<std::string, std::vector<std::string>>& paths,
                         const std::optional<array::Array>& sizes,
                         const std::optional<objects::ColorType>& colors,
                         const std::optional<std::variant<std::string, std::vector<std::string>>>& axes,
                         bool resetXformOpProperties)
    : objects::Xform()
{
    // Get or create prims.
    std::vector<std::string> meshPaths;
    std::vector<std::string> planePaths;
    auto [existentPaths, nonexistentPaths] = this->resolvePaths(paths);
    // Get prims.
    if (!existentPaths.empty())
    {
        m_paths = std::move(existentPaths);
        const std::vector<std::vector<std::string>> children = this->getChildren();
        for (size_t i = 0; i < m_paths.size(); ++i)
        {
            auto [planePath, meshPath] = findCompositePaths(m_paths[i], children[i]);
            planePaths.push_back(std::move(planePath));
            meshPaths.push_back(std::move(meshPath));
        }
        m_planes.emplace(planePaths, /*widths=*/std::nullopt, /*lengths=*/std::nullopt, /*axes=*/std::nullopt,
                         /*colors=*/std::nullopt, /*resetXformOpProperties=*/false);
        m_meshes.emplace(meshPaths, /*primitives=*/std::nullopt, /*colors=*/std::nullopt,
                         /*resetXformOpProperties=*/false);
    }
    // Create prims.
    else
    {
        m_paths = std::move(nonexistentPaths);
        for (const std::string& path : m_paths)
        {
            this->getStage().definePrim(path, "Xform");
            planePaths.push_back(path + "/Plane");
            meshPaths.push_back(path + "/Mesh");
        }

        const objects::ColorType resolvedColors = colors.has_value() ? *colors : "gray";
        const std::vector<std::string> resolvedAxes =
            axes.has_value() ? _resolveStringList(*axes, std::nullopt) :
                               std::vector<std::string>(m_paths.size(), this->getStage().getUpAxis());
        const std::vector<float> resolvedSizes = resolveSizes(sizes, m_paths.size());
        auto [meshOrientations, meshScales] = computeMeshTransforms(resolvedAxes, resolvedSizes);
        const array::Array sizeValues =
            array::Array(resolvedSizes).reshape(array::Shape({ int64_t{ -1 }, int64_t{ 1 } }));

        m_planes.emplace(planePaths, /*widths=*/sizeValues, /*lengths=*/sizeValues, /*axes=*/resolvedAxes,
                         /*colors=*/resolvedColors);
        m_planes->updateExtents();
        m_meshes.emplace(meshPaths, /*primitives=*/std::string("Plane"), /*colors=*/resolvedColors);
        m_meshes->setLocalPoses(
            /*translations=*/std::nullopt,
            /*orientations=*/array::Array(meshOrientations).reshape(array::Shape({ int64_t{ -1 }, int64_t{ 4 } })));
        m_meshes->setLocalScales(array::Array(meshScales).reshape(array::Shape({ int64_t{ -1 }, int64_t{ 3 } })));
        // The Plane child drives collision only; it is not renderable by Hydra, so it is authored as a guide.
        m_planes->setAttributeValues("purpose", std::vector<std::string>(m_paths.size(), "guide"));
        m_planes->applyApi("PhysicsCollisionAPI");
    }
    // Initialize instance from arguments.
    _initialize(resetXformOpProperties);
    // The collision APIs live on the Plane children, so the collider wrapper is placed over them. They are not
    // applied here: the create branch authors the collision API a Plane supports, and the wrap branch must leave
    // externally authored prims as they are.
    m_colliders.emplace(planePaths, /*approximations=*/std::nullopt, /*applyCollisionApis=*/false,
                        /*resetXformOpProperties=*/false);
}

objects::shapes::Plane& GroundPlane::planes()
{
    return *m_planes;
}

objects::Mesh& GroundPlane::meshes()
{
    return *m_meshes;
}

void GroundPlane::setOffsets(const std::optional<array::Array>& contactOffsets,
                             const std::optional<array::Array>& restOffsets,
                             const std::optional<array::Array>& indices)
{
    m_colliders->setOffsets(contactOffsets, restOffsets, indices);
}

std::tuple<array::Array, array::Array> GroundPlane::getOffsets(const std::optional<array::Array>& indices)
{
    return m_colliders->getOffsets(indices);
}

void GroundPlane::setTorsionalPatchRadii(const array::Array& radii, const std::optional<array::Array>& indices, bool minimum)
{
    m_colliders->setTorsionalPatchRadii(radii, indices, minimum);
}

array::Array GroundPlane::getTorsionalPatchRadii(const std::optional<array::Array>& indices, bool minimum)
{
    return m_colliders->getTorsionalPatchRadii(indices, minimum);
}

void GroundPlane::setEnabledCollisions(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    m_colliders->setEnabledCollisions(enabled, indices);
}

array::Array GroundPlane::getEnabledCollisions(const std::optional<array::Array>& indices)
{
    return m_colliders->getEnabledCollisions(indices);
}

array::Array GroundPlane::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    const objects::Prim prims(paths);
    const std::vector<bool> areXformable = prims.isA("Xformable").flatten().get<std::vector<bool>>();
    const std::vector<std::vector<std::string>> children = prims.getChildren();
    std::vector<bool> result(children.size());
    for (std::size_t i = 0; i < children.size(); ++i)
    {
        // Same composite structure the constructor requires: a Plane child for collision, a Mesh child
        // for rendering, and an optional visual material.
        if (!areXformable[i] || (children[i].size() != 2 && children[i].size() != 3))
        {
            result[i] = false;
            continue;
        }
        const std::vector<std::string> typeNames = objects::Prim(children[i], /*resolvePaths=*/false).getTypeName();
        result[i] = std::find(typeNames.begin(), typeNames.end(), "Plane") != typeNames.end() &&
                    std::find(typeNames.begin(), typeNames.end(), "Mesh") != typeNames.end();
    }
    return array::Array(result).reshape(array::Shape({ -1, 1 }));
}

} // namespace physics
} // namespace prims
} // namespace foundation
} // namespace isaacsim

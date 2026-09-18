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
#include <isaacsim/foundation/prims/physics/ColliderBody.hpp>

#include <numeric>
#include <sstream>
#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace prims
{
namespace physics
{

namespace
{

std::string joinPaths(const std::vector<std::string>& v)
{
    std::ostringstream ss;
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        if (i)
            ss << ", ";
        ss << "'" << v[i] << "'";
    }
    return ss.str();
}

} // namespace

ColliderBody::ColliderBody(const std::variant<std::string, std::vector<std::string>>& paths,
                           const std::optional<std::variant<std::string, std::vector<std::string>>>& approximations,
                           bool applyCollisionApis,
                           bool resetXformOpProperties)
    : isaacsim::foundation::objects::Xform()
{
    // Get prims.
    auto [existentPaths, nonexistentPaths] = this->resolvePaths(paths);
    if (!nonexistentPaths.empty())
    {
        throw std::runtime_error("Specified paths must correspond to existing prims: " + joinPaths(nonexistentPaths));
    }
    m_paths = std::move(existentPaths);
    // Initialize instance from arguments.
    _initialize(resetXformOpProperties);
    if (applyCollisionApis)
    {
        this->applyCollisionApis();
    }
    if (approximations.has_value())
    {
        this->setCollisionApproximations(*approximations);
    }
}

void ColliderBody::applyCollisionApis(const std::optional<array::Array>& indices)
{
    this->applyApi("PhysicsCollisionAPI", std::nullopt, indices);
    this->applyApi("PhysicsMeshCollisionAPI", std::nullopt, indices);
    this->applyApi("PhysxCollisionAPI", std::nullopt, indices);
}

void ColliderBody::removeCollisionApis(const std::optional<array::Array>& indices)
{
    this->removeApi("PhysicsCollisionAPI", std::nullopt, indices);
    this->removeApi("PhysicsMeshCollisionAPI", std::nullopt, indices);
    this->removeApi("PhysxCollisionAPI", std::nullopt, indices);
    this->removeApi("PhysxConvexHullCollisionAPI", std::nullopt, indices);
    this->removeApi("PhysxConvexDecompositionCollisionAPI", std::nullopt, indices);
    this->removeApi("PhysxTriangleMeshSimplificationCollisionAPI", std::nullopt, indices);
    this->removeApi("PhysxTriangleMeshCollisionAPI", std::nullopt, indices);
    this->removeApi("PhysxSphereFillCollisionAPI", std::nullopt, indices);
    this->removeApi("PhysxSDFMeshCollisionAPI", std::nullopt, indices);
    this->removeApi("PhysxCookedDataAPI", "convexHull", indices);
    this->removeApi("PhysxCookedDataAPI", "convexDecomposition", indices);
    this->removeApi("PhysxCookedDataAPI", "triangleMesh", indices);
}

void ColliderBody::setOffsets(const std::optional<array::Array>& contactOffsets,
                              const std::optional<array::Array>& restOffsets,
                              const std::optional<array::Array>& indices)
{
    if (!contactOffsets.has_value() && !restOffsets.has_value())
    {
        throw std::invalid_argument(
            "Both 'contactOffsets' and 'restOffsets' are not defined. Define at least one of them");
    }
    this->applyApi("PhysxCollisionAPI", std::nullopt, indices);
    const int64_t batchSize = _resolveIndexedSize(indices);
    if (contactOffsets.has_value())
    {
        this->setAttributeValues("physxCollision:contactOffset",
                                 contactOffsets->reshape(array::Shape({ int64_t{ -1 }, int64_t{ 1 } }))
                                     .broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })),
                                 indices);
    }
    if (restOffsets.has_value())
    {
        this->setAttributeValues("physxCollision:restOffset",
                                 restOffsets->reshape(array::Shape({ int64_t{ -1 }, int64_t{ 1 } }))
                                     .broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })),
                                 indices);
    }
}

std::tuple<array::Array, array::Array> ColliderBody::getOffsets(const std::optional<array::Array>& indices)
{
    this->applyApi("PhysxCollisionAPI", std::nullopt, indices);
    return {
        std::get<array::Array>(this->getAttributeValues("physxCollision:contactOffset", indices)),
        std::get<array::Array>(this->getAttributeValues("physxCollision:restOffset", indices)),
    };
}

void ColliderBody::setTorsionalPatchRadii(const array::Array& radii,
                                          const std::optional<array::Array>& indices,
                                          bool minimum)
{
    this->applyApi("PhysxCollisionAPI", std::nullopt, indices);
    const int64_t batchSize = _resolveIndexedSize(indices);
    const std::string attributeName =
        minimum ? "physxCollision:minTorsionalPatchRadius" : "physxCollision:torsionalPatchRadius";
    this->setAttributeValues(
        attributeName,
        radii.reshape(array::Shape({ int64_t{ -1 }, int64_t{ 1 } })).broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })),
        indices);
}

array::Array ColliderBody::getTorsionalPatchRadii(const std::optional<array::Array>& indices, bool minimum)
{
    this->applyApi("PhysxCollisionAPI", std::nullopt, indices);
    const std::string attributeName =
        minimum ? "physxCollision:minTorsionalPatchRadius" : "physxCollision:torsionalPatchRadius";
    return std::get<array::Array>(this->getAttributeValues(attributeName, indices));
}

void ColliderBody::setCollisionApproximations(const std::variant<std::string, std::vector<std::string>>& approximations,
                                              const std::optional<array::Array>& indices)
{
    this->applyApi("PhysicsMeshCollisionAPI", std::nullopt, indices);
    this->setAttributeValues("physics:approximation", _resolveStringList(approximations, indices), indices);
}

std::vector<std::string> ColliderBody::getCollisionApproximations(const std::optional<array::Array>& indices)
{
    this->applyApi("PhysicsMeshCollisionAPI", std::nullopt, indices);
    return std::get<std::vector<std::string>>(this->getAttributeValues("physics:approximation", indices));
}

void ColliderBody::setEnabledCollisions(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    const int64_t batchSize = _resolveIndexedSize(indices);
    const std::vector<int64_t> resolvedIndices = _resolveIndexValues(indices);
    const std::vector<bool> enabledValues =
        enabled.flatten().broadcastTo(array::Shape({ batchSize })).get<std::vector<bool>>();
    for (size_t i = 0; i < resolvedIndices.size(); ++i)
    {
        Prim prim = Prim(m_paths[resolvedIndices[i]], /*resolvePaths=*/false);
        const bool value = enabledValues[i];
        if (value)
        {
            prim.applyApi("PhysicsCollisionAPI");
            prim.applyApi("PhysicsMeshCollisionAPI");
            prim.applyApi("PhysxCollisionAPI");
        }
        else if (!prim.hasApi("PhysicsCollisionAPI").item<bool>())
        {
            continue;
        }
        prim.setAttributeValues("physics:collisionEnabled", array::Array(std::vector<bool>{ value }));
    }
}

array::Array ColliderBody::getEnabledCollisions(const std::optional<array::Array>& indices)
{
    std::vector<bool> result(_resolveIndexedSize(indices), false);
    const std::vector<int64_t> resolvedIndices = _resolveIndexValues(indices);
    for (size_t i = 0; i < resolvedIndices.size(); ++i)
    {
        Prim prim = Prim(m_paths[resolvedIndices[i]], /*resolvePaths=*/false);
        if (prim.hasApi("PhysicsCollisionAPI").item<bool>())
        {
            result[i] = std::get<array::Array>(prim.getAttributeValues("physics:collisionEnabled")).item<bool>();
        }
    }
    return array::Array(result).reshape(array::Shape({ -1, 1 }));
}

array::Array ColliderBody::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    // A collider body is a transformable prim carrying the Collision API.
    const isaacsim::foundation::objects::Prim prims(paths);
    const std::vector<bool> areXformable = prims.isA("Xformable").flatten().get<std::vector<bool>>();
    const std::vector<bool> haveApi = prims.hasApi("PhysicsCollisionAPI").flatten().get<std::vector<bool>>();
    std::vector<bool> result(areXformable.size());
    for (std::size_t i = 0; i < result.size(); ++i)
    {
        result[i] = areXformable[i] && haveApi[i];
    }
    return array::Array(result).reshape(array::Shape({ -1, 1 }));
}

} // namespace physics
} // namespace prims
} // namespace foundation
} // namespace isaacsim

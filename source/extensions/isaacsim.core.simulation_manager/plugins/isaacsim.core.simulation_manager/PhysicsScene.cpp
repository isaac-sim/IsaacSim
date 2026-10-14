// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/core/simulation_manager/PhysicsScene.hpp>
#include <omni/usd/UsdContext.h>

#include <cmath>
#include <stdexcept>

namespace isaacsim
{
namespace core
{
namespace simulation_manager
{

PhysicsScene::PhysicsScene(const std::string& path) : m_path(path)
{
    auto usdContext = omni::usd::UsdContext::getContext();
    pxr::UsdStagePtr stage = usdContext ? usdContext->getStage() : pxr::UsdStagePtr();
    if (!stage)
    {
        throw std::runtime_error("Cannot create a PhysicsScene without a USD stage");
    }
    if (stage->GetPrimAtPath(pxr::SdfPath(path)).IsValid())
    {
        pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath(path));
        if (!prim.IsA<pxr::UsdPhysicsScene>())
        {
            throw std::invalid_argument("Prim at path '" + path + "' is not a PhysicsScene");
        }
        m_prim = prim;
    }
    else
    {
        m_prim = stage->DefinePrim(pxr::SdfPath(path), pxr::TfToken("PhysicsScene"));
    }
    m_physicsScene = pxr::UsdPhysicsScene(m_prim);
}

pxr::GfVec3d PhysicsScene::getGravity()
{
    float magnitude;
    pxr::GfVec3f direction;
    pxr::UsdStagePtr stage = m_prim.GetStage();
    if (!stage)
    {
        return pxr::GfVec3d(0.0);
    }
    double metersPerUnit = UsdGeomGetStageMetersPerUnit(stage);
    m_physicsScene.GetGravityMagnitudeAttr().Get(&magnitude);
    m_physicsScene.GetGravityDirectionAttr().Get(&direction);
    if (std::isinf(magnitude) && magnitude < 0.0f && direction == pxr::GfVec3f(0.0f))
    {
        return pxr::GfVec3d(0.0, 0.0, -9.81);
    }
    if (!std::isfinite(magnitude) || !std::isfinite(direction[0]) || !std::isfinite(direction[1]) ||
        !std::isfinite(direction[2]))
    {
        return pxr::GfVec3d(0.0);
    }
    return pxr::GfVec3d(direction.GetNormalized()) * static_cast<double>(magnitude) / metersPerUnit;
}

bool PhysicsScene::isValid() const
{
    return m_prim.IsValid() && m_prim.IsActive();
}

std::vector<std::string> getPhysicsScenePaths()
{
    auto usdContext = omni::usd::UsdContext::getContext();
    return getPhysicsScenePaths(usdContext ? usdContext->getStage() : pxr::UsdStagePtr());
}

std::vector<std::string> getPhysicsScenePaths(uint64_t stageId)
{
    pxr::UsdStagePtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    if (!stage)
    {
        CARB_LOG_ERROR("Stage ID (%lu) doesn't correspond to an existing stage", stageId);
        return std::vector<std::string>();
    }
    return getPhysicsScenePaths(stage);
}

std::vector<std::string> getPhysicsScenePaths(const pxr::UsdStagePtr& stage)
{
    std::vector<std::string> paths;
    if (!stage)
    {
        return paths;
    }
    for (const pxr::UsdPrim& prim : stage->Traverse())
    {
        if (prim.IsA<pxr::UsdPhysicsScene>())
        {
            paths.push_back(prim.GetPath().GetString());
        }
    }
    return paths;
}

} // namespace simulation_manager
} // namespace core
} // namespace isaacsim

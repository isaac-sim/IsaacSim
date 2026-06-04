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

#include <isaacsim/core/simulation_manager/PhysicsScene.h>
#include <omni/usd/UsdContext.h>

namespace isaacsim
{
namespace core
{
namespace simulation_manager
{

PhysicsScene::PhysicsScene(const std::string& path) : m_path(path)
{
    pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
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
    double metersPerUnit = UsdGeomGetStageMetersPerUnit(omni::usd::UsdContext::getContext()->getStage());
    m_physicsScene.GetGravityMagnitudeAttr().Get(&magnitude);
    m_physicsScene.GetGravityDirectionAttr().Get(&direction);
    return pxr::GfVec3d(direction) * static_cast<double>(magnitude) / metersPerUnit;
}

bool PhysicsScene::isValid() const
{
    return m_prim.IsValid() && m_prim.IsActive();
}

std::vector<std::string> getPhysicsScenePaths()
{
    return getPhysicsScenePaths(omni::usd::UsdContext::getContext()->getStage());
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

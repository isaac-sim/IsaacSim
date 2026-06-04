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

#include <isaacsim/core/simulation_manager/UsdNoticeListener.h>
#include <physxSchema/physxSceneAPI.h>
///
#include <omni/usd/UsdContext.h>
///
#include <iostream>


namespace isaacsim
{
namespace core
{
namespace simulation_manager
{

UsdNoticeListener::UsdNoticeListener()
{
    this->m_callbackIter = 0;
    this->m_enableFlag = true;
}

void UsdNoticeListener::handle(const pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    // TODO: only listen to the stage of interest here once Isaac can work with multiple stages
    pxr::UsdStagePtr attachedStage = omni::usd::UsdContext::getContext()->getStage();
    pxr::UsdStageWeakPtr stage = objectsChanged.GetStage();

    if (!this->m_enableFlag || stage != attachedStage)
    {
        return;
    }


    for (const pxr::SdfPath& path : objectsChanged.GetResyncedPaths())
    {
        if (path.IsAbsoluteRootOrPrimPath())
        {
            const pxr::SdfPath primPath =
                stage->GetPseudoRoot().GetPath() == path ? stage->GetPseudoRoot().GetPath() : path.GetPrimPath();
            pxr::UsdPrim prim = stage->GetPrimAtPath(primPath);
            if (prim.IsValid() == false || !prim.IsActive())
            {
                if (this->m_physicsScenes.find(primPath) != this->m_physicsScenes.end())
                {
                    this->m_physicsScenes.erase(primPath);
                }
                std::vector<int> deletionKeys;
                std::transform(this->m_deletionCallbacks.begin(), this->m_deletionCallbacks.end(),
                               std::back_inserter(deletionKeys), [](const auto& p) { return p.first; });
                for (auto const& key : deletionKeys)
                {
                    this->m_deletionCallbacks[key](primPath.GetString());
                }
            }
            else
            {
                static const pxr::TfToken s_kGkindToken("kind");
                const auto& changedFields = objectsChanged.GetChangedFields(primPath);
                const bool typeNameChange = std::find(changedFields.begin(), changedFields.end(),
                                                      pxr::SdfFieldKeys->TypeName) != changedFields.end();
                if (typeNameChange && std::find(changedFields.begin(), changedFields.end(),
                                                PXR_NS::UsdTokens->apiSchemas) == changedFields.end())
                {
                    if (this->m_physicsScenes.count(primPath) == 0)
                    {
                        static pxr::TfToken s_physicsSceneType("PhysicsScene");
                        if (prim.GetTypeName() == s_physicsSceneType)
                        {
                            this->m_physicsScenes.emplace(primPath, pxr::PhysxSchemaPhysxSceneAPI(prim));
                            for (auto const& [key, AdditionFunc] : this->m_physicsSceneAdditionCallbacks)
                            {
                                (void)key;
                                AdditionFunc(primPath.GetString());
                            }
                        }
                    }
                }
            }
        }
    }
}


std::map<int, std::function<void(const std::string&)>>& UsdNoticeListener::getDeletionCallbacks()
{
    return this->m_deletionCallbacks;
}

std::map<int, std::function<void(const std::string&)>>& UsdNoticeListener::getPhysicsSceneAdditionCallbacks()
{
    return this->m_physicsSceneAdditionCallbacks;
}

std::map<pxr::SdfPath, pxr::PhysxSchemaPhysxSceneAPI>& UsdNoticeListener::getPhysicsScenes()
{
    return this->m_physicsScenes;
}

int& UsdNoticeListener::getCallbackIter()
{
    return this->m_callbackIter;
}
void UsdNoticeListener::enable(const bool& flag)
{
    this->m_enableFlag = flag;
}

bool UsdNoticeListener::isEnabled()
{
    return this->m_enableFlag;
}

} // namespace simulation_manager
} // namespace core
} // namespace isaacsim

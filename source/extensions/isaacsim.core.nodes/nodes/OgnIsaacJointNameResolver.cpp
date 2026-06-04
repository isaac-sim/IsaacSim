// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/core/includes/UsdUtilities.h>
#include <isaacsim/core/nodes/ICoreNodes.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/UsdContextIncludes.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>

#include <OgnIsaacJointNameResolverDatabase.h>
#include <unordered_map>


namespace isaacsim
{
namespace core
{
namespace nodes
{

class OgnIsaacJointNameResolver : public isaacsim::core::includes::BaseResetNode
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnIsaacJointNameResolverDatabase::sPerInstanceState<OgnIsaacJointNameResolver>(nodeObj, instanceId);
        state.m_robotPath = "";
    }

    static bool compute(OgnIsaacJointNameResolverDatabase& db)
    {
        const GraphContextObj& context = db.abi_context();

        auto& state = db.perInstanceState<OgnIsaacJointNameResolver>();
        if (state.m_firstFrame)
        {

            const auto& prim = db.inputs.targetPrim();
            state.m_robotPath = db.inputs.robotPath();

            if (state.m_robotPath.empty())
            {
                if (!prim.empty())
                {
                    state.m_robotPath = omni::fabric::toSdfPath(prim[0]).GetText();
                }
                else
                {
                    db.logError("OmniGraph Error: no articulation root prim found");
                    return false;
                }
            }

            state.m_firstFrame = false;

            const auto stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

            if (!stage)
            {
                db.logError("Could not find USD stage %ld", stageId);
                return false;
            }

            const pxr::UsdPrim startPrim = stage->GetPrimAtPath(pxr::SdfPath(state.m_robotPath));

            if (!startPrim.IsValid())
            {
                db.logError("%s prim is invalid", state.m_robotPath.c_str());
                return false;
            }

            // If the supplied prim does not itself carry UsdPhysicsArticulationRootAPI, descend
            // looking for the first descendant that does. Handles assets where the user-supplied
            // prim is an ancestor Xform (e.g. an IsaacRobotAPI root whose articulation root sits
            // on a deeper base_link). Mirrors the descend-for-articulation behavior that the
            // Python Articulation wrapper has had via fetch_articulation_root_api_prim_paths.
            pxr::UsdPrim effectiveStart = startPrim;
            if (!effectiveStart.HasAPI<pxr::UsdPhysicsArticulationRootAPI>())
            {
                for (const pxr::UsdPrim& descendant : pxr::UsdPrimRange(startPrim))
                {
                    if (descendant.HasAPI<pxr::UsdPhysicsArticulationRootAPI>())
                    {
                        effectiveStart = descendant;
                        break;
                    }
                }
            }

            if (!effectiveStart.HasAPI<pxr::UsdPhysicsArticulationRootAPI>())
            {
                db.logError("Articulation not found for prim %s", state.m_robotPath.c_str());
                return false;
            }

            // Traverse from the user-supplied prim (not `effectiveStart`) so override discovery
            // covers the full intended subtree. Joints may live as siblings of the articulation
            // root (e.g. `/World/Robot/joint_left` next to `/World/Robot/base_link`) and would be
            // missed if we restricted the walk to descendants of the articulation-root link.
            for (const pxr::UsdPrim& currentPrim : pxr::UsdPrimRange(startPrim))
            {
                if (!currentPrim.HasAttribute(isaacsim::core::includes::g_kIsaacNameOveride))
                    continue;
                const std::string primNameOverride = isaacsim::core::includes::getName(currentPrim);
                const std::string primName = currentPrim.GetName().GetString();
                if (primNameOverride != primName)
                    state.m_nameOverrideMap.emplace(primNameOverride, primName);
            }
        }

        state.resolvePrims(db);

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

    void resolvePrims(OgnIsaacJointNameResolverDatabase& db)
    {
        const auto& inNames = db.inputs.jointNames();
        auto& outNames = db.outputs.jointNames();
        outNames.resize(inNames.size());

        for (size_t i = 0; i < inNames.size(); ++i)
        {
            const std::string primNameString = db.tokenToString(inNames[i]);
            const auto it = m_nameOverrideMap.find(primNameString);
            outNames[i] = db.stringToToken(it != m_nameOverrideMap.end() ? it->second.c_str() : primNameString.c_str());
        }

        db.outputs.robotPath() = m_robotPath;
    }

    void reset() override
    {
        m_firstFrame = true;
        m_robotPath = "";
        m_nameOverrideMap.clear();
    }

private:
    std::unordered_map<std::string, std::string> m_nameOverrideMap;
    bool m_firstFrame = true;
    std::string m_robotPath;
};

REGISTER_OGN_NODE()
}
}
}

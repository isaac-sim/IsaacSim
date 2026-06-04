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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "isaacsim/core/includes/UsdUtilities.h"

#include <isaacsim/core/includes/Conversions.h>
#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <omni/fabric/FabricUSD.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/vec3d.h>

#include <OgnIsaacReadLightBeamSensorDatabase.h>


namespace isaacsim
{
namespace sensors
{
namespace physx
{
class OgnIsaacReadLightBeamSensor
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnIsaacReadLightBeamSensorDatabase::sPerInstanceState<OgnIsaacReadLightBeamSensor>(nodeObj, instanceId);

        state.m_lightBeamSensorInterface = carb::getCachedInterface<LightBeamSensorInterface>();

        if (!state.m_lightBeamSensorInterface)
        {
            CARB_LOG_ERROR("Failed to acquire omni::isaac::sensor interface");
            return;
        }
    }

    static bool compute(OgnIsaacReadLightBeamSensorDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacReadLightBeamSensor>();

        // Find our stage
        const GraphContextObj& context = db.abi_context();
        long stageId = context.iContext->getStageId(context);
        auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
        if (!stage)
        {
            db.logError("Could not find USD stage %ld", stageId);
            return false;
        }
        double unitScale = UsdGeomGetStageMetersPerUnit(stage);
        const auto& prim = db.inputs.lightbeamPrim();
        const char* primPath;

        if (!prim.empty())
        {
            primPath = omni::fabric::toSdfPath(prim[0]).GetText();
        }
        else
        {
            db.logError("Invalid Light Beam Sensor prim");
            return false;
        }

        uint8_t* beamHitData = state.m_lightBeamSensorInterface->getBeamHitData(primPath);
        float* linearDepthData = state.m_lightBeamSensorInterface->getLinearDepthData(primPath);
        carb::Float3* hitPosData = state.m_lightBeamSensorInterface->getHitPosData(primPath);
        int numRays = state.m_lightBeamSensorInterface->getNumRays(primPath);
        carb::Float3* rayOrigins = state.m_lightBeamSensorInterface->getBeamOrigins(primPath);
        carb::Float3* rayEndPoints = state.m_lightBeamSensorInterface->getBeamEndPoints(primPath);

        // Clear vectors
        state.m_beamHitData.clear();
        state.m_linearDepthData.clear();
        state.m_hitPosData.clear();
        state.m_beamOrigins.clear();
        state.m_beamEndPoints.clear();

        for (int i = 0; i < numRays; i++)
        {
            state.m_beamHitData.push_back(beamHitData[i]);
            state.m_linearDepthData.push_back(linearDepthData[i]);
            state.m_hitPosData.push_back(isaacsim::core::includes::conversions::asGfVec3f(hitPosData[i]) * unitScale);
            state.m_beamOrigins.push_back(isaacsim::core::includes::conversions::asGfVec3f(rayOrigins[i]) * unitScale);
            state.m_beamEndPoints.push_back(isaacsim::core::includes::conversions::asGfVec3f(rayEndPoints[i]) * unitScale);
        }

        // fill in outputs
        db.outputs.numRays() = numRays;
        db.outputs.beamHitData().resize(state.m_beamHitData.size());
        db.outputs.hitPosData().resize(state.m_hitPosData.size());
        db.outputs.linearDepthData().resize(state.m_linearDepthData.size());
        db.outputs.beamOrigins().resize(state.m_beamOrigins.size());
        db.outputs.beamEndPoints().resize(state.m_beamEndPoints.size());

        memcpy(db.outputs.beamHitData().data(), &state.m_beamHitData[0], state.m_beamHitData.size() * sizeof(bool));
        memcpy(db.outputs.hitPosData().data(), &state.m_hitPosData[0], state.m_hitPosData.size() * sizeof(GfVec3f));
        memcpy(db.outputs.linearDepthData().data(), &state.m_linearDepthData[0],
               state.m_linearDepthData.size() * sizeof(float));
        memcpy(db.outputs.beamOrigins().data(), &state.m_beamOrigins[0], state.m_beamOrigins.size() * sizeof(GfVec3f));
        memcpy(db.outputs.beamEndPoints().data(), &state.m_beamEndPoints[0],
               state.m_beamEndPoints.size() * sizeof(GfVec3f));

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

private:
    LightBeamSensorInterface* m_lightBeamSensorInterface = nullptr;
    std::vector<GfVec3f> m_hitPosData;
    std::vector<float> m_linearDepthData;
    std::vector<uint8_t> m_beamHitData;
    std::vector<GfVec3f> m_beamOrigins;
    std::vector<GfVec3f> m_beamEndPoints;
};


REGISTER_OGN_NODE()
} // sensor
} // graph
} // omni

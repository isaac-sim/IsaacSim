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

#ifdef _WIN32
#    pragma warning(push)
#    pragma warning(disable : 4996)
#endif

#define CARB_EXPORTS

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "LightBeamSensor.h"

#include "isaacsim/core/includes/Pose.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <carb/Framework.h>

#include <isaacsim/core/includes/Conversions.h>
#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSceneQuery.h>

#include <PxActor.h>

#if defined(_WIN32)
#    include <PxArticulationLink.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wpragmas"
#    include <PxArticulationLink.h>
#    pragma GCC diagnostic pop
#endif

#include <PxRigidDynamic.h>
#include <PxScene.h>
#include <map>
#include <string>
#include <vector>

namespace isaacsim
{
namespace sensors
{
namespace physx
{

LightBeamSensor::LightBeamSensor(omni::physx::IPhysx* physXInterface) : RangeSensorComponent(physXInterface)
{
}

void LightBeamSensor::onStart()
{
    RangeSensorComponent::onStart();
}

void LightBeamSensor::onComponentChange()
{
    RangeSensorComponent::onComponentChange();

    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kCurtainLengthAttr), m_curtainLength);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kNumRaysAttr), m_numRays);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kCurtainAxisAttr), m_curtainAxis);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kForwardAxisAttr), m_forwardAxis);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kMinRangeAttr), m_minRange);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kMaxRangeAttr), m_maxRange);

    m_metersPerUnit = static_cast<float>(UsdGeomGetStageMetersPerUnit(this->m_stage));
    m_minRange = pxr::GfClamp(m_minRange, 0, 1e9f);
    m_maxRange = pxr::GfClamp(m_maxRange, m_minRange, 1e9f);
    m_maxDepth = m_maxRange / m_metersPerUnit;
    m_minDepth = m_minRange / m_metersPerUnit;
    m_linearDepth.assign(m_numRays, 0);
    m_hitPos.assign(m_numRays, { 0, 0, 0 });
    m_beamHit.assign(m_numRays, 0);
    m_beamOrigins.assign(m_numRays, { 0, 0, 0 });
    m_beamEndPoints.assign(m_numRays, { 0, 0, 0 });
}

void LightBeamSensor::scan(const ::physx::PxVec3& origin, const ::physx::PxQuat& worldRotation)
{
    if (!m_pxScene)
    {
        return;
    }

    ::physx::PxVec3 unitDir =
        worldRotation.rotate(isaacsim::core::includes::conversions::asPxVec3(m_forwardAxis)).getNormalized();
    ::physx::PxVec3 unitCurtain =
        worldRotation.rotate(isaacsim::core::includes::conversions::asPxVec3(m_curtainAxis)).getNormalized();

    auto lightbeamLambda = [&]()
    {
        // for each ray in the light curtain
        for (int ray = 0; ray < m_numRays; ray++)
        {
            // increase casting origin by unit offset
            ::physx::PxVec3 rayOffset = ray * m_curtainLength / m_numRays * unitCurtain;
            ::physx::PxRaycastHit raycastHit;

            // Calculate the start point of the ray
            ::physx::PxVec3 startPoint = origin + unitDir * m_minDepth + rayOffset;

            // Project the start point out to prevent collisions from origin
            const bool hit = ::physx::PxSceneQueryExt::raycastSingle(
                *m_pxScene, startPoint, unitDir, m_maxDepth, m_hitFlags, raycastHit);

            // Store the start point (in world coordinates)
            m_beamOrigins[ray] = { startPoint.x, startPoint.y, startPoint.z };

            if (hit)
            {
                m_beamHit[ray] = 1;
                // calculate the distance and position of the ray hit
                m_linearDepth[ray] = (raycastHit.distance + m_minDepth) * m_metersPerUnit; // in meters
                ::physx::PxVec3 hitPosRel = worldRotation.rotateInv(raycastHit.position - origin);
                m_hitPos[ray] = { hitPosRel.x, hitPosRel.y, hitPosRel.z }; // relative to the sensor location
                // Calculate and store the end point (in world coordinates)
                ::physx::PxVec3 endPoint = raycastHit.position;
                m_beamEndPoints[ray] = { endPoint.x, endPoint.y, endPoint.z };
            }
            else
            {
                m_beamHit[ray] = 0;
                m_linearDepth[ray] = m_maxDepth * m_metersPerUnit; // in meters
                ::physx::PxVec3 hitPos = origin + unitDir * (m_maxDepth + m_minDepth) + rayOffset;
                // store the end point (in world coordinates)
                m_beamEndPoints[ray] = { hitPos.x, hitPos.y, hitPos.z };
                ::physx::PxVec3 hitPosRel = worldRotation.rotateInv(hitPos - origin);
                m_hitPos[ray] = { hitPosRel.x, hitPosRel.y, hitPosRel.z };
            }
        }
    };

    // call lambda
    lightbeamLambda();
}

void LightBeamSensor::onPhysicsStep()
{
    if (!m_pxScene)
    {
        CARB_LOG_ERROR("Physics Scene does not exist");
        return;
    }

    auto worldMat = isaacsim::core::includes::pose::computeWorldXformNoCache(m_stage, m_usdrtStage, m_prim.GetPath());

    m_worldTranslation = isaacsim::core::includes::conversions::asPxVec3(worldMat.ExtractTranslation());
    m_worldRotation = isaacsim::core::includes::conversions::asPxQuat(worldMat.ExtractRotation());

    // run full scan
    scan(m_worldTranslation, m_worldRotation);

    if (m_previousEnabled != this->m_enabled)
    {
        if (m_enabled)
        {
            this->onPhysicsStep(); // force on physics step to run to get up to date value
        }
        else
        {
            this->onStop();
        }
        m_previousEnabled = this->m_enabled;
    }
}

} // physx
} // sensors
} // isaacsim

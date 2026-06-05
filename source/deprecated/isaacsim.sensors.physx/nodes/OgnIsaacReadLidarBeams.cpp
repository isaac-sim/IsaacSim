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

#include "isaacsim/core/includes/UsdUtilities.h"

#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/robot/schema/sensor_tokens.h>
#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <omni/fabric/FabricUSD.h>

#include <OgnIsaacReadLidarBeamsDatabase.h>

using namespace isaacsim::robot::schema::sensors;

namespace isaacsim
{
namespace core
{
namespace nodes
{

class OgnIsaacReadLidarBeams : public isaacsim::core::includes::BaseResetNode
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacReadLidarBeamsDatabase::sPerInstanceState<OgnIsaacReadLidarBeams>(nodeObj, instanceId);

        state.m_lidarSensorInterface = carb::getCachedInterface<isaacsim::sensors::physx::LidarSensorInterface>();

        if (!state.m_lidarSensorInterface)
        {
            CARB_LOG_ERROR("Failed to acquire isaacsim::sensors::physx interface");
            return;
        }
    }

    static bool compute(OgnIsaacReadLidarBeamsDatabase& db)
    {
        const GraphContextObj& context = db.abi_context();

        auto& state = db.perInstanceState<OgnIsaacReadLidarBeams>();

        if (state.m_firstFrame)
        {
            const auto& prim = db.inputs.lidarPrim();
            const char* primPath;
            if (!prim.empty())
            {
                primPath = omni::fabric::toSdfPath(prim[0]).GetText();
            }
            else
            {
                db.logError("no prim path found for the lidar");
                return false;
            }


            state.m_firstFrame = false;


            // Find our stage
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
            if (!stage)
            {
                db.logError("Could not find USD stage %ld", stageId);
                return false;
            }

            // Verify we have a valid lidar prim
            pxr::UsdPrim targetPrim = stage->GetPrimAtPath(pxr::SdfPath(primPath));
            if (targetPrim.GetTypeName() != kLidarType)
            {
                db.logError("Prim is not a Lidar Prim");
                return false;
            }

            state.m_sensorPrim = targetPrim;

            if (!state.m_lidarSensorInterface->isLidarSensor(primPath))
            {
                db.logError("Prim is not registered with Lidar extension");
                return false;
            }

            state.m_lidarPrimPath = primPath;

            return true;
        }

        state.readLidar(db);
        return true;
    }


    void readLidar(OgnIsaacReadLidarBeamsDatabase& db)
    {
        if (!m_lidarSensorInterface->isLidarSensor(m_lidarPrimPath))
        {
            db.logError("Invalid Lidar Reference, Prim is not registered with Lidar extension");
            return;
        }

        auto& numRows = db.outputs.numRows();
        auto& numCols = db.outputs.numCols();

        numRows = m_lidarSensorInterface->getNumRows(m_lidarPrimPath);
        numCols = m_lidarSensorInterface->getNumCols(m_lidarPrimPath);

        int numColsTicked = m_lidarSensorInterface->getNumColsTicked(m_lidarPrimPath);

        size_t numBeams = numColsTicked * numRows;

        float* azimuthData = m_lidarSensorInterface->getAzimuthData(m_lidarPrimPath);
        // float* zenithData = m_lidarSensorInterface->getZenithData(m_lidarPrimPath);
        float* beamTimes = m_lidarSensorInterface->getBeamTimeData(m_lidarPrimPath);
        float* ranges = m_lidarSensorInterface->getLinearDepthData(m_lidarPrimPath);
        uint8_t* intensities = m_lidarSensorInterface->getIntensityData(m_lidarPrimPath);

        if (!azimuthData || !beamTimes || !ranges || !intensities)
        {
            return;
        }

        auto& horizontalFov = db.outputs.horizontalFov();
        auto& horizontalResolution = db.outputs.horizontalResolution();
        auto& depthRange = db.outputs.depthRange();
        auto& rotationRate = db.outputs.rotationRate();
        auto& verticalFov = db.outputs.verticalFov();
        auto& verticalResolution = db.outputs.verticalResolution();

        isaacsim::core::includes::safeGetAttribute(m_sensorPrim.GetAttribute(kHorizontalFovAttr), horizontalFov);
        isaacsim::core::includes::safeGetAttribute(
            m_sensorPrim.GetAttribute(kHorizontalResolutionAttr), horizontalResolution);
        isaacsim::core::includes::safeGetAttribute(m_sensorPrim.GetAttribute(kMinRangeAttr), depthRange[0]);
        isaacsim::core::includes::safeGetAttribute(m_sensorPrim.GetAttribute(kMaxRangeAttr), depthRange[1]);
        isaacsim::core::includes::safeGetAttribute(m_sensorPrim.GetAttribute(kRotationRateAttr), rotationRate);
        isaacsim::core::includes::safeGetAttribute(m_sensorPrim.GetAttribute(kVerticalFovAttr), verticalFov);
        isaacsim::core::includes::safeGetAttribute(
            m_sensorPrim.GetAttribute(kVerticalResolutionAttr), verticalResolution);

        size_t numBeamsTotal = numRows * numCols;

        if (horizontalFov <= 0.0)
        {
            db.logError("Lidar Prim %s: Horizontal FOV must be greater than 0.0", m_lidarPrimPath);
            return;
        }
        if (horizontalFov > 360.0)
        {
            db.logError("Lidar Prim %s: Horizontal FOV must be less than or equal to 360.0", m_lidarPrimPath);
            return;
        }
        if (horizontalResolution <= 0.0)
        {
            db.logError("Lidar Prim %s: Horizontal Resolution must be greater than 0.0", m_lidarPrimPath);
            return;
        }
        if (rotationRate < 0.0)
        {
            db.logError("Lidar Prim %s: Rotation Rate must be equal to or greater than 0.0", m_lidarPrimPath);
            return;
        }
        if (verticalFov <= 0.0)
        {
            db.logError("Lidar Prim %s: Vertical FOV must be greater than 0.0", m_lidarPrimPath);
            return;
        }
        if (verticalResolution <= 0.0)
        {
            db.logError("Lidar Prim %s: Vertical Resolution must be greater than 0.0", m_lidarPrimPath);
            return;
        }

        uint64_t currSequenceNum = m_lidarSensorInterface->getSequenceNumber(m_lidarPrimPath);

        if (currSequenceNum == m_prevSequenceNumber)
        {
            return;
        }

        if (currSequenceNum < m_prevSequenceNumber)
        {
            m_resetLaserScan = true;
        }

        m_prevSequenceNumber = currSequenceNum;

        carb::Float2 azimuthRange = m_lidarSensorInterface->getAzimuthRange(m_lidarPrimPath);
        carb::Float2 zenithRange = m_lidarSensorInterface->getZenithRange(m_lidarPrimPath);

        auto& azimuthRangeOutput = db.outputs.azimuthRange();
        auto& zenithRangeOutput = db.outputs.zenithRange();

        azimuthRangeOutput[0] = azimuthRange.x / static_cast<float>(M_PI) * 180.0f;
        azimuthRangeOutput[1] = azimuthRange.y / static_cast<float>(M_PI) * 180.0f;

        zenithRangeOutput[0] = zenithRange.x / static_cast<float>(M_PI) * 180.0f;
        zenithRangeOutput[1] = zenithRange.y / static_cast<float>(M_PI) * 180.0f;

        if (m_resetLaserScan)
        {
            m_beamTimeData.clear();
            m_intensitiesData.clear();
            m_rangesData.clear();

            m_numBeamsRemaining = numBeamsTotal;

            bool foundStart = false;
            for (m_beamIdx = 0; m_beamIdx < numBeams; m_beamIdx++)
            {
                if (azimuthData[m_beamIdx] == azimuthRange.x)
                {
                    foundStart = true;
                    break;
                }
            }
            if (!foundStart)
            {
                return;
            }
            m_resetLaserScan = false;
        }

        if (m_numBeamsRemaining > numBeams)
        {
            for (size_t i = m_beamIdx; i < numBeams; i++)
            {
                m_beamTimeData.push_back(beamTimes[i]);
                m_intensitiesData.push_back(intensities[i]);
                m_rangesData.push_back(ranges[i]);
                m_numBeamsRemaining--;
            }
            m_beamIdx = 0;
        }

        else
        {

            // Save data up to maximum FOV
            size_t idx;
            for (idx = 0; idx < m_numBeamsRemaining; idx++)
            {
                m_beamTimeData.push_back(beamTimes[idx]);
                m_intensitiesData.push_back(intensities[idx]);
                m_rangesData.push_back(ranges[idx]);
            }

            size_t buffSize = m_rangesData.size();

            db.outputs.beamTimeData.resize(buffSize);
            db.outputs.linearDepthData.resize(buffSize);
            db.outputs.intensitiesData.resize(buffSize);

            std::memcpy(db.outputs.beamTimeData().data(), &m_beamTimeData[0], m_beamTimeData.size() * sizeof(float));
            std::memcpy(db.outputs.linearDepthData().data(), &m_rangesData[0], m_rangesData.size() * sizeof(float));
            std::memcpy(
                db.outputs.intensitiesData().data(), &m_intensitiesData[0], m_intensitiesData.size() * sizeof(uint8_t));

            db.outputs.execOut() = kExecutionAttributeStateEnabled;


            // Reset fields for new lidar scan
            m_beamTimeData.clear();
            m_rangesData.clear();
            m_intensitiesData.clear();

            if (idx < numBeams)
            {
                if (azimuthData[idx] != azimuthRange.x)
                {
                    m_resetLaserScan = true;
                    return;
                }
            }

            // Save remaining data
            size_t numBeamsOffset = numBeams - m_numBeamsRemaining;
            for (size_t j = 0; j < numBeamsOffset; j++)
            {
                m_beamTimeData.push_back(beamTimes[idx]);
                m_intensitiesData.push_back(intensities[idx]);
                m_rangesData.push_back(ranges[idx]);
                idx++;
            }
            m_numBeamsRemaining = numBeamsTotal - numBeamsOffset;
        }
    }

    virtual void reset()
    {
        m_resetLaserScan = true;
        m_firstFrame = true;
    }


private:
    isaacsim::sensors::physx::LidarSensorInterface* m_lidarSensorInterface = nullptr;
    pxr::UsdPrim m_sensorPrim;

    const char* m_lidarPrimPath = nullptr;

    std::vector<uint8_t> m_intensitiesData;
    std::vector<float> m_rangesData;
    std::vector<float> m_beamTimeData;

    uint64_t m_prevSequenceNumber = 0;

    bool m_resetLaserScan = true;
    size_t m_numBeamsRemaining = 0;

    size_t m_beamIdx = 0;

    bool m_firstFrame = true;
};

REGISTER_OGN_NODE()
} // nodes
} // graph
} // omni

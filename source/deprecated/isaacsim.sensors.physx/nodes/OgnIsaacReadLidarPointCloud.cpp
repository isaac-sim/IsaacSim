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

#include <OgnIsaacReadLidarPointCloudDatabase.h>

using namespace isaacsim::robot::schema::sensors;

namespace isaacsim
{
namespace core
{
namespace nodes
{

class OgnIsaacReadLidarPointCloud : public isaacsim::core::includes::BaseResetNode
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnIsaacReadLidarPointCloudDatabase::sPerInstanceState<OgnIsaacReadLidarPointCloud>(nodeObj, instanceId);

        state.m_lidarSensorInterface = carb::getCachedInterface<isaacsim::sensors::physx::LidarSensorInterface>();

        if (!state.m_lidarSensorInterface)
        {
            CARB_LOG_ERROR("Failed to acquire isaacsim::sensors::physx interface");
            return;
        }
    }

    static bool compute(OgnIsaacReadLidarPointCloudDatabase& db)
    {
        const GraphContextObj& context = db.abi_context();

        auto& state = db.perInstanceState<OgnIsaacReadLidarPointCloud>();

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

            state.m_unitScale = UsdGeomGetStageMetersPerUnit(stage);

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

        state.readLidarPointCloud(db);
        return true;
    }


    void readLidarPointCloud(OgnIsaacReadLidarPointCloudDatabase& db)
    {
        float maxRange = 100;

        isaacsim::core::includes::safeGetAttribute(m_sensorPrim.GetAttribute(kMaxRangeAttr), maxRange);

        carb::Float3* lidarData = m_lidarSensorInterface->getPointCloud(m_lidarPrimPath);
        // float* theta = m_lidarSensorInterface->getAzimuthData(m_lidarPrimPath);
        float* ranges = m_lidarSensorInterface->getLinearDepthData(m_lidarPrimPath);

        if (!ranges || !lidarData)
        {
            return;
        }

        int numColsTicked = m_lidarSensorInterface->getNumColsTicked(m_lidarPrimPath);
        int numCols = m_lidarSensorInterface->getNumCols(m_lidarPrimPath);
        int numRows = m_lidarSensorInterface->getNumRows(m_lidarPrimPath);

        size_t numBeams = numColsTicked * numRows;
        size_t numBeamsTotal = numRows * numCols;

        uint64_t currSequenceNum = m_lidarSensorInterface->getSequenceNumber(m_lidarPrimPath);


        if (currSequenceNum == m_prevSequenceNumber)
        {
            return;
        }

        if (currSequenceNum < m_prevSequenceNumber)
        {
            m_resetPCL = true;
        }

        m_prevSequenceNumber = currSequenceNum;

        if (m_resetPCL)
        {
            m_pointsData.clear();
            m_numBeamsRemainingPCL = numBeamsTotal;
            m_resetPCL = false;
        }


        if (m_numBeamsRemainingPCL > numBeams)
        {
            for (size_t i = 0; i < numBeams; i++)
            {

                if (ranges[i] >= maxRange)
                {
                    continue;
                }

                GfVec3f point = { lidarData[i].x, lidarData[i].y, lidarData[i].z };
                m_pointsData.push_back(point * m_unitScale);
            }
            m_numBeamsRemainingPCL -= numBeams;
        }
        else
        {

            // Save data up to maximum FOV
            size_t i = 0;
            for (i = 0; i < m_numBeamsRemainingPCL; i++)
            {
                if (ranges[i] >= maxRange)
                {
                    continue;
                }

                GfVec3f point = { lidarData[i].x, lidarData[i].y, lidarData[i].z };
                m_pointsData.push_back(point * m_unitScale);
            }

            db.outputs.data().resize(m_pointsData.size());

            memcpy(db.outputs.data().data(), &m_pointsData[0], m_pointsData.size() * sizeof(GfVec3f));

            db.outputs.execOut() = kExecutionAttributeStateEnabled;


            m_pointsData.clear();

            // Save remaining data
            size_t numBeamsOffset = numBeams - m_numBeamsRemainingPCL;
            for (size_t j = 0; j < numBeamsOffset; j++)
            {
                if (ranges[i] >= maxRange)
                {
                    i++;
                    continue;
                }
                GfVec3f point = { lidarData[i].x, lidarData[i].y, lidarData[i].z };
                m_pointsData.push_back(point * m_unitScale);
                i++;
            }
            m_numBeamsRemainingPCL = numBeamsTotal - numBeamsOffset;
        }
    }
    static bool updateNodeVersion(const GraphContextObj& context, const NodeObj& nodeObj, int oldVersion, int newVersion)
    {
        if (oldVersion < newVersion)
        {
            const INode* const iNode = nodeObj.iNode;
            if (oldVersion < 2)
            {
                iNode->removeAttribute(nodeObj, "outputs:pointCloudData");
                CARB_LOG_ERROR(
                    "outputs:pointCloudData renamed to outputs:data, downstream connections will need to be re-connected");
            }
            return true;
        }
        return false;
    }
    virtual void reset()
    {
        m_resetPCL = true;
        m_firstFrame = true;
    }


private:
    isaacsim::sensors::physx::LidarSensorInterface* m_lidarSensorInterface = nullptr;
    pxr::UsdPrim m_sensorPrim;

    const char* m_lidarPrimPath = nullptr;

    std::vector<GfVec3f> m_pointsData;

    uint64_t m_prevSequenceNumber = 0;

    bool m_resetPCL = true;
    size_t m_numBeamsRemainingPCL = 0;

    bool m_firstFrame = true;

    double m_unitScale = 1.0;
};

REGISTER_OGN_NODE()
} // nodes
} // graph
} // omni

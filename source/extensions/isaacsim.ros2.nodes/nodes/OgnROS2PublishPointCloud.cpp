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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <carb/profiler/Profile.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <isaacsim/ros2/core/Ros2Node.h>
#include <isaacsim/ros2/nodes/PointCloudPublisher.h>

#include <OgnROS2PublishPointCloudDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2PublishPointCloud : public Ros2Node
{
public:
    static bool compute(OgnROS2PublishPointCloudDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishPointCloud>();
        const auto& nodeObj = db.abi_node();

        if (!state.m_pub.isInitialized())
        {
            const GraphContextObj& context = db.abi_context();
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

            if (!state.m_pub.initialize(
                    std::string(nodeObj.iNode->getPrimPath(nodeObj)),
                    collectNamespace(db.inputs.nodeNamespace(),
                                     stage->GetPrimAtPath(pxr::SdfPath(nodeObj.iNode->getPrimPath(nodeObj)))),
                    db.inputs.topicName(), db.inputs.frameId(), db.inputs.queueSize(), db.inputs.qosProfile(),
                    db.inputs.context()))
            {
                db.logError("Unable to create ROS2 publisher");
                return false;
            }

            carb::settings::ISettings* threadSettings = carb::getCachedInterface<carb::settings::ISettings>();
            static constexpr char s_kThreadDisable[] = "/exts/isaacsim.ros2.bridge/publish_multithreading_disabled";
            state.m_multithreadingDisabled = threadSettings->getAsBool(s_kThreadDisable);
            return true;
        }

        return state.publishLidar(db);
    }


    bool publishLidar(OgnROS2PublishPointCloudDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] Lidar Point Cloud Pub");
        auto& state = db.perInstanceState<OgnROS2PublishPointCloud>();
        auto tasking = carb::getCachedInterface<carb::tasking::ITasking>();

        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] wait for previous publish");
            state.m_tasks.wait();
        }

        // Cartesian data + individual metadata pointers, gated by output* flags
        isaacsim::ros2::nodes::PointCloudMetadata metadata;
        metadata.intensityPtr =
            db.inputs.outputIntensity() ? reinterpret_cast<float*>(db.inputs.intensityPtr()) : nullptr;
        metadata.timestampPtr =
            db.inputs.outputTimestamp() ? reinterpret_cast<uint64_t*>(db.inputs.timestampPtr()) : nullptr;
        metadata.emitterIdPtr =
            db.inputs.outputEmitterId() ? reinterpret_cast<uint32_t*>(db.inputs.emitterIdPtr()) : nullptr;
        metadata.channelIdPtr =
            db.inputs.outputChannelId() ? reinterpret_cast<uint32_t*>(db.inputs.channelIdPtr()) : nullptr;
        metadata.materialIdPtr =
            db.inputs.outputMaterialId() ? reinterpret_cast<uint32_t*>(db.inputs.materialIdPtr()) : nullptr;
        metadata.tickIdPtr = db.inputs.outputTickId() ? reinterpret_cast<uint32_t*>(db.inputs.tickIdPtr()) : nullptr;
        metadata.hitNormalPtr =
            db.inputs.outputHitNormal() ? reinterpret_cast<GfVec3f*>(db.inputs.hitNormalPtr()) : nullptr;
        metadata.velocityPtr = db.inputs.outputVelocity() ? reinterpret_cast<GfVec3f*>(db.inputs.velocityPtr()) : nullptr;
        metadata.objectIdPtr =
            db.inputs.outputObjectId() ? reinterpret_cast<uint32_t*>(db.inputs.objectIdPtr()) : nullptr;
        metadata.echoIdPtr = db.inputs.outputEchoId() ? reinterpret_cast<uint8_t*>(db.inputs.echoIdPtr()) : nullptr;
        metadata.tickStatePtr =
            db.inputs.outputTickState() ? reinterpret_cast<uint8_t*>(db.inputs.tickStatePtr()) : nullptr;
        metadata.radialVelocityMSPtr =
            db.inputs.outputRadialVelocityMS() ? reinterpret_cast<float*>(db.inputs.radialVelocityMSPtr()) : nullptr;

        if (db.inputs.cudaDeviceIndex() == -1)
        {
            if (db.inputs.dataPtr() != 0)
            {
                if (db.inputs.bufferSize() == 0)
                {
                    return false;
                }
                state.m_pub.getPointCloudMessage()->generateBuffer(
                    db.inputs.timeStamp(), state.m_pub.getFrameId(), db.inputs.bufferSize(), metadata.intensityPtr,
                    metadata.timestampPtr, metadata.emitterIdPtr, metadata.channelIdPtr, metadata.materialIdPtr,
                    metadata.tickIdPtr, metadata.hitNormalPtr, metadata.velocityPtr, metadata.objectIdPtr,
                    metadata.echoIdPtr, metadata.tickStatePtr, metadata.radialVelocityMSPtr);
                // Data is on host as ptr
                if (state.m_pub.getPointCloudMessage()->getOrderedFields().empty())
                {
                    // Direct copy when no metadata interleaving is needed
                    memcpy(state.m_pub.getPointCloudMessage()->getBufferPtr(),
                           reinterpret_cast<void*>(db.inputs.dataPtr()),
                           state.m_pub.getPointCloudMessage()->getTotalBytes());
                }
                else
                {
                    // Host interleave: fill buffer with xyz + metadata per point
                    isaacsim::ros2::nodes::fillPointCloudBufferHost(
                        reinterpret_cast<uint8_t*>(state.m_pub.getPointCloudMessage()->getBufferPtr()),
                        reinterpret_cast<const float3*>(db.inputs.dataPtr()),
                        state.m_pub.getPointCloudMessage()->getOrderedFields(),
                        state.m_pub.getPointCloudMessage()->getPointStep(),
                        state.m_pub.getPointCloudMessage()->getNumPoints());
                }
            }
            else
            {
                const size_t totalBytes = sizeof(GfVec3f) * db.inputs.data.size();
                if (totalBytes == 0)
                {
                    return false;
                }
                state.m_pub.getPointCloudMessage()->generateBuffer(
                    db.inputs.timeStamp(), state.m_pub.getFrameId(), totalBytes, metadata.intensityPtr,
                    metadata.timestampPtr, metadata.emitterIdPtr, metadata.channelIdPtr, metadata.materialIdPtr,
                    metadata.tickIdPtr, metadata.hitNormalPtr, metadata.velocityPtr, metadata.objectIdPtr,
                    metadata.echoIdPtr, metadata.tickStatePtr, metadata.radialVelocityMSPtr);
                // Data is on host as ogn data, copy from cpu
                {
                    memcpy(state.m_pub.getPointCloudMessage()->getBufferPtr(),
                           reinterpret_cast<const uint8_t*>(db.inputs.data.cpu().data()),
                           state.m_pub.getPointCloudMessage()->getTotalBytes());
                }
            }

            if (state.m_multithreadingDisabled)
            {
                CARB_PROFILE_ZONE(1, "[IsaacSim] Publish PCL");
                state.m_pub.send();
            }
            else
            {
                tasking->addTask(carb::tasking::Priority::eHigh, state.m_tasks,
                                 [&state]
                                 {
                                     CARB_PROFILE_ZONE(1, "[IsaacSim] Publish PCL Thread");
                                     state.m_pub.send();
                                 });
            }
        }
        else if (db.inputs.dataPtr() != 0)
        {
            const void* devicePtr = reinterpret_cast<const void*>(db.inputs.dataPtr());
            const size_t bufferSize = db.inputs.bufferSize();
            const double timestamp = db.inputs.timeStamp();
            const int cudaDeviceIndex = db.inputs.cudaDeviceIndex();

            if (state.m_multithreadingDisabled)
            {
                state.m_pub.publishFromDevice(devicePtr, bufferSize, timestamp, cudaDeviceIndex, metadata);
            }
            else
            {
                tasking->addTask(
                    carb::tasking::Priority::eHigh, state.m_tasks,
                    [&state, devicePtr, bufferSize, timestamp, cudaDeviceIndex, metadata]() mutable
                    { state.m_pub.publishFromDevice(devicePtr, bufferSize, timestamp, cudaDeviceIndex, metadata); });
            }
        }

        return true;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublishPointCloudDatabase::sPerInstanceState<OgnROS2PublishPointCloud>(nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] wait for previous publish");
            m_tasks.wait();
        }
        m_pub.reset();
        Ros2Node::reset();
    }

private:
    isaacsim::ros2::nodes::PointCloudPublisher m_pub;

    carb::tasking::TaskGroup m_tasks;

    bool m_multithreadingDisabled = false;
};

REGISTER_OGN_NODE()

#ifdef _WIN32
#    pragma warning(pop)
#endif

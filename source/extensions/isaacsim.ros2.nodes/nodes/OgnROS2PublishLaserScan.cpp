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

#include <carb/profiler/Profile.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <isaacsim/ros2/core/Ros2Node.h>

#include <GenericModelOutput.h>
#include <OgnROS2PublishLaserScanDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2PublishLaserScan : public Ros2Node
{
public:
    static bool compute(OgnROS2PublishLaserScanDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishLaserScan>();

        // spin once calls reset automatically if it was not successful
        const auto& nodeObj = db.abi_node();
        if (!state.isInitialized())
        {
            const GraphContextObj& context = db.abi_context();
            // Find our stage
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

            if (!state.initializeNodeHandle(
                    std::string(nodeObj.iNode->getPrimPath(nodeObj)),
                    collectNamespace(db.inputs.nodeNamespace(),
                                     stage->GetPrimAtPath(pxr::SdfPath(nodeObj.iNode->getPrimPath(nodeObj)))),
                    db.inputs.context()))
            {
                db.logError("Unable to create ROS2 node, please check that namespace is valid");
                return false;
            }
        }

        // Publisher was not valid, create a new one
        if (!state.m_publisher)
        {
            CARB_PROFILE_ZONE(0, "[IsaacSim] setup publisher");
            // Setup ROS publisher
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
                return false;
            }

            state.m_message = state.m_factory->createLaserScanMessage();

            Ros2QoSProfile qos;
            const std::string& qosProfile = db.inputs.qosProfile();
            if (qosProfile.empty())
            {
                qos.depth = db.inputs.queueSize();
            }
            else
            {
                if (!jsonToRos2QoSProfile(qos, qosProfile))
                {
                    return false;
                }
            }

            state.m_publisher = state.m_factory->createPublisher(
                state.m_nodeHandle.get(), fullTopicName.c_str(), state.m_message->getTypeSupportHandle(), qos);

            state.m_frameId = db.inputs.frameId();

            // Get extension settings for multithreading
            carb::settings::ISettings* threadSettings = carb::getCachedInterface<carb::settings::ISettings>();
            static constexpr char s_kThreadDisable[] = "/exts/isaacsim.ros2.bridge/publish_multithreading_disabled";
            state.m_multithreadingDisabled = threadSettings->getAsBool(s_kThreadDisable);
            return true;
        }
        return state.publishLidar(db);
    }

    bool publishMessage(OgnROS2PublishLaserScanDatabase& db,
                        const float* rangeDataSrc,
                        const uint8_t* intensityDataSrc,
                        size_t buffSize)
    {
        auto& state = db.perInstanceState<OgnROS2PublishLaserScan>();
        auto tasking = carb::getCachedInterface<carb::tasking::ITasking>();

        state.m_message->writeHeader(db.inputs.timeStamp(), state.m_frameId);
        state.m_message->generateBuffers(buffSize);
        state.m_message->writeData(db.inputs.azimuthRange(), db.inputs.rotationRate(), db.inputs.depthRange(),
                                   db.inputs.horizontalResolution(), db.inputs.horizontalFov());

        std::vector<float>& rangeData = state.m_message->getRangeData();
        memcpy(rangeData.data(), rangeDataSrc, sizeof(float) * buffSize);

        std::vector<float>& intensitiesData = state.m_message->getIntensitiesData();
        std::transform(intensityDataSrc, intensityDataSrc + buffSize, intensitiesData.begin(),
                       [](uint8_t val) { return static_cast<float>(val); });

        if (state.m_multithreadingDisabled)
        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] LaserScan publisher publish");
            state.m_publisher.get()->publish(state.m_message->getPtr());
        }
        else
        {
            tasking->addTask(carb::tasking::Priority::eHigh, state.m_tasks,
                             [&state]
                             {
                                 CARB_PROFILE_ZONE(1, "[IsaacSim] LaserScan publisher publish");
                                 state.m_publisher.get()->publish(state.m_message->getPtr());
                             });
        }
        return true;
    }

    bool publishFromGMO(OgnROS2PublishLaserScanDatabase& db)
    {
        CARB_PROFILE_ZONE(1, "[IsaacSim] publish laserscan from GMO");

        auto* gmo = omni::sensors::getModelOutputPtrFromBuffer(reinterpret_cast<void*>(db.inputs.dataPtr()));
        if (!gmo || gmo->numElements == 0)
        {
            CARB_LOG_INFO("ROS2PublishLaserScan: GMO buffer is empty or invalid. Skipping.");
            return false;
        }

        const float horizontalFov = db.inputs.horizontalFov();
        const float horizontalRes = db.inputs.horizontalResolution();
        if (horizontalRes <= 0.0f || horizontalFov <= 0.0f)
        {
            db.logError(
                "horizontalFov (%f) and horizontalResolution (%f) must be positive", horizontalFov, horizontalRes);
            return false;
        }

        const size_t numOutputElements = static_cast<size_t>(horizontalFov / horizontalRes);
        const float azimuthRangeStart = db.inputs.azimuthRange()[0];
        const size_t numInputElements = static_cast<size_t>(gmo->numElements);

        if (m_linearDepthBuffer.capacity() == 0)
        {
            m_linearDepthBuffer.reserve(numOutputElements);
            m_intensitiesBuffer.reserve(numOutputElements);
        }
        m_linearDepthBuffer.resize(numOutputElements);
        m_intensitiesBuffer.resize(numOutputElements);
        std::fill(m_linearDepthBuffer.begin(), m_linearDepthBuffer.end(), -1.0f);
        std::fill(m_intensitiesBuffer.begin(), m_intensitiesBuffer.end(), 0);

        // Map GMO elements into flat scan slots by azimuth
        const bool isCartesian = gmo->elementsCoordsType == omni::sensors::CoordsType::CARTESIAN;
        const float radToDeg = 180.0f / static_cast<float>(M_PI);

        for (size_t inIdx = 0; inIdx < numInputElements; inIdx++)
        {
            float azimuth;
            float distance;
            if (isCartesian)
            {
                const float x = gmo->elements.x[inIdx];
                const float y = gmo->elements.y[inIdx];
                const float z = gmo->elements.z[inIdx];
                distance = sqrtf(x * x + y * y + z * z);
                azimuth = atan2f(y, x) * radToDeg;
            }
            else
            {
                azimuth = gmo->elements.x[inIdx];
                distance = gmo->elements.z[inIdx];
            }
            uint8_t intensity = static_cast<uint8_t>(gmo->elements.scalar[inIdx] * 255.0f);
            size_t outIdx = static_cast<size_t>((azimuth - azimuthRangeStart) / horizontalRes);
            if (outIdx >= numOutputElements)
            {
                outIdx = numOutputElements - 1;
            }
            m_linearDepthBuffer[outIdx] = distance;
            m_intensitiesBuffer[outIdx] = intensity;
        }

        return publishMessage(db, m_linearDepthBuffer.data(), m_intensitiesBuffer.data(), numOutputElements);
    }

    bool publishLidar(OgnROS2PublishLaserScanDatabase& db)
    {
        CARB_PROFILE_ZONE(1, "[IsaacSim] publish laserscan function");
        auto& state = db.perInstanceState<OgnROS2PublishLaserScan>();

        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] wait for previous publish");
            // Wait for last message to publish before starting next
            state.m_tasks.wait();
        }
        // Check if subscription count is 0
        if (!m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return false;
        }

        // GMO path: process GenericModelOutput buffer directly
        if (db.inputs.dataPtr() != 0)
        {
            return publishFromGMO(db);
        }

        // Legacy path: use pre-processed linearDepthData/intensitiesData
        size_t buffSize = db.inputs.numCols() * db.inputs.numRows();
        if (buffSize == 0)
        {
            return false;
        }
        if (db.inputs.numRows() != 1)
        {
            db.logError(
                "Number of rows must be equal to 1. High LOD not supported for LaserScan, only 2D Lidar Supported for LaserScan. Please disable Lidar High LOD setting");
            return false;
        }

        if (!db.inputs.linearDepthData.isValid())
        {
            db.logError("linearDepthData buffer is invalid");
            return false;
        }

        if (buffSize != db.inputs.linearDepthData.size())
        {
            db.logError("Lidar data with %d rows and %d columns does not match input buffer array size of %d",
                        db.inputs.numRows(), db.inputs.numCols(), db.inputs.linearDepthData.size());
            return false;
        }

        const float* depthData = db.inputs.linearDepthData().data();
        const uint8_t* intensityData = nullptr;

        if (!db.inputs.intensitiesData.isValid() || db.inputs.intensitiesData.size() == 0 ||
            db.inputs.intensitiesData.size() != db.inputs.linearDepthData.size())
        {
            if (db.inputs.intensitiesData.isValid() && db.inputs.intensitiesData.size() > 0)
            {
                CARB_LOG_WARN_ONCE(
                    "intensitiesData size (%zu) != linearDepthData size (%zu); synthesizing binary intensities",
                    db.inputs.intensitiesData.size(), db.inputs.linearDepthData.size());
            }
            m_intensitiesBuffer.resize(buffSize);
            for (size_t i = 0; i < buffSize; i++)
            {
                m_intensitiesBuffer[i] = (depthData[i] > 0.0f) ? 255 : 0;
            }
            intensityData = m_intensitiesBuffer.data();
        }
        else
        {
            intensityData = static_cast<const uint8_t*>(db.inputs.intensitiesData().data());
        }

        return publishMessage(db, depthData, intensityData, buffSize);
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublishLaserScanDatabase::sPerInstanceState<OgnROS2PublishLaserScan>(nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] wait for previous publish");
            // Wait for last message to publish before starting next
            m_tasks.wait();
        }
        m_publisher.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();
    }


private:
    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2LaserScanMessage> m_message = nullptr;

    std::string m_frameId = "sim_lidar";

    carb::tasking::TaskGroup m_tasks;

    bool m_multithreadingDisabled = false;

    // Reusable buffers for GMO flat scan processing
    std::vector<float> m_linearDepthBuffer;
    std::vector<uint8_t> m_intensitiesBuffer;
};

REGISTER_OGN_NODE()

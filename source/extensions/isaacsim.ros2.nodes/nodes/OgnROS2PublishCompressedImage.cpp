// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <OgnROS2PublishCompressedImageDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2PublishCompressedImage : public Ros2Node
{
public:
    static bool compute(OgnROS2PublishCompressedImageDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishCompressedImage>();
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
            CARB_PROFILE_ZONE(0, "[IsaacSim] setup compressed image publisher");
            // Setup ROS publisher
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
                return false;
            }

            state.m_message = state.m_factory->createCompressedImageMessage();

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

            return true;
        }

        return state.publishCompressedImage(db);
    }

    bool publishCompressedImage(OgnROS2PublishCompressedImageDatabase& db)
    {
        CARB_PROFILE_ZONE(1, "[IsaacSim] publish compressed image function");
        auto& state = db.perInstanceState<OgnROS2PublishCompressedImage>();

        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] wait for previous publish");
            // Wait for last message to publish before starting next
            state.m_tasks.wait();
        }

        // Check if subscription count is 0
        if (!state.m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return false;
        }

        state.m_message->writeHeader(db.inputs.timeStamp(), state.m_frameId);

        // Get data pointer and size
        const uint8_t* dataPtr = nullptr;
        size_t dataSize = 0;

        if (db.inputs.dataPtr() != 0 && db.inputs.bufferSize() > 0)
        {
            // Data is provided as a pointer
            dataPtr = reinterpret_cast<const uint8_t*>(db.inputs.dataPtr());
            dataSize = db.inputs.bufferSize();
        }
        else if (db.inputs.data.size() > 0)
        {
            // Data is provided as OGN array
            dataPtr = reinterpret_cast<const uint8_t*>(db.inputs.data.cpu().data());
            dataSize = db.inputs.data.size();
        }

        // Skip publishing if no data available
        if (dataPtr == nullptr || dataSize == 0)
        {
            // No data available yet - this can happen during initialization
            return true;
        }

        // Get the format string
        std::string format = db.tokenToString(db.inputs.input_format());

        // Write the compressed data to the message
        state.m_message->writeData(dataPtr, dataSize, format);

        // Publish the message
        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] compressed image publisher publish");
            state.m_publisher.get()->publish(state.m_message->getPtr());
        }

        return true;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnROS2PublishCompressedImageDatabase::sPerInstanceState<OgnROS2PublishCompressedImage>(nodeObj, instanceId);
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
    std::shared_ptr<Ros2CompressedImageMessage> m_message = nullptr;

    std::string m_frameId = "sim_camera";

    carb::tasking::TaskGroup m_tasks;
};

REGISTER_OGN_NODE()

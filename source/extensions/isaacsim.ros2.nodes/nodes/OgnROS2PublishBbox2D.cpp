// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/ros2/core/Ros2Factory.h>
#include <isaacsim/ros2/core/Ros2Node.h>

#include <OgnROS2PublishBbox2DDatabase.h>

using namespace isaacsim::ros2::core;

struct Bbox2DData
{
    uint32_t semanticId;
    int32_t xMin;
    int32_t yMin;
    int32_t xMax;
    int32_t yMax;
    float occlusionRatio;
};

class OgnROS2PublishBbox2D : public Ros2Node
{
public:
    static bool compute(OgnROS2PublishBbox2DDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishBbox2D>();

        // Spin once calls reset automatically if it was not successful
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
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
                return false;
            }

            state.m_message = state.m_factory->createBoundingBox2DMessage();
            if (!state.m_message->getPtr())
            {
                CARB_LOG_ERROR("Unable to find Detection2DArray message type");
                return false;
            }

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

        return state.publishDetectionArray(db);
    }

    bool publishDetectionArray(OgnROS2PublishBbox2DDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishBbox2D>();

        // Check if subscription count is 0
        if (!m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return false;
        }

        size_t bytes = db.inputs.data().size();
        size_t numBbox = bytes / sizeof(Bbox2DData);
        const Bbox2DData* bboxData = reinterpret_cast<const Bbox2DData*>(db.inputs.data().data());

        state.m_message->writeHeader(db.inputs.timeStamp(), state.m_frameId);
        state.m_message->writeBboxData(bboxData, numBbox);
        state.m_publisher.get()->publish(state.m_message->getPtr());
        return true;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublishBbox2DDatabase::sPerInstanceState<OgnROS2PublishBbox2D>(nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        m_publisher.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();
    }

private:
    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2BoundingBox2DMessage> m_message = nullptr;

    std::string m_frameId = "sim_camera";
};

REGISTER_OGN_NODE()

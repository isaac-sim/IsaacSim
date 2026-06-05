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

#include <isaacsim/ros2/core/Ros2Node.h>

#include <OgnROS2PublishImuDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2PublishImu : public Ros2Node
{
public:
    static bool compute(OgnROS2PublishImuDatabase& db)
    {
        const GraphContextObj& context = db.abi_context();
        auto& state = db.perInstanceState<OgnROS2PublishImu>();

        // Spin once calls reset automatically if it was not successful
        const auto& nodeObj = db.abi_node();
        if (!state.isInitialized())
        {
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

        // Either publisher was not valid, create a new one
        if (!state.m_publisher)
        {
            // Find our stage
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

            if (!stage)
            {
                db.logError("Could not find USD stage %ld", stageId);
                return false;
            }

            // Setup ROS IMU publisher
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
                return false;
            }

            state.m_message = state.m_factory->createImuMessage();

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

        state.publishImu(db);
        return true;
    }

    void publishImu(OgnROS2PublishImuDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishImu>();

        // Check if subscription count is 0
        if (!m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return;
        }

        state.m_message->writeHeader(db.inputs.timeStamp(), state.m_frameId);

        if (!db.inputs.publishLinearAcceleration())
        {
            state.m_message->writeAcceleration(true);
        }
        else
        {
            auto& linearAcceleration = db.inputs.linearAcceleration();
            std::vector<double> acceleration{ linearAcceleration[0], linearAcceleration[1], linearAcceleration[2] };
            state.m_message->writeAcceleration(false, acceleration);
        }

        if (!db.inputs.publishAngularVelocity())
        {
            state.m_message->writeVelocity(true);
        }
        else
        {
            auto& angularVelocity = db.inputs.angularVelocity();
            std::vector<double> velocity{ angularVelocity[0], angularVelocity[1], angularVelocity[2] };
            state.m_message->writeVelocity(false, velocity);
        }

        if (!db.inputs.publishOrientation())
        {
            state.m_message->writeOrientation(true);
        }
        else
        {
            auto& orientation = db.inputs.orientation();
            std::vector<double> orientationVector{ orientation.GetImaginary()[0], orientation.GetImaginary()[1],
                                                   orientation.GetImaginary()[2], orientation.GetReal() };
            state.m_message->writeOrientation(false, orientationVector);
        }

        state.m_publisher.get()->publish(state.m_message->getPtr());
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublishImuDatabase::sPerInstanceState<OgnROS2PublishImu>(nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        m_publisher.reset(); // Publisher should be reset before we reset the handle.
        Ros2Node::reset();
    }


private:
    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2ImuMessage> m_message = nullptr;
    std::string m_frameId = "sim_imu";
};

REGISTER_OGN_NODE()

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

#include <isaacsim/ros2/core/Ros2Node.h>

#include <OgnROS2SubscribeAckermannDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2SubscribeAckermann : public Ros2Node
{
public:
    static bool compute(OgnROS2SubscribeAckermannDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeAckermann>();
        state.m_nodeObj = db.abi_node();
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

        // Subscriber was not valid, create a new one
        if (!state.m_subscriber)
        {
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 subscriber, invalid topic name");
                return false;
            }

            state.m_message = state.m_factory->createAckermannDriveStampedMessage();
            if (!state.m_message->getPtr())
            {
                CARB_LOG_ERROR("Unable to find AckermannDriveStamped message type");
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

            state.m_subscriber = state.m_factory->createSubscriber(
                state.m_nodeHandle.get(), fullTopicName.c_str(), state.m_message->getTypeSupportHandle(), qos);
            return true;
        }

        return state.subscriberCallback(db);
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnROS2SubscribeAckermannDatabase::sPerInstanceState<OgnROS2SubscribeAckermann>(nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        if (!m_nodeObj.iNode)
        {
            return;
        }
        GraphObj graphObj{ m_nodeObj.iNode->getGraph(m_nodeObj) };
        GraphContextObj context{ graphObj.iGraph->getDefaultGraphContext(graphObj) };

        // For acceleration
        AttributeObj accelerationAttr = m_nodeObj.iNode->getAttribute(m_nodeObj, "outputs:acceleration");
        auto accelerationHandle =
            accelerationAttr.iAttribute->getAttributeDataHandle(accelerationAttr, kAccordingToContextIndex);
        double* accelerationCommand = getDataW<double>(context, accelerationHandle);
        *accelerationCommand = 0.0;

        // For jerk
        AttributeObj jerkAttr = m_nodeObj.iNode->getAttribute(m_nodeObj, "outputs:jerk");
        auto jerkHandle = jerkAttr.iAttribute->getAttributeDataHandle(jerkAttr, kAccordingToContextIndex);
        double* jerkCommand = getDataW<double>(context, jerkHandle);
        *jerkCommand = 0.0;

        // For speed
        AttributeObj speedAttr = m_nodeObj.iNode->getAttribute(m_nodeObj, "outputs:speed");
        auto speedHandle = speedAttr.iAttribute->getAttributeDataHandle(speedAttr, kAccordingToContextIndex);
        double* speedCommand = getDataW<double>(context, speedHandle);
        *speedCommand = 0.0;

        // For steeringAngle
        AttributeObj steeringAngleAttr = m_nodeObj.iNode->getAttribute(m_nodeObj, "outputs:steeringAngle");
        auto steeringAngleHandle =
            steeringAngleAttr.iAttribute->getAttributeDataHandle(steeringAngleAttr, kAccordingToContextIndex);
        double* steeringAngleCommand = getDataW<double>(context, steeringAngleHandle);
        *steeringAngleCommand = 0.0;

        // For steeringAngleVelocity
        AttributeObj steeringAngleVelocityAttr =
            m_nodeObj.iNode->getAttribute(m_nodeObj, "outputs:steeringAngleVelocity");
        auto steeringAngleVelocityHandle = steeringAngleVelocityAttr.iAttribute->getAttributeDataHandle(
            steeringAngleVelocityAttr, kAccordingToContextIndex);
        double* steeringAngleVelocityCommand = getDataW<double>(context, steeringAngleVelocityHandle);
        *steeringAngleVelocityCommand = 0.0;

        // For timeStamp
        AttributeObj timeStampAttr = m_nodeObj.iNode->getAttribute(m_nodeObj, "outputs:timeStamp");
        auto timeStampHandle = timeStampAttr.iAttribute->getAttributeDataHandle(timeStampAttr, kAccordingToContextIndex);
        double* timeStampCommand = getDataW<double>(context, timeStampHandle);
        *timeStampCommand = 0.0;


        m_subscriber.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();
    }

    bool subscriberCallback(OgnROS2SubscribeAckermannDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeAckermann>();

        if (state.m_subscriber->spin(state.m_message->getPtr()))
        {
            std::string frameId;

            auto& timeStamp = db.outputs.timeStamp();
            auto& steeringAngle = db.outputs.steeringAngle();
            auto& steeringAngleVelocity = db.outputs.steeringAngleVelocity();
            auto& speed = db.outputs.speed();
            auto& acceleration = db.outputs.acceleration();
            auto& jerk = db.outputs.jerk();

            state.m_message->readData(
                timeStamp, frameId, steeringAngle, steeringAngleVelocity, speed, acceleration, jerk);

            db.outputs.frameId() = frameId;

            db.outputs.execOut() = kExecutionAttributeStateEnabled;
            return true;
        }
        return false;
    }

private:
    std::shared_ptr<Ros2Subscriber> m_subscriber = nullptr;
    std::shared_ptr<Ros2AckermannDriveStampedMessage> m_message = nullptr;
    NodeObj m_nodeObj;
};

REGISTER_OGN_NODE()

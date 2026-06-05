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

#include <OgnROS2SubscribeTwistDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2SubscribeTwist : public Ros2Node
{
public:
    static bool compute(OgnROS2SubscribeTwistDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeTwist>();
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

            state.m_message = state.m_factory->createTwistMessage();

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
        auto& state = OgnROS2SubscribeTwistDatabase::sPerInstanceState<OgnROS2SubscribeTwist>(nodeObj, instanceId);
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

        AttributeObj linearAttr = m_nodeObj.iNode->getAttribute(m_nodeObj, "outputs:linearVelocity");
        auto linearHandle = linearAttr.iAttribute->getAttributeDataHandle(linearAttr, kAccordingToContextIndex);
        double* linearCommand = getDataW<double>(context, linearHandle);
        if (linearCommand)
        {
            linearCommand[0] = 0.0;
            linearCommand[1] = 0.0;
            linearCommand[2] = 0.0;
        }

        AttributeObj angularAttr = m_nodeObj.iNode->getAttribute(m_nodeObj, "outputs:angularVelocity");
        auto angularHandle = angularAttr.iAttribute->getAttributeDataHandle(angularAttr, kAccordingToContextIndex);
        double* angularCommand = getDataW<double>(context, angularHandle);
        if (angularCommand)
        {
            angularCommand[0] = 0.0;
            angularCommand[1] = 0.0;
            angularCommand[2] = 0.0;
        }

        m_subscriber.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();
    }

    bool subscriberCallback(OgnROS2SubscribeTwistDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeTwist>();

        if (state.m_subscriber->spin(state.m_message->getPtr()))
        {
            auto& linearVelocity = db.outputs.linearVelocity();
            auto& angularVelocity = db.outputs.angularVelocity();

            state.m_message->readData(linearVelocity, angularVelocity);
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
            return true;
        }
        return false;
    }

private:
    std::shared_ptr<Ros2Subscriber> m_subscriber = nullptr;
    std::shared_ptr<Ros2TwistMessage> m_message = nullptr;
    NodeObj m_nodeObj;
};

REGISTER_OGN_NODE()

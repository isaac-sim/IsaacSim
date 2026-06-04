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

#include <isaacsim/ros2/core/Ros2Node.h>
#include <omni/fabric/FabricUSD.h>

#include <OgnROS2SubscribeJointStateDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2SubscribeJointState : public Ros2Node
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnROS2SubscribeJointStateDatabase::sPerInstanceState<OgnROS2SubscribeJointState>(nodeObj, instanceId);
        state.m_nodeObj = nodeObj;
    }

    static bool compute(OgnROS2SubscribeJointStateDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeJointState>();

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
            // Setup ROS publisher
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 subscriber, invalid topic name");
                return false;
            }

            state.m_message = state.m_factory->createJointStateMessage();

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

    bool subscriberCallback(OgnROS2SubscribeJointStateDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeJointState>();

        if (state.m_subscriber->spin(state.m_message->getPtr()))
        {
            size_t numActuators = state.m_message->getNumJoints();
            if (numActuators == 0)
            {
                db.logWarning("No joints found");
                return false;
            }

            // Check if all sub-message size match size of actuators before setting data
            if (state.m_message->checkValid())
            {
                db.outputs.positionCommand().resize(numActuators);
                db.outputs.velocityCommand().resize(numActuators);
                db.outputs.effortCommand().resize(numActuators);
                db.outputs.jointNames().resize(numActuators);

                state.m_message->readData(m_jointNames, db.outputs.positionCommand().data(),
                                          db.outputs.velocityCommand().data(), db.outputs.effortCommand().data(),
                                          db.outputs.timeStamp());

                for (size_t i = 0; i < numActuators; i++)
                {
                    db.outputs.jointNames().at(i) = db.stringToToken(m_jointNames[i]);
                }

                db.outputs.execOut() = kExecutionAttributeStateEnabled;
            }
            else
            {
                db.logWarning("Please ensure size of position, velocity and effort arrays match the number of actuators");
                return false;
            }
        }
        return true;
    }

    static bool updateNodeVersion(const GraphContextObj& context, const NodeObj& nodeObj, int oldVersion, int newVersion)
    {
        if (oldVersion < newVersion)
        {
            const INode* const iNode = nodeObj.iNode;
            if (oldVersion < 2)
            {
                iNode->removeAttribute(nodeObj, "inputs:targetPrim");
            }
            return true;
        }
        return false;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnROS2SubscribeJointStateDatabase::sPerInstanceState<OgnROS2SubscribeJointState>(nodeObj, instanceId);
        state.reset();
    }

    virtual void reset()
    {
        auto db = OgnROS2SubscribeJointStateDatabase(m_nodeObj);

        db.outputs.jointNames.resize(0);
        db.outputs.positionCommand.resize(0);
        db.outputs.velocityCommand.resize(0);
        db.outputs.effortCommand.resize(0);

        m_subscriber.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();
    }

private:
    std::shared_ptr<Ros2Subscriber> m_subscriber = nullptr;
    std::shared_ptr<Ros2JointStateMessage> m_message = nullptr;

    // Names will be extracted as strings and later converted to tokens
    std::vector<char*> m_jointNames;

    NodeObj m_nodeObj;
};

REGISTER_OGN_NODE()

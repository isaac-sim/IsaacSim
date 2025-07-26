// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/ros2/bridge/Ros2Node.h>

#include <OgnROS2PublishOdometryDatabase.h>

using namespace isaacsim::ros2::bridge;

class OgnROS2PublishOdometry : public Ros2Node
{
public:
    static bool compute(OgnROS2PublishOdometryDatabase& db)
    {
        const GraphContextObj& context = db.abi_context();
        auto& state = db.perInstanceState<OgnROS2PublishOdometry>();

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

            state.m_zUp = UsdGeomGetStageUpAxis(stage) == "Z" ? true : false;
            state.m_unitScale = UsdGeomGetStageMetersPerUnit(stage);

            auto& robotFrontVector = db.inputs.robotFront();
            state.m_robotFront = pxr::GfVec3d(robotFrontVector[0], robotFrontVector[1], 0.0); // Z-component of zero
                                                                                              // will always be assumed

            state.m_robotFront = pxr::GfGetNormalized(state.m_robotFront, 1.0f);

            if (state.m_zUp)
            {
                state.m_robotUp = pxr::GfVec3d(0.0, 0.0, 1.0);
                state.m_robotSide = pxr::GfCross(state.m_robotUp, state.m_robotFront);
            }
            else
            {
                state.m_robotUp = pxr::GfVec3d(0.0, 1.0, 0.0);
                state.m_robotSide = pxr::GfCross(state.m_robotUp, state.m_robotFront);
            }


            // Setup ROS odom publisher
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
                return false;
            }

            state.m_message = state.m_factory->createOdometryMessage();

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

            state.m_odometryFrameId = db.inputs.odomFrameId();
            state.m_chassisFrameId = db.inputs.chassisFrameId();

            return true;
        }

        state.publishOdom(db);
        return true;
    }

    void publishOdom(OgnROS2PublishOdometryDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishOdometry>();

        // Check if subscription count is 0
        if (!m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return;
        }
        auto& linearVelocity = db.inputs.linearVelocity();
        auto& angularVelocity = db.inputs.angularVelocity();
        auto& position = db.inputs.position();
        auto& orientation = db.inputs.orientation();

        bool publishRawVelocities = db.inputs.publishRawVelocities();

        state.m_message->writeHeader(db.inputs.timeStamp(), state.m_odometryFrameId);
        state.m_message->writeData(state.m_chassisFrameId, linearVelocity, angularVelocity, m_robotFront, m_robotSide,
                                   m_robotUp, m_unitScale, position, orientation, publishRawVelocities);

        state.m_publisher.get()->publish(state.m_message->getPtr());
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublishOdometryDatabase::sPerInstanceState<OgnROS2PublishOdometry>(nodeObj, instanceId);
        state.reset();
    }

    virtual void reset()
    {
        m_publisher.reset(); // Publisher should be reset before we reset the handle.
        Ros2Node::reset();
    }

private:
    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2OdometryMessage> m_message = nullptr;

    double m_unitScale;
    bool m_zUp = true;

    // The front of the robot
    pxr::GfVec3d m_robotFront = pxr::GfVec3d(1.0, 0.0, 0.0);
    pxr::GfVec3d m_robotSide = pxr::GfVec3d(0.0, 1.0, 0.0);
    pxr::GfVec3d m_robotUp = pxr::GfVec3d(0.0, 0.0, 1.0);

    std::string m_odometryFrameId = "odom";
    std::string m_chassisFrameId = "base_link";
};

REGISTER_OGN_NODE()

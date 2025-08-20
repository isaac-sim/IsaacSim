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

#include <OgnROS2PublishCameraInfoDatabase.h>

using namespace isaacsim::ros2::bridge;

class OgnROS2PublishCameraInfo : public Ros2Node
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
    }

    static bool compute(OgnROS2PublishCameraInfoDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishCameraInfo>();

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

            state.m_message = state.m_factory->createCameraInfoMessage();

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
            return true;
        }

        state.m_frameId = db.inputs.frameId();
        state.publishCameraInfo(db);
        return true;
    }

    void publishCameraInfo(OgnROS2PublishCameraInfoDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishCameraInfo>();

        // Check if subscription count is 0
        if (!m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return;
        }

        state.m_message->writeHeader(db.inputs.timeStamp(), state.m_frameId);
        state.m_message->writeResolution(db.inputs.height(), db.inputs.width());
        if (db.inputs.k().data())
        {
            state.m_message->writeIntrinsicMatrix(db.inputs.k().data(), db.inputs.k().size());
        }
        if (db.inputs.r().data())
        {
            state.m_message->writeRectificationMatrix(db.inputs.r().data(), db.inputs.r().size());
        }
        if (db.inputs.p().data())
        {
            state.m_message->writeProjectionMatrix(db.inputs.p().data(), db.inputs.p().size());
        }
        std::string physicalDistortion = db.tokenToString(db.inputs.physicalDistortionModel());
        if (physicalDistortion.length() > 0)
        {
            std::vector<double> coeff;
            for (size_t i = 0; i < db.inputs.physicalDistortionCoefficients().size(); i++)
            {
                coeff.push_back(db.inputs.physicalDistortionCoefficients()[i]);
            }
            state.m_message->writeDistortionParameters(coeff, physicalDistortion);
        }
        else
        {
            std::vector<double> empty;
            state.m_message->writeDistortionParameters(empty, db.tokenToString(db.inputs.projectionType()));
        }

        state.m_publisher.get()->publish(state.m_message->getPtr());
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublishCameraInfoDatabase::sPerInstanceState<OgnROS2PublishCameraInfo>(nodeObj, instanceId);
        state.reset();
    }

    virtual void reset()
    {
        m_publisher.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();
    }

private:
    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2CameraInfoMessage> m_message = nullptr;

    std::string m_frameId = "sim_camera";
};

REGISTER_OGN_NODE()

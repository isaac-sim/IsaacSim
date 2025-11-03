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

#include "isaacsim/core/includes/UsdUtilities.h"

#include <carb/Framework.h>
#include <carb/Types.h>

#include <isaacsim/core/includes/Math.h>
#include <isaacsim/ros2/bridge/Ros2Node.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/physics/tensors/IArticulationView.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/TensorApi.h>

#include <OgnROS2PublishJointStateDatabase.h>

using namespace isaacsim::ros2::bridge;

class OgnROS2PublishJointState : public Ros2Node
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublishJointStateDatabase::sPerInstanceState<OgnROS2PublishJointState>(nodeObj, instanceId);
        state.m_tensorInterface = carb::getCachedInterface<omni::physics::tensors::TensorApi>();
        if (!state.m_tensorInterface)
        {
            CARB_LOG_ERROR("Failed to acquire Tensor Api interface\n");
            return;
        }
    }

    static bool compute(OgnROS2PublishJointStateDatabase& db)
    {
        const GraphContextObj& context = db.abi_context();
        auto& state = db.perInstanceState<OgnROS2PublishJointState>();


        // Spin once calls reset automatically if it was not successful
        const auto& nodeObj = db.abi_node();
        if (!state.isInitialized())
        {
            // Find our stage
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
            state.m_simView = state.m_tensorInterface->createSimulationView(stageId);

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
            // Find our stage
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
            if (!stage)
            {
                db.logError("Could not find USD stage %ld", stageId);
                return false;
            }

            const auto& prim = db.inputs.targetPrim();
            const char* primPath;
            if (!prim.empty())
            {
                if (!stage->GetPrimAtPath(omni::fabric::toSdfPath(prim[0])))
                {
                    db.logError("The prim %s is not valid. Please specify at least one valid chassis prim",
                                omni::fabric::toSdfPath(prim[0]).GetText());
                    return false;
                }
                primPath = omni::fabric::toSdfPath(prim[0]).GetText();
            }
            else
            {
                db.logError("Could not find target prim");
                return false;
            }

            state.m_unitScale = UsdGeomGetStageMetersPerUnit(stage);

            // Verify we have a valid articulation prim
            if (state.m_articulation)
                state.m_articulation->release();
            state.m_articulation = state.m_simView->createArticulationView(std::vector<std::string>{ primPath });
            if (!state.m_articulation)
            {
                db.logError("Prim %s is not an articulation", primPath);
                return false;
            }

            // Setup ROS publisher
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
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

            state.m_publisher = state.m_factory->createPublisher(
                state.m_nodeHandle.get(), fullTopicName.c_str(), state.m_message->getTypeSupportHandle(), qos);
            return true;
        }

        return state.publishJointStates(db, context);
    }

    bool publishJointStates(OgnROS2PublishJointStateDatabase& db, const GraphContextObj& context)
    {
        auto& state = db.perInstanceState<OgnROS2PublishJointState>();

        // Check if subscription count is 0
        if (!m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return false;
        }

        double stageUnits = 1.0 / m_unitScale;

        long stageId = context.iContext->getStageId(context);
        m_stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

        state.m_message->writeData(db.inputs.timeStamp(), m_articulation, m_stage, m_jointPositions, m_jointVelocities,
                                   m_jointEfforts, m_dofTypes, stageUnits, db.inputs.cudaDeviceIndex());
        state.m_publisher.get()->publish(state.m_message->getPtr());
        return true;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublishJointStateDatabase::sPerInstanceState<OgnROS2PublishJointState>(nodeObj, instanceId);
        state.reset();
    }

    virtual void reset()
    {
        if (m_articulation)
        {
            m_articulation->release();
            m_articulation = nullptr;
        }
        if (m_simView)
        {
            m_simView->release(true);
            m_simView = nullptr;
        }

        m_stage = nullptr;
        m_jointPositions.clear();
        m_jointVelocities.clear();
        m_jointEfforts.clear();
        m_dofTypes.clear();
        m_publisher.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();
    }

private:
    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2JointStateMessage> m_message = nullptr;

    pxr::UsdStageWeakPtr m_stage = nullptr;
    omni::physics::tensors::TensorApi* m_tensorInterface = nullptr;
    omni::physics::tensors::ISimulationView* m_simView = nullptr;
    omni::physics::tensors::IArticulationView* m_articulation = nullptr;
    std::vector<float> m_jointPositions;
    std::vector<float> m_jointVelocities;
    std::vector<float> m_jointEfforts;
    std::vector<uint8_t> m_dofTypes;

    double m_unitScale = 1;
};

REGISTER_OGN_NODE()

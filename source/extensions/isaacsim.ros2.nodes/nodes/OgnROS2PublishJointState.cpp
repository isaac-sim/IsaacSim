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

#include <carb/Framework.h>
#include <carb/logging/Log.h>

#include <isaacsim/core/includes/PhysicsEngine.h>
#include <isaacsim/ros2/core/Ros2Node.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/physics/tensors/IArticulationView.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/TensorApi.h>

#include <OgnROS2PublishJointStateDatabase.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace isaacsim::ros2::core;

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
        const auto& nodeObj = db.abi_node();

        if (!state.isInitialized())
        {
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
            if (!stage)
            {
                db.logError("Could not find USD stage %ld", stageId);
                return false;
            }
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

        const bool hasSensorInputs = useSensorInputs(db);
        const auto& prim = db.inputs.targetPrim();
        const bool hasTargetPrim = !prim.empty();

        if (hasSensorInputs)
        {
            std::vector<std::string> jointNames;
            std::vector<double> positions, velocities, efforts;
            std::vector<uint8_t> dofTypes;
            double stageMetersPerUnit = 0.0;
            if (!validateAndGatherSensorInputs(
                    db, jointNames, positions, velocities, efforts, dofTypes, stageMetersPerUnit))
            {
                return false;
            }
            if (!state.m_publisher && !createPublisherForSensorPath(db, state))
            {
                return false;
            }
            return publishFromSensorInputs(
                db, state, jointNames, positions, velocities, efforts, dofTypes, stageMetersPerUnit);
        }

        if (hasTargetPrim)
        {
            if (!state.m_deprecationWarningLogged)
            {
                CARB_LOG_WARN(
                    "[ROS2 Publish Joint State] Reading from targetPrim is deprecated. Connect an Isaac Read Joint "
                    "State node and use its outputs instead.");
                state.m_deprecationWarningLogged = true;
            }
            if (!state.m_publisher && !createPublisherForLegacyPath(db, state, context))
            {
                return false;
            }
            return state.publishJointStates(db, context);
        }

        db.logError("Specify targetPrim or connect Isaac Read Joint State node outputs (jointNames, positions, etc.).");
        return false;
    }

    static bool useSensorInputs(OgnROS2PublishJointStateDatabase& db)
    {
        const size_t nNames = db.inputs.jointNames().size();
        const size_t nPos = db.inputs.jointPositions().size();
        const size_t nVel = db.inputs.jointVelocities().size();
        const size_t nEff = db.inputs.jointEfforts().size();
        const size_t nTypes = db.inputs.jointDofTypes().size();
        return (nNames > 0 || nPos > 0 || nVel > 0 || nEff > 0 || nTypes > 0);
    }

    static bool validateAndGatherSensorInputs(OgnROS2PublishJointStateDatabase& db,
                                              std::vector<std::string>& jointNames,
                                              std::vector<double>& positions,
                                              std::vector<double>& velocities,
                                              std::vector<double>& efforts,
                                              std::vector<uint8_t>& dofTypes,
                                              double& stageMetersPerUnit)
    {
        const size_t n = db.inputs.jointNames().size();
        if (n == 0)
        {
            db.logError(
                "Joint state from sensor: jointNames is empty. Connect all required outputs from Isaac Read Joint State.");
            return false;
        }
        if (db.inputs.jointPositions().size() != n || db.inputs.jointVelocities().size() != n ||
            db.inputs.jointEfforts().size() != n || db.inputs.jointDofTypes().size() != n)
        {
            db.logError(
                "Joint state from sensor: jointNames, jointPositions, jointVelocities, jointEfforts, and jointDofTypes "
                "must have the same length.");
            return false;
        }
        stageMetersPerUnit = static_cast<double>(db.inputs.stageMetersPerUnit());
        if (stageMetersPerUnit <= 0.0 || !std::isfinite(stageMetersPerUnit))
        {
            db.logError("Joint state from sensor: stageMetersPerUnit must be a positive finite value.");
            return false;
        }
        jointNames.resize(n);
        positions.resize(n);
        velocities.resize(n);
        efforts.resize(n);
        dofTypes.resize(n);
        for (size_t i = 0; i < n; i++)
        {
            jointNames[i] = db.tokenToString(db.inputs.jointNames()[i]);
            positions[i] = db.inputs.jointPositions()[i];
            velocities[i] = db.inputs.jointVelocities()[i];
            efforts[i] = db.inputs.jointEfforts()[i];
            dofTypes[i] = db.inputs.jointDofTypes()[i];
        }
        return true;
    }

    static bool createPublisherForSensorPath(OgnROS2PublishJointStateDatabase& db, OgnROS2PublishJointState& state)
    {
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

    static bool createPublisherForLegacyPath(OgnROS2PublishJointStateDatabase& db,
                                             OgnROS2PublishJointState& state,
                                             const GraphContextObj& context)
    {
        long stageId = context.iContext->getStageId(context);
        auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
        if (!stage)
        {
            db.logError("Could not find USD stage %ld", stageId);
            return false;
        }
        const auto& prim = db.inputs.targetPrim();
        if (prim.empty())
        {
            db.logError("Could not find target prim");
            return false;
        }
        if (!stage->GetPrimAtPath(omni::fabric::toSdfPath(prim[0])))
        {
            db.logError("The prim %s is not valid. Please specify at least one valid chassis prim",
                        omni::fabric::toSdfPath(prim[0]).GetText());
            return false;
        }
        const char* primPath = omni::fabric::toSdfPath(prim[0]).GetText();
        state.m_unitScale = UsdGeomGetStageMetersPerUnit(stage);
        if (!state.m_simView)
        {
            state.m_simView = state.m_tensorInterface->createSimulationView(
                stageId, isaacsim::core::includes::getActivePhysicsEngineName());
            if (!state.m_simView)
            {
                db.logError("Failed to create simulation view - physics backend may not be initialized");
                return false;
            }
        }
        if (state.m_articulation)
        {
            state.m_articulation->release();
        }
        state.m_articulation = state.m_simView->createArticulationView(std::vector<std::string>{ primPath });
        if (!state.m_articulation)
        {
            db.logError("Prim %s is not an articulation", primPath);
            return false;
        }
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

    static bool publishFromSensorInputs(OgnROS2PublishJointStateDatabase& db,
                                        OgnROS2PublishJointState& state,
                                        const std::vector<std::string>& jointNames,
                                        const std::vector<double>& positions,
                                        const std::vector<double>& velocities,
                                        const std::vector<double>& efforts,
                                        const std::vector<uint8_t>& dofTypes,
                                        double stageMetersPerUnit)
    {
        if (!state.m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return false;
        }
        const double timeStamp =
            (db.inputs.sensorTime() > 0.0f) ? static_cast<double>(db.inputs.sensorTime()) : db.inputs.timeStamp();
        state.m_message->writeData(timeStamp, jointNames, positions, velocities, efforts, dofTypes, stageMetersPerUnit);
        state.m_publisher.get()->publish(state.m_message->getPtr());
        return true;
    }

    bool publishJointStates(OgnROS2PublishJointStateDatabase& db, const GraphContextObj& context)
    {
        auto& state = db.perInstanceState<OgnROS2PublishJointState>();
        if (!m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return false;
        }
        const double stageUnits = 1.0 / m_unitScale;
        long stageId = context.iContext->getStageId(context);
        m_stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
        state.m_message->writeData(db.inputs.timeStamp(), m_articulation, m_stage, m_jointPositions, m_jointVelocities,
                                   m_jointEfforts, m_dofTypes, stageUnits);
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
    bool m_deprecationWarningLogged = false;
};

REGISTER_OGN_NODE()

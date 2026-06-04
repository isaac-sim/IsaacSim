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

#include <isaacsim/core/includes/PhysicsEngine.h>
#include <isaacsim/core/includes/PoseTree.h>
#include <isaacsim/ros2/core/Ros2Node.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/TensorApi.h>

#include <OgnROS2PublishTransformTreeDatabase.h>
#include <iomanip>
#include <sstream>

using namespace isaacsim::ros2::core;
using namespace omni::physics::tensors;
class OgnROS2PublishTransformTree : public Ros2Node
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnROS2PublishTransformTreeDatabase::sPerInstanceState<OgnROS2PublishTransformTree>(nodeObj, instanceId);

        state.m_tensorInterface = carb::getCachedInterface<TensorApi>();
        if (!state.m_tensorInterface)
        {
            CARB_LOG_ERROR("Failed to acquire Tensor Api interface\n");
            return;
        }

        state.m_firstIteration = true;
    }

    static bool compute(OgnROS2PublishTransformTreeDatabase& db)
    {
        const GraphContextObj& context = db.abi_context();
        auto& state = db.perInstanceState<OgnROS2PublishTransformTree>();

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
                                     stage->GetPrimAtPath(pxr::SdfPath(nodeObj.iNode->getPrimPath(nodeObj))), true),
                    db.inputs.context()))
            {
                db.logError("Unable to create ROS2 node, please check that namespace is valid");
                return false;
            }
        }

        // Publisher was not valid, create a new one
        if (!state.m_publisher)
        {
            //  Find our stage
            state.m_stageId = context.iContext->getStageId(context);
            state.m_usdStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(state.m_stageId));
            if (!state.m_usdStage)
            {
                db.logError("Could not find USD stage %ld", state.m_stageId);
                return false;
            }

            state.m_stageUnits = UsdGeomGetStageMetersPerUnit(state.m_usdStage);

            // Check if external transform data is provided (preferred path via OgnIsaacComputeTransformTree)
            const auto& externalParentFrames = db.inputs.parentFrames();
            if (!externalParentFrames.empty())
            {
                state.m_useExternalData = true;
            }
            else
            {
                // Deprecated path: compute transforms internally from targetPrims
                CARB_LOG_WARN_ONCE(
                    "OgnROS2PublishTransformTree: using targetPrims for internal computation is deprecated. "
                    "Connect OgnIsaacComputeTransformTree to inputs:parentFrames/childFrames/translations/orientations instead.");
                state.m_useExternalData = false;

                state.m_simView = state.m_tensorInterface->createSimulationView(
                    state.m_stageId, isaacsim::core::includes::getActivePhysicsEngineName());
                if (!state.m_simView)
                {
                    db.logError("Failed to create simulation view - physics backend may not be initialized");
                    return false;
                }

                //  Finding target prims
                const auto& targetPrims = db.inputs.targetPrims();
                if (!targetPrims.empty())
                {
                    state.m_targets.resize(targetPrims.size());

                    for (size_t i = 0; i < targetPrims.size(); i++)
                    {
                        pxr::SdfPath primPath = omni::fabric::toSdfPath(targetPrims[i]);
                        if (primPath.IsEmpty() || !state.m_usdStage->GetPrimAtPath(primPath))
                        {
                            db.logError(
                                "The prim %s is not valid. Please specify at least one valid target prim for the ROS pose tree component",
                                primPath.GetText());
                            return false;
                        }
                        state.m_targets[i] = primPath;
                    }
                }
                else
                {
                    db.logError("Please specify at least one valid target prim for the ROS pose tree component");
                    return false;
                }

                // Finding Parent Prim
                const auto& parentPrim = db.inputs.parentPrim();
                if (!parentPrim.empty())
                {
                    state.m_parentPath = omni::fabric::toSdfPath(parentPrim[0]);
                }
                else
                {
                    state.m_parentPath = pxr::SdfPath();
                }

                // Reset this object
                state.m_poseTree =
                    std::make_unique<isaacsim::core::includes::posetree::PoseTree>(state.m_stageId, state.m_simView);
                state.m_poseTree->setParentPrimPath(state.m_parentPath, "world");
                state.m_poseTree->setTargetPrimPaths(state.m_targets);
            }

            // Setup ROS publisher
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
                return false;
            }

            state.m_message = state.m_factory->createTfTreeMessage();

            Ros2QoSProfile qos;
            const std::string& qosProfile = db.inputs.qosProfile();
            if (db.inputs.staticPublisher())
            {
                qos.depth = 1;
                qos.durability = Ros2QoSDurabilityPolicy::eTransientLocal;
            }
            else if (qosProfile.empty())
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

        return state.publishTF(db, context);
    }

    bool publishTF(OgnROS2PublishTransformTreeDatabase& db, const GraphContextObj& context)
    {
        auto& state = db.perInstanceState<OgnROS2PublishTransformTree>();

        // If we're a static publisher we only publish once on the first iteration.
        // The message will persist as long as the simulation is playing.
        // If we're not a static publisher, we publish every tick only if
        // we have subscribers or m_publishWithoutVerification is true.
        bool isStaticPublisher = db.inputs.staticPublisher();
        if (isStaticPublisher)
        {
            if (!state.m_firstIteration)
            {
                return false;
            }
            state.m_firstIteration = false;
        }
        else
        {
            // Check if subscription count is 0
            if (!m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
            {
                return false;
            }
        }

        const double time = db.inputs.timeStamp();
        std::vector<TfTransformStamped> transforms;

        if (m_useExternalData)
        {
            // External data path: consume parallel arrays from OgnIsaacComputeTransformTree
            const auto& parentFrames = db.inputs.parentFrames();
            const auto& childFrames = db.inputs.childFrames();
            const auto& translations = db.inputs.translations();
            const auto& orientations = db.inputs.orientations();

            if (parentFrames.empty())
            {
                return false;
            }

            const double stageUnits = m_stageUnits;
            const size_t count = parentFrames.size();
            if (childFrames.size() != count || translations.size() != count || orientations.size() != count)
            {
                db.logError(
                    "External transform arrays have mismatched sizes: parentFrames=%zu, childFrames=%zu, "
                    "translations=%zu, orientations=%zu",
                    count, childFrames.size(), translations.size(), orientations.size());
                return false;
            }

            for (size_t i = 0; i < count; i++)
            {
                TfTransformStamped msg;
                msg.timeStamp = time;
                msg.parentFrame = db.tokenToString(parentFrames[i]);
                msg.childFrame = db.tokenToString(childFrames[i]);

                const pxr::GfVec3d& trans = translations[i];
                msg.translationX = trans[0] * stageUnits;
                msg.translationY = trans[1] * stageUnits;
                msg.translationZ = trans[2] * stageUnits;

                // orientations are stored as (x, y, z, w)
                const pxr::GfVec4d& ori = orientations[i];
                msg.rotationX = ori[0];
                msg.rotationY = ori[1];
                msg.rotationZ = ori[2];
                msg.rotationW = ori[3];

                transforms.push_back(msg);
            }
        }
        else
        {
            // Legacy path: internal PoseTree computation from targetPrims
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
            if (!stage)
            {
                db.logError("Could not find USD stage %ld", stageId);
                return false;
            }

            const double stageUnits = m_stageUnits;

            std::function<void(const std::string&, const std::string&, const physx::PxTransform&)> addPoseLambda =
                [stageUnits, &transforms, &time](
                    const std::string& parentFrame, const std::string& childFrame, const physx::PxTransform& t)
            {
                TfTransformStamped currentMsg;
                currentMsg.timeStamp = time;
                currentMsg.childFrame = childFrame;
                currentMsg.parentFrame = parentFrame;

                currentMsg.translationX = t.p.x * static_cast<float>(stageUnits);
                currentMsg.translationY = t.p.y * static_cast<float>(stageUnits);
                currentMsg.translationZ = t.p.z * static_cast<float>(stageUnits);

                currentMsg.rotationX = t.q.x;
                currentMsg.rotationY = t.q.y;
                currentMsg.rotationZ = t.q.z;
                currentMsg.rotationW = t.q.w;

                transforms.push_back(currentMsg);
            };

            m_poseTree->processAllFrames(addPoseLambda);
        }

        state.m_message->writeData(time, transforms);
        state.m_publisher.get()->publish(state.m_message->getPtr());

        return true;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnROS2PublishTransformTreeDatabase::sPerInstanceState<OgnROS2PublishTransformTree>(nodeObj, instanceId);
        state.reset();
    }

    virtual void reset()
    {
        m_publisher.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();
        m_poseTree.reset();
        m_firstIteration = true;
        m_useExternalData = false;
    }

private:
    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2TfTreeMessage> m_message = nullptr;

    bool m_firstIteration = true;
    bool m_useExternalData = false;

    TensorApi* m_tensorInterface = nullptr;
    ISimulationView* m_simView = nullptr;
    double m_stageUnits = 1;
    pxr::SdfPath m_parentPath;
    pxr::SdfPathVector m_targets;

    long m_stageId = 0;
    pxr::UsdStageRefPtr m_usdStage;
    std::unique_ptr<isaacsim::core::includes::posetree::PoseTree> m_poseTree;
};

REGISTER_OGN_NODE()

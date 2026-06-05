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

#include "isaacsim/core/includes/UsdUtilities.h"

#include <isaacsim/ros2/core/Ros2Node.h>
#include <omni/fabric/FabricUSD.h>
#include <physxSchema/physxArticulationAPI.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/meshCollisionAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#include <OgnROS2SubscribeTransformTreeDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2SubscribeTransformTree : public Ros2Node
{

public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnROS2SubscribeTransformTreeDatabase::sPerInstanceState<OgnROS2SubscribeTransformTree>(nodeObj, instanceId);

        state.m_usdStage = nullptr;
        state.m_anonLayer = nullptr;
        state.m_nodeId = nodeObj.nodeHandle;

        state.m_startupState = 0;
    }

    static bool compute(OgnROS2SubscribeTransformTreeDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeTransformTree>();

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
                                     stage->GetPrimAtPath(pxr::SdfPath(nodeObj.iNode->getPrimPath(nodeObj))), true),
                    db.inputs.context()))
            {
                db.logError("Unable to create ROS2 node, please check that namespace is valid");
                return false;
            }
        }

        // Subscriber was not valid, create a new one
        if (!state.m_subscriber)
        {
            //  Find our stage
            state.m_usdStage = omni::usd::UsdContext::getContext()->getStage();
            if (!state.m_usdStage)
            {
                db.logError("Could not find USD stage");
                return false;
            }

            // Create subscriber
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 subscriber, invalid topic name");
                return false;
            }

            // Create message and subscriber
            state.m_message = state.m_factory->createTfTreeMessage();

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

    static void releaseInstance(const NodeObj& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnROS2SubscribeTransformTreeDatabase::sPerInstanceState<OgnROS2SubscribeTransformTree>(nodeObj, instanceId);
        state.reset();
    }

    virtual void reset()
    {
        m_startupState = 0;
        if (m_anonLayer == nullptr)
        {
            return;
        }

        m_subscriber.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();

        // IMPORTANT NOTE
        // It seems that removing the anonymous layer triggers some sort of internal update in
        // OmniGraph that destroys and recreates all the nodes. This causes releaseInstance to
        // be called, followed by deconstruction of this node. When returning from the Remove
        // method this instance of the node will have already been deconstructed, so any changes
        // after the removal can result in memory corruption. A cleaner approach would be
        // queueing up some sort of function that gets called later, which handles removing
        // the layer.
        auto anonLayer = m_anonLayer;
        auto usdStage = m_usdStage;
        m_anonLayer.Reset();
        m_anonLayer = nullptr;
        m_usdStage = nullptr;

        pxr::SdfLayerHandle session = usdStage->GetSessionLayer();
        session->GetSubLayerPaths().Remove(anonLayer->GetIdentifier());
    }

    bool subscriberCallback(OgnROS2SubscribeTransformTreeDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeTransformTree>();

        // An error occurred when we parsed the inputs
        if (m_startupState == -1)
        {
            return false;
        }

        // First tick, we set things up and disable physics
        if (m_startupState == 0)
        {
            // Get the mapping from ROS2 frame to prim, and other stuff
            if (!buildFramePrimsMapAndSet(db))
            {
                m_startupState = -1;
                return false;
            }

            disablePhysicsArticulationAPIs(db);
            m_startupState++;
            return true;
        }

        if (m_startupState == 1)
        {
            disablePhysicsRigidBodiesAndJoints(db);
            m_startupState++;
            return true;
        }

        // Receive all the messages that are available
        bool gotMessage = false;
        while (state.m_subscriber->spin(state.m_message->getPtr()))
        {
            pxr::UsdEditContext editContext(m_usdStage, m_anonLayer);
            gotMessage = true;

            // Get the tfMessages
            std::vector<TfTransformStamped> transforms;
            state.m_message->readData(transforms);

            for (size_t i = 0; i < transforms.size(); i++)
            {
                std::string childFrame = transforms[i].childFrame;
                std::string parentFrame = transforms[i].parentFrame;

                if (m_framePrimsMap.count(childFrame) == 0)
                {
                    continue;
                }
                if (m_framePrimsMap.count(parentFrame) == 0)
                {
                    continue;
                }

                // We are given TFMessages with a transform between the child and parent frame, however in the scene
                // the corresponding prim may have a different parent.  Given a child to parent transform,
                // we need to calculate the child to usd parent transform.  This is done by combining child to parent,
                // parent to world and the inverse of the usd parent to world transforms
                std::string childPrimPath = m_framePrimsMap[childFrame];
                std::string parentPrimPath = m_framePrimsMap[parentFrame];

                pxr::UsdPrim childPrim = m_usdStage->GetPrimAtPath(pxr::SdfPath(childPrimPath));
                pxr::UsdPrim parentPrim = m_usdStage->GetPrimAtPath(pxr::SdfPath(parentPrimPath));
                pxr::UsdPrim usdParentPrim = childPrim.GetParent();

                pxr::GfMatrix4d parentToWorldTransform = isaacsim::core::includes::getWorldTransformMatrix(parentPrim);
                pxr::GfMatrix4d usdParentToWorldTransform =
                    isaacsim::core::includes::getWorldTransformMatrix(usdParentPrim);

                // The transform we're given in the TFMessage
                pxr::GfMatrix4d childTransform;

                childTransform.SetIdentity();
                childTransform.SetTranslateOnly(
                    pxr::GfVec3d(transforms[i].translationX, transforms[i].translationY, transforms[i].translationZ));
                childTransform.SetRotateOnly(pxr::GfQuatd(
                    transforms[i].rotationW,
                    pxr::GfVec3d(transforms[i].rotationX, transforms[i].rotationY, transforms[i].rotationZ)));

                // Now compose the final child to usd parent transform
                pxr::GfMatrix4d newChildTransform;
                newChildTransform = childTransform * parentToWorldTransform * usdParentToWorldTransform.GetInverse();

                // Extract the translation and rotation from our new transform
                pxr::GfVec3d translation, scale;
                pxr::GfQuatd rotation;
                translation = newChildTransform.ExtractTranslation();
                rotation = newChildTransform.ExtractRotationQuat();
                scale.Set(1.0, 1.0, 1.0);

                // Next we take our new translation rotation and scale, and apply it to our prim.
                // Since this may be in a reference, we are unable to clear out all the new xformOps.
                // Below we go through the existing xformOps, either creating or overwriting the existing
                // ones.  Then we set the xformOp order.
                pxr::UsdGeomXform xform(childPrim);
                pxr::UsdGeomXformOp translateXformOp, orientXformOp, scaleXformOp;

                // Go through existing xformOps, extracting the translate, orient and scale ones
                bool resetsXFormStack = false;
                std::vector<pxr::UsdGeomXformOp> xformOps = xform.GetOrderedXformOps(&resetsXFormStack);
                for (const pxr::UsdGeomXformOp& xformOp : xformOps)
                {
                    if (xformOp.GetOpType() == pxr::UsdGeomXformOp::TypeTranslate)
                    {
                        translateXformOp = xformOp;
                    }
                    else if (xformOp.GetOpType() == pxr::UsdGeomXformOp::TypeOrient)
                    {
                        orientXformOp = xformOp;
                    }
                    else if (xformOp.GetOpType() == pxr::UsdGeomXformOp::TypeScale)
                    {
                        scaleXformOp = xformOp;
                    }
                }

                // Add the XformOps if they didn't exist
                if (!translateXformOp)
                {
                    translateXformOp =
                        xform.AddXformOp(pxr::UsdGeomXformOp::TypeTranslate, pxr::UsdGeomXformOp::PrecisionDouble);
                }
                if (!orientXformOp)
                {
                    orientXformOp =
                        xform.AddXformOp(pxr::UsdGeomXformOp::TypeOrient, pxr::UsdGeomXformOp::PrecisionDouble);
                }
                if (!scaleXformOp)
                {
                    scaleXformOp = xform.AddXformOp(pxr::UsdGeomXformOp::TypeScale, pxr::UsdGeomXformOp::PrecisionDouble);
                }

                // Set the XformOps with the proper precision
                if (translateXformOp.GetPrecision() == pxr::UsdGeomXformOp::PrecisionDouble)
                {
                    translateXformOp.Set(translation);
                }
                else
                {
                    translateXformOp.Set(pxr::GfVec3f(translation));
                }
                if (orientXformOp.GetPrecision() == pxr::UsdGeomXformOp::PrecisionDouble)
                {
                    orientXformOp.Set(rotation);
                }
                else
                {
                    orientXformOp.Set(pxr::GfQuatf(rotation));
                }
                if (scaleXformOp.GetPrecision() == pxr::UsdGeomXformOp::PrecisionDouble)
                {
                    scaleXformOp.Set(scale);
                }
                else
                {
                    scaleXformOp.Set(pxr::GfVec3f(scale));
                }

                // Clear the old xformOpOrder, and set the new one
                xform.ClearXformOpOrder();
                xform.SetXformOpOrder({ translateXformOp, orientXformOp, scaleXformOp });
            }
        }

        if (gotMessage)
        {
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
        }
        return gotMessage;
    }

    void disablePhysicsArticulationAPIs(OgnROS2SubscribeTransformTreeDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeTransformTree>();

        // Create anonymous layer, where we disable the physics on the articulation / prims,
        // and where we update the transforms
        std::string layerName = "anon_ros2_subscribe_transform_tree_" + std::to_string(state.m_nodeId);
        state.m_anonLayer = pxr::SdfLayer::CreateAnonymous(layerName);

        pxr::SdfLayerHandle session = state.m_usdStage->GetSessionLayer();
        session->GetSubLayerPaths().push_back(state.m_anonLayer->GetIdentifier());

        pxr::UsdEditContext editContext(state.m_usdStage, state.m_anonLayer);
        {
            pxr::SdfChangeBlock changeBlock;

            // Disable the articulation API on all the provide articulation roots
            for (const std::string& path : state.m_articulationRoots)
            {
                pxr::UsdPrim prim = state.m_usdStage->GetPrimAtPath(pxr::SdfPath(path));
                if (!prim.HasAPI<pxr::PhysxSchemaPhysxArticulationAPI>())
                {
                    db.logWarning("Articulation Root %s doesn't have PhysxSchemaPhysxArticulationAPI", path.c_str());
                    continue;
                }
                pxr::PhysxSchemaPhysxArticulationAPI articulationAPI(prim);
                articulationAPI.GetArticulationEnabledAttr().Set(false);
            }
        }
    }

    void disablePhysicsRigidBodiesAndJoints(OgnROS2SubscribeTransformTreeDatabase& db)
    {
        pxr::UsdEditContext editContext(m_usdStage, m_anonLayer);
        {
            pxr::SdfChangeBlock changeBlock;

            // Disable rigid bodies, and joints that connect pairs of prims in our articulation
            for (const pxr::UsdPrim& prim : m_usdStage->Traverse())
            {
                if (prim.IsA<pxr::UsdPhysicsJoint>())
                {
                    pxr::UsdPhysicsJoint joint(prim);
                    pxr::SdfPathVector targets0, targets1;
                    joint.GetBody0Rel().GetTargets(&targets0);
                    joint.GetBody1Rel().GetTargets(&targets1);
                    if (targets0.empty() || targets1.empty())
                    {
                        continue;
                    }
                    if (m_primPaths.count(targets0.at(0).GetPrimPath().GetString()) > 0 ||
                        m_primPaths.count(targets1.at(0).GetPrimPath().GetString()) > 0)
                    {
                        joint.GetJointEnabledAttr().Set(false);
                    }
                }

                if (m_primPaths.count(prim.GetPath().GetString()) > 0)
                {
                    if (!prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
                    {
                        continue;
                    }
                    pxr::UsdPhysicsRigidBodyAPI rigidBody(prim);
                    rigidBody.GetRigidBodyEnabledAttr().Set(false);
                }
            }
        }
    }

private:
    bool buildFramePrimsMapAndSet(OgnROS2SubscribeTransformTreeDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeTransformTree>();

        state.m_framePrimsMap.clear();
        state.m_primPaths.clear();
        state.m_articulationRoots.clear();

        if (db.inputs.frameNamesMap().size() % 2 != 0)
        {
            db.logError("The frameNamesMap must have an even length in OgnROS2SubscribeTransformTree node");
            return false;
        }

        // Read in the frameNamesMap, checking for duplicates
        for (std::size_t i = 0; i < db.inputs.frameNamesMap().size() / 2; ++i)
        {
            const std::string isaacPrimPath = db.tokenToString(db.inputs.frameNamesMap()[2 * i]);
            const std::string frameName = db.tokenToString(db.inputs.frameNamesMap()[2 * i + 1]);

            if (state.m_primPaths.count(isaacPrimPath) != 0)
            {
                db.logError("Encountered duplicate prim path \"%s\" in OgnROS2SubscribeTransformTree frameNamesMap",
                            isaacPrimPath.c_str());
                return false;
            }
            if (state.m_framePrimsMap.count(frameName) != 0)
            {
                db.logError("Encountered duplicate frame name \"%s\" in OgnROS2SubscribeTransformTree frameNamesMap",
                            frameName.c_str());
                return false;
            }
            if (!state.m_usdStage->GetPrimAtPath(pxr::SdfPath(isaacPrimPath)))
            {
                db.logError("The provided prim path \"%s\" is invalid in OgnROS2SubscribeTransformTree frameNamesMap",
                            isaacPrimPath.c_str());
                return false;
            }

            state.m_framePrimsMap[frameName] = isaacPrimPath;
            state.m_primPaths.insert(isaacPrimPath);
        }

        for (std::size_t i = 0; i < db.inputs.articulationRoots().size(); ++i)
        {
            const std::string articulationPath = db.tokenToString(db.inputs.articulationRoots()[i]);
            pxr::UsdPrim prim = state.m_usdStage->GetPrimAtPath(pxr::SdfPath(articulationPath));
            if (!prim || !prim.HasAPI<pxr::PhysxSchemaPhysxArticulationAPI>())
            {
                db.logError(
                    "Articulation Root \"%s\" doesn't have PhysxSchemaPhysxArticulationAPI in OgnROS2SubscribeTransformTree node",
                    articulationPath.c_str());
                return false;
            }
            state.m_articulationRoots.push_back(articulationPath);
        }
        return true;
    }

    std::shared_ptr<Ros2Subscriber> m_subscriber = nullptr;
    std::shared_ptr<Ros2TfTreeMessage> m_message = nullptr;

    std::map<std::string, std::string> m_framePrimsMap;
    std::set<std::string> m_primPaths;
    std::vector<std::string> m_articulationRoots;

    long m_stageId = 0;
    pxr::UsdStageRefPtr m_usdStage;
    pxr::SdfLayerRefPtr m_anonLayer;

    uint64_t m_nodeId = 0;
    int m_startupState = 0;
};

REGISTER_OGN_NODE()

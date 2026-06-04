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

#include <carb/Defines.h>
#include <carb/eventdispatcher/IEventDispatcher.h>
#include <carb/logging/Logger.h>

#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/core/nodes/ICoreNodes.h>
#include <omni/timeline/TimelineTypes.h>
#include <omni/usd/UsdContextIncludes.h>
//
#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>
#include <omni/usd/UsdContext.h>

#include <OgnOnPhysicsStepDatabase.h>
#include <algorithm>
#include <chrono>

namespace isaacsim
{
namespace core
{
namespace nodes
{


omni::graph::core::INode* g_iNode;
omni::physics::IPhysicsSimulation* g_physicsSimulationInterface;
omni::physics::IPhysics* g_physicsInterface;
struct PhysicsStepData
{
    std::vector<NodeHandle> nodes;
    omni::physics::SubscriptionId stepSubscription = omni::physics::kInvalidSubscriptionId;
};
struct HandleIdPair
{
    GraphHandle graphHandle;
    GraphInstanceID instanceId;
};
namespace
{
std::map<GraphHandle, PhysicsStepData> g_graphsWithStepNode;
omni::physics::SubscriptionId g_simulationRegistrySubscription = omni::physics::kInvalidSubscriptionId;
}
class OgnOnPhysicsStep
{
public:
    OgnOnPhysicsStep() = default;

    static void start(const NodeObj& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnOnPhysicsStepDatabase::sPerInstanceState<OgnOnPhysicsStep>(nodeObj, instanceId);
        auto graphObj = g_iNode->getGraph(nodeObj);
        auto pipelineStage = graphObj.iGraph->getPipelineStage(graphObj);
        if (pipelineStage != kGraphPipelineStage_OnDemand && state.m_initialized)
        {
            // NodeGraph changed from on-demand since last execution - unsubscribe node from step events
            unsubscribe(nodeObj);
        }
        else if (!state.m_initialized)
        {
            initialize(nodeObj, instanceId);
        }

        state.m_startTime = std::chrono::high_resolution_clock::now();
    }
    static void subscribeGraphToStepEvents(PhysicsStepData& data, HandleIdPair* graphHandlePairPtr)
    {
        if (data.stepSubscription != omni::physics::kInvalidSubscriptionId)
        {
            g_physicsSimulationInterface->unsubscribePhysicsOnStepEvents(data.stepSubscription);
        }
        data.stepSubscription = g_physicsSimulationInterface->subscribePhysicsOnStepEvents(
            false, 0,
            [graphHandlePairPtr](float dt, const omni::physics::PhysicsStepContext&)
            { onPhysicsStep(dt, graphHandlePairPtr); });
    }

    static void onSimulationRegistryEvent(omni::physics::SimulationRegistryEventType::Enum eventType,
                                          omni::physics::SimulationId,
                                          const char*,
                                          void*)
    {
        if (eventType != omni::physics::SimulationRegistryEventType::eSIMULATION_REGISTERED)
        {
            return;
        }
        for (auto& entry : g_graphsWithStepNode)
        {
            if (!entry.second.nodes.empty())
            {
                NodeObj node = g_iNode->getNodeFromHandle(entry.second.nodes[0]);
                auto& state = OgnOnPhysicsStepDatabase::sSharedState<OgnOnPhysicsStep>(node);
                subscribeGraphToStepEvents(entry.second, &state.m_graphHandlePair);
            }
        }
    }

    static void initialize(const NodeObj& nodeObj, GraphInstanceID instanceId)
    {
        if (!g_physicsSimulationInterface)
        {
            g_physicsSimulationInterface = carb::getCachedInterface<omni::physics::IPhysicsSimulation>();
        }
        if (!g_physicsInterface)
        {
            g_physicsInterface = carb::getCachedInterface<omni::physics::IPhysics>();
        }
        if (!g_iNode)
        {
            g_iNode = carb::getCachedInterface<omni::graph::core::INode>();
        }

        if (g_simulationRegistrySubscription == omni::physics::kInvalidSubscriptionId && g_physicsInterface)
        {
            g_simulationRegistrySubscription =
                g_physicsInterface->subscribeSimulationRegistryEvents(onSimulationRegistryEvent, nullptr);
        }

        auto graphObj = g_iNode->getGraph(nodeObj);
        auto pipelineStage = graphObj.iGraph->getPipelineStage(graphObj);
        auto& state = OgnOnPhysicsStepDatabase::sPerInstanceState<OgnOnPhysicsStep>(nodeObj, instanceId);
        if (!state.m_timelineEventSub)
        {
            auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
            state.m_timelineEventSub = ed->observeEvent(
                carb::RStringKey("IsaacSimOGNPhysicStepsTimelineEventHandler"), carb::eventdispatcher::kDefaultOrder,
                omni::timeline::kGlobalEventPlay,
                [nodeObj, instanceId](const carb::eventdispatcher::Event&) { start(nodeObj, instanceId); });
        }
        if (pipelineStage != kGraphPipelineStage_OnDemand)
        {
            CARB_LOG_ERROR(
                "Physics OnSimulationStep node detected in a non on-demand Graph. Node will only trigger events if the parent Graph is set to compute on-demand. (%s))",
                g_iNode->getPrimPath(nodeObj));
        }
        else
        {
            if (g_graphsWithStepNode.find(graphObj.graphHandle) == g_graphsWithStepNode.end())
            {
                g_graphsWithStepNode[graphObj.graphHandle] = PhysicsStepData();
                state.m_graphHandlePair = HandleIdPair{ graphObj.graphHandle, instanceId };
                subscribeGraphToStepEvents(g_graphsWithStepNode[graphObj.graphHandle], &state.m_graphHandlePair);
            }
            g_graphsWithStepNode[graphObj.graphHandle].nodes.push_back(nodeObj.nodeHandle);
            state.m_initialized = true;
        }
    }
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        initialize(nodeObj, instanceId);
    }


    static void unsubscribe(const NodeObj& nodeObj)
    {
        const INode* const iNode = nodeObj.iNode;
        if (!iNode)
        {
            return;
        }
        auto graphObj = g_iNode->getGraph(nodeObj);
        auto graphData = g_graphsWithStepNode.find(graphObj.graphHandle);

        if (graphData != g_graphsWithStepNode.end())
        {
            graphData->second.nodes.erase(
                std::remove(graphData->second.nodes.begin(), graphData->second.nodes.end(), nodeObj.nodeHandle),
                graphData->second.nodes.end());
            if (graphData->second.nodes.empty())
            {
                g_physicsSimulationInterface->unsubscribePhysicsOnStepEvents(graphData->second.stepSubscription);
                g_graphsWithStepNode.erase(graphData);
            }
        }
        if (g_graphsWithStepNode.empty() && g_simulationRegistrySubscription != omni::physics::kInvalidSubscriptionId)
        {
            g_physicsInterface->unsubscribeSimulationRegistryEvents(g_simulationRegistrySubscription);
            g_simulationRegistrySubscription = omni::physics::kInvalidSubscriptionId;
        }
        auto& state = OgnOnPhysicsStepDatabase::sSharedState<OgnOnPhysicsStep>(nodeObj);
        state.m_initialized = false;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        unsubscribe(nodeObj);
        auto& state = OgnOnPhysicsStepDatabase::sPerInstanceState<OgnOnPhysicsStep>(nodeObj, instanceId);
        state.m_timelineEventSub.reset();
    }


    static void onPhysicsStep(float timeElapsed, HandleIdPair* idpair)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] OgnOnPhysicsStep::onPhysicsStep");
        auto graphHandle = idpair->graphHandle;
        auto instanceId = idpair->instanceId;
        auto graphData = g_graphsWithStepNode.find(graphHandle);
        // Sanity check if graph exists
        if (graphData != g_graphsWithStepNode.end())
        {
            // Double sanity check if there are step nodes in this graph
            if (!graphData->second.nodes.empty())
            {
                NodeObj node = g_iNode->getNodeFromHandle(graphData->second.nodes[0]);
                auto graphObj = g_iNode->getGraph(node);
                // Iterate over all step nodes enabling them to receive the evaluate input
                for (auto handle : graphData->second.nodes)
                {
                    NodeObj currentNode = g_iNode->getNodeFromHandle(handle);
                    auto& state = OgnOnPhysicsStepDatabase::sPerInstanceState<OgnOnPhysicsStep>(currentNode, instanceId);
                    state.m_dt = timeElapsed;
                    state.m_isSet = true;
                }

                graphObj.iGraph->evaluate(graphObj);
            }
        }
    }

    static bool compute(OgnOnPhysicsStepDatabase& db)
    {
        auto& state = db.perInstanceState<OgnOnPhysicsStep>();
        // Update node with trigger event
        if (state.m_isSet)
        {
            state.m_isSet = false;
            db.outputs.deltaSimulationTime() = state.m_dt;
            db.outputs.step() = kExecutionAttributeStateEnabled;
            auto end = std::chrono::high_resolution_clock::now();
            db.outputs.deltaSystemTime() =
                std::chrono::duration_cast<std::chrono::microseconds>(end - state.m_startTime).count() * 1.0e-6f;
            state.m_startTime = end;
        }
        else
        {
            CARB_LOG_INFO("Graph evaluated outside physics step. A step will not be triggered this time.(%s))",
                          g_iNode->getPrimPath(db.abi_node()));
        }
        return true;
    }


private:
    float m_dt = 0.0f;
    bool m_initialized = false;
    bool m_isSet = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
    HandleIdPair m_graphHandlePair;
    carb::eventdispatcher::ObserverGuard m_timelineEventSub;
};

REGISTER_OGN_NODE()
} // core_nodes
} // isaac
} // omni

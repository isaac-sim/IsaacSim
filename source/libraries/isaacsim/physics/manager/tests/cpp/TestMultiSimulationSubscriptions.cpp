// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <doctest/doctest.h>
#include <isaacsim/physics/manager/PhysicsBenchmark.hpp>
#include <isaacsim/physics/manager/PhysicsSimulation.hpp>
#include <isaacsim/physics/registration/Physics.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>


using namespace isaacsim::physics::registration;
namespace manager = isaacsim::physics::manager;

// ---------------------------------------------------------------------------
// Mock simulator that tracks subscription callbacks
// ---------------------------------------------------------------------------
class MultiSimulationMockSimulator
{
public:
    MultiSimulationMockSimulator()
        : m_nextStepSubscriptionId(0),
          m_nextContactSubscriptionId(0),
          m_stepCallbackCount(0),
          m_contactCallbackCount(0),
          m_publishCount(0)
    {
    }

    SimulationFunctions getSimulationFunctions()
    {
        SimulationFunctions functions;
        functions.initialize = [](void*, const char*) -> bool { return true; };
        functions.close = []() { return true; };
        functions.getAttachedStage = []() -> long { return 0; };
        functions.simulateAsynchronously = [this](float elapsedTime, float)
        {
            PhysicsStepContext context;
            context.scenePath = 0;
            context.simulationId = SimulationId(1);
            for (const auto& pair : m_stepEventCallbacks)
            {
                pair.second(elapsedTime, context);
            }
        };
        functions.fetchResults = [this]()
        {
            const ContactEventHeaderVector eventHeaders;
            const ContactDataVector contactData;
            const FrictionAnchorsDataVector frictionAnchors;
            for (const auto& pair : m_contactEventCallbacks)
            {
                pair.second(eventHeaders, contactData, frictionAnchors);
            }
        };
        functions.simulate = [&functions](float elapsedTime, float currentTime)
        {
            functions.simulateAsynchronously(elapsedTime, currentTime);
            functions.fetchResults();
        };
        functions.checkResults = []() -> bool { return true; };
        functions.publishTransformsToStage = [this]()
        {
            ++m_publishCount;
            return true;
        };
        functions.flushChanges = []() {};
        functions.pauseChangeTracking = [](bool) {};
        functions.isChangeTrackingPaused = []() -> bool { return false; };
        functions.getSimulationTimeStepsPerSecond = [](long, PathToken) -> uint32_t { return 60; };
        functions.getSimulationTimestamp = []() -> uint64_t { return 0; };
        functions.getSimulationStepCount = []() -> uint64_t { return 0; };

        functions.subscribePhysicsContactReportEvents = [this](OnContactReportEventFunction onEvent) -> SubscriptionId
        {
            m_nextContactSubscriptionId = SubscriptionId(m_nextContactSubscriptionId.id + 1);
            SubscriptionId id = m_nextContactSubscriptionId;
            m_contactEventCallbacks[id] = onEvent;
            m_contactCallbackCount++;
            return id;
        };
        functions.unsubscribePhysicsContactReportEvents = [this](SubscriptionId subscriptionId)
        {
            auto callbackIterator = m_contactEventCallbacks.find(subscriptionId);
            if (callbackIterator != m_contactEventCallbacks.end())
            {
                m_contactEventCallbacks.erase(callbackIterator);
                m_contactCallbackCount--;
            }
        };
        functions.subscribePhysicsOnStepEvents = [this](bool, int, OnPhysicsStepEventFunction onUpdate) -> SubscriptionId
        {
            m_nextStepSubscriptionId = SubscriptionId(m_nextStepSubscriptionId.id + 1);
            SubscriptionId id = m_nextStepSubscriptionId;
            m_stepEventCallbacks[id] = onUpdate;
            m_stepCallbackCount++;
            return id;
        };
        functions.unsubscribePhysicsOnStepEvents = [this](SubscriptionId subscriptionId)
        {
            auto callbackIterator = m_stepEventCallbacks.find(subscriptionId);
            if (callbackIterator != m_stepEventCallbacks.end())
            {
                m_stepEventCallbacks.erase(callbackIterator);
                m_stepCallbackCount--;
            }
        };
        return functions;
    }

    int getStepCallbackCount() const
    {
        return m_stepCallbackCount;
    }
    int getContactCallbackCount() const
    {
        return m_contactCallbackCount;
    }
    int getPublishCount() const
    {
        return m_publishCount;
    }

    // Invoke all registered step callbacks
    void fireStepCallbacks(float timeStep)
    {
        PhysicsStepContext context;
        context.scenePath = 0;
        context.simulationId = SimulationId(1);
        for (const auto& pair : m_stepEventCallbacks)
        {
            pair.second(timeStep, context);
        }
    }

    // Invoke all registered contact callbacks
    void fireContactCallbacks()
    {
        const ContactEventHeaderVector eventHeaders;
        const ContactDataVector contactData;
        const FrictionAnchorsDataVector frictionAnchors;
        for (const auto& pair : m_contactEventCallbacks)
        {
            pair.second(eventHeaders, contactData, frictionAnchors);
        }
    }

private:
    SubscriptionId m_nextStepSubscriptionId;
    SubscriptionId m_nextContactSubscriptionId;
    std::unordered_map<SubscriptionId, OnPhysicsStepEventFunction, SubscriptionIdHash> m_stepEventCallbacks;
    std::unordered_map<SubscriptionId, OnContactReportEventFunction, SubscriptionIdHash> m_contactEventCallbacks;
    std::atomic<int> m_stepCallbackCount;
    std::atomic<int> m_contactCallbackCount;
    std::atomic<int> m_publishCount;
};

// ---------------------------------------------------------------------------
// Mock benchmark that tracks subscription callbacks
// ---------------------------------------------------------------------------
class MultiSimulationMockBenchmark
{
public:
    MultiSimulationMockBenchmark() : m_nextSubscriptionId(0), m_profileStatisticsCallbackCount(0)
    {
    }

    BenchmarkFunctions getBenchmarkFunctions()
    {
        BenchmarkFunctions functions;
        functions.subscribeProfileStatisticsEvents = [this](ProfileStatisticsNotificationFunction onEvent) -> SubscriptionId
        {
            m_nextSubscriptionId = SubscriptionId(m_nextSubscriptionId.id + 1);
            SubscriptionId id = m_nextSubscriptionId;
            m_profileStatisticsCallbacks[id] = onEvent;
            m_profileStatisticsCallbackCount++;
            return id;
        };
        functions.unsubscribeProfileStatisticsEvents = [this](SubscriptionId subscriptionId)
        {
            auto callbackIterator = m_profileStatisticsCallbacks.find(subscriptionId);
            if (callbackIterator != m_profileStatisticsCallbacks.end())
            {
                m_profileStatisticsCallbacks.erase(callbackIterator);
                m_profileStatisticsCallbackCount--;
            }
        };
        return functions;
    }

    void fireProfileStatistics(const std::string& prefix)
    {
        std::vector<PhysicsProfileStatistics> statistics = { { prefix + "_Zone1", 10.0f }, { prefix + "_Zone2", 5.0f } };
        for (const auto& pair : m_profileStatisticsCallbacks)
        {
            pair.second(statistics);
        }
    }

    int getProfileStatisticsCallbackCount() const
    {
        return m_profileStatisticsCallbackCount;
    }

private:
    SubscriptionId m_nextSubscriptionId;
    std::unordered_map<SubscriptionId, ProfileStatisticsNotificationFunction, SubscriptionIdHash> m_profileStatisticsCallbacks;
    std::atomic<int> m_profileStatisticsCallbackCount;
};

//=============================================================================
// TEST: Multi-Simulation Subscription Fan-Out
//=============================================================================
TEST_CASE("Multi-Simulation Subscription Tests")
{

    // Two mock simulators + benchmarks
    MultiSimulationMockSimulator mockSimulation1, mockSimulation2;
    MultiSimulationMockBenchmark mockBenchmark1, mockBenchmark2;

    Simulation simulation1;
    simulation1.simulationFunctions = mockSimulation1.getSimulationFunctions();
    simulation1.benchmarkFunctions = mockBenchmark1.getBenchmarkFunctions();

    Simulation simulation2;
    simulation2.simulationFunctions = mockSimulation2.getSimulationFunctions();
    simulation2.benchmarkFunctions = mockBenchmark2.getBenchmarkFunctions();

    SimulationId simulationId1 = registerSimulation(simulation1, "MockSimulator1");
    REQUIRE(simulationId1 != g_kInvalidSimulationId);

    SimulationId simulationId2 = registerSimulation(simulation2, "MockSimulator2");
    REQUIRE(simulationId2 != g_kInvalidSimulationId);

    SUBCASE("OnStep events subscription across multiple simulations")
    {
        std::atomic<int> callbackCount{ 0 };

        OnPhysicsStepEventFunction stepCallback = [&callbackCount](float, const PhysicsStepContext&) { callbackCount++; };

        const SubscriptionId subscriptionId = manager::subscribePhysicsOnStepEvents(true, 0, stepCallback);
        REQUIRE(subscriptionId != g_kInvalidSubscriptionId);
        REQUIRE(mockSimulation1.getStepCallbackCount() == 1);
        REQUIRE(mockSimulation2.getStepCallbackCount() == 1);

        // Fire step events from both simulations
        callbackCount = 0;
        mockSimulation1.fireStepCallbacks(1.0f / 60.0f);
        mockSimulation2.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callbackCount == 2);

        manager::unsubscribePhysicsOnStepEvents(subscriptionId);
        REQUIRE(mockSimulation1.getStepCallbackCount() == 0);
        REQUIRE(mockSimulation2.getStepCallbackCount() == 0);

        // Fire again — no callbacks
        callbackCount = 0;
        mockSimulation1.fireStepCallbacks(1.0f / 60.0f);
        mockSimulation2.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callbackCount == 0);
    }

    SUBCASE("Contact report events subscription across multiple simulations")
    {
        std::atomic<int> callbackCount{ 0 };

        OnContactReportEventFunction contactCallback =
            [&callbackCount](const ContactEventHeaderVector&, const ContactDataVector&, const FrictionAnchorsDataVector&)
        { callbackCount++; };

        const SubscriptionId subscriptionId = manager::subscribePhysicsContactReportEvents(contactCallback);
        REQUIRE(subscriptionId != g_kInvalidSubscriptionId);
        REQUIRE(mockSimulation1.getContactCallbackCount() == 1);
        REQUIRE(mockSimulation2.getContactCallbackCount() == 1);

        callbackCount = 0;
        mockSimulation1.fireContactCallbacks();
        mockSimulation2.fireContactCallbacks();
        REQUIRE(callbackCount == 2);

        manager::unsubscribePhysicsContactReportEvents(subscriptionId);
        REQUIRE(mockSimulation1.getContactCallbackCount() == 0);
        REQUIRE(mockSimulation2.getContactCallbackCount() == 0);

        callbackCount = 0;
        mockSimulation1.fireContactCallbacks();
        mockSimulation2.fireContactCallbacks();
        REQUIRE(callbackCount == 0);
    }

    SUBCASE("Profile statistics events subscription across multiple simulations")
    {
        std::atomic<int> callbackCount{ 0 };
        std::vector<std::string> receivedZones;

        ProfileStatisticsNotificationFunction statisticsCallback =
            [&](const std::vector<PhysicsProfileStatistics>& statistics)
        {
            callbackCount++;
            for (const auto& statistic : statistics)
            {
                receivedZones.push_back(statistic.zoneName);
            }
        };

        const SubscriptionId subscriptionId = manager::subscribeProfileStatisticsEvents(statisticsCallback);
        REQUIRE(subscriptionId != g_kInvalidSubscriptionId);
        REQUIRE(mockBenchmark1.getProfileStatisticsCallbackCount() == 1);
        REQUIRE(mockBenchmark2.getProfileStatisticsCallbackCount() == 1);

        callbackCount = 0;
        receivedZones.clear();
        mockBenchmark1.fireProfileStatistics("Benchmark1");
        mockBenchmark2.fireProfileStatistics("Benchmark2");
        REQUIRE(callbackCount == 2);

        // Verify we received zones from both benchmarks
        bool foundBenchmark1Zone = false;
        bool foundBenchmark2Zone = false;
        for (const auto& zone : receivedZones)
        {
            if (zone.find("Benchmark1") != std::string::npos)
            {
                foundBenchmark1Zone = true;
            }
            if (zone.find("Benchmark2") != std::string::npos)
            {
                foundBenchmark2Zone = true;
            }
        }
        REQUIRE(foundBenchmark1Zone);
        REQUIRE(foundBenchmark2Zone);

        manager::unsubscribeProfileStatisticsEvents(subscriptionId);
        REQUIRE(mockBenchmark1.getProfileStatisticsCallbackCount() == 0);
        REQUIRE(mockBenchmark2.getProfileStatisticsCallbackCount() == 0);

        callbackCount = 0;
        mockBenchmark1.fireProfileStatistics("Benchmark1");
        mockBenchmark2.fireProfileStatistics("Benchmark2");
        REQUIRE(callbackCount == 0);
    }

    SUBCASE("Multiple subscriptions across multiple simulations")
    {
        std::atomic<int> callback1Count{ 0 };
        std::atomic<int> callback2Count{ 0 };

        OnPhysicsStepEventFunction stepCallback1 = [&callback1Count](float, const PhysicsStepContext&)
        { callback1Count++; };
        OnPhysicsStepEventFunction stepCallback2 = [&callback2Count](float, const PhysicsStepContext&)
        { callback2Count++; };

        const SubscriptionId subscriptionId1 = manager::subscribePhysicsOnStepEvents(true, 0, stepCallback1);
        const SubscriptionId subscriptionId2 = manager::subscribePhysicsOnStepEvents(false, 1, stepCallback2);
        REQUIRE(subscriptionId1 != g_kInvalidSubscriptionId);
        REQUIRE(subscriptionId2 != g_kInvalidSubscriptionId);

        REQUIRE(mockSimulation1.getStepCallbackCount() == 2);
        REQUIRE(mockSimulation2.getStepCallbackCount() == 2);

        mockSimulation1.fireStepCallbacks(1.0f / 60.0f);
        mockSimulation2.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callback1Count == 2);
        REQUIRE(callback2Count == 2);

        // Unsubscribe first only
        manager::unsubscribePhysicsOnStepEvents(subscriptionId1);
        REQUIRE(mockSimulation1.getStepCallbackCount() == 1);
        REQUIRE(mockSimulation2.getStepCallbackCount() == 1);

        callback1Count = 0;
        callback2Count = 0;
        mockSimulation1.fireStepCallbacks(1.0f / 60.0f);
        mockSimulation2.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callback1Count == 0);
        REQUIRE(callback2Count == 2);

        manager::unsubscribePhysicsOnStepEvents(subscriptionId2);
        REQUIRE(mockSimulation1.getStepCallbackCount() == 0);
        REQUIRE(mockSimulation2.getStepCallbackCount() == 0);
    }

    SUBCASE("Subscription with one inactive simulation")
    {
        // Deactivate second simulation
        deactivateSimulation(simulationId2);

        std::atomic<int> callbackCount{ 0 };
        OnPhysicsStepEventFunction stepCallback = [&callbackCount](float, const PhysicsStepContext&) { callbackCount++; };

        // Subscribe to all — both should get subscribed (subscription doesn't check active).
        const SubscriptionId subscriptionId = manager::subscribePhysicsOnStepEvents(true, 0, stepCallback);
        REQUIRE(subscriptionId != g_kInvalidSubscriptionId);
        REQUIRE(mockSimulation1.getStepCallbackCount() == 1);
        REQUIRE(mockSimulation2.getStepCallbackCount() == 1);

        // Only fire from active simulation
        callbackCount = 0;
        // Active simulation fires
        mockSimulation1.fireStepCallbacks(1.0f / 60.0f);
        REQUIRE(callbackCount == 1);

        // Inactive simulation2 would not normally be called by umbrella simulate()
        // but the mock still has the subscription registered
        REQUIRE(mockSimulation2.getStepCallbackCount() == 1);

        manager::unsubscribePhysicsOnStepEvents(subscriptionId);
        REQUIRE(mockSimulation1.getStepCallbackCount() == 0);
        REQUIRE(mockSimulation2.getStepCallbackCount() == 0);

        // Reactivate for cleanup
        activateSimulation(simulationId2);
    }

    SUBCASE("Transform publication dispatches only to active simulations")
    {
        REQUIRE(manager::publishTransformsToStage());
        REQUIRE(mockSimulation1.getPublishCount() == 1);
        REQUIRE(mockSimulation2.getPublishCount() == 1);

        deactivateSimulation(simulationId2);
        REQUIRE(manager::publishTransformsToStage());
        REQUIRE(mockSimulation1.getPublishCount() == 2);
        REQUIRE(mockSimulation2.getPublishCount() == 1);
        activateSimulation(simulationId2);
    }

    // Cleanup
    unregisterSimulation(simulationId1);
    unregisterSimulation(simulationId2);
}

TEST_CASE("Transform publication reports unsupported backends")
{
    Simulation simulation;
    const SimulationId simulationId = registerSimulation(simulation, "NoTransformPublisher");
    REQUIRE(simulationId != g_kInvalidSimulationId);
    REQUIRE_FALSE(manager::publishTransformsToStage());
    unregisterSimulation(simulationId);
}

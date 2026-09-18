// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <isaacsim/physics/manager/PhysicsSimulation.hpp>
// clang-format on

#include "SimulationSnapshot.hpp"
#include "SubscriptionStore.hpp"

#include <isaacsim/common/logging/Logging.hpp>
#include <isaacsim/physics/registration/Physics.hpp>

#include <exception>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace manager
{

using namespace registration;

namespace
{
const isaacsim::common::logging::Logger g_kLogger("isaacsim.physics.manager");
details::SubscriptionStore g_contactSubscriptions;
details::SubscriptionStore g_stepSubscriptions;
}

// Initialize physics simulation from a caller-owned ovstage and/or USD identifier
InitializeResult initialize(void* ovstage, const char* usdIdentifier)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();

    // Transactional across backends: initialize() is broadcast to every active backend,
    // so a partial success (e.g. ovphysx attaches the ovstage, then Newton fails) must
    // not report a plain failure with a stage still attached -- the caller pins its
    // keepAlive on Ok, and also on FailedDirty (below) when the rollback could not
    // confirm the detach. Roll the succeeded backends back before returning, so Failed
    // means nothing is attached and FailedDirty means a stage may still be.
    std::vector<const details::SimulationSnapshotRecord*> succeeded;
    // Returns true if the rollback is "dirty" -- a close() did not confirm its detach
    // (returned false or threw), so a stage may still be attached.
    auto rollback = [&succeeded]() -> bool
    {
        bool dirty = false;
        for (auto iterator = succeeded.rbegin(); iterator != succeeded.rend(); ++iterator)
        {
            const auto& functions = (*iterator)->simulation.simulationFunctions;
            if (!functions.close)
            {
                continue;
            }
            // close() is fallible (ovphysx's detach can fail on a pending op) and, for a
            // Python backend like Newton, can throw. Best-effort per backend: one
            // backend's failure must not abort the rollback of the others or escape into
            // the caller, where it would leak past the enum return. A close that cannot
            // confirm the detach marks the rollback dirty and is logged.
            try
            {
                if (!functions.close())
                {
                    ISAACSIM_LOG_ERROR(g_kLogger,
                                       "initialize rollback: close() for simulation '{}' did not detach; a stage may remain "
                                       "attached after a failed initialize.",
                                       (*iterator)->simulationName.c_str());
                    dirty = true;
                }
            }
            catch (const std::exception& error)
            {
                ISAACSIM_LOG_ERROR(g_kLogger, "initialize rollback: close() for simulation '{}' threw: {}",
                                   (*iterator)->simulationName, error.what());
                dirty = true;
            }
            catch (...)
            {
                ISAACSIM_LOG_ERROR(g_kLogger,
                                   "initialize rollback: close() for simulation '{}' threw an unknown exception",
                                   (*iterator)->simulationName);
                dirty = true;
            }
        }
        return dirty;
    };

    // Classify a backend failure. rollback() closes the succeeded backends and reports
    // whether that left anything attached; combine it with whether the FAILED backend
    // itself still holds a stage -- e.g. ovphysx's detach of the previous stage failed, so
    // it returned false without attaching the new one but kept the old one. That is asked
    // via the explicit hasAttachedStage() query, NOT getAttachedStage(): the StageCache id
    // is legally 0 for a caller-owned ovstage attached with an empty USD identifier, so a
    // retained zero-id stage must not be misread as "nothing attached". Either source of a
    // remaining stage means FailedDirty; only a clean rollback with nothing left attached
    // is Failed (the "nothing attached" contract a C++ caller relies on to free its stage).
    //
    // The query is best-effort: a Python backend's callback can throw, so contain it and
    // treat any failure conservatively as dirty -- it must never escape past the enum
    // return (that would skip the owner-pin and contradict the no-propagation contract).
    auto createFailureResult = [&rollback](const details::SimulationSnapshotRecord& failed) -> InitializeResult
    {
        bool dirty = rollback();
        const auto& functions = failed.simulation.simulationFunctions;
        if (functions.hasAttachedStage)
        {
            try
            {
                if (functions.hasAttachedStage())
                {
                    dirty = true;
                }
            }
            catch (...)
            {
                dirty = true;
            }
        }
        return dirty ? InitializeResult::eFailedDirty : InitializeResult::eFailed;
    };

    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFunctions.initialize)
        {
            bool initialized = false;
            try
            {
                initialized = simulation.second.simulation.simulationFunctions.initialize(ovstage, usdIdentifier);
            }
            catch (const std::exception& error)
            {
                // A backend that throws is the same failure as one that returns false, and is
                // handled the same way: roll back and encode the outcome in the enum. Symmetric
                // with the !initialized path, so a dirty rollback surfaces as FailedDirty (the caller pins
                // its owner) rather than an escaping exception that skips the pin. Not propagated.
                ISAACSIM_LOG_ERROR(g_kLogger, "initialize for simulation {} threw: {}",
                                   simulation.second.simulationName, error.what());
                return createFailureResult(simulation.second);
            }
            catch (...)
            {
                ISAACSIM_LOG_ERROR(g_kLogger, "initialize for simulation {} threw an unknown exception",
                                   simulation.second.simulationName);
                return createFailureResult(simulation.second);
            }
            if (!initialized)
            {
                ISAACSIM_LOG_ERROR(g_kLogger, "Failed to initialize for simulation {}", simulation.second.simulationName);
                return createFailureResult(simulation.second);
            }
            succeeded.push_back(&simulation.second);
        }
    }
    return InitializeResult::eOk;
}

// Close the simulation
bool close()
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    bool allClosed = true;
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFunctions.close)
        {
            if (!simulation.second.simulation.simulationFunctions.close())
            {
                allClosed = false;
            }
        }
    }
    return allClosed;
}

// Get currently attached USD stage
long getAttachedStage()
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFunctions.getAttachedStage)
        {
            return simulation.second.simulation.simulationFunctions.getAttachedStage();
        }
    }
    return 0;
}

// Simulate physics asynchronously
void simulateAsynchronously(float elapsedTime, float currentTime)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFunctions.simulateAsynchronously)
        {
            simulation.second.simulation.simulationFunctions.simulateAsynchronously(elapsedTime, currentTime);
        }
    }
}

// Simulate physics synchronously - runs simulation and waits for results
void simulate(float elapsedTime, float currentTime)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFunctions.simulate)
        {
            simulation.second.simulation.simulationFunctions.simulate(elapsedTime, currentTime);
        }
    }
}

// Fetch simulation results
void fetchResults()
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFunctions.fetchResults)
        {
            simulation.second.simulation.simulationFunctions.fetchResults();
        }
    }
}

// Check if simulation finished
bool checkResults()
{
    bool workDone = true;
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFunctions.checkResults)
        {
            if (!simulation.second.simulation.simulationFunctions.checkResults())
            {
                workDone = false;
            }
        }
    }
    return workDone;
}

bool publishTransformsToStage()
{
    bool participated = false;
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    for (const auto& simulation : simulations)
    {
        if (!simulation.second.isActive)
        {
            continue;
        }
        const auto& publish = simulation.second.simulation.simulationFunctions.publishTransformsToStage;
        if (!publish || !publish())
        {
            return false;
        }
        participated = true;
    }
    return participated;
}

// Flush changes
void flushChanges()
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFunctions.flushChanges)
        {
            simulation.second.simulation.simulationFunctions.flushChanges();
        }
    }
}

// Pause change tracking
void pauseChangeTracking(bool pause)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFunctions.pauseChangeTracking)
        {
            simulation.second.simulation.simulationFunctions.pauseChangeTracking(pause);
        }
    }
}

// Check if change tracking is paused
bool isChangeTrackingPaused(SimulationId simulationId)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    details::SimulationSnapshots::const_iterator simulationIterator = simulations.find(simulationId);
    if (simulationIterator != simulations.end() &&
        simulationIterator->second.simulation.simulationFunctions.isChangeTrackingPaused)
    {
        return simulationIterator->second.simulation.simulationFunctions.isChangeTrackingPaused();
    }
    return false;
}

// Subscribe to physics simulation contact report events
SubscriptionId subscribePhysicsContactReportEvents(OnContactReportEventFunction onEvent)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();

    std::vector<details::SimulationSubscription> simulationSubscriptions;

    for (const auto& simulation : simulations)
    {
        if (simulation.second.simulation.simulationFunctions.subscribePhysicsContactReportEvents)
        {
            SubscriptionId simulationSubscriptionId =
                simulation.second.simulation.simulationFunctions.subscribePhysicsContactReportEvents(onEvent);
            if (simulationSubscriptionId != g_kInvalidSubscriptionId)
            {
                simulationSubscriptions.push_back({ simulation.first, simulationSubscriptionId });
            }
        }
    }

    if (simulationSubscriptions.empty())
    {
        return g_kInvalidSubscriptionId;
    }

    return g_contactSubscriptions.add(std::move(simulationSubscriptions));
}

// Unsubscribe from physics simulation contact report events
void unsubscribePhysicsContactReportEvents(SubscriptionId subscriptionId)
{
    if (subscriptionId == g_kInvalidSubscriptionId)
    {
        return;
    }

    std::vector<details::SimulationSubscription> simulationSubscriptions = g_contactSubscriptions.remove(subscriptionId);

    if (simulationSubscriptions.empty())
    {
        return;
    }

    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();

    for (const details::SimulationSubscription& simulationSubscription : simulationSubscriptions)
    {
        auto simulationIterator = simulations.find(simulationSubscription.simulationId);
        if (simulationIterator != simulations.end() &&
            simulationIterator->second.simulation.simulationFunctions.unsubscribePhysicsContactReportEvents)
        {
            simulationIterator->second.simulation.simulationFunctions.unsubscribePhysicsContactReportEvents(
                simulationSubscription.subscriptionId);
        }
    }
}

// Get physics simulation time steps per second
uint32_t getSimulationTimeStepsPerSecond(SimulationId simulationId, long stageId, PathToken scenePath)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    details::SimulationSnapshots::const_iterator simulationIterator = simulations.find(simulationId);
    if (simulationIterator != simulations.end() &&
        simulationIterator->second.simulation.simulationFunctions.getSimulationTimeStepsPerSecond)
    {
        return simulationIterator->second.simulation.simulationFunctions.getSimulationTimeStepsPerSecond(
            stageId, scenePath);
    }
    return 0;
}

// Get physics simulation timestamp
uint64_t getSimulationTimestamp(SimulationId simulationId)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    details::SimulationSnapshots::const_iterator simulationIterator = simulations.find(simulationId);
    if (simulationIterator != simulations.end() &&
        simulationIterator->second.simulation.simulationFunctions.getSimulationTimestamp)
    {
        return simulationIterator->second.simulation.simulationFunctions.getSimulationTimestamp();
    }
    return 0;
}

// Get the number of physics steps performed in the active simulation
uint64_t getSimulationStepCount(SimulationId simulationId)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    details::SimulationSnapshots::const_iterator simulationIterator = simulations.find(simulationId);
    if (simulationIterator != simulations.end() &&
        simulationIterator->second.simulation.simulationFunctions.getSimulationStepCount)
    {
        return simulationIterator->second.simulation.simulationFunctions.getSimulationStepCount();
    }
    return 0;
}

// Subscribe to physics pre/post step events
SubscriptionId subscribePhysicsOnStepEvents(bool preStep, int order, OnPhysicsStepEventFunction onUpdate)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();

    std::vector<details::SimulationSubscription> simulationSubscriptions;

    for (const auto& simulation : simulations)
    {
        if (simulation.second.simulation.simulationFunctions.subscribePhysicsOnStepEvents)
        {
            SubscriptionId simulationSubscriptionId =
                simulation.second.simulation.simulationFunctions.subscribePhysicsOnStepEvents(preStep, order, onUpdate);
            if (simulationSubscriptionId != g_kInvalidSubscriptionId)
            {
                simulationSubscriptions.push_back({ simulation.first, simulationSubscriptionId });
            }
        }
    }

    if (simulationSubscriptions.empty())
    {
        return g_kInvalidSubscriptionId;
    }

    return g_stepSubscriptions.add(std::move(simulationSubscriptions));
}

// Unsubscribe from physics pre/post step events
void unsubscribePhysicsOnStepEvents(SubscriptionId subscriptionId)
{
    if (subscriptionId == g_kInvalidSubscriptionId)
    {
        return;
    }

    std::vector<details::SimulationSubscription> simulationSubscriptions = g_stepSubscriptions.remove(subscriptionId);

    if (simulationSubscriptions.empty())
    {
        return;
    }

    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();

    for (const details::SimulationSubscription& simulationSubscription : simulationSubscriptions)
    {
        auto simulationIterator = simulations.find(simulationSubscription.simulationId);
        if (simulationIterator != simulations.end() &&
            simulationIterator->second.simulation.simulationFunctions.unsubscribePhysicsOnStepEvents)
        {
            simulationIterator->second.simulation.simulationFunctions.unsubscribePhysicsOnStepEvents(
                simulationSubscription.subscriptionId);
        }
    }
}

// Check if simulation is capable of simulating given schema names
bool isCapableOfSimulating(SimulationId simulationId, const char** schemaNames, size_t schemaNamesCount, bool* isCapable)
{
    const details::SimulationSnapshots simulations = details::getSimulationSnapshots();
    details::SimulationSnapshots::const_iterator simulationIterator = simulations.find(simulationId);
    if (simulationIterator != simulations.end() &&
        simulationIterator->second.simulation.simulationFunctions.isCapableOfSimulating)
    {
        return simulationIterator->second.simulation.simulationFunctions.isCapableOfSimulating(
            schemaNames, schemaNamesCount, isCapable);
    }
    return false;
}

} // namespace manager
} // namespace physics
} // namespace isaacsim

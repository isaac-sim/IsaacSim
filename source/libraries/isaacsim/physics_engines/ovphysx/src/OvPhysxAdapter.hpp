// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#pragma once

#include <isaacsim/physics/registration/Physics.hpp>
#include <isaacsim/physics/registration/simulator/Simulation.hpp>

// Only the types header is needed in the interface; the full C API (ovphysx.h)
// is included only in .cpp files to minimize compile-time dependencies.
#include <ovphysx/ovphysx_types.h>

// ovstage instance type for the attached-Stage member; the full ovstage +
// population APIs are included only in the .cpp.
#include <ovstage/ovstage_api/ovstage_api_types.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{

// OvPhysxAdapter owns one ovphysx instance handle, fills a Simulation struct with
// C++ lambdas (no Python), and registers it with Physics.
//
// Lifecycle (mirrors the Python OvPhysxSimulationRegistry + OvPhysxStage):
//   1. create() -- ovphysx_initialize() + ovphysx_create_instance()
//   2. registerWithManager(name) -- fills SimulationFunctions/SceneQueryFunctions/etc.,
//                                    calls Physics::registerSimulation()
//   3. [manager calls] initialize(ovstage, usdIdentifier) -> attach the caller's
//                        borrowed ovstage; simulate(timeStep) -> ovphysx_step_sync()
//   4. unregisterFromManager() + destroy()
class OvPhysxAdapter
{
public:
    // Configuration mirrors Python OvPhysxConfig. All fields optional; zero-init is valid.
    struct Configuration
    {
        int gpuOrdinal{ 0 };
        bool useGpuPipeline{ false }; // DirectGPU / suppressReadback
    };

    static std::shared_ptr<OvPhysxAdapter> create(const Configuration& configuration);
    static std::shared_ptr<OvPhysxAdapter> create()
    {
        return create(Configuration{});
    }
    ~OvPhysxAdapter();

    // Not copyable.
    OvPhysxAdapter(const OvPhysxAdapter&) = delete;
    OvPhysxAdapter& operator=(const OvPhysxAdapter&) = delete;

    // Register this adapter with Physics; returns the assigned SimulationId.
    // Throws on failure (invalid handle returned).
    size_t registerWithManager(const std::string& name = "ovphysx");

    // Unregister and release manager resources. Idempotent.
    void unregisterFromManager();

    // The ovphysx instance handle (used by entity views to create tensor bindings).
    ovphysx_handle_t getHandle() const noexcept
    {
        return m_handle;
    }

    // SimulationId assigned by the manager (0xFFffFFff == not registered).
    size_t getSimulationId() const noexcept
    {
        return m_simulationId;
    }

    // USD StageCache ID set by PhysicsSimulation::initialize(). 0 = not yet initialized.
    long getAttachedStageId() const noexcept
    {
        return m_attachedStageId;
    }

private:
    explicit OvPhysxAdapter(const Configuration& configuration);

    void _buildSimulation(isaacsim::physics::registration::Simulation& simulation);

    // _detachOvstage detaches the currently-attached (caller-owned) Stage; a no-op
    // when none is attached. The Stage is never destroyed here -- the caller owns it.
    // Returns false, keeping the Stage attached, when ovphysx_detach_ovstage fails.
    bool _detachOvstage();

    // SimulationFunctions implementations
    bool _doInitialize(void* callerOvstage, const char* usdIdentifier);
    bool _doClose();
    long _doGetAttachedStage();
    // Ground-truth "is a caller-owned Stage currently attached", independent of the
    // StageCache id (which is legally 0 for an ovstage attached with an empty USD
    // identifier). True whenever ovphysx still borrows a Stage.
    bool _doHasAttachedStage() const
    {
        return m_ovstage != nullptr;
    }
    void _doSimulateAsynchronously(float elapsed, float current);
    void _doSimulate(float elapsed, float current);
    void _doFetchResults();
    bool _doCheckResults();
    bool _doPublishTransformsToStage();
    void _doFlushChanges();
    void _doPauseChangeTracking(bool pause);
    bool _doIsChangeTrackingPaused();
    isaacsim::physics::registration::SubscriptionId _doSubscribeContactReport(
        isaacsim::physics::registration::OnContactReportEventFunction callback);
    void _doUnsubscribeContactReport(isaacsim::physics::registration::SubscriptionId subscriptionId);
    uint32_t _doGetTimeStepsPerSecond(long stageId, isaacsim::physics::registration::PathToken scenePath);
    uint64_t _doGetTimestamp();
    uint64_t _doGetStepCount();
    isaacsim::physics::registration::SubscriptionId _doSubscribeStepEvents(
        bool preStep, int order, isaacsim::physics::registration::OnPhysicsStepEventFunction callback);
    void _doUnsubscribeStepEvents(isaacsim::physics::registration::SubscriptionId subscriptionId);
    bool _doIsCapableOfSimulating(const char** schemaNames, size_t count, bool* isCapable);

    // SceneQueryFunctions helpers
    void _fillSceneQueryFunctions(isaacsim::physics::registration::SceneQueryFunctions& sceneQueryFunctions);

    void _fireStepSubscribers(bool preStep, float elapsed);

    // Read the completed step's contact report from ovphysx and deliver it to the subscribers.
    // The engine returns views into internal buffers that the next step overwrites, so the data
    // is copied into the manager's vectors before any subscriber runs.
    void _fireContactReportSubscribers();

    // RAII refcount for the process-global ovphysx_initialize()/ovphysx_shutdown(). Declared
    // first so it is released last, and released even if the constructor body throws after it.
    struct GlobalLifecycleGuard
    {
        GlobalLifecycleGuard();
        ~GlobalLifecycleGuard();
        GlobalLifecycleGuard(const GlobalLifecycleGuard&) = delete;
        GlobalLifecycleGuard& operator=(const GlobalLifecycleGuard&) = delete;
    };
    GlobalLifecycleGuard m_lifecycle;

    ovphysx_handle_t m_handle{ OVPHYSX_INVALID_HANDLE };
    // The attached ovstage Stage (caller-owned; ovphysx captures the pointer on
    // attach, so it must outlive the attachment). nullptr = nothing attached.
    ovstage_instance_t* m_ovstage{ nullptr };
    long m_attachedStageId{ 0 };
    bool m_initialized{ false };
    bool m_changeTrackingPaused{ false };
    uint64_t m_transformJournalGeneration{ 0 };
    ovstage_ordinal_t m_transformJournalBaseOrdinal{ 0 };
    ovstage_ordinal_t m_lastTransformJournalOrdinal{ 0 };
    size_t m_simulationId{ isaacsim::physics::registration::g_kInvalidSimulationId.id };

    std::atomic<uint64_t> m_timestamp{ 0 };
    std::atomic<uint64_t> m_stepCount{ 0 };

    // Pending asynchronous step operation. Written by _doSimulateAsynchronously and consumed
    // by _doCheckResults / _doFetchResults. The manager's contract is that these never execute
    // concurrently; if that changes, these need std::atomic / a mutex (and note that ovphysx
    // makes waiting on one op index from multiple threads undefined behavior).
    ovphysx_op_index_t m_pendingStepOperation{ 0 };
    // An op index is outstanding and still needs consuming. Cleared by whichever of
    // _doCheckResults (non-blocking poll) or _doFetchResults (blocking wait) consumes it --
    // an op index is single-use, so only one of them may wait on it.
    bool m_hasPendingStep{ false };
    // A dispatched asynchronous step still owes its post-step notification. Tracked
    // separately because a _doCheckResults poll can consume the op index before
    // _doFetchResults runs, and the post-step event must still fire exactly once.
    bool m_postStepPending{ false };
    float m_lastStepElapsed{ 0.0f }; // for post-step subscribers in _doFetchResults

    // Step subscribers: { preStep, order, callback }
    struct StepSubscriber
    {
        bool preStep;
        int order;
        isaacsim::physics::registration::OnPhysicsStepEventFunction callback;
    };
    std::mutex m_subscriberMutex;
    size_t m_nextSubscriptionId{ 1 };
    std::map<size_t, StepSubscriber> m_stepSubscribers;

    // Contact report subscribers
    std::map<size_t, isaacsim::physics::registration::OnContactReportEventFunction> m_contactSubscribers;
};

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim

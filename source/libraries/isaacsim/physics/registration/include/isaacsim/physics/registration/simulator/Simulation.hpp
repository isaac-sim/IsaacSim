// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "../Types.hpp"
#include "ContactEvent.hpp"

#include <cstdint>
#include <functional>

namespace isaacsim
{
namespace physics
{
namespace registration
{

/**
 * @brief Strong identifier for a registered physics simulation.
 */
class SimulationId
{
public:
    /**
     * @brief Constructs an invalid simulation identifier.
     */
    SimulationId() : id(0xFFFFFFFF)
    {
    }

    /**
     * @brief Constructs a simulation identifier from an integer value.
     *
     * @param[in] value The identifier value.
     */
    SimulationId(size_t value) : id(value)
    {
    }

    /**
     * @brief Returns a hash value for this identifier.
     *
     * @return The identifier value.
     */
    size_t computeHash() const
    {
        return id;
    }

    /**
     * @brief Compares two simulation identifiers for equality.
     *
     * @param[in] other The identifier to compare with.
     * @return @c true if both identifiers contain the same value; otherwise, @c false.
     */
    bool operator==(const SimulationId& other) const
    {
        return (id == other.id);
    }

    /**
     * @brief Compares two simulation identifiers for inequality.
     *
     * @param[in] other The identifier to compare with.
     * @return @c true if the identifiers contain different values; otherwise, @c false.
     */
    bool operator!=(const SimulationId& other) const
    {
        return (id != other.id);
    }

    /** @brief The simulation identifier value. */
    size_t id;
};

/**
 * @brief Hash function object for @ref SimulationId.
 */
class SimulationIdHash
{
public:
    /**
     * @brief Computes a hash value for a simulation identifier.
     *
     * @param[in] simulationId The simulation identifier to hash.
     * @return The simulation identifier's hash value.
     */
    size_t operator()(const SimulationId& simulationId) const
    {
        return simulationId.computeHash();
    }
};

/**
 * @brief Sentinel value representing an invalid simulation identifier.
 */
const SimulationId g_kInvalidSimulationId = SimulationId(0xFFFFFFFF);

/**
 * @brief Strong identifier for an event subscription.
 */
class SubscriptionId
{
public:
    /**
     * @brief Constructs an invalid subscription identifier.
     */
    SubscriptionId() : id(0xFFFFFFFFFF)
    {
    }

    /**
     * @brief Constructs a subscription identifier from an integer value.
     *
     * @param[in] value The identifier value.
     */
    SubscriptionId(size_t value) : id(value)
    {
    }

    /**
     * @brief Returns a hash value for this identifier.
     *
     * @return The identifier value.
     */
    size_t computeHash() const
    {
        return id;
    }

    /**
     * @brief Compares two subscription identifiers for equality.
     *
     * @param[in] other The identifier to compare with.
     * @return @c true if both identifiers contain the same value; otherwise, @c false.
     */
    bool operator==(const SubscriptionId& other) const
    {
        return (id == other.id);
    }

    /**
     * @brief Compares two subscription identifiers for inequality.
     *
     * @param[in] other The identifier to compare with.
     * @return @c true if the identifiers contain different values; otherwise, @c false.
     */
    bool operator!=(const SubscriptionId& other) const
    {
        return (id != other.id);
    }

    /** @brief The subscription identifier value. */
    size_t id;
};

/**
 * @brief Hash function object for @ref SubscriptionId.
 */
class SubscriptionIdHash
{
public:
    /**
     * @brief Computes a hash value for a subscription identifier.
     *
     * @param[in] subscriptionId The subscription identifier to hash.
     * @return The subscription identifier's hash value.
     */
    size_t operator()(const SubscriptionId& subscriptionId) const
    {
        return subscriptionId.computeHash();
    }
};

/**
 * @brief Sentinel value representing an invalid subscription identifier.
 */
const SubscriptionId g_kInvalidSubscriptionId = SubscriptionId(0xFFFFFFFFFF);

/**
 * @brief Identifies the scene and simulation associated with a physics step event.
 */
struct PhysicsStepContext
{
    /** @brief The encoded USD path of the physics scene. */
    PathToken scenePath;

    /** @brief The identifier of the simulation that stepped. */
    SimulationId simulationId;
};

/**
 * @brief Callback invoked for a physics step event.
 *
 * @param[in] elapsedTime The simulated time since the previous physics step, in seconds.
 * @param[in] context The scene and simulation associated with the step.
 *
 * @note The callback borrows @p context. The reference remains valid only for the duration of the callback invocation
 *       and must not be retained.
 */
using OnPhysicsStepEventFunction = std::function<void(float elapsedTime, const PhysicsStepContext& context)>;

/**
 * @brief Specifies how a force-like value affects a body.
 */
enum class ForceModeType : uint32_t
{
    /** @brief Applies a force with units of mass times distance per second squared. */
    eForce,

    /** @brief Applies a linear impulse with units of mass times distance per second. */
    eImpulse,

    /** @brief Applies a mass-independent velocity change with units of distance per second. */
    eVelocityChange,

    /** @brief Applies a mass-independent acceleration with units of distance per second squared. */
    eAcceleration
};

/**
 * @brief Initializes the simulation backend from caller-supplied stage inputs.
 *
 * @param[in] ovstage An opaque, caller-owned @c ovstage_instance_t pointer, or @c nullptr when no ovstage instance is
 *                    supplied. The backend does not take ownership. After a successful attachment, the caller must
 *                    keep the pointed-to stage alive until @c DetachStageFunction returns @c true.
 * @param[in] usdIdentifier A decimal USD StageCache identifier, such as @c "123", or @c nullptr or an empty string
 *                          when no StageCache identifier is supplied. This value is an identifier, not a file path.
 * @return @c true if backend initialization succeeded; otherwise, @c false.
 *
 * @note A null @p ovstage together with a null or empty @p usdIdentifier represents an explicit empty scene.
 *       Initialization may succeed without attaching a borrowed stage pointer.
 * @note After a failed call, use @c HasAttachedStageFunction to determine whether the backend still retains a
 * previously attached stage. Its caller-owned stage must remain alive until @c DetachStageFunction succeeds.
 */
using AttachStageFunction = std::function<bool(void* ovstage, const char* usdIdentifier)>;

/**
 * @brief Detaches the currently attached stage.
 *
 * @return @c true if the stage was fully detached; @c false if detachment failed and the stage remains attached.
 */
using DetachStageFunction = std::function<bool()>;

/**
 * @brief Returns the StageCache identifier of the attached stage.
 *
 * @return The StageCache identifier, or zero when the attached stage has no StageCache identifier or no stage is
 *         attached.
 */
using GetAttachedStageFunction = std::function<long()>;

/**
 * @brief Reports whether the backend currently has an attached stage.
 *
 * Unlike @c GetAttachedStageFunction, this query distinguishes an attached stage without a StageCache identifier from
 * no attached stage.
 *
 * @return @c true if a stage is attached; otherwise, @c false.
 */
using HasAttachedStageFunction = std::function<bool()>;

/**
 * @brief Starts an asynchronous physics simulation step.
 *
 * The backend simulates exactly @p elapsedTime without substepping. Callers should use a fixed time step no greater
 * than 1/60 second when possible.
 *
 * @param[in] elapsedTime The amount of time to simulate, in seconds.
 * @param[in] currentTime The current simulation time, in seconds. Backends may use this value when evaluating
 *                        time-sampled transforms.
 */
using SimulateAsynchronouslyFunction = std::function<void(float elapsedTime, float currentTime)>;

/**
 * @brief Executes a synchronous physics simulation step.
 *
 * The backend simulates exactly @p elapsedTime without substepping. Callers should use a fixed time step no greater
 * than 1/60 second when possible. The backend completes the step and makes its results available before returning.
 * Callers must not automatically invoke @c FetchResultsFunction afterward; doing so may repeat post-step or
 * contact-event delivery. Use @c FetchResultsFunction to complete a step started through @c
 * SimulateAsynchronouslyFunction.
 *
 * @param[in] elapsedTime The amount of time to simulate, in seconds.
 * @param[in] currentTime The current simulation time, in seconds. Backends may use this value when evaluating
 *                        time-sampled transforms.
 */
using SimulateFunction = std::function<void(float elapsedTime, float currentTime)>;

/**
 * @brief Waits for simulation completion and fetches the resulting state.
 *
 * @note This function blocks until simulation finishes.
 * @note Use this function to complete a step started through @c SimulateAsynchronouslyFunction. Do not automatically
 * invoke it after
 *       @c SimulateFunction because the synchronous operation already completes its result processing.
 */
using FetchResultsFunction = std::function<void()>;

/**
 * @brief Queries the backend-reported readiness of the current simulation step.
 *
 * @return @c true if the backend reports that its current step is ready; otherwise, @c false.
 *
 * @note This is backend-reported status. A backend may always return @c true when it does not independently expose
 *       pending-work state.
 */
using CheckResultsFunction = std::function<bool()>;

/**
 * @brief Publishes the latest simulated transforms to the attached stage.
 *
 * @return @c true if the backend published its current transforms; otherwise, @c false.
 *
 * @note This operation does not advance simulation. Backends that do not support stage publication leave the
 *       corresponding function-table entry empty.
 */
using PublishTransformsToStageFunction = std::function<bool()>;

/**
 * @brief Processes all buffered physics changes.
 *
 * Use this operation when later changes depend on earlier buffered changes having already been applied.
 */
using FlushChangesFunction = std::function<void()>;

/**
 * @brief Pauses or resumes physics change tracking.
 *
 * @param[in] pause @c true to pause change tracking; @c false to resume it.
 */
using PauseChangeTrackingFunction = std::function<void(bool pause)>;

/**
 * @brief Reports whether physics change tracking is paused.
 *
 * @return @c true if change tracking is paused; otherwise, @c false.
 */
using IsChangeTrackingPausedFunction = std::function<bool()>;

/**
 * @brief Subscribes to physics contact-report events.
 *
 * @param[in] onEvent The callback to invoke for contact reports.
 * @return A subscription identifier for use with @c UnsubscribePhysicsContactReportEventsFunction, or
 *         @c g_kInvalidSubscriptionId if the subscription fails.
 *
 * @note @p onEvent borrows its contact-event, contact-data, and friction-anchor vectors. They remain valid only for
 *       the duration of the callback invocation and must not be retained by reference.
 * @note Backend event dispatch must contain exceptions raised by @p onEvent so that one subscriber does not suppress
 *       later subscribers.
 */
using SubscribePhysicsContactReportEventsFunction = std::function<SubscriptionId(OnContactReportEventFunction onEvent)>;

/**
 * @brief Unsubscribes from physics contact-report events.
 *
 * @param[in] subscriptionId The identifier returned by @c SubscribePhysicsContactReportEventsFunction.
 */
using UnsubscribePhysicsContactReportEventsFunction = std::function<void(SubscriptionId subscriptionId)>;

/**
 * @brief Returns the simulation frequency for a physics scene.
 *
 * @param[in] stageId The identifier of the USD stage containing the scene.
 * @param[in] scenePath The encoded USD path of the physics scene. An empty path selects the first available scene.
 * @return The current simulation frequency, in steps per second.
 */
using GetSimulationTimeStepsPerSecondFunction = std::function<uint32_t(long stageId, PathToken scenePath)>;

/**
 * @brief Returns the current simulation timestamp.
 *
 * The timestamp increases with every simulation step.
 *
 * @return The current simulation timestamp.
 */
using GetSimulationTimestampFunction = std::function<uint64_t()>;

/**
 * @brief Returns the number of physics steps performed by the active simulation.
 *
 * The step count resets to zero when a new simulation starts.
 *
 * @return The number of steps since the active simulation started, or zero when no simulation is active.
 */
using GetSimulationStepCountFunction = std::function<uint64_t()>;

/**
 * @brief Subscribes to physics pre-step or post-step events.
 *
 * @param[in] preStep @c true to invoke the callback immediately before each physics step; @c false to invoke it
 *                    immediately after each physics step.
 * @param[in] order The callback order. Lower values run before higher values.
 * @param[in] onUpdate The callback to invoke for each selected step event.
 * @return A subscription identifier for use with @c UnsubscribePhysicsOnStepEventsFunction, or
 *         @c g_kInvalidSubscriptionId if the subscription fails.
 *
 * @note Step-event subscriptions must not be changed from within @p onUpdate.
 * @note Backend event dispatch must contain exceptions raised by @p onUpdate so that one subscriber does not suppress
 *       later subscribers.
 */
using SubscribePhysicsOnStepEventsFunction =
    std::function<SubscriptionId(bool preStep, int order, OnPhysicsStepEventFunction onUpdate)>;

/**
 * @brief Unsubscribes from physics pre-step or post-step events.
 *
 * @param[in] subscriptionId The identifier returned by @c SubscribePhysicsOnStepEventsFunction.
 *
 * @note Step-event subscriptions must not be changed from within a step-event callback.
 */
using UnsubscribePhysicsOnStepEventsFunction = std::function<void(SubscriptionId subscriptionId)>;

/**
 * @brief Queries whether the simulation supports a list of USD schemas.
 *
 * Each input name may identify either an applied schema API or a typed schema. The output at index @c i corresponds to
 * the input at index @c i.
 *
 * @param[in] schemaNames An array of @p schemaNamesCount null-terminated schema names. Must be non-null and reference
 *                        at least @p schemaNamesCount entries when @p schemaNamesCount is nonzero.
 * @param[in] schemaNamesCount The number of entries in @p schemaNames and @p isCapable.
 * @param[out] isCapable An array with capacity for @p schemaNamesCount results. Must be non-null when
 *                       @p schemaNamesCount is nonzero. Its contents are defined only when the function returns
 *                       @c true.
 * @return @c true if the capability query completed successfully and populated @p isCapable; otherwise, @c false.
 */
using IsCapableOfSimulatingFunction =
    std::function<bool(const char** schemaNames, size_t schemaNamesCount, bool* isCapable)>;

/**
 * @brief Function table for simulation lifecycle, stepping, and event operations.
 */
struct SimulationFunctions
{
    /** @brief Initializes the backend from stage inputs. */
    AttachStageFunction initialize{};

    /** @brief Detaches the current USD stage. */
    DetachStageFunction close{};

    /** @brief Returns the attached stage's StageCache identifier. */
    GetAttachedStageFunction getAttachedStage{};

    /** @brief Reports whether a stage is attached. */
    HasAttachedStageFunction hasAttachedStage{};

    /** @brief Starts an asynchronous simulation step. */
    SimulateAsynchronouslyFunction simulateAsynchronously{};

    /** @brief Executes a synchronous simulation step. */
    SimulateFunction simulate{};

    /** @brief Waits for and fetches simulation results. */
    FetchResultsFunction fetchResults{};

    /** @brief Queries backend-reported simulation readiness. */
    CheckResultsFunction checkResults{};

    /** @brief Publishes the latest simulated transforms to the attached stage. */
    PublishTransformsToStageFunction publishTransformsToStage{};

    /** @brief Processes buffered physics changes. */
    FlushChangesFunction flushChanges{};

    /** @brief Pauses or resumes physics change tracking. */
    PauseChangeTrackingFunction pauseChangeTracking{};

    /** @brief Reports whether change tracking is paused. */
    IsChangeTrackingPausedFunction isChangeTrackingPaused{};

    /** @brief Subscribes to contact reports. */
    SubscribePhysicsContactReportEventsFunction subscribePhysicsContactReportEvents{};

    /** @brief Unsubscribes from contact reports. */
    UnsubscribePhysicsContactReportEventsFunction unsubscribePhysicsContactReportEvents{};

    /** @brief Returns a scene's simulation frequency. */
    GetSimulationTimeStepsPerSecondFunction getSimulationTimeStepsPerSecond{};

    /** @brief Returns the current simulation timestamp. */
    GetSimulationTimestampFunction getSimulationTimestamp{};

    /** @brief Returns the active simulation's step count. */
    GetSimulationStepCountFunction getSimulationStepCount{};

    /** @brief Subscribes to pre-step or post-step events. */
    SubscribePhysicsOnStepEventsFunction subscribePhysicsOnStepEvents{};

    /** @brief Unsubscribes from step events. */
    UnsubscribePhysicsOnStepEventsFunction unsubscribePhysicsOnStepEvents{};

    /** @brief Queries support for USD schemas. */
    IsCapableOfSimulatingFunction isCapableOfSimulating{};
};

} // namespace registration
} // namespace physics
} // namespace isaacsim

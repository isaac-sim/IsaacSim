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

#include <isaacsim/physics/manager/Export.h>
#include <isaacsim/physics/registration/Types.hpp>
#include <isaacsim/physics/registration/simulator/ContactEvent.hpp>
#include <isaacsim/physics/registration/simulator/Simulation.hpp>
#include <isaacsim/physics/registration/simulator/Simulator.hpp>

namespace isaacsim
{
namespace physics
{
namespace manager
{
/**
 * @brief Result of initializing all active physics backends.
 */
enum class InitializeResult : int
{
    /** @brief Every participating backend initialization callback succeeded. */
    eOk = 0,
    /** @brief Initialization failed without a manager-observed cleanup problem. */
    eFailed = 1,
    /** @brief Initialization failed with a manager-observed or indeterminate cleanup problem. */
    eFailedDirty = 2,
};

/**
 * @brief Initializes all active physics backends from a stage.
 *
 * Each backend consumes the stage representation it supports. The caller retains ownership of `ovstage`, and this
 * function does not extend its lifetime. A null `ovstage` together with a null or empty `usdIdentifier` is valid input
 * that requests initialization of an empty scene. The manager does not detach an existing stage before dispatching
 * initialization; each backend controls replacement of any stage it already holds.
 *
 * The caller must keep a non-null `ovstage` alive until every backend that accepted it has detached. A successful call
 * to `close()` normally establishes this when all participating backends expose correct cleanup callbacks. A failure
 * result alone does not prove detachment when a backend omits cleanup or attachment-status callbacks.
 *
 * If a backend fails, successfully initialized backends that expose a close callback are closed in reverse order. The
 * function returns `InitializeResult::eFailedDirty` when a close callback fails or throws, or when the failed backend's
 * attachment-status callback reports an attached stage or throws. Otherwise, it returns `InitializeResult::eFailed`.
 * These results describe manager-observed cleanup; they cannot guarantee absolute attachment state when callbacks are
 * absent or report inaccurate state.
 *
 * @param[in] ovstage Opaque pointer to a caller-owned `ovstage_instance_t`, or `nullptr` when unavailable.
 * @param[in] usdIdentifier Numeric USD stage-cache identifier, such as `"123"`, or `nullptr` or an empty string when
 *                          unavailable. The value is interpreted as an identifier, not as a file path.
 * @return `InitializeResult::eOk` when every participating initialization callback succeeds,
 *         `InitializeResult::eFailed` when initialization fails without an observed cleanup problem, or
 *         `InitializeResult::eFailedDirty` when cleanup fails or its status cannot be determined.
 *
 * @note The OV PhysX backend reads `ovstage` at ordinal 1. Populate that ordinal when using this backend.
 * @note Exceptions raised by backend initialization or rollback callbacks are converted to a failure result.
 */
ISAACSIM_PHYSICS_MANAGER_API InitializeResult initialize(void* ovstage, const char* usdIdentifier);

/**
 * @brief Requests every active physics backend to close and detach its stage.
 *
 * @return `true` if every participating backend reported that it closed and detached its stage; otherwise, `false`.
 *
 * @note Call this function again if a backend reports a transient detach failure.
 * @note Active backends without a close callback are skipped and do not affect the return value. Therefore, a `true`
 *       result does not prove detachment for a backend that omits its close callback.
 * @note Exceptions raised by a backend close callback propagate to the caller and prevent later backends from being
 *       closed during that call.
 */
ISAACSIM_PHYSICS_MANAGER_API bool close();

/**
 * @brief Gets the USD stage identifier reported by the first active backend that exposes the query.
 *
 * @return The value reported by the first active backend that exposes the query, or `0` if none does. A value of `0`
 *         may also represent an attached caller-owned stage without a StageCache identifier.
 *
 * @note Exceptions raised by the selected backend query callback propagate to the caller.
 */
ISAACSIM_PHYSICS_MANAGER_API long getAttachedStage();

/**
 * @brief Starts an asynchronous simulation step on every active physics backend.
 *
 * The exact requested interval is simulated without automatic substepping. Prefer fixed time steps no larger than
 * 1/60 second.
 *
 * @param[in] elapsedTime Duration of the simulation step, in seconds.
 * @param[in] currentTime Current simulation time, in seconds. A backend may use this value when applying time-sampled
 *                        transformations.
 *
 * @note Exceptions raised by a backend simulation callback propagate to the caller and prevent dispatch to later
 *       backends.
 */
ISAACSIM_PHYSICS_MANAGER_API void simulateAsynchronously(float elapsedTime, float currentTime);

/**
 * @brief Runs a synchronous simulation step on every active physics backend.
 *
 * The exact requested interval is simulated without automatic substepping. Each backend's synchronous callback
 * completes before dispatch proceeds to the next backend. Prefer fixed time steps no larger than 1/60 second.
 *
 * @param[in] elapsedTime Duration of the simulation step, in seconds.
 * @param[in] currentTime Current simulation time, in seconds. A backend may use this value when applying time-sampled
 *                        transformations.
 *
 * @note Exceptions raised by a backend simulation callback propagate to the caller and prevent dispatch to later
 *       backends.
 */
ISAACSIM_PHYSICS_MANAGER_API void simulate(float elapsedTime, float currentTime);

/**
 * @brief Waits for active simulations to finish and fetches their results.
 *
 * @note This function blocks until every participating backend finishes.
 * @note Exceptions raised by a backend result callback propagate to the caller and prevent dispatch to later backends.
 */
ISAACSIM_PHYSICS_MANAGER_API void fetchResults();

/**
 * @brief Collects result-readiness status from active physics backends.
 *
 * @return `true` if every participating result-status callback reports ready; `false` if any reports not ready.
 *
 * @note This function does not independently verify that backend work has completed.
 * @note Active backends that do not expose a result-status callback are not considered.
 * @note Exceptions raised by a backend result-status callback propagate to the caller and prevent checks of later
 *       backends.
 */
ISAACSIM_PHYSICS_MANAGER_API bool checkResults();

/**
 * @brief Publishes the latest simulated transforms to the stages attached to active physics backends.
 *
 * @return `true` if at least one active backend participated and every active backend published successfully;
 *         `false` if no backend participated, a backend does not support publication, or a backend reported failure.
 *
 * @note This operation does not advance simulation.
 * @note If several backends are active, an earlier backend may have published before a later backend reports failure.
 * @note Exceptions raised by a backend publication callback propagate to the caller and prevent dispatch to later
 *       backends.
 */
ISAACSIM_PHYSICS_MANAGER_API bool publishTransformsToStage();

/**
 * @brief Forces every active physics backend to process buffered scene changes.
 *
 * Use this function when a later edit depends on an earlier buffered edit. For example, flush after adding a prim and
 * before redirecting an existing relationship to that prim.
 *
 * @note Exceptions raised by a backend flush callback propagate to the caller and prevent dispatch to later backends.
 */
ISAACSIM_PHYSICS_MANAGER_API void flushChanges();

/**
 * @brief Pauses or resumes scene-change tracking for every active physics backend.
 *
 * @param[in] pause `true` to pause change tracking; `false` to resume it.
 *
 * @note Exceptions raised by a backend change-tracking callback propagate to the caller and prevent dispatch to later
 *       backends.
 */
ISAACSIM_PHYSICS_MANAGER_API void pauseChangeTracking(bool pause);

/**
 * @brief Checks whether scene-change tracking is paused for a simulation.
 *
 * @param[in] simulationId Identifier of the simulation to query.
 * @return `true` if tracking is paused; `false` if tracking is active, the simulation is unknown, or the backend does
 *         not expose the query.
 *
 * @note Exceptions raised by the selected backend query callback propagate to the caller.
 * @note The simulation may be queried while inactive, but it must remain registered.
 */
ISAACSIM_PHYSICS_MANAGER_API bool isChangeTrackingPaused(registration::SimulationId simulationId);

/**
 * @brief Subscribes to contact-report events from physics backends.
 *
 * @param[in] onEvent Callback invoked when a contact report is available.
 * @return A subscription identifier for use with `unsubscribePhysicsContactReportEvents()`, or
 *         `registration::g_kInvalidSubscriptionId` if no backend accepted the subscription.
 *
 * @note `onEvent` borrows the contact-event, contact-data, and friction-anchor vectors. They remain valid only for the
 *       duration of the callback invocation and must not be retained by reference.
 * @note Exceptions raised by a backend subscription callback propagate to the caller. If an exception occurs after
 *       another backend accepted the subscription, that earlier backend subscription may remain active without an
 *       aggregate subscription identifier being returned.
 */
ISAACSIM_PHYSICS_MANAGER_API registration::SubscriptionId subscribePhysicsContactReportEvents(
    registration::OnContactReportEventFunction onEvent);

/**
 * @brief Unsubscribes from physics contact-report events.
 *
 * An invalid or unknown subscription identifier is ignored.
 *
 * @param[in] subscriptionId Subscription identifier returned by `subscribePhysicsContactReportEvents()`.
 *
 * @note Exceptions raised by a backend unsubscription callback propagate to the caller and prevent later backends from
 *       being unsubscribed during that call.
 */
ISAACSIM_PHYSICS_MANAGER_API void unsubscribePhysicsContactReportEvents(registration::SubscriptionId subscriptionId);

/**
 * @brief Gets the configured simulation frequency for a physics scene.
 *
 * @param[in] simulationId Identifier of the simulation to query.
 * @param[in] stageId USD stage-cache identifier containing the scene.
 * @param[in] scenePath Encoded USD path of the physics scene, or `0` to query the first available scene.
 * @return The configured number of simulation steps per second, or `0` if the simulation is unknown or does not
 *         expose the query.
 *
 * @note Exceptions raised by the selected backend query callback propagate to the caller.
 * @note The simulation may be queried while inactive, but it must remain registered.
 */
ISAACSIM_PHYSICS_MANAGER_API uint32_t getSimulationTimeStepsPerSecond(registration::SimulationId simulationId,
                                                                      long stageId,
                                                                      registration::PathToken scenePath);

/**
 * @brief Gets a simulation's monotonically increasing step timestamp.
 *
 * @param[in] simulationId Identifier of the simulation to query.
 * @return The current simulation timestamp, or `0` if the simulation is unknown or does not expose the query.
 *
 * @note Exceptions raised by the selected backend query callback propagate to the caller.
 * @note The simulation may be queried while inactive, but it must remain registered.
 */
ISAACSIM_PHYSICS_MANAGER_API uint64_t getSimulationTimestamp(registration::SimulationId simulationId);

/**
 * @brief Gets the physics step count reported by a simulation.
 *
 * The count resets when a new simulation starts.
 *
 * @param[in] simulationId Identifier of the simulation to query.
 * @return The backend's current step count, or `0` if the simulation is unknown or does not expose the query. A backend
 *         may increment this count when a step starts, before asynchronous work completes.
 *
 * @note Exceptions raised by the selected backend query callback propagate to the caller.
 * @note The simulation may be queried while inactive, but it must remain registered.
 */
ISAACSIM_PHYSICS_MANAGER_API uint64_t getSimulationStepCount(registration::SimulationId simulationId);

/**
 * @brief Subscribes to events before or after each physics step.
 *
 * @param[in] preStep `true` to invoke the callback before each step; `false` to invoke it after each step.
 * @param[in] order Callback priority. Lower values run before higher values.
 * @param[in] onUpdate Callback invoked for each matching step event.
 * @return A subscription identifier for use with `unsubscribePhysicsOnStepEvents()`, or
 *         `registration::g_kInvalidSubscriptionId` if no backend accepted the subscription.
 *
 * @note Do not modify step-event subscriptions from within `onUpdate`.
 * @note Exceptions raised by a backend subscription callback propagate to the caller. If an exception occurs after
 *       another backend accepted the subscription, that earlier backend subscription may remain active without an
 *       aggregate subscription identifier being returned.
 */
ISAACSIM_PHYSICS_MANAGER_API registration::SubscriptionId subscribePhysicsOnStepEvents(
    bool preStep, int order, registration::OnPhysicsStepEventFunction onUpdate);

/**
 * @brief Unsubscribes from physics step events.
 *
 * An invalid or unknown subscription identifier is ignored.
 *
 * @param[in] subscriptionId Subscription identifier returned by `subscribePhysicsOnStepEvents()`.
 *
 * @note Do not modify step-event subscriptions from within a step-event callback.
 * @note Exceptions raised by a backend unsubscription callback propagate to the caller and prevent later backends from
 *       being unsubscribed during that call.
 */
ISAACSIM_PHYSICS_MANAGER_API void unsubscribePhysicsOnStepEvents(registration::SubscriptionId subscriptionId);

/**
 * @brief Queries whether a simulation supports USD schema types or applied schema APIs.
 *
 * @param[in] simulationId Identifier of the simulation to query.
 * @param[in] schemaNames Array of null-terminated schema type or applied-schema API names. Must be non-null and
 *                        reference at least `schemaNamesCount` entries when `schemaNamesCount` is nonzero. A null
 *                        element requests an unsupported result for that entry.
 * @param[in] schemaNamesCount Number of entries in `schemaNames` and `isCapable`.
 * @param[out] isCapable Array with capacity for `schemaNamesCount` results. Must be non-null when `schemaNamesCount` is
 *                       nonzero. Its contents are defined only when the function returns `true`.
 * @return `true` if the backend completed the capability query and populated `isCapable`; otherwise, `false`.
 *
 * @note Exceptions raised by the selected backend query callback propagate to the caller.
 * @note The simulation may be queried while inactive, but it must remain registered.
 */
ISAACSIM_PHYSICS_MANAGER_API bool isCapableOfSimulating(registration::SimulationId simulationId,
                                                        const char** schemaNames,
                                                        size_t schemaNamesCount,
                                                        bool* isCapable);
} // namespace manager
} // namespace physics
} // namespace isaacsim

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

#include "Types.hpp"
#include "simulator/Simulator.hpp"

#include <isaacsim/physics/registration/Export.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace isaacsim
{
namespace physics
{
namespace registration
{

/**
 * @brief Types of events emitted by the simulation registry.
 *
 * Registry events are emitted after the associated registry action completes.
 */
enum class SimulationRegistryEventType : uint32_t
{
    /** @brief A new simulation was registered. */
    eSimulationRegistered,

    /** @brief A simulation was unregistered. Its identifier is no longer valid when the callback runs. */
    eSimulationUnregistered,

    /**
     * @brief A simulation activation request completed.
     *
     * Registration creates an active simulation without emitting this event. An explicit activation request emits
     * the event even when the simulation was already active.
     */
    eSimulationActivated,

    /**
     * @brief A simulation deactivation request completed.
     *
     * An explicit deactivation request emits the event even when the simulation was already inactive.
     */
    eSimulationDeactivated,
};

/**
 * @brief Callback invoked after a simulation registry event.
 *
 * @param[in] eventType The type of registry event.
 * @param[in] simulationId The identifier of the affected simulation.
 * @param[in] simulationName The name of the affected simulation.
 * @param[in] userData The caller-owned data pointer supplied when subscribing. The registry does not take ownership.
 *
 * @note References passed to the callback remain valid only for the duration of the callback invocation.
 * @note Exceptions raised by the callback are caught and logged by the registry and do not propagate to the registry
 *       operation that emitted the event.
 */
using OnSimulationRegistryEventFunction = std::function<void(
    SimulationRegistryEventType eventType, const SimulationId& simulationId, const std::string& simulationName, void* userData)>;

/**
 * @brief Registers a simulation with the physics system.
 *
 * The simulation is initially active. Registration emits an
 * @c SimulationRegistryEventType::eSimulationRegistered event.
 * The registry copies @p simulation; the caller does not need to retain the original function-table object.
 *
 * @param[in] simulation The simulation function table to register.
 * @param[in] simulationName The human-readable simulation name.
 * @return The identifier assigned to the registered simulation.
 */
ISAACSIM_PHYSICS_REGISTRATION_API SimulationId registerSimulation(const Simulation& simulation,
                                                                  const std::string& simulationName);

/**
 * @brief Unregisters a simulation from the physics system.
 *
 * @param[in] simulationId The identifier of the simulation to unregister.
 */
ISAACSIM_PHYSICS_REGISTRATION_API void unregisterSimulation(const SimulationId& simulationId);

/**
 * @brief Retrieves a registered simulation.
 *
 * @param[in] simulationId The simulation identifier.
 * @return A non-owning, registry-owned pointer to the simulation, or @c nullptr if @p simulationId is not registered.
 *         The caller must not delete the pointer. A non-null pointer remains valid until @p simulationId is
 *         unregistered.
 */
ISAACSIM_PHYSICS_REGISTRATION_API const Simulation* getSimulation(const SimulationId& simulationId);

/**
 * @brief Retrieves the name of a registered simulation.
 *
 * @param[in] simulationId The simulation identifier.
 * @return The simulation name, or an empty string if @p simulationId is not registered.
 */
ISAACSIM_PHYSICS_REGISTRATION_API std::string getSimulationName(const SimulationId& simulationId);

/**
 * @brief Returns the number of registered simulations.
 *
 * @return The number of simulations currently registered with the physics system.
 */
ISAACSIM_PHYSICS_REGISTRATION_API size_t getSimulationCount();

/**
 * @brief Copies registered simulation identifiers into a caller-provided buffer.
 *
 * Identifiers are copied in unspecified order. When @p bufferSize is smaller than the number of registered simulations,
 * the subset copied is unspecified.
 *
 * @param[out] simulationIds The destination buffer, or @c nullptr to copy no identifiers and return zero.
 * @param[in] bufferSize The capacity of @p simulationIds, in elements.
 * @return The number of simulation identifiers copied. This value does not exceed @p bufferSize.
 */
ISAACSIM_PHYSICS_REGISTRATION_API size_t getSimulationIds(SimulationId* simulationIds, size_t bufferSize);

/**
 * @brief Activates a registered simulation.
 *
 * @param[in] simulationId The simulation identifier.
 *
 * @note A registered simulation emits
 *       @c SimulationRegistryEventType::eSimulationActivated even if it was already
 *       active.
 */
ISAACSIM_PHYSICS_REGISTRATION_API void activateSimulation(const SimulationId& simulationId);

/**
 * @brief Deactivates a registered simulation.
 *
 * @param[in] simulationId The simulation identifier.
 *
 * @note A registered simulation emits
 *       @c SimulationRegistryEventType::eSimulationDeactivated even if it was already
 *       inactive.
 */
ISAACSIM_PHYSICS_REGISTRATION_API void deactivateSimulation(const SimulationId& simulationId);

/**
 * @brief Reports whether a registered simulation is active.
 *
 * @param[in] simulationId The simulation identifier.
 * @return @c true if the simulation is registered and active; otherwise, @c false.
 */
ISAACSIM_PHYSICS_REGISTRATION_API bool isSimulationActive(const SimulationId& simulationId);

/**
 * @brief Retrieves the active simulation registered under a given name.
 *
 * Names are matched exactly. A simulation and the tensor factories that resolve it are registered under one
 * name, so an exact match is what keeps distinctly named simulations apart.
 *
 * @param[in] simulationName The simulation name to match.
 * @return The identifier of the active simulation registered under @p simulationName, or
 *         @c g_kInvalidSimulationId when no such simulation is active.
 * @throws std::runtime_error If more than one active simulation carries that name.
 */
ISAACSIM_PHYSICS_REGISTRATION_API SimulationId getActiveSimulationId(const std::string& simulationName);

/**
 * @brief Subscribes to simulation registry events.
 *
 * @param[in] onEvent The callback to invoke for registry events.
 * @param[in] userData A caller-owned data pointer to pass to @p onEvent. The registry does not take ownership; when
 *                     non-null, the pointer must remain valid until unsubscription completes and every event dispatch
 *                     that snapshotted the subscription before removal has finished.
 * @return A subscription identifier for use with @c unsubscribeSimulationRegistryEvents(), or
 *         @c g_kInvalidSubscriptionId if @p onEvent is empty.
 */
ISAACSIM_PHYSICS_REGISTRATION_API SubscriptionId
subscribeSimulationRegistryEvents(OnSimulationRegistryEventFunction onEvent, void* userData = nullptr);

/**
 * @brief Unsubscribes from simulation registry events.
 *
 * @param[in] subscriptionId The identifier returned by @c subscribeSimulationRegistryEvents().
 *
 * @note Event dispatch snapshots the subscription set before invoking callbacks. Removing a subscription does not
 *       cancel an invocation that is already present in such a snapshot.
 */
ISAACSIM_PHYSICS_REGISTRATION_API void unsubscribeSimulationRegistryEvents(SubscriptionId subscriptionId);
} // namespace registration
} // namespace physics
} // namespace isaacsim

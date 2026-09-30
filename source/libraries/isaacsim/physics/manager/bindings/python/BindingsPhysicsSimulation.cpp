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

#include "BindingsPhysics.hpp"
#include "NanobindHelpers.hpp"

#include <isaacsim/physics/manager/PhysicsSimulation.hpp>

#include <memory>

namespace nb = nanobind;

namespace
{
// Keepalive for caller-owned objects backing attached stages.
// A borrowed native instance must outlive its attachment, so the simulation holds
// a reference to each such Python owner until it can confirm the stage is detached
// -- a downstream caller can drop its own reference without freeing it under us.
// Kept engine-agnostic: owners are opaque here.
//
// FailedDirty cannot say WHICH stage remains attached -- the new one (a dirty
// rollback) or a previous one (a re-initialize whose detach of the old stage
// failed) -- so the keepAlive holds a set of owners, not a single slot, and drops
// owners only on positive detach confirmation: Ok (previous stages detached, keep
// only the new owner) or a fully successful close (everything detached, drop all).
// A dirty failure at worst over-holds a not-yet-freed owner until the next such
// event. Leaked on purpose; a static-storage nb object would be destroyed during
// interpreter teardown, after Python has finalized.
nb::list& getAttachedStageOwners()
{
    static nb::list* s_owners = new nb::list();
    return *s_owners;
}
} // namespace

void isaacsim::physics::manager::details::bindPhysicsSimulation(nb::module_& module)
{
    using namespace isaacsim::physics::manager;
    using namespace isaacsim::physics::registration;

    module.def(
        "initialize",
        [](uintptr_t ovstage, const char* usdIdentifier, nb::object owner)
        {
            const InitializeResult result = initialize(reinterpret_cast<void*>(ovstage), usdIdentifier);
            if (result == InitializeResult::eOk)
            {
                getAttachedStageOwners() = nb::list();
                if (!owner.is_none())
                {
                    getAttachedStageOwners().append(owner);
                }
            }
            else if (result == InitializeResult::eFailedDirty && !owner.is_none())
            {
                getAttachedStageOwners().append(owner);
            }
            return result == InitializeResult::eOk;
        },
        nb::arg("ovstage"), nb::arg("usd_identifier"), nb::arg("owner") = nb::none(),
        "Attach a USD stage to all active physics simulations.");
    module.def(
        "close",
        []()
        {
            const bool closed = isaacsim::physics::manager::close();
            if (closed)
            {
                getAttachedStageOwners() = nb::list();
            }
            return closed;
        },
        "Detach the current stage from all active physics simulations.");
    module.def("get_attached_stage", &getAttachedStage, "Return the identifier of the attached USD stage.");
    module.def("simulate_async", &simulateAsynchronously, nb::arg("elapsed_time"), nb::arg("current_time"),
               "Start an asynchronous simulation step.");
    module.def("simulate", &simulate, nb::arg("elapsed_time"), nb::arg("current_time"),
               "Advance all active simulations synchronously.");
    module.def("fetch_results", &fetchResults, "Fetch results for the pending asynchronous simulation step.");
    module.def("check_results", &checkResults, "Return whether pending asynchronous simulation results are ready.");
    module.def("publish_transforms_to_stage", &publishTransformsToStage,
               "Publish the latest simulated transforms to attached stages.");
    module.def("flush_changes", &flushChanges, "Flush tracked USD changes to all active simulations.");
    module.def("pause_change_tracking", &pauseChangeTracking, nb::arg("pause"),
               "Pause or resume USD change tracking for all active simulations.");
    module.def("is_change_tracking_paused", &isChangeTrackingPaused, nb::arg("simulation_id"),
               "Return whether USD change tracking is paused for a simulation.");
    module.def(
        "subscribe_physics_contact_report_events",
        [](OnContactReportEventFunction onEvent)
        {
            const SubscriptionId id = subscribePhysicsContactReportEvents(std::move(onEvent));
            return PythonSubscription(
                id, [](SubscriptionId subscriptionId) { unsubscribePhysicsContactReportEvents(subscriptionId); });
        },
        nb::arg("contact_report_fn"), "Subscribe to contact-report events from active simulations.");
    module.def("get_simulation_time_steps_per_second", &getSimulationTimeStepsPerSecond, nb::arg("simulation_id"),
               nb::arg("stage_id"), nb::arg("scene_path"), "Return the configured update rate for a physics scene.");
    module.def("get_simulation_timestamp", &getSimulationTimestamp, nb::arg("simulation_id"),
               "Return the latest timestamp reported by a simulation.");
    module.def("get_simulation_step_count", &getSimulationStepCount, nb::arg("simulation_id"),
               "Return the number of completed steps for a simulation.");
    module.def(
        "subscribe_physics_on_step_events",
        [](bool preStep, int order, OnPhysicsStepEventFunction onUpdate)
        {
            const SubscriptionId id = subscribePhysicsOnStepEvents(preStep, order, std::move(onUpdate));
            return PythonSubscription(
                id, [](SubscriptionId subscriptionId) { unsubscribePhysicsOnStepEvents(subscriptionId); });
        },
        nb::arg("pre_step"), nb::arg("order"), nb::arg("on_update"),
        "Subscribe to ordered callbacks before or after each physics step.");
    module.def(
        "is_capable_of_simulating",
        [](SimulationId simulationId, const std::vector<std::string>& schemaNames)
        {
            std::vector<const char*> schemaNamePointers;
            schemaNamePointers.reserve(schemaNames.size());
            for (const auto& name : schemaNames)
            {
                schemaNamePointers.push_back(name.c_str());
            }
            std::unique_ptr<bool[]> isCapableOutput(new bool[schemaNames.size()]);
            const bool success = isCapableOfSimulating(
                simulationId, schemaNamePointers.data(), schemaNamePointers.size(), isCapableOutput.get());
            if (!success)
            {
                return nb::make_tuple(false, nb::list());
            }
            nb::list result;
            for (size_t i = 0; i < schemaNames.size(); ++i)
            {
                result.append(isCapableOutput[i]);
            }
            return nb::make_tuple(true, result);
        },
        nb::arg("simulation_id"), nb::arg("schema_names"),
        "Return whether a simulation supports each requested USD schema.");
}

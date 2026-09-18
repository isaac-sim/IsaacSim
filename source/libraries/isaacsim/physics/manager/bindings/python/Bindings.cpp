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
#include "isaacsim/physics/manager/PhysicsEvent.hpp"
#include "isaacsim/physics/manager/PhysicsManager.hpp"

#include <isaacsim/physics/registration/Physics.hpp>
#include <isaacsim/physics/registration/simulator/ContactEvent.hpp>
#include <isaacsim/physics/registration/simulator/Simulation.hpp>
#include <isaacsim/physics/registration/simulator/Simulator.hpp>
#include <nanobind/stl/optional.h>

namespace nb = nanobind;
using namespace isaacsim::physics::manager;

NB_MODULE(_bindings, module)
{
    module.doc() = "Manage physics simulations, scene queries, interactions, profiling, and tensor views.";

    // Register the shared vocabulary before any manager signatures refer to it. This keeps generated annotations in
    // Python syntax and lets manager functions reuse the canonical registration-module types.
    nb::module_::import_("isaacsim.physics.registration.bindings._bindings");

    // PhysicsEvent.hpp
    nb::enum_<PhysicsEvent>(module, "PhysicsEvent", "Identify a physics manager lifecycle event.")
        .value("PHYSICS_PRE_STEP", PhysicsEvent::ePhysicsPreStep, "Before a simulation step begins.")
        .value("PHYSICS_POST_STEP", PhysicsEvent::ePhysicsPostStep, "After a simulation step completes.")
        .value("PHYSICS_SETUP", PhysicsEvent::ePhysicsSetup, "After simulation setup completes.")
        .value("PHYSICS_INITIALIZED", PhysicsEvent::ePhysicsInitialized, "After simulation initialization completes.")
        .value("PHYSICS_INVALIDATED", PhysicsEvent::ePhysicsInvalidated, "After simulation state is invalidated.");

    // PhysicsManager.hpp
    nb::class_<PhysicsManager>(module, "PhysicsManager", "Coordinate registered physics simulation backends.")
        .def_static("get_instance", &PhysicsManager::getInstance, nb::rv_policy::reference,
                    "Return the process-wide physics manager. The manager remains owned by the module.")
        .def("is_initialized", &PhysicsManager::isInitialized, "Return whether physics simulation is initialized.")
        .def("setup", &PhysicsManager::configure, nb::arg("dt") = 1.0f / 60.0f,
             "Configure the simulation time step before initialization.")
        .def(
            "initialize",
            [](PhysicsManager& self, uintptr_t ovstage, int64_t usdStageId)
            { return self.initialize(reinterpret_cast<void*>(ovstage), usdStageId); },
            nb::arg("ovstage_instance"), nb::arg("usd_stage_id"),
            "Initialize simulation against an OVStage instance and USD stage identifier.")
        .def("invalidate", &PhysicsManager::invalidate, "Invalidate all active physics simulation state.")
        .def("step", &PhysicsManager::step, nb::kw_only(), nb::arg("steps") = 1, nb::arg("callback") = nb::none(),
             "Advance the active physics simulation by one or more steps.")
        .def("publish_transforms_to_stage", &PhysicsManager::publishTransformsToStage,
             "Publish simulated transforms to the current stage.")
        .def("get_simulated_time", &PhysicsManager::getSimulatedTime, "Return the elapsed simulated time in seconds.")
        .def("get_simulated_physics_steps", &PhysicsManager::getSimulatedPhysicsStepCount,
             "Return the number of completed physics steps.")
        .def("register_callback", &PhysicsManager::registerCallback, nb::arg("callback"), nb::arg("event"),
             nb::kw_only(), nb::arg("order") = 0, "Register a callback for a physics lifecycle event.")
        .def("deregister_callback", &PhysicsManager::deregisterCallback, nb::arg("uid"),
             "Remove the callback with the given identifier.")
        .def("deregister_all_callbacks", &PhysicsManager::deregisterAllCallbacks,
             "Remove every registered physics lifecycle callback.")
        .def("get_registered_physics_engines", &PhysicsManager::getRegisteredPhysicsEngines,
             "Return registered engine names and their active state.")
        .def("switch_physics_engine", &PhysicsManager::switchPhysicsEngine, nb::arg("engine"),
             "Select the registered physics engine used for simulation.")
        .def("create_entity", &PhysicsManager::createEntity, nb::kw_only(), nb::arg("engine"), nb::arg("entity"),
             nb::arg("paths"), nb::arg("options") = nb::none(),
             "Create an entity view using a registered physics engine.");

    bindPythonSubscription(module);
    details::bindPhysicsSceneQuery(module);
    details::bindPhysicsInteraction(module);
    details::bindPhysicsBenchmarks(module);
    details::bindPhysicsSimulation(module);
    details::bindTensorViews(module);
    details::bindTensorCreate(module);
}

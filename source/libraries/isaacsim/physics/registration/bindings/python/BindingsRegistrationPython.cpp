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

#include "NanobindHelpers.hpp"

#include <isaacsim/physics/registration/Physics.hpp>
#include <isaacsim/physics/registration/simulator/ContactEvent.hpp>
#include <isaacsim/physics/registration/simulator/Simulation.hpp>
#include <isaacsim/physics/registration/simulator/Simulator.hpp>
#include <nanobind/stl/bind_vector.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace nb = nanobind;

// Tensor register API (compiled as a separate TU in this module).
namespace isaacsim
{
namespace physics
{
namespace registration
{
namespace details
{
void bindTensors(nb::module_& module);
}
} // namespace registration
} // namespace physics
} // namespace isaacsim

NB_MAKE_OPAQUE(isaacsim::physics::registration::ContactEventHeaderVector)
NB_MAKE_OPAQUE(isaacsim::physics::registration::ContactDataVector)
NB_MAKE_OPAQUE(isaacsim::physics::registration::FrictionAnchorsDataVector)

namespace
{

/**
 * @brief Own a Python callable from C++ and release it with the GIL held.
 *
 * The std::function holding this wrapper may be destroyed by C++ (simulation teardown,
 * reassignment) on a thread that does not hold the GIL; decrementing the reference count
 * there corrupts the interpreter state. Copies share one holder via std::shared_ptr so
 * only the final release touches the reference count.
 */
struct GilSafeCallable
{
    nb::callable function;

    explicit GilSafeCallable(nb::callable callable) : function(std::move(callable))
    {
    }

    GilSafeCallable(const GilSafeCallable&) = delete;
    GilSafeCallable& operator=(const GilSafeCallable&) = delete;

    ~GilSafeCallable()
    {
        nb::gil_scoped_acquire globalInterpreterLockAcquire;
        function.reset();
    }
};

} // namespace

NB_MODULE(_bindings, module)
{
    using namespace isaacsim::physics::registration;

    module.doc() = "Register physics-engine simulations, callbacks, and tensor factories.";

    details::bindPythonSubscription(module);

    nb::class_<Float3>(module, "Float3", "Store a three-component floating-point value.")
        .def(nb::init<>(), "Create a value initialized to zero.")
        .def(
            "__init__",
            [](Float3* self, float x, float y, float z) {
                new (self) Float3{ x, y, z };
            },
            nb::arg("x"), nb::arg("y"), nb::arg("z"), "Create a value from its three components.")
        .def_rw("x", &Float3::x, "First component.")
        .def_rw("y", &Float3::y, "Second component.")
        .def_rw("z", &Float3::z, "Third component.");

    nb::class_<Float4>(module, "Float4", "Store a four-component floating-point value.")
        .def(nb::init<>(), "Create a value initialized to zero.")
        .def(
            "__init__",
            [](Float4* self, float x, float y, float z, float w) {
                new (self) Float4{ x, y, z, w };
            },
            nb::arg("x"), nb::arg("y"), nb::arg("z"), nb::arg("w"), "Create a value from its four components.")
        .def_rw("x", &Float4::x, "First component.")
        .def_rw("y", &Float4::y, "Second component.")
        .def_rw("z", &Float4::z, "Third component.")
        .def_rw("w", &Float4::w, "Fourth component.");

    nb::class_<SimulationId>(module, "SimulationId", "A unique identifier for a physics simulation instance.")
        .def(nb::init<>(), "Create an invalid simulation identifier.")
        .def(nb::init<size_t>(), nb::arg("id"), "Create an identifier from an integer value.")
        .def(
            "__eq__", [](const SimulationId& left, const SimulationId& right) { return left == right; },
            nb::is_operator(), "Return whether two identifiers are equal.")
        .def(
            "__ne__", [](const SimulationId& left, const SimulationId& right) { return left != right; },
            nb::is_operator(), "Return whether two identifiers differ.")
        .def("__hash__", &SimulationId::computeHash, "Return a hash of the identifier.")
        .def(
            "__int__", [](const SimulationId& simulationId) { return simulationId.id; },
            "Return the identifier as an integer.")
        .def(
            "__index__", [](const SimulationId& simulationId) { return simulationId.id; },
            "Return the identifier for integer-index contexts.")
        .def_ro("id", &SimulationId::id, "Integer identifier value.");

    nb::class_<PhysicsStepContext>(module, "PhysicsStepContext", "Identify the scene and simulation for a step callback.")
        .def(nb::init<>(), "Create an empty step context.")
        .def_rw("scene_path", &PhysicsStepContext::scenePath, "Path of the simulated physics scene.")
        .def_rw("simulation_id", &PhysicsStepContext::simulationId, "Identifier of the simulation backend.");

    nb::enum_<ForceModeType>(module, "ForceMode", "Modes used when applying forces to physics objects.")
        .value("FORCE", ForceModeType::eForce, "Apply a continuous force.")
        .value("IMPULSE", ForceModeType::eImpulse, "Apply an instantaneous impulse.")
        .value("VELOCITY_CHANGE", ForceModeType::eVelocityChange, "Apply an instantaneous velocity change.")
        .value("ACCELERATION", ForceModeType::eAcceleration, "Apply a continuous acceleration.")
        .export_values();

    nb::enum_<ContactEventType>(module, "ContactEventType", "Lifecycle states reported for a contact pair.")
        .value("CONTACT_FOUND", ContactEventType::eContactFound, "A contact pair began touching.")
        .value("CONTACT_LOST", ContactEventType::eContactLost, "A contact pair stopped touching.")
        .value("CONTACT_PERSIST", ContactEventType::eContactPersist, "A contact pair remains touching.")
        .export_values();

    nb::class_<ContactEventHeader>(module, "ContactEventHeader", "Describe one pair of objects in a contact event.")
        .def(nb::init<>(), "Create an empty contact-event header.")
        .def_rw("type", &ContactEventHeader::type, "Lifecycle state of the contact pair.")
        .def_rw("stage_id", &ContactEventHeader::stageId, "Identifier of the stage containing the contact.")
        .def_rw("actor0", &ContactEventHeader::actor0, "First actor path identifier.")
        .def_rw("actor1", &ContactEventHeader::actor1, "Second actor path identifier.")
        .def_rw("collider0", &ContactEventHeader::collider0, "First collider path identifier.")
        .def_rw("collider1", &ContactEventHeader::collider1, "Second collider path identifier.")
        .def_rw(
            "contact_data_offset", &ContactEventHeader::contactDataOffset, "Offset of this pair's first contact point.")
        .def_rw("num_contact_data", &ContactEventHeader::contactDataCount, "Number of contact points for this pair.")
        .def_rw("friction_anchors_data_offset", &ContactEventHeader::frictionAnchorsDataOffset,
                "Offset of this pair's first friction anchor.")
        .def_rw("num_friction_anchors_data", &ContactEventHeader::frictionAnchorDataCount,
                "Number of friction anchors for this pair.")
        .def_rw("proto_index0", &ContactEventHeader::prototypeIndex0, "Prototype index for the first actor.")
        .def_rw("proto_index1", &ContactEventHeader::prototypeIndex1, "Prototype index for the second actor.");

    nb::class_<ContactData>(module, "ContactData", "Describe one contact point.")
        .def(nb::init<>(), "Create empty contact-point data.")
        .def_rw("position", &ContactData::position, "World-space contact position.")
        .def_rw("normal", &ContactData::normal, "World-space contact normal.")
        .def_rw("separation", &ContactData::separation, "Signed separation distance.")
        .def_rw("impulse", &ContactData::impulse, "Normal impulse magnitude.");

    nb::class_<FrictionAnchor>(module, "FrictionAnchor", "Describe one friction anchor.")
        .def(nb::init<>(), "Create empty friction-anchor data.")
        .def_rw("position", &FrictionAnchor::position, "World-space anchor position.")
        .def_rw("impulse", &FrictionAnchor::impulse, "Friction impulse at the anchor.");

    nb::bind_vector<ContactEventHeaderVector>(
        module, "ContactEventHeaderVector", "Mutable sequence of contact-event headers.");
    nb::bind_vector<ContactDataVector>(module, "ContactDataVector", "Mutable sequence of contact points.");
    nb::bind_vector<FrictionAnchorsDataVector>(
        module, "FrictionAnchorsDataVector", "Mutable sequence of friction anchors.");

    // Bind scene-query and profiling vocabulary before the callback collections that reference it. Besides making
    // these shared types available from their owning registration module, the ordering gives generated Python stubs
    // valid local type names instead of unresolved C++ qualified names.
    nb::class_<SceneQueryHitObject>(module, "SceneQueryHitObject", "Identify the objects reported by a scene query.")
        .def(nb::init<>(), "Create an empty scene-query object hit.")
        .def_rw("collision", &SceneQueryHitObject::collision, "Path of the hit collision shape.")
        .def_rw("rigid_body", &SceneQueryHitObject::rigidBody, "Path of the hit rigid body.")
        .def_rw("proto_index", &SceneQueryHitObject::prototypeIndex, "Prototype index for an instanced hit.");

    nb::class_<SceneQueryHitLocation, SceneQueryHitObject>(
        module, "SceneQueryHitLocation", "Describe the location and material of a scene-query hit.")
        .def(nb::init<>(), "Create an empty located scene-query hit.")
        .def_rw("normal", &SceneQueryHitLocation::normal, "Surface normal at the hit.")
        .def_rw("position", &SceneQueryHitLocation::position, "World-space hit position.")
        .def_rw("distance", &SceneQueryHitLocation::distance, "Distance from the query origin.")
        .def_rw("face_index", &SceneQueryHitLocation::faceIndex, "Index of the hit mesh face.")
        .def_rw("material", &SceneQueryHitLocation::material, "Path of the hit physics material.");

    nb::class_<OverlapHit, SceneQueryHitObject>(module, "OverlapHit", "Describe an overlap query result.")
        .def(nb::init<>(), "Create an empty overlap hit.");
    nb::class_<RaycastHit, SceneQueryHitLocation>(module, "RaycastHit", "Describe a raycast query result.")
        .def(nb::init<>(), "Create an empty raycast hit.");
    nb::class_<SweepHit, SceneQueryHitLocation>(module, "SweepHit", "Describe a shape-sweep query result.")
        .def(nb::init<>(), "Create an empty shape-sweep hit.");

    nb::class_<PhysicsProfileStatistics>(module, "PhysicsProfileStats", "Timing data for one physics profiling zone.")
        .def(nb::init<>(), "Create empty profiling statistics.")
        .def_rw("zone_name", &PhysicsProfileStatistics::zoneName, "Name of the profiling zone.")
        .def_rw("ms", &PhysicsProfileStatistics::elapsedMilliseconds, "Time in milliseconds for this zone.");

    nb::class_<SimulationFunctions>(module, "SimulationFns", "Callbacks that implement a physics simulation backend.")
        .def(nb::init<>(), "Create an empty simulation callback collection.")
        // Convert the opaque C++ ovstage pointer to an integer address for Python
        // backends (matching physics_manager.initialize's `ovstage: int` contract).
        // The default std::function caster would deliver a nanobind PyCapsule instead.
        .def_prop_rw(
            "initialize",
            [](SimulationFunctions& self) -> nb::object
            {
                if (!self.initialize)
                {
                    return nb::none();
                }
                return nb::cpp_function(
                    [attachFunction = self.initialize](uintptr_t ovstage, nb::object usdIdentifier) -> bool
                    {
                        const char* usdIdentifierCString = nullptr;
                        std::string usdIdentifierStorage;
                        if (!usdIdentifier.is_none())
                        {
                            usdIdentifierStorage = nb::cast<std::string>(usdIdentifier);
                            usdIdentifierCString = usdIdentifierStorage.c_str();
                        }
                        return attachFunction(reinterpret_cast<void*>(ovstage), usdIdentifierCString);
                    });
            },
            [](SimulationFunctions& self, nb::object pythonFunction)
            {
                if (pythonFunction.is_none())
                {
                    self.initialize = nullptr;
                    return;
                }
                // nb::borrow does not type-check; reject non-callables here rather than
                // failing inside the simulation step where the traceback is opaque.
                if (!nb::isinstance<nb::callable>(pythonFunction))
                {
                    throw nb::type_error("SimulationFns.initialize must be callable or None");
                }
                auto callback = std::make_shared<GilSafeCallable>(nb::borrow<nb::callable>(pythonFunction));
                self.initialize = [callback](void* ovstage, const char* usdIdentifier) -> bool
                {
                    nb::gil_scoped_acquire globalInterpreterLockAcquire;
                    nb::object usdIdentifierObject = usdIdentifier ? nb::cast(usdIdentifier) : nb::none();
                    // Let Python exceptions propagate to physics.manager's initialize try/catch
                    // so the traceback is logged; do not swallow them here.
                    return nb::cast<bool>(callback->function(reinterpret_cast<uintptr_t>(ovstage), usdIdentifierObject));
                };
            },
            "Callback that initializes a simulation against an OVStage instance.")
        .def_rw("close", &SimulationFunctions::close, "Callback that closes simulation resources.")
        .def_rw("get_attached_stage", &SimulationFunctions::getAttachedStage,
                "Callback that returns the attached stage identifier.")
        .def_rw("has_attached_stage", &SimulationFunctions::hasAttachedStage,
                "Callback that reports whether a stage is attached.")
        .def_rw("simulate_async", &SimulationFunctions::simulateAsynchronously,
                "Callback that starts an asynchronous simulation step.")
        .def_rw("simulate", &SimulationFunctions::simulate, "Callback that starts a simulation step.")
        .def_rw("fetch_results", &SimulationFunctions::fetchResults,
                "Callback that waits for and applies simulation results.")
        .def_rw("check_results", &SimulationFunctions::checkResults,
                "Callback that reports whether simulation results are ready.")
        .def_rw("publish_transforms_to_stage", &SimulationFunctions::publishTransformsToStage,
                "Callback that publishes simulated transforms to the stage.")
        .def_rw("flush_changes", &SimulationFunctions::flushChanges,
                "Callback that flushes pending stage changes to simulation.")
        .def_rw("pause_change_tracking", &SimulationFunctions::pauseChangeTracking,
                "Callback that pauses or resumes stage-change tracking.")
        .def_rw("is_change_tracking_paused", &SimulationFunctions::isChangeTrackingPaused,
                "Callback that reports whether stage-change tracking is paused.")
        .def_rw("subscribe_physics_contact_report_events", &SimulationFunctions::subscribePhysicsContactReportEvents,
                "Callback that subscribes to physics contact-report events.")
        .def_rw("unsubscribe_physics_contact_report_events", &SimulationFunctions::unsubscribePhysicsContactReportEvents,
                "Callback that unsubscribes from physics contact-report events.")
        .def_rw("get_simulation_time_steps_per_second", &SimulationFunctions::getSimulationTimeStepsPerSecond,
                "Callback that returns the configured simulation frequency.")
        .def_rw("get_simulation_timestamp", &SimulationFunctions::getSimulationTimestamp,
                "Callback that returns the current simulation timestamp.")
        .def_rw("get_simulation_step_count", &SimulationFunctions::getSimulationStepCount,
                "Callback that returns the current simulation step count.")
        .def_rw("subscribe_physics_on_step_events", &SimulationFunctions::subscribePhysicsOnStepEvents,
                "Callback that subscribes to simulation step events.")
        .def_rw("unsubscribe_physics_on_step_events", &SimulationFunctions::unsubscribePhysicsOnStepEvents,
                "Callback that unsubscribes from simulation step events.")
        .def_prop_rw(
            "is_capable_of_simulating",
            [](SimulationFunctions& self) -> nb::object
            {
                if (!self.isCapableOfSimulating)
                {
                    return nb::none();
                }
                return nb::cpp_function(
                    [capabilityFunction = self.isCapableOfSimulating](const std::vector<std::string>& schemaNames)
                    {
                        std::vector<const char*> schemaNamePointers;
                        schemaNamePointers.reserve(schemaNames.size());
                        for (const auto& schemaName : schemaNames)
                        {
                            schemaNamePointers.push_back(schemaName.c_str());
                        }
                        auto capabilityResults = std::make_unique<bool[]>(schemaNames.size());
                        const bool succeeded = capabilityFunction(
                            schemaNamePointers.data(), schemaNamePointers.size(), capabilityResults.get());
                        if (!succeeded)
                        {
                            return nb::make_tuple(false, nb::list());
                        }
                        nb::list result;
                        for (size_t i = 0; i < schemaNames.size(); ++i)
                        {
                            result.append(capabilityResults[i]);
                        }
                        return nb::make_tuple(true, result);
                    });
            },
            [](SimulationFunctions& self, nb::object pythonFunction)
            {
                if (pythonFunction.is_none())
                {
                    self.isCapableOfSimulating = nullptr;
                    return;
                }
                nb::callable callback = nb::borrow<nb::callable>(pythonFunction);
                self.isCapableOfSimulating = [callback](const char** schemaNames, size_t schemaNameCount,
                                                        bool* capabilityResults) -> bool
                {
                    nb::gil_scoped_acquire globalInterpreterLockAcquire;
                    try
                    {
                        nb::list pythonSchemaNames;
                        for (size_t schemaIndex = 0; schemaIndex < schemaNameCount; ++schemaIndex)
                        {
                            pythonSchemaNames.append(schemaNames[schemaIndex]);
                        }
                        const nb::tuple result = nb::cast<nb::tuple>(callback(pythonSchemaNames));
                        const bool success = nb::cast<bool>(result[0]);
                        if (!success)
                        {
                            return false;
                        }
                        const nb::list capabilities = nb::cast<nb::list>(result[1]);
                        const size_t resultCount =
                            capabilities.size() < schemaNameCount ? capabilities.size() : schemaNameCount;
                        for (size_t resultIndex = 0; resultIndex < resultCount; ++resultIndex)
                        {
                            capabilityResults[resultIndex] = nb::cast<bool>(capabilities[resultIndex]);
                        }
                        return true;
                    }
                    catch (nb::python_error& error)
                    {
                        error.discard_as_unraisable("is_capable_of_simulating");
                        return false;
                    }
                };
            },
            "Callback that reports support for stage schema types.");

    nb::class_<InteractionFunctions>(module, "InteractionFns", "Callbacks that implement interactive physics tools.")
        .def(nb::init<>(), "Create an empty interaction callback collection.")
        .def_rw("handle_raycast", &InteractionFunctions::handleRaycast,
                "Callback that handles an interactive raycast request.")
        .def_rw("get_prim_debug_data", &InteractionFunctions::getPrimDebugData,
                "Callback that returns debug data for a prim.");

    nb::class_<SceneQueryFunctions>(module, "SceneQueryFns", "Callbacks that implement physics scene queries.")
        .def(nb::init<>(), "Create an empty scene-query callback collection.")
        .def_rw("raycast_closest", &SceneQueryFunctions::raycastClosest, "Callback that returns the closest raycast hit.")
        .def_rw("raycast_all", &SceneQueryFunctions::raycastAll, "Callback that returns every raycast hit.")
        .def_rw("raycast_any", &SceneQueryFunctions::raycastAny, "Callback that reports whether a raycast hits.")
        .def_rw("sweep_sphere_closest", &SceneQueryFunctions::sweepSphereClosest,
                "Callback that returns the closest sphere-sweep hit.")
        .def_rw("sweep_sphere_all", &SceneQueryFunctions::sweepSphereAll, "Callback that returns every sphere-sweep hit.")
        .def_rw("sweep_sphere_any", &SceneQueryFunctions::sweepSphereAny,
                "Callback that reports whether a sphere sweep hits.")
        .def_rw("sweep_box_closest", &SceneQueryFunctions::sweepBoxClosest,
                "Callback that returns the closest box-sweep hit.")
        .def_rw("sweep_box_all", &SceneQueryFunctions::sweepBoxAll, "Callback that returns every box-sweep hit.")
        .def_rw("sweep_box_any", &SceneQueryFunctions::sweepBoxAny, "Callback that reports whether a box sweep hits.")
        .def_rw("sweep_shape_closest", &SceneQueryFunctions::sweepShapeClosest,
                "Callback that returns the closest shape-sweep hit.")
        .def_rw("sweep_shape_all", &SceneQueryFunctions::sweepShapeAll, "Callback that returns every shape-sweep hit.")
        .def_rw(
            "sweep_shape_any", &SceneQueryFunctions::sweepShapeAny, "Callback that reports whether a shape sweep hits.")
        .def_rw("overlap_sphere", &SceneQueryFunctions::overlapSphere,
                "Callback that returns overlapping shapes for a sphere.")
        .def_rw("overlap_sphere_any", &SceneQueryFunctions::overlapSphereAny,
                "Callback that reports whether a sphere overlaps a shape.")
        .def_rw("overlap_box", &SceneQueryFunctions::overlapBox, "Callback that returns overlapping shapes for a box.")
        .def_rw("overlap_box_any", &SceneQueryFunctions::overlapBoxAny,
                "Callback that reports whether a box overlaps a shape.")
        .def_rw("overlap_shape", &SceneQueryFunctions::overlapShape,
                "Callback that returns shapes overlapping a stage shape.")
        .def_rw("overlap_shape_any", &SceneQueryFunctions::overlapShapeAny,
                "Callback that reports whether a stage shape overlaps another shape.");

    nb::class_<BenchmarkFunctions>(module, "BenchmarkFns", "Callbacks that expose backend profiling data.")
        .def(nb::init<>(), "Create an empty profiling callback collection.")
        .def_rw("subscribe_profile_stats_events", &BenchmarkFunctions::subscribeProfileStatisticsEvents,
                "Callback that subscribes to physics profiling statistics.")
        .def_rw("unsubscribe_profile_stats_events", &BenchmarkFunctions::unsubscribeProfileStatisticsEvents,
                "Callback that unsubscribes from physics profiling statistics.");

    nb::class_<Simulation>(module, "Simulation", "Group the callback collections registered by a physics engine.")
        .def(nb::init<>(), "Create an empty physics simulation registration.")
        .def_rw("simulation_fns", &Simulation::simulationFunctions, "Simulation lifecycle callbacks.")
        .def_rw("scene_query_fns", &Simulation::sceneQueryFunctions, "Physics scene-query callbacks.")
        .def_rw("interaction_fns", &Simulation::interactionFunctions, "Interactive physics callbacks.")
        .def_rw("benchmark_fns", &Simulation::benchmarkFunctions, "Physics profiling callbacks.");

    nb::enum_<SimulationRegistryEventType>(
        module, "SimulationRegistryEventType", "Changes reported by the simulation registry.")
        .value("SIMULATION_REGISTERED", SimulationRegistryEventType::eSimulationRegistered,
               "A simulation backend was registered.")
        .value("SIMULATION_UNREGISTERED", SimulationRegistryEventType::eSimulationUnregistered,
               "A simulation backend was unregistered.")
        .value("SIMULATION_ACTIVATED", SimulationRegistryEventType::eSimulationActivated,
               "A simulation backend was activated.")
        .value("SIMULATION_DEACTIVATED", SimulationRegistryEventType::eSimulationDeactivated,
               "A simulation backend was deactivated.")
        .export_values();

    module.def("register_simulation", &registerSimulation, nb::arg("simulation"), nb::arg("name"),
               "Register a physics simulation backend.");
    module.def("unregister_simulation", &unregisterSimulation, nb::arg("id"), "Unregister a physics simulation backend.");
    module.def(
        "get_simulation",
        [](const SimulationId& simulationId) -> nb::object
        {
            const Simulation* simulation = getSimulation(simulationId);
            if (simulation)
            {
                return nb::cast(*simulation);
            }
            return nb::none();
        },
        nb::arg("id"), "Return a registered simulation, or None when the identifier is unknown.");
    module.def("get_simulation_name", &getSimulationName, nb::arg("id"), "Return the registered name of a simulation.");
    module.def("get_num_simulations", &getSimulationCount, "Return the number of registered simulations.");
    module.def(
        "get_simulation_ids",
        []()
        {
            std::vector<SimulationId> simulationIds(getSimulationCount());
            simulationIds.resize(getSimulationIds(simulationIds.data(), simulationIds.size()));
            return simulationIds;
        },
        "Return the identifiers of all registered simulations.");
    module.def("activate_simulation", &activateSimulation, nb::arg("id"), "Activate a registered simulation.");
    module.def("deactivate_simulation", &deactivateSimulation, nb::arg("id"), "Deactivate a registered simulation.");
    module.def("is_simulation_active", &isSimulationActive, nb::arg("id"),
               "Return whether a registered simulation is active.");
    module.def("get_active_simulation_id", &getActiveSimulationId, nb::arg("name"),
               "Return the identifier of the active simulation registered under a name, matched exactly, or the "
               "invalid identifier when no such simulation is active. Raise RuntimeError when more than one active "
               "simulation carries that name.");
    module.def(
        "subscribe_simulation_registry_events",
        [](std::function<void(SimulationRegistryEventType, const SimulationId&, const std::string&)> onEvent)
        {
            auto wrapper = [onEvent = std::move(onEvent)](SimulationRegistryEventType eventType,
                                                          const SimulationId& simulationId,
                                                          const std::string& simulationName, void* /*userData*/)
            { onEvent(eventType, simulationId, simulationName); };
            const SubscriptionId subscriptionId = subscribeSimulationRegistryEvents(wrapper, nullptr);
            return details::PythonSubscription(subscriptionId, [](SubscriptionId idToUnsubscribe)
                                               { unsubscribeSimulationRegistryEvents(idToUnsubscribe); });
        },
        nb::arg("on_event"), "Subscribe to simulation registry events. Returns a Subscription handle.");

    // Module-level constants
    module.attr("k_invalid_simulation_id") = g_kInvalidSimulationId;
    module.attr("k_invalid_subscription_id") = g_kInvalidSubscriptionId;

    // Tensor register API (entity/simulation-view factory registry + enums/types).
    details::bindTensors(module);
}

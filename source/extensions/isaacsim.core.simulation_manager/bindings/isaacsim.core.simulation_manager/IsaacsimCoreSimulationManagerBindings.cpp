// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/core/includes/BindingsPythonUtils.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <isaacsim/core/simulation_manager/PhysicsScene.h>
#include <pybind11/functional.h>

CARB_BINDINGS("isaacsim.core.simulation_manager.python")

namespace
{

namespace py = pybind11;

PYBIND11_MODULE(_simulation_manager, m)
{
    using namespace isaacsim::core::simulation_manager;

    m.doc() = R"pbdoc(Omniverse Isaac Sim Simulation Manager Interface

This module provides access to the Simulation Manager which handles events and callbacks
for simulation-related activities, such as physics scene additions and object deletions.
It also manages USD notice handling to track stage changes.)pbdoc";

    // Bind RationalTime (if not already bound elsewhere)
    pybind11::class_<omni::fabric::RationalTime>(m, "RationalTime")
        .def_readonly("numerator", &omni::fabric::RationalTime::numerator, "Numerator of the rational time")
        .def_readonly("denominator", &omni::fabric::RationalTime::denominator, "Denominator of the rational time")
        .def(
            "to_float",
            [](const omni::fabric::RationalTime& rt) {
                return rt.denominator != 0 ? static_cast<double>(rt.numerator) / static_cast<double>(rt.denominator) :
                                             0.0;
            },
            "Convert to floating point seconds")
        .def("__repr__", [](const omni::fabric::RationalTime& rt)
             { return "<RationalTime " + std::to_string(rt.numerator) + "/" + std::to_string(rt.denominator) + ">"; });

    // Bind TimeSampleStorage::TimeData
    pybind11::class_<TimeSampleStorage::TimeData>(m, "TimeData")
        .def_readonly("sim_time", &TimeSampleStorage::TimeData::simTime, "Simulation time in seconds")
        .def_readonly("sim_time_monotonic", &TimeSampleStorage::TimeData::simTimeMonotonic,
                      "Monotonic simulation time in seconds")
        .def_readonly("system_time", &TimeSampleStorage::TimeData::systemTime, "System (real-world) time in seconds")
        .def("__repr__",
             [](const TimeSampleStorage::TimeData& td)
             {
                 return "<TimeData sim=" + std::to_string(td.simTime) + " mono=" + std::to_string(td.simTimeMonotonic) +
                        " sys=" + std::to_string(td.systemTime) + ">";
             });

    // Bind TimeSampleStorage::Entry
    pybind11::class_<TimeSampleStorage::Entry>(m, "TimeSampleEntry")
        .def_readonly("time", &TimeSampleStorage::Entry::time, "Rational time of this sample")
        .def_readonly("data", &TimeSampleStorage::Entry::data, "Time data for this sample")
        .def_readonly("valid", &TimeSampleStorage::Entry::valid, "Whether this entry is valid")
        .def("__repr__",
             [](const TimeSampleStorage::Entry& entry)
             {
                 return "<TimeSampleEntry time=" + std::to_string(entry.time.numerator) + "/" +
                        std::to_string(entry.time.denominator) + " valid=" + (entry.valid ? "True" : "False") + ">";
             });

    // Bind PhysicsScene
    py::class_<PhysicsScene>(m, "PhysicsScene")
        .def(py::init(
                 [](py::object prim)
                 {
                     if (py::isinstance<pxr::UsdPrim>(prim))
                     {
                         return new PhysicsScene(prim.cast<pxr::UsdPrim>());
                     }
                     else if (py::isinstance<pxr::SdfPath>(prim))
                     {
                         return new PhysicsScene(prim.cast<pxr::SdfPath>());
                     }
                     else if (py::isinstance<py::str>(prim))
                     {
                         return new PhysicsScene(prim.cast<py::str>().cast<std::string>());
                     }
                     throw py::value_error("Invalid prim type. Must be a UsdPrim, SdfPath, or std::string.");
                 }),
             "Constructor")
        .def_property_readonly("path", &PhysicsScene::getPath, "Path to the USD Physics Scene prim");

    m.def("get_physics_scene_paths",
          [](std::optional<uint64_t> stageId)
          {
              if (stageId.has_value())
              {
                  return getPhysicsScenePaths(stageId.value());
              }
              return getPhysicsScenePaths();
          });

    // carb interface
    carb::defineInterfaceClass<ISimulationManager>(
        m, "ISimulationManager", "acquire_simulation_manager_interface", "release_simulation_manager_interface")
        .def("register_deletion_callback", &ISimulationManager::registerDeletionCallback,
             R"pbdoc(
                Register a callback for deletion events.

                Args:
                    callback: Function to be called when an object is deleted. Takes a string path parameter.

                Returns:
                    int: Unique identifier for the registered callback.
             )pbdoc")
        .def("register_physics_scene_addition_callback", &ISimulationManager::registerPhysicsSceneAdditionCallback,
             R"pbdoc(
                Register a callback for physics scene addition events.

                Args:
                    callback: Function to be called when a physics scene is added. Takes a string path parameter.

                Returns:
                    int: Unique identifier for the registered callback.
             )pbdoc")
        .def("deregister_callback", &ISimulationManager::deregisterCallback,
             R"pbdoc(
                Deregister a previously registered callback.

                Args:
                    callback_id: The unique identifier of the callback to deregister.

                Returns:
                    bool: True if callback was successfully deregistered, False otherwise.
             )pbdoc")
        .def("reset", &ISimulationManager::reset,
             R"pbdoc(
                Reset the simulation manager to its initial state.

                Calls all registered deletion callbacks with the root path ('/'),
                clears all registered callbacks, clears the physics scenes list,
                and resets the callback iterator to 0.
             )pbdoc")
        .def("cleanup_invalid_physics_scenes", &ISimulationManager::cleanupInvalidPhysicsScenes,
             R"pbdoc(
                Remove any tracked physics scenes with invalid prims.

                Iterates through the internally tracked physics scenes and removes any
                whose underlying USD prim is no longer valid. This handles cases where
                physics scene prims become invalid without triggering USD notices
                (e.g., layer removal operations). Also triggers deletion callbacks for
                any removed physics scenes.

                Returns:
                    list[str]: The paths of physics scenes that were removed.
             )pbdoc")
        .def("set_callback_iter", &ISimulationManager::setCallbackIter,
             R"pbdoc(
                Set the callback iteration counter.

                Args:
                    val: New value for the callback iteration counter.
             )pbdoc")
        .def("get_callback_iter", &ISimulationManager::getCallbackIter,
             R"pbdoc(
                Get the current callback iteration counter.

                Returns:
                    int: The current callback iteration counter value.
             )pbdoc")
        .def("enable_usd_notice_handler", &ISimulationManager::enableUsdNoticeHandler,
             R"pbdoc(
                Enable or disable the USD notice handler.

                Args:
                    flag: True to enable the handler, False to disable.
             )pbdoc")
        .def("enable_fabric_usd_notice_handler", &ISimulationManager::enableFabricUsdNoticeHandler,
             R"pbdoc(
                Enable or disable the USD notice handler for a specific fabric stage.

                Args:
                    stage_id: ID of the fabric stage.
                    flag: True to enable the handler, False to disable.
             )pbdoc")
        .def("is_fabric_usd_notice_handler_enabled", &ISimulationManager::isFabricUsdNoticeHandlerEnabled,
             R"pbdoc(
                Check if the USD notice handler is enabled for a specific fabric stage.

                Args:
                    stage_id: ID of the fabric stage to check.

                Returns:
                    bool: True if the handler is enabled for the stage, False otherwise.
             )pbdoc")
        .def("get_simulation_time", &ISimulationManager::getSimulationTime,
             R"pbdoc(
                Get the current simulation time.

                Returns:
                    double: The current simulation time.
             )pbdoc")
        .def("get_simulation_time_monotonic", &ISimulationManager::getSimulationTimeMonotonic,
             R"pbdoc(
                Get the current simulation time. This time does not reset when the simulation is stopped.

                Returns:
                    double: The current simulation time.
             )pbdoc")
        .def("get_system_time", &ISimulationManager::getSystemTime,
             R"pbdoc(
                Get the current system time.

                Returns:
                    double: The current system time.
             )pbdoc")
        .def("get_current_time", &ISimulationManager::getCurrentTime,
             R"pbdoc(
                Get the current frame time from best available source.
                
                This returns the current frame time using the same priority order as TimeSampleStorage:
                1. StageReaderWriter's getFrameTime() - provides FSD frame time
                2. Timeline interface - fallback for animation/UI time
                
                This is useful for testing to track exact frame times being written to storage.

                Returns:
                    RationalTime: Current rational time or kInvalidRationalTime if unavailable.
             )pbdoc")
        .def("get_num_physics_steps", &ISimulationManager::getNumPhysicsSteps,
             R"pbdoc(
                Get the current physics step count.

                Returns:
                    int: The current physics step count.
             )pbdoc")
        .def("is_simulating", &ISimulationManager::isSimulating,
             R"pbdoc(
                Get the current simulation pause state.

                Returns:
                    bool: True if the simulation is paused, False otherwise.
             )pbdoc")
        .def("is_paused", &ISimulationManager::isPaused,
             R"pbdoc(
                Get the current simulation pause state.

                Returns:
                    bool: True if the simulation is paused, False otherwise.
             )pbdoc")
        .def("get_simulation_time_at_time", &ISimulationManager::getSimulationTimeAtTime,
             R"pbdoc(
                Gets simulation time in seconds at a specific rational time.

                Args:
                    time (omni.fabric.RationalTime): The rational time to query.

                Returns:
                    float: The simulation time in seconds at the specified time.
             )pbdoc")
        .def(
            "get_simulation_time_at_time",
            [](ISimulationManager* iface, const std::tuple<int64_t, uint64_t>& timeTuple)
            {
                int64_t numerator = std::get<0>(timeTuple);
                uint64_t denominator = std::get<1>(timeTuple);
                omni::fabric::RationalTime time(numerator, denominator);
                return iface->getSimulationTimeAtTime(time);
            },
            R"pbdoc(
                Gets simulation time in seconds at a specific rational time.

                Args:
                    time (tuple): A tuple of (numerator, denominator) representing the rational time.

                Returns:
                    float: The simulation time in seconds at the specified time.
             )pbdoc")
        .def("get_simulation_time_monotonic_at_time", &ISimulationManager::getSimulationTimeMonotonicAtTime,
             R"pbdoc(
                Gets monotonic simulation time in seconds at a specific rational time.

                Args:
                    time (omni.fabric.RationalTime): The rational time to query.

                Returns:
                    float: The monotonic simulation time in seconds at the specified time.
             )pbdoc")
        .def(
            "get_simulation_time_monotonic_at_time",
            [](ISimulationManager* iface, const std::tuple<int64_t, uint64_t>& timeTuple)
            {
                int64_t numerator = std::get<0>(timeTuple);
                uint64_t denominator = std::get<1>(timeTuple);
                omni::fabric::RationalTime time(numerator, denominator);
                return iface->getSimulationTimeMonotonicAtTime(time);
            },
            R"pbdoc(
                Gets monotonic simulation time in seconds at a specific rational time.

                Args:
                    time (tuple): A tuple of (numerator, denominator) representing the rational time.

                Returns:
                    float: The monotonic simulation time in seconds at the specified time.
             )pbdoc")
        .def("get_system_time_at_time", &ISimulationManager::getSystemTimeAtTime,
             R"pbdoc(
                Gets system (real-world) time in seconds at a specific rational time.

                Args:
                    time (omni.fabric.RationalTime): The rational time to query.

                Returns:
                    float: The system time in seconds at the specified time.
             )pbdoc")
        .def(
            "get_system_time_at_time",
            [](ISimulationManager* iface, const std::tuple<int64_t, uint64_t>& timeTuple)
            {
                int64_t numerator = std::get<0>(timeTuple);
                uint64_t denominator = std::get<1>(timeTuple);
                omni::fabric::RationalTime time(numerator, denominator);
                return iface->getSystemTimeAtTime(time);
            },
            R"pbdoc(
                Gets system (real-world) time in seconds at a specific rational time.

                Args:
                    time (tuple): A tuple of (numerator, denominator) representing the rational time.

                Returns:
                    float: The system time in seconds at the specified time.
             )pbdoc")
        .def("get_all_samples", &ISimulationManager::getAllSamples,
             R"pbdoc(
                Get all stored samples for testing and validation.
                
                Returns:
                    list[TimeSampleEntry]: List of sample entries with structured data.
                    Each entry has: 
                        - time (RationalTime): rational time of this sample
                        - data (TimeData): time data with sim_time, sim_time_monotonic, system_time
                        - valid (bool): whether this entry is valid
             )pbdoc")
        .def("get_sample_count", &ISimulationManager::getSampleCount,
             R"pbdoc(
                Get the count of stored samples.
                
                Returns:
                    int: Number of stored samples.
             )pbdoc")
        .def("log_statistics", &ISimulationManager::logStatistics,
             R"pbdoc(
                Log sample storage statistics for debugging.
             )pbdoc")
        .def("get_sample_range", &ISimulationManager::getSampleRange,
             R"pbdoc(
                Get the time range of stored samples.
                
                Returns:
                    tuple[RationalTime, RationalTime] or None: Pair of (earliest_time, latest_time) or None if empty.
                    Each time is a RationalTime object with numerator, denominator, and to_float() method.
             )pbdoc")
        .def("get_buffer_capacity", &ISimulationManager::getBufferCapacity,
             R"pbdoc(
                Get the maximum buffer capacity for time sample storage.
                
                Returns:
                    int: Maximum number of samples that can be stored in the circular buffer.
             )pbdoc");
}

} // namespace

// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/BindingsPythonUtils.h>
#include <carb/BindingsUtils.h>

#include <isaacsim/robot/surface_gripper/ISurfaceGripper.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>

CARB_BINDINGS("isaacsim.robot.surface_gripper.python")


namespace
{

enum class GripperStatus
{
    Open, // NOLINT(readability-identifier-naming) Python API
    Closing, // NOLINT(readability-identifier-naming) Python API
    Closed, // NOLINT(readability-identifier-naming) Python API
};

namespace py = pybind11;

template <typename InterfaceType>
py::class_<InterfaceType> defineInterfaceClass(py::module& m,
                                               const char* className,
                                               const char* acquireFuncName,
                                               const char* releaseFuncName = nullptr,
                                               const char* docString = "")
{
    m.def(
        acquireFuncName,
        [](const char* pluginName, const char* libraryPath)
        {
            return libraryPath ? carb::acquireInterfaceFromLibraryForBindings<InterfaceType>(libraryPath) :
                                 carb::acquireInterfaceForBindings<InterfaceType>(pluginName);
        },
        py::arg("plugin_name") = nullptr, py::arg("library_path") = nullptr, py::return_value_policy::reference,
        "Acquire Surface Gripper interface. This is the base object that all of the Surface Gripper functions are defined on");

    if (releaseFuncName)
    {
        m.def(
            releaseFuncName, [](InterfaceType* iface) { carb::getFramework()->releaseInterface(iface); },
            "Release Surface Gripper interface. Generally this does not need to be called, the Surface Gripper interface is released on extension shutdown");
    }

    return py::class_<InterfaceType>(m, className, docString);
}


PYBIND11_MODULE(_surface_gripper, m)
{
    using namespace carb;
    using namespace isaacsim::robot::surface_gripper;
    // We use carb data types, must import bindings for them
    auto carbModule = py::module::import("carb");

    // Expose GripperStatus enum to Python
    py::enum_<GripperStatus>(m, "GripperStatus", py::arithmetic(), "Enumeration of surface gripper statuses.")
        .value("Open", GripperStatus::Open, "Gripper is open.")
        .value("Closed", GripperStatus::Closed, "Gripper is closed.")
        .value("Closing", GripperStatus::Closing, "Gripper is in the process of closing.")
        .export_values()
        .attr("__module__") = "_surface_gripper";

    defineInterfaceClass<SurfaceGripperInterface>(
        m, "SurfaceGripperInterface", "acquire_surface_gripper_interface", "release_surface_gripper_interface")
        .def(
            "set_gripper_action",
            [](const SurfaceGripperInterface* iface, const char* primPath, const float action) -> bool
            { return iface ? iface->setGripperAction(primPath, action) : false; },
            py::arg("prim_path"), py::arg("action"),
            R"doc(
                Sets the action of a surface gripper.
                
                Values less than -0.3 will open the gripper, values greater than 0.3
                will close the gripper, and values in between have no effect.
                
                Args:
                    prim_path (str): USD path to the gripper.
                    action (float): Action value, typically in range [-1.0, 1.0].
                    
                Returns:
                    bool: True if successful, False otherwise.
            )doc")
        .def(
            "open_gripper",
            [](const SurfaceGripperInterface* iface, const char* primPath) -> bool
            { return iface ? iface->openGripper(primPath) : false; },
            py::arg("prim_path"),
            R"doc(
                Opens/releases a surface gripper.
                
                Commands the specified gripper to release any held objects and return
                to the open state.
                
                Args:
                    prim_path (str): USD path to the gripper.
                    
                Returns:
                    bool: True if successful, False otherwise.
            )doc")
        .def(
            "close_gripper",
            [](const SurfaceGripperInterface* iface, const char* primPath) -> bool
            { return iface ? iface->closeGripper(primPath) : false; },
            py::arg("prim_path"),
            R"doc(
                Closes/activates a surface gripper.
                
                Commands the specified gripper to attempt to grip objects in contact
                with its surface.
                
                Args:
                    prim_path (str): USD path to the gripper.
                    
                Returns:
                    bool: True if successful, False otherwise.
            )doc")
        .def(
            "get_gripper_status",
            [](const SurfaceGripperInterface* iface, const char* primPath) -> GripperStatus
            {
                if (!iface)
                {
                    return GripperStatus::Open;
                }
                return static_cast<GripperStatus>(iface->getGripperStatus(primPath));
            },
            py::arg("prim_path"),
            R"doc(
                Gets the status of a surface gripper.
                
                Args:
                    prim_path (str): USD path to the gripper.
                    
                Returns:
                    GripperStatus: Current status (Open, Closed, or Closing).
            )doc")
        .def(
            "set_write_to_usd",
            [](const SurfaceGripperInterface* iface, bool writeToUsd) -> bool
            { return iface ? iface->setWriteToUsd(writeToUsd) : false; },
            py::arg("write_to_usd"),
            R"doc(
                Sets whether to write gripper state to USD.
                
                Controls whether gripper state changes are persisted to the USD stage
                or maintained only in memory for improved performance.
                
                Args:
                    write_to_usd (bool): True to write state to USD, False to keep in memory only.
                    
                Returns:
                    bool: True if successful, False otherwise.
            )doc")
        // .def(
        //     "get_all_grippers",
        //     [](const SurfaceGripperInterface* iface) -> std::vector<std::string>
        //     {
        //         return iface ? iface->getAllGrippers() : std::vector<std::string>();
        //     },
        //     "Gets a list of all surface grippers in the scene.")
        .def(
            "get_gripped_objects",
            [](const SurfaceGripperInterface* iface, const char* primPath) -> std::vector<std::string>
            { return iface ? iface->getGrippedObjects(primPath) : std::vector<std::string>(); },
            py::arg("prim_path"),
            R"doc(
                Gets a list of objects currently gripped.
                
                Returns the USD paths of all objects that are currently held by the
                specified gripper.
                
                Args:
                    prim_path (str): USD path to the gripper.
                    
                Returns:
                    list[str]: List of USD paths for all gripped objects, empty if none.
            )doc")
        .def(
            "get_gripper_status_batch",
            [](const SurfaceGripperInterface* iface, const std::vector<std::string>& primPaths) -> std::vector<int>
            {
                if (!iface)
                {
                    return {};
                }
                std::vector<const char*> cpaths;
                cpaths.reserve(primPaths.size());
                for (const auto& p : primPaths)
                {
                    cpaths.push_back(p.c_str());
                }
                return iface->getGripperStatusBatch(cpaths.data(), cpaths.size());
            },
            py::arg("prim_paths"),
            R"doc(
                Gets statuses for multiple surface grippers in parallel.
                
                Batch operation that queries the status of multiple grippers efficiently.
                
                Args:
                    prim_paths (list[str]): List of USD paths for grippers to query.
                    
                Returns:
                    list[int]: List of status codes corresponding to each gripper path.
            )doc")
        .def(
            "open_gripper_batch",
            [](const SurfaceGripperInterface* iface, const std::vector<std::string>& primPaths) -> std::vector<bool>
            {
                if (!iface)
                {
                    return {};
                }
                std::vector<const char*> cpaths;
                cpaths.reserve(primPaths.size());
                for (const auto& p : primPaths)
                {
                    cpaths.push_back(p.c_str());
                }
                return iface->openGripperBatch(cpaths.data(), cpaths.size());
            },
            py::arg("prim_paths"),
            R"doc(
                Opens multiple surface grippers in parallel.
                
                Batch operation that opens multiple grippers efficiently.
                
                Args:
                    prim_paths (list[str]): List of USD paths for grippers to open.
                    
                Returns:
                    list[bool]: List of success flags corresponding to each gripper path.
            )doc")
        .def(
            "close_gripper_batch",
            [](const SurfaceGripperInterface* iface, const std::vector<std::string>& primPaths) -> std::vector<bool>
            {
                if (!iface)
                {
                    return {};
                }
                std::vector<const char*> cpaths;
                cpaths.reserve(primPaths.size());
                for (const auto& p : primPaths)
                {
                    cpaths.push_back(p.c_str());
                }
                return iface->closeGripperBatch(cpaths.data(), cpaths.size());
            },
            py::arg("prim_paths"),
            R"doc(
                Closes multiple surface grippers in parallel.
                
                Batch operation that closes multiple grippers efficiently.
                
                Args:
                    prim_paths (list[str]): List of USD paths for grippers to close.
                    
                Returns:
                    list[bool]: List of success flags corresponding to each gripper path.
            )doc")
        .def(
            "set_gripper_action_batch",
            [](const SurfaceGripperInterface* iface, const std::vector<std::string>& primPaths,
               const std::vector<float>& actions) -> std::vector<bool>
            {
                if (!iface || primPaths.size() != actions.size())
                {
                    return {};
                }
                std::vector<const char*> cpaths;
                cpaths.reserve(primPaths.size());
                for (const auto& p : primPaths)
                {
                    cpaths.push_back(p.c_str());
                }
                return iface->setGripperActionBatch(cpaths.data(), actions.data(), actions.size());
            },
            py::arg("prim_paths"), py::arg("actions"),
            R"doc(
                Sets actions for multiple surface grippers in parallel.
                
                Batch operation that sets gripper actions for multiple grippers efficiently.
                
                Args:
                    prim_paths (list[str]): List of USD paths for grippers to control.
                    actions (list[float]): List of action values corresponding to each gripper.
                    
                Returns:
                    list[bool]: List of success flags corresponding to each gripper path.
                    
                Note:
                    The prim_paths and actions lists must have the same length.
            )doc")
        .def(
            "get_gripped_objects_batch",
            [](const SurfaceGripperInterface* iface,
               const std::vector<std::string>& primPaths) -> std::vector<std::vector<std::string>>
            {
                if (!iface)
                {
                    return {};
                }
                std::vector<const char*> cpaths;
                cpaths.reserve(primPaths.size());
                for (const auto& p : primPaths)
                {
                    cpaths.push_back(p.c_str());
                }
                return iface->getGrippedObjectsBatch(cpaths.data(), cpaths.size());
            },
            py::arg("prim_paths"),
            R"doc(
                Gets gripped objects for multiple surface grippers in parallel.
                
                Batch operation that retrieves the list of gripped objects for
                multiple grippers efficiently.
                
                Args:
                    prim_paths (list[str]): List of USD paths for grippers to query.
                    
                Returns:
                    list[list[str]]: List of lists containing USD paths of gripped objects
                                     for each gripper.
            )doc");
}
}

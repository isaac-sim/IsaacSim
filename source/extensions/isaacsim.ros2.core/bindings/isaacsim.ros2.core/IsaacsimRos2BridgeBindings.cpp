// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

/** @file
 * @brief Python bindings for the ROS 2 core interface
 * @details
 * This file provides Python bindings for the ROS 2 core functionality in Isaac Sim.
 * It exposes the core ROS 2 core interface to Python, allowing Python scripts to
 * interact with ROS 2 functionality through the bridge.
 */

#include <carb/BindingsPythonUtils.h>

#include <isaacsim/ros2/core/IRos2Core.h>
#include <pybind11/numpy.h>

CARB_BINDINGS("isaacsim.ros2.core.python")

namespace
{

/**
 * @brief Python module definition for ROS 2 core bindings
 * @details
 * Creates and configures the Python module that exposes ROS 2 core functionality.
 * The module provides access to the Ros2Bridge interface and its methods for
 * managing ROS 2 communication in Isaac Sim.
 *
 * @param[in,out] m Python module object to configure
 */
PYBIND11_MODULE(_ros2_core, m)
{
    using namespace carb;
    using namespace isaacsim::ros2::core;

    m.doc() = R"doc(
        Isaac Sim ROS 2 Bridge Interface
        
        This module provides Python bindings for the ROS 2 core functionality in Isaac Sim.
        It enables communication between Isaac Sim and ROS 2, allowing simulation components
        to interact with ROS 2 nodes, topics, services, and actions.
        
        The bridge supports creating nodes, publishers, subscribers, service clients, service
        servers, action clients, and action servers directly from Python scripts in Isaac Sim.
    )doc";

    {
        auto cls = defineInterfaceClass<Ros2Bridge>(
            m, "Ros2Core", "acquire_ros2_core_interface", "release_ros2_core_interface");

        cls.doc() = R"doc(
            Main interface class for ROS 2 core functionality.
            
            This class provides the core interface for interacting with ROS 2 functionality
            within Isaac Sim. It manages the lifecycle of ROS 2 context handlers and provides
            factory access for creating ROS 2 related objects.
            
            The Ros2Bridge serves as the entry point for all ROS 2 operations in Isaac Sim,
            including node creation, message publishing, and service handling.
        )doc";

        cls.def("get_startup_status", wrapInterfaceFunction(&Ros2Bridge::getStartupStatus),
                R"doc(
                    Checks the initialization status of the ROS 2 core.
                    
                    This method verifies if both the factory and context handler objects have
                    been properly instantiated. These objects are created when the Ros2Bridge
                    interface is first acquired after the plugin is loaded.
                    
                    Returns:
                        bool: True if both factory and context handler are successfully 
                        instantiated, False otherwise.
                        
                    Note:
                        This method should be called before attempting to use any other
                        methods of this interface. Using other methods when this returns
                        False may result in undefined behavior.
                )doc");
    }
}

} // namespace anonymous

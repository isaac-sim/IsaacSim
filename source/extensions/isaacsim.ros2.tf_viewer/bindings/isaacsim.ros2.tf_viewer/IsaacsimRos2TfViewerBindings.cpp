// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/ros2/tf_viewer/ITransformListener.h>

CARB_BINDINGS("isaacsim.ros2.tf_viewer")


namespace py = pybind11;

PYBIND11_MODULE(_transform_listener, m)
{
    using namespace isaacsim::ros2::tf_viewer;

    m.doc() = R"doc(
    ROS 2 Transform Listener module for Isaac Sim.

    This module provides an interface to ROS 2 transform (tf) functionality,
    allowing Isaac Sim to listen to and process transform messages from ROS 2.
    It enables visualization and utilization of coordinate frames broadcast
    through the ROS 2 tf2 system.
    
    The module allows applications to:
    - Connect to a ROS 2 system
    - Subscribe to transform messages
    - Process and retrieve transformation data
    - Visualize transform hierarchies within Isaac Sim
    )doc";

    carb::defineInterfaceClass<ITransformListener>(
        m, "ITransformListener", "acquire_transform_listener_interface", "release_transform_listener_interface")
        .def("initialize", &ITransformListener::initialize,
             R"doc(
             Initialize the transform listener with the specified ROS 2 distribution.

             Args:
                 ros_distro (str): The ROS 2 distribution name to use (e.g., "humble", "foxy").

             Returns:
                 bool: True if initialization was successful, False otherwise.
             )doc",
             py::arg("ros_distro"))
        .def("finalize", &ITransformListener::finalize,
             R"doc(
             Finalize and clean up the transform listener.

             Unsubscribes from tf topics and cleans up ROS 2 resources.
             )doc")
        .def("spin", &ITransformListener::spin,
             R"doc(
             Process incoming transform messages.
             
             This method should be called regularly to process new transform data
             from ROS 2.

             Returns:
                 bool: True if transforms were processed successfully, False otherwise.
             )doc")
        .def("reset", &ITransformListener::reset,
             R"doc(
             Reset the transform buffer.
             
             Clears all stored transform data.
             )doc")
        .def(
            "get_transforms",
            [](ITransformListener& m, std::string rootFrame)
            {
                m.computeTransforms(rootFrame);
                return std::make_tuple(m.getFrames(), m.getTransforms(), m.getRelations());
            },
            R"doc(
             Get all transforms relative to the specified root frame.
             
             This method computes transforms for all frames relative to the specified
             root frame and returns the frame data, transform data, and frame relationships.

             Args:
                 root_frame (str): The reference frame ID to compute transforms against.

             Returns:
                 tuple: A tuple containing three elements:
                     - list of str: Available frame IDs
                     - dict: Map of frame IDs to transforms (translation: xyz, rotation: xyzw)
                     - list of tuple: Parent-child relationships between frames
             )doc",
            py::arg("root_frame"));
}

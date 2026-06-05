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

#include <carb/BindingsPythonUtils.h>
#include <carb/logging/Log.h>

#include <isaacsim/ros2/nodes/IRos2Nodes.h>
#include <isaacsim/ros2/nodes/SrtxPublisherFactory.h>
#include <pybind11/stl.h>

CARB_BINDINGS("isaacsim.ros2.nodes.python")

namespace
{

namespace py = pybind11;

py::capsule wrapDescriptorAsCapsule(isaacsim::ros2::nodes::SrtxFrameCallbackDescriptor* desc)
{
    return py::capsule(desc, "SrtxFrameCallbackDescriptor",
                       [](PyObject* cap)
                       {
                           auto* d = static_cast<isaacsim::ros2::nodes::SrtxFrameCallbackDescriptor*>(
                               PyCapsule_GetPointer(cap, "SrtxFrameCallbackDescriptor"));
                           if (d)
                           {
                               delete d;
                           }
                       });
}

PYBIND11_MODULE(_ros2_nodes, m)
{
    // clang-format off
    using namespace carb;
    using namespace isaacsim::ros2::nodes;

    m.doc() = R"pbdoc(
        Internal interface that is automatically called when the extension is loaded so that Omnigraph nodes are registered.

        Example:

            # import  isaacsim.ros2.nodes.bindings._ros2_nodes as _ros2_nodes

            # Acquire the interface
            interface = _ros2_nodes.acquire_interface()

            # Use the interface
            # ...

            # Release the interface
            _ros2_nodes.release_interface(interface)
    )pbdoc";

    defineInterfaceClass<IRos2Nodes>(
        m,
        "IRos2Nodes",
        "acquire_interface",
        "release_interface"
    );

    m.def("create_image_publisher_capsule",
          [](const std::string& topicName,
             const std::string& frameId,
             const std::string& nodeNamespace,
             uint64_t queueSize,
             const std::string& qosProfile) -> py::capsule
          {
              auto* desc = createImagePublisherDescriptor(
                  topicName, frameId, nodeNamespace, queueSize, qosProfile);
              if (!desc)
              {
                  throw std::runtime_error("Failed to initialize Ros2SrtxImagePublisher");
              }
              return wrapDescriptorAsCapsule(desc);
          },
          py::arg("topic_name"),
          py::arg("frame_id"),
          py::arg("node_namespace"),
          py::arg("queue_size"),
          py::arg("qos_profile") = "",
          R"pbdoc(
              Create a ROS 2 Image publisher and return a PyCapsule wrapping
              the C-ABI callback descriptor.

              The capsule is named "SrtxFrameCallbackDescriptor" and is intended
              to be passed to omni.replicator.srtx's register_frame_callback().

              Args:
                  topic_name: ROS 2 topic name to publish on.
                  frame_id: TF frame_id for the published message header.
                  node_namespace: ROS 2 node namespace.
                  queue_size: Publisher queue depth.
                  qos_profile: JSON-encoded QoS profile (empty string for defaults).

              Returns:
                  PyCapsule containing the callback descriptor.
          )pbdoc");

    m.def("create_camera_info_publisher_capsule",
          [](const std::string& topicName,
             const std::string& frameId,
             const std::string& nodeNamespace,
             uint64_t queueSize,
             const std::string& qosProfile,
             uint32_t width,
             uint32_t height,
             const std::string& distortionModel,
             const std::vector<double>& k,
             const std::vector<double>& r,
             const std::vector<double>& p,
             const std::vector<double>& d) -> py::capsule
          {
              auto* desc = createCameraInfoPublisherDescriptor(
                  topicName, frameId, nodeNamespace, queueSize, qosProfile, width, height, distortionModel, k, r, p, d);
              if (!desc)
              {
                  throw std::runtime_error("Failed to initialize Ros2SrtxCameraInfoPublisher");
              }
              return wrapDescriptorAsCapsule(desc);
          },
          py::arg("topic_name"),
          py::arg("frame_id"),
          py::arg("node_namespace"),
          py::arg("queue_size"),
          py::arg("qos_profile"),
          py::arg("width"),
          py::arg("height"),
          py::arg("distortion_model"),
          py::arg("k"),
          py::arg("r"),
          py::arg("p"),
          py::arg("d"),
          R"pbdoc(
              Create a ROS 2 CameraInfo publisher and return a PyCapsule wrapping
              the C-ABI callback descriptor.

              The capsule is named "SrtxFrameCallbackDescriptor" and is intended
              to be passed to omni.replicator.srtx's register_frame_callback().
          )pbdoc");

    m.def("create_lidar_publisher_capsule",
          [](const std::string& topicName,
             const std::string& frameId,
             const std::string& nodeNamespace,
             uint64_t queueSize,
             const std::string& qosProfile) -> py::capsule
          {
              auto* desc = createLidarPublisherDescriptor(
                  topicName, frameId, nodeNamespace, queueSize, qosProfile);
              if (!desc)
              {
                  throw std::runtime_error("Failed to initialize Ros2SrtxLidarPublisher");
              }
              return wrapDescriptorAsCapsule(desc);
          },
          py::arg("topic_name"),
          py::arg("frame_id"),
          py::arg("node_namespace"),
          py::arg("queue_size"),
          py::arg("qos_profile") = "",
          R"pbdoc(
              Create a ROS 2 PointCloud2 (lidar) publisher and return a PyCapsule
              wrapping the C-ABI callback descriptor.

              The capsule is named "SrtxFrameCallbackDescriptor" and is intended
              to be passed to omni.replicator.srtx's register_frame_callback().

              Args:
                  topic_name: ROS 2 topic name to publish on.
                  frame_id: TF frame_id for the published message header.
                  node_namespace: ROS 2 node namespace.
                  queue_size: Publisher queue depth.
                  qos_profile: JSON-encoded QoS profile (empty string for defaults).

              Returns:
                  PyCapsule containing the callback descriptor.
          )pbdoc");

    m.def("create_laser_scan_publisher_capsule",
          [](const std::string& topicName,
             const std::string& frameId,
             const std::string& nodeNamespace,
             uint64_t queueSize,
             const std::string& qosProfile,
             float azimuthRangeStart,
             float azimuthRangeEnd,
             float depthRangeMin,
             float depthRangeMax,
             float rotationRate,
             float horizontalResolution,
             float horizontalFov) -> py::capsule
          {
              auto* desc = createLaserScanPublisherDescriptor(
                  topicName, frameId, nodeNamespace, queueSize, qosProfile,
                  azimuthRangeStart, azimuthRangeEnd, depthRangeMin, depthRangeMax,
                  rotationRate, horizontalResolution, horizontalFov);
              if (!desc)
              {
                  throw std::runtime_error("Failed to initialize Ros2SrtxLaserScanPublisher");
              }
              return wrapDescriptorAsCapsule(desc);
          },
          py::arg("topic_name"),
          py::arg("frame_id"),
          py::arg("node_namespace"),
          py::arg("queue_size"),
          py::arg("qos_profile") = "",
          py::arg("azimuth_range_start") = -180.0f,
          py::arg("azimuth_range_end") = 180.0f,
          py::arg("depth_range_min") = 0.0f,
          py::arg("depth_range_max") = 100.0f,
          py::arg("rotation_rate") = 20.0f,
          py::arg("horizontal_resolution") = 1.0f,
          py::arg("horizontal_fov") = 360.0f,
          R"pbdoc(
              Create a ROS 2 LaserScan publisher and return a PyCapsule
              wrapping the C-ABI callback descriptor.

              The capsule is named "SrtxFrameCallbackDescriptor" and is intended
              to be passed to omni.replicator.srtx's register_frame_callback().

              Args:
                  topic_name: ROS 2 topic name to publish on.
                  frame_id: TF frame_id for the published message header.
                  node_namespace: ROS 2 node namespace.
                  queue_size: Publisher queue depth.
                  qos_profile: JSON-encoded QoS profile (empty string for defaults).
                  azimuth_range_start: Scan start angle in degrees.
                  azimuth_range_end: Scan end angle in degrees.
                  depth_range_min: Minimum range in meters.
                  depth_range_max: Maximum range in meters.
                  rotation_rate: Scan frequency in Hz.
                  horizontal_resolution: Angular resolution in degrees.
                  horizontal_fov: Horizontal field of view in degrees.

              Returns:
                  PyCapsule containing the callback descriptor.
          )pbdoc");
}
} // namespace anonymous

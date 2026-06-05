// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cstdint>
#include <string>
#include <vector>

#if defined(_MSC_VER)
#    define ISAACSIM_ROS2_NODES_API __declspec(dllexport)
#else
#    define ISAACSIM_ROS2_NODES_API __attribute__((visibility("default")))
#endif

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/// C-ABI callback signature matching omni::replicator::srtx::SrtxFrameCallbackFn.
using SrtxFrameCallbackFn = void (*)(void* userData,
                                     const uint8_t* data,
                                     size_t dataSize,
                                     const uint32_t* shape,
                                     size_t shapeLen,
                                     const char* format,
                                     const char* outputPath,
                                     double timestamp);

/// Descriptor passed through PyCapsule between isaacsim.ros2.nodes (creator)
/// and omni.replicator.srtx (consumer).
struct SrtxFrameCallbackDescriptor
{
    SrtxFrameCallbackFn fn; ///< Function pointer called per frame.
    void* userData; ///< Opaque pointer forwarded to @c fn.
    void (*destructor)(void*); ///< Cleanup function for @c userData (may be null).
};

/// Create an image publisher callback descriptor.
/// Returns a heap-allocated descriptor (caller owns it), or nullptr on failure.
/// The descriptor's destructor will clean up the publisher when called.
ISAACSIM_ROS2_NODES_API SrtxFrameCallbackDescriptor* createImagePublisherDescriptor(const std::string& topicName,
                                                                                    const std::string& frameId,
                                                                                    const std::string& nodeNamespace,
                                                                                    uint64_t queueSize,
                                                                                    const std::string& qosProfile);

/// Create a camera info publisher callback descriptor.
/// Returns a heap-allocated descriptor (caller owns it), or nullptr on failure.
ISAACSIM_ROS2_NODES_API SrtxFrameCallbackDescriptor* createCameraInfoPublisherDescriptor(const std::string& topicName,
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
                                                                                         const std::vector<double>& d);

/// Create a lidar publisher callback descriptor.
/// Returns a heap-allocated descriptor (caller owns it), or nullptr on failure.
ISAACSIM_ROS2_NODES_API SrtxFrameCallbackDescriptor* createLidarPublisherDescriptor(const std::string& topicName,
                                                                                    const std::string& frameId,
                                                                                    const std::string& nodeNamespace,
                                                                                    uint64_t queueSize,
                                                                                    const std::string& qosProfile);

/// Create a laser scan publisher callback descriptor.
/// Returns a heap-allocated descriptor (caller owns it), or nullptr on failure.
/// The extra parameters provide scan metadata read from the lidar prim.
ISAACSIM_ROS2_NODES_API SrtxFrameCallbackDescriptor* createLaserScanPublisherDescriptor(const std::string& topicName,
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
                                                                                        float horizontalFov);

} // namespace nodes
} // namespace ros2
} // namespace isaacsim

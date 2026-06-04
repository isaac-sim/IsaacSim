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

#include "Ros2SrtxPublisher.h"

#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/// Publishes SRTX lidar data (kGeneralData / GenericModelOutput) as
/// sensor_msgs/msg/LaserScan to a ROS 2 topic.
class Ros2SrtxLaserScanPublisher : public Ros2SrtxPublisher
{
public:
    Ros2SrtxLaserScanPublisher() = default;
    ~Ros2SrtxLaserScanPublisher() override = default;

    Ros2SrtxLaserScanPublisher(const Ros2SrtxLaserScanPublisher&) = delete;
    Ros2SrtxLaserScanPublisher& operator=(const Ros2SrtxLaserScanPublisher&) = delete;

    /// Not used — call the overload with scan metadata instead.
    bool initialize(const std::string& topicName,
                    const std::string& frameId,
                    const std::string& nodeNamespace,
                    uint64_t queueSize,
                    const std::string& qosProfile) override;

    bool initialize(const std::string& topicName,
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

    void publishData(const uint8_t* data, size_t dataSize, double timestamp) override;

    /// C-ABI trampoline matching SrtxFrameCallbackFn.
    static void trampoline(void* userData,
                           const uint8_t* data,
                           size_t dataSize,
                           const uint32_t* shape,
                           size_t shapeLen,
                           const char* format,
                           const char* outputPath,
                           double timestamp);

    static void destroy(void* userData);

private:
    std::shared_ptr<isaacsim::ros2::core::Ros2LaserScanMessage> m_message;

    float m_azimuthRangeStart = -180.0f;
    float m_azimuthRangeEnd = 180.0f;
    float m_depthRangeMin = 0.0f;
    float m_depthRangeMax = 100.0f;
    float m_rotationRate = 20.0f;
    float m_horizontalResolution = 1.0f;
    float m_horizontalFov = 360.0f;

    size_t m_numOutputElements = 0;
};

} // namespace nodes
} // namespace ros2
} // namespace isaacsim

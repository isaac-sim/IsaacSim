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

#include <carb/Framework.h>
#include <carb/logging/Log.h>

#include <isaacsim/ros2/core/IRos2Core.h>
#include <isaacsim/ros2/core/Ros2Factory.h>
#include <isaacsim/ros2/core/Ros2Message.h>
#include <isaacsim/ros2/core/Ros2QoS.h>
#include <isaacsim/ros2/nodes/SrtxPublisherFactory.h>

#include <cstdint>
#include <memory>
#include <string>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/// Abstract base for SRTX-to-ROS 2 publishers.
class Ros2SrtxPublisher
{
public:
    Ros2SrtxPublisher() = default;
    virtual ~Ros2SrtxPublisher() = default;

    Ros2SrtxPublisher(const Ros2SrtxPublisher&) = delete;
    Ros2SrtxPublisher& operator=(const Ros2SrtxPublisher&) = delete;

    virtual bool initialize(const std::string& topicName,
                            const std::string& frameId,
                            const std::string& nodeNamespace,
                            uint64_t queueSize,
                            const std::string& qosProfile) = 0;

    virtual void publishImage(const uint8_t* data,
                              uint32_t width,
                              uint32_t height,
                              const std::string& encoding,
                              size_t bufferSize,
                              double timestamp)
    {
    }

    virtual void publishData(const uint8_t* data, size_t dataSize, double timestamp)
    {
    }

    bool isInitialized() const
    {
        return m_initialized;
    }

protected:
    isaacsim::ros2::core::Ros2Bridge* m_ros2Bridge = nullptr;
    isaacsim::ros2::core::Ros2Factory* m_factory = nullptr;
    std::shared_ptr<isaacsim::ros2::core::Ros2NodeHandle> m_nodeHandle;
    std::shared_ptr<isaacsim::ros2::core::Ros2Publisher> m_publisher;
    std::string m_frameId;
    bool m_initialized = false;
};

} // namespace nodes
} // namespace ros2
} // namespace isaacsim

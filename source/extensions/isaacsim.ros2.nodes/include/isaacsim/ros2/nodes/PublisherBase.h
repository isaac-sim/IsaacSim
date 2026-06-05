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

#include <isaacsim/ros2/core/IRos2Core.h>
#include <isaacsim/ros2/core/Ros2Factory.h>

#include <memory>
#include <string>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/// Common base for all standalone ROS 2 publishers in the library.
///
/// Handles ROS 2 node handle creation, publisher creation with QoS,
/// subscription-count gating, and teardown. Derived classes override
/// onCreateMessage() to create the specific message type and provide
/// typed publish methods.
class PublisherBase
{
public:
    PublisherBase();
    virtual ~PublisherBase();

    PublisherBase(const PublisherBase&) = delete;
    PublisherBase& operator=(const PublisherBase&) = delete;

    /// One-time ROS 2 setup. Calls onCreateMessage() to create the message,
    /// then creates the publisher. Safe to call multiple times (no-op after first).
    bool initialize(const std::string& nodeName,
                    const std::string& namespaceName,
                    const std::string& topicName,
                    const std::string& frameId,
                    uint64_t queueSize,
                    const std::string& qosProfile,
                    uint64_t context = 0);

    /// Publish the message that was previously filled by a prepare() call.
    /// Thread-safe: can be called from a background task after prepare() completes.
    void send();

    /// Release ROS 2 resources and reset the publisher to an uninitialized state.
    virtual void reset();

    /// Return true if the publisher has been successfully initialized.
    bool isInitialized() const
    {
        return m_publisher != nullptr;
    }

protected:
    /// Override to create the ROS 2 message. Return the type-support handle
    /// pointer (from message->getTypeSupportHandle()). Store the typed
    /// message in a member of the derived class.
    virtual const void* onCreateMessage() = 0;

    /// Override to return the concrete message pointer for send().
    virtual isaacsim::ros2::core::Ros2Message* getMessagePtr() = 0;

    /// Returns false when there are no subscribers and publish_without_verification is off.
    bool shouldPublish() const;

    isaacsim::ros2::core::Ros2Bridge* m_ros2Bridge = nullptr; ///< Cached bridge singleton.
    isaacsim::ros2::core::Ros2Factory* m_factory = nullptr; ///< Cached factory from the bridge.
    std::shared_ptr<isaacsim::ros2::core::Ros2ContextHandle>* m_contextHandle = nullptr; ///< ROS 2 context handle.
    std::shared_ptr<isaacsim::ros2::core::Ros2NodeHandle> m_nodeHandle; ///< ROS 2 node handle.
    std::shared_ptr<isaacsim::ros2::core::Ros2Publisher> m_publisher; ///< Underlying ROS 2 publisher.
    std::string m_frameId; ///< TF frame ID stamped into published messages.
    bool m_publishWithoutVerification = false; ///< When true, publish even with no subscribers.
};

} // namespace nodes
} // namespace ros2
} // namespace isaacsim

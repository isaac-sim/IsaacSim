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


#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/ros2/nodes/PublisherBase.h>

using namespace isaacsim::ros2::core;

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

PublisherBase::PublisherBase()
{
    m_ros2Bridge = carb::getCachedInterface<Ros2Bridge>();
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
    static constexpr char s_kSetting[] = "/exts/isaacsim.ros2.bridge/publish_without_verification";
    m_publishWithoutVerification = settings->getAsBool(s_kSetting);
    m_factory = m_ros2Bridge->getFactory();
}

PublisherBase::~PublisherBase()
{
    PublisherBase::reset();
}

bool PublisherBase::initialize(const std::string& nodeName,
                               const std::string& namespaceName,
                               const std::string& topicName,
                               const std::string& frameId,
                               uint64_t queueSize,
                               const std::string& qosProfile,
                               uint64_t context)
{
    if (m_publisher)
    {
        return true;
    }

    std::string sanitizedNodeName = nodeName;
    std::replace_if(
        sanitizedNodeName.begin(), sanitizedNodeName.end(), [](auto ch) { return !(::isalnum(ch) || ch == '_'); }, '_');

    if (!m_factory->validateNodeName(sanitizedNodeName))
    {
        CARB_LOG_ERROR("PublisherBase: invalid ROS2 node name '%s'", sanitizedNodeName.c_str());
        return false;
    }

    std::string ns = namespaceName;
    while (!ns.empty() && !std::isalnum(ns.front()))
    {
        ns.erase(ns.begin());
    }
    while (!ns.empty() && !std::isalnum(ns.back()))
    {
        ns.pop_back();
    }
    if (!ns.empty())
    {
        ns = "/" + ns;
    }

    if (context)
    {
        void* ptr = m_ros2Bridge->getHandle(context);
        if (!ptr)
        {
            CARB_LOG_ERROR("PublisherBase: ROS2 context handle not found");
            return false;
        }
        m_contextHandle = reinterpret_cast<std::shared_ptr<Ros2ContextHandle>*>(ptr);
    }
    else
    {
        m_contextHandle =
            reinterpret_cast<std::shared_ptr<Ros2ContextHandle>*>(m_ros2Bridge->getDefaultContextHandleAddr());
    }

    if (ns.empty() || !m_factory->validateNamespaceName(ns))
    {
        m_nodeHandle = m_factory->createNodeHandle(sanitizedNodeName.c_str(), "", m_contextHandle->get());
    }
    else
    {
        m_nodeHandle = m_factory->createNodeHandle(sanitizedNodeName.c_str(), ns.c_str(), m_contextHandle->get());
    }

    if (!m_nodeHandle || !m_nodeHandle->getNode())
    {
        CARB_LOG_ERROR("PublisherBase: failed to create ROS2 node handle");
        return false;
    }

    std::string fullTopicName = topicName;
    if (!topicName.empty())
    {
        std::string trimmedTopic = topicName;
        while (!trimmedTopic.empty() && !std::isalnum(trimmedTopic.front()))
        {
            trimmedTopic.erase(trimmedTopic.begin());
        }
        while (!trimmedTopic.empty() && !std::isalnum(trimmedTopic.back()))
        {
            trimmedTopic.pop_back();
        }
        fullTopicName = ns + "/" + trimmedTopic;
    }

    if (!m_factory->validateTopicName(fullTopicName))
    {
        CARB_LOG_ERROR("PublisherBase: invalid topic name '%s'", fullTopicName.c_str());
        return false;
    }

    const void* typeSupport = onCreateMessage();
    if (!typeSupport)
    {
        CARB_LOG_ERROR("PublisherBase: failed to create message");
        return false;
    }

    Ros2QoSProfile qos;
    if (qosProfile.empty())
    {
        qos.depth = queueSize;
    }
    else
    {
        if (!jsonToRos2QoSProfile(qos, qosProfile))
        {
            CARB_LOG_ERROR("PublisherBase: failed to parse QoS profile");
            return false;
        }
    }

    m_publisher = m_factory->createPublisher(m_nodeHandle.get(), fullTopicName.c_str(), typeSupport, qos);

    m_frameId = frameId;
    return m_publisher != nullptr;
}

bool PublisherBase::shouldPublish() const
{
    return m_publishWithoutVerification || (m_publisher && m_publisher->getSubscriptionCount() > 0);
}

void PublisherBase::send()
{
    auto* msg = getMessagePtr();
    if (m_publisher && msg)
    {
        m_publisher->publish(msg->getPtr());
    }
}

void PublisherBase::reset()
{
    m_publisher.reset();
    m_nodeHandle.reset();
}

} // namespace nodes
} // namespace ros2
} // namespace isaacsim

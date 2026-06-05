// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "Ros2Impl.h"

#include <isaacsim/ros2/core/Ros2Macros.h>
#include <rcl/rcl.h>

namespace isaacsim
{
namespace ros2
{
namespace core
{

Ros2PublisherImpl::Ros2PublisherImpl(Ros2NodeHandle* nodeHandle,
                                     const char* topicName,
                                     const void* typeSupport,
                                     const Ros2QoSProfile& qos)
    : m_nodeHandle(nodeHandle),
      m_publisher(std::shared_ptr<rcl_publisher_t>(new rcl_publisher_t,
                                                   [nodeHandle](rcl_publisher_t* pub)
                                                   {
                                                       rcl_ret_t ret = rcl_publisher_fini(
                                                           pub, static_cast<rcl_node_t*>(nodeHandle->getNode()));
                                                       if (RCL_RET_OK != ret)
                                                       {
                                                           RCL_ERROR_MSG(Ros2PublisherImpl, rcl_publisher_fini);
                                                       }
                                                       delete pub;
                                                   }))
{
    // Allocate memory for publisher
    // Init publisher
    (*m_publisher) = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t publisherOptions = rcl_publisher_get_default_options();
    publisherOptions.qos = Ros2QoSProfileConverter::convert(qos);
    rcl_ret_t rc = rcl_publisher_init(m_publisher.get(), static_cast<rcl_node_t*>(m_nodeHandle->getNode()),
                                      static_cast<const rosidl_message_type_support_t*>(typeSupport), topicName,
                                      &publisherOptions);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(Ros2PublisherImpl, rcl_publisher_init);
        m_publisher.reset();
        return;
    }
}

Ros2PublisherImpl::~Ros2PublisherImpl()
{
    m_publisher.reset();
    return;
}

void Ros2PublisherImpl::publish(const void* msg)
{
    rcl_ret_t rc = rcl_publish(m_publisher.get(), msg, nullptr);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(publish, rcl_publish);
    }
}

size_t Ros2PublisherImpl::getSubscriptionCount()
{
    size_t subscriptionCount = 0;
    rcl_ret_t rc = rcl_publisher_get_subscription_count(m_publisher.get(), &subscriptionCount);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(getSubscriptionCount, rcl_publisher_get_subscription_count);
    }
    return subscriptionCount;
}

} // namespace core
} // namespace ros2
} // namespace isaacsim

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

Ros2SubscriberImpl::Ros2SubscriberImpl(Ros2NodeHandle* nodeHandle,
                                       const char* topicName,
                                       const void* typeSupport,
                                       const Ros2QoSProfile& qos)
    : m_nodeHandle(nodeHandle),
      m_waitSetInitialized(false),
      m_subscription(std::shared_ptr<rcl_subscription_t>(
          new rcl_subscription_t,
          [nodeHandle](rcl_subscription_t* subscription)
          {
              // Intentionally capture node by copy so shared_ptr can be
              // transferred to copies
              rcl_ret_t ret = rcl_subscription_fini(subscription, static_cast<rcl_node_t*>(nodeHandle->getNode()));
              if (RCL_RET_OK != ret)
              {
                  RCL_ERROR_MSG(Ros2SubscriberImpl, rcl_subscription_fini);
              }
              delete subscription;
          }))
{
    (*m_subscription) = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscriptionOptions = rcl_subscription_get_default_options();
    subscriptionOptions.qos = Ros2QoSProfileConverter::convert(qos);

    rcl_ret_t rc = rcl_subscription_init(m_subscription.get(), static_cast<rcl_node_t*>(m_nodeHandle->getNode()),
                                         static_cast<const rosidl_message_type_support_t*>(typeSupport), topicName,
                                         &subscriptionOptions);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(Ros2SubscriberImpl, rcl_subscription_init);
        m_subscription.reset();
        return;
    }
}
Ros2SubscriberImpl::~Ros2SubscriberImpl()
{
    if (m_waitSetInitialized)
    {
        rcl_ret_t rc = rcl_wait_set_fini(&m_waitSet);
        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(~Ros2SubscriberImpl, rcl_wait_set_fini);
        }
        m_waitSetInitialized = false;
    }

    m_subscription.reset();
    return;
}

bool Ros2SubscriberImpl::spin(void* msg)
{
    if (!m_waitSetInitialized)
    {
        m_waitSet = rcl_get_zero_initialized_wait_set();
        rcl_ret_t rc = rcl_wait_set_init(&m_waitSet, 1, 0, 0, 0, 0, 0,
                                         static_cast<rcl_context_t*>(m_nodeHandle->getContextHandle()->getContext()),
                                         rcl_get_default_allocator());

        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(spin, rcl_wait_set_init);
            return false;
        }
        m_waitSetInitialized = true;
    }
    else
    {
        rcl_ret_t rc = rcl_wait_set_clear(&m_waitSet);
        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(spin, rcl_wait_set_clear);
            return false;
        }

        rc = rcl_wait_set_add_subscription(&m_waitSet, m_subscription.get(), nullptr);
        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(spin, rcl_wait_set_add_subscription);
            return false;
        }
        rc = rcl_wait(&m_waitSet, 0);
        CARB_LOG_WARN_ONCE("Subscriber created, check topic name and message type if not active");
        if (rc != RCL_RET_OK)
        {
            // This keeps printing an error if the publisher is not active.
            // Ideally only want to notify user once, when the subscription is created
            // RCL_WARN_MSG(spin, rcl_wait);
            return false;
        }
        if (m_waitSet.subscriptions[0])
        {
            rmw_message_info_t messageInfo;
            rcl_ret_t ret = rcl_take(m_subscription.get(), msg, &messageInfo, nullptr);
            if (ret != RCL_RET_OK)
            {
                RCL_ERROR_MSG(spin, rcl_take);
                return false;
            }
            return true;
        }
    }
    return false;
}

} // namespace core
} // namespace ros2
} // namespace isaacsim

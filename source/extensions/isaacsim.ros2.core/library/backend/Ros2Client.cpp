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
#include <rcl/client.h>
#include <rcl/rcl.h>

namespace isaacsim
{
namespace ros2
{
namespace core
{

Ros2ClientImpl::Ros2ClientImpl(Ros2NodeHandle* nodeHandle,
                               const char* serviceName,
                               const void* typeSupport,
                               const Ros2QoSProfile& qos)
    : m_nodeHandle(nodeHandle),
      m_waitSetInitialized(false),
      m_client(std::shared_ptr<rcl_client_t>(new rcl_client_t,
                                             [nodeHandle](rcl_client_t* client)
                                             {
                                                 // Intentionally capture node by copy so shared_ptr can be
                                                 // transferred to copies
                                                 rcl_ret_t ret = rcl_client_fini(
                                                     client, static_cast<rcl_node_t*>(nodeHandle->getNode()));
                                                 if (RCL_RET_OK != ret)
                                                 {
                                                     RCL_ERROR_MSG(Ros2ClientImpl, rcl_client_fini);
                                                 }
                                                 delete client;
                                             }))
{

    (*m_client) = rcl_get_zero_initialized_client();
    rcl_client_options_t clientOptions = rcl_client_get_default_options();
    clientOptions.qos = Ros2QoSProfileConverter::convert(qos);
    rcl_ret_t rc =
        rcl_client_init(m_client.get(), static_cast<rcl_node_t*>(m_nodeHandle->getNode()),
                        static_cast<const rosidl_service_type_support_t*>(typeSupport), serviceName, &clientOptions);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(Ros2ClientImpl, rcl_client_init);
        m_client.reset();
    }
}

Ros2ClientImpl::~Ros2ClientImpl()
{
    if (m_waitSetInitialized)
    {
        rcl_ret_t rc = rcl_wait_set_fini(&m_waitSet);
        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(~Ros2ClientImpl, rcl_wait_set_fini);
        }
        m_waitSetInitialized = false;
    }
    m_client.reset();
}

bool Ros2ClientImpl::sendRequest(void* requestMsg)
{
    if (!m_waitSetInitialized)
    {
        m_waitSet = rcl_get_zero_initialized_wait_set();
        rcl_ret_t rc = rcl_wait_set_init(&m_waitSet, 0, 0, 0, 1, 0, 0,
                                         static_cast<rcl_context_t*>(m_nodeHandle->getContextHandle()->getContext()),
                                         rcl_get_default_allocator());

        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(sendRequest, rcl_wait_set_init);
            return false;
        }
        m_waitSetInitialized = true;
    }

    int64_t sequenceNumber;
    rcl_ret_t rc = rcl_send_request(m_client.get(), requestMsg, &sequenceNumber);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(sendRequest, rcl_send_request);
        return false;
    }
    return true;
}

bool Ros2ClientImpl::takeResponse(void* responseMsg)
{
    while (true)
    {
        rcl_ret_t rc = rcl_wait_set_clear(&m_waitSet);
        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(sendRequest, rcl_wait_set_clear);
            return false;
        }
        rc = rcl_wait_set_add_client(&m_waitSet, m_client.get(), nullptr);
        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(sendRequest, rcl_wait_set_add_subscription);
            return false;
        }
        rc = rcl_wait(&m_waitSet, 0);
        if (rc == RCL_RET_TIMEOUT)
        {
            return false;
        }
        if (rc != RCL_RET_OK)
        {
            // This keeps printing an error if the publisher is not active.
            // Ideally only want to notify user once, when the subscription is created
            RCL_WARN_MSG(takeResponse, rcl_wait);
            return false;
        }
        for (size_t i = 0; i < m_waitSet.size_of_clients; i++)
        {
            if (m_waitSet.clients[0])
            {
                rmw_request_id_t requestId;
                rc = rcl_take_response(m_client.get(), &requestId, responseMsg);
                if (rc == RCL_RET_OK)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

} // namespace core
} // namespace ros2
} // namespace isaacsim

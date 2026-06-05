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

Ros2ServiceImpl::Ros2ServiceImpl(Ros2NodeHandle* nodeHandle,
                                 const char* serviceName,
                                 const void* typeSupport,
                                 const Ros2QoSProfile& qos)
    : m_nodeHandle(nodeHandle),
      m_waitSetInitialized(false),
      m_service(std::shared_ptr<rcl_service_t>(new rcl_service_t,
                                               [nodeHandle](rcl_service_t* service)
                                               {
                                                   // Intentionally capture node by copy so shared_ptr can be
                                                   // transfered to copies
                                                   rcl_ret_t ret = rcl_service_fini(
                                                       service, static_cast<rcl_node_t*>(nodeHandle->getNode()));
                                                   if (RCL_RET_OK != ret)
                                                   {
                                                       RCL_ERROR_MSG(Ros2ServiceImpl, rcl_service_fini);
                                                   }
                                                   delete service;
                                               }))
{
    (*m_service) = rcl_get_zero_initialized_service();
    rcl_service_options_t serviceOptions = rcl_service_get_default_options();
    serviceOptions.qos = Ros2QoSProfileConverter::convert(qos);

    rcl_ret_t rc =
        rcl_service_init(m_service.get(), static_cast<rcl_node_t*>(m_nodeHandle->getNode()),
                         static_cast<const rosidl_service_type_support_t*>(typeSupport), serviceName, &serviceOptions);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(Ros2ServiceImpl, rcl_service_init);
        m_service.reset();
    }
}
Ros2ServiceImpl::~Ros2ServiceImpl()
{
    if (m_waitSetInitialized)
    {
        rcl_ret_t rc = rcl_wait_set_fini(&m_waitSet);
        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(~Ros2ServiceImpl, rcl_wait_set_fini);
        }
        m_waitSetInitialized = false;
    }
    m_service.reset();
}

bool Ros2ServiceImpl::takeRequest(void* requestMsg)
{
    if (!m_waitSetInitialized)
    {
        m_waitSet = rcl_get_zero_initialized_wait_set();
        rcl_ret_t rc = rcl_wait_set_init(&m_waitSet, 0, 0, 0, 0, 1, 0,
                                         static_cast<rcl_context_t*>(m_nodeHandle->getContextHandle()->getContext()),
                                         rcl_get_default_allocator());

        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(takeRequest, rcl_wait_set_init);
            return false;
        }
        m_waitSetInitialized = true;
    }

    rcl_ret_t rc = rcl_wait_set_clear(&m_waitSet);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(takeRequest, rcl_wait_set_clear);
        return false;
    }
    rc = rcl_wait_set_add_service(&m_waitSet, m_service.get(), nullptr);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(takeRequest, rcl_wait_set_add_service);
        return false;
    }
    rc = rcl_wait(&m_waitSet, 0);
    // CARB_LOG_WARN_ONCE("Subscriber created, check topic name and message type if not active");
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT)
    {
        RCL_WARN_MSG(takeRequest, rcl_wait);
        return false;
    }
    if (m_waitSet.services[0])
    {
        rc = rcl_take_request(m_service.get(), &m_requestId, requestMsg);
        if (rc != RCL_RET_OK)
        {
            RCL_ERROR_MSG(takeRequest, rcl_take_request);
            return false;
        }
        return true;
    }
    return false;
}

bool Ros2ServiceImpl::sendResponse(void* responseMsg)
{
    if (!m_waitSetInitialized)
    {
        return false;
    }
    rcl_ret_t rc = rcl_send_response(m_service.get(), &m_requestId, responseMsg);
    if (rc != RCL_RET_OK)
    {
        RCL_ERROR_MSG(sendResponse, rcl_send_response);
        return false;
    }
    return true;
}

} // namespace core
} // namespace ros2
} // namespace isaacsim

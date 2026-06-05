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

Ros2NodeHandleImpl::Ros2NodeHandleImpl(const char* name, const char* namespaceName, Ros2ContextHandle* contextHandle)
    : m_contextHandle(contextHandle),
      m_node(std::shared_ptr<rcl_node_t>(new rcl_node_t,
                                         [](rcl_node_t* node)
                                         {
                                             rcl_ret_t ret = rcl_node_fini(node);
                                             if (RCL_RET_OK != ret)
                                             {
                                                 RCL_ERROR_MSG(Ros2NodeHandleImpl, rcl_node_fini);
                                             }
                                             delete node;
                                         }))
{
    rcl_ret_t rc;
    if (m_node != nullptr)
    {
        (*m_node) = rcl_get_zero_initialized_node();
        rcl_node_options_t nodeOptions = rcl_node_get_default_options();
        rc = rcl_node_init(m_node.get(), name, namespaceName,
                           static_cast<rcl_context_t*>(m_contextHandle->getContext()), &nodeOptions);
        if (rc != RCL_RET_OK)
        {
            m_node.reset();
            RCL_ERROR_MSG(Ros2NodeHandleImpl, rcl_node_init);
            return;
        }
    }
}

Ros2NodeHandleImpl::~Ros2NodeHandleImpl()
{
    m_node.reset();
}

Ros2ContextHandle* Ros2NodeHandleImpl::getContextHandle()
{
    return m_contextHandle;
}

void* Ros2NodeHandleImpl::getNode()
{
    return m_node.get();
}

} // namespace core
} // namespace ros2
} // namespace isaacsim

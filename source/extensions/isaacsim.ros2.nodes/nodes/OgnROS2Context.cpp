// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/ros2/core/IRos2Core.h>
#include <isaacsim/ros2/core/Ros2Factory.h>

#include <OgnROS2ContextDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2Context : public isaacsim::core::includes::BaseResetNode
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2ContextDatabase::sPerInstanceState<OgnROS2Context>(nodeObj, instanceId);
        state.m_ros2Bridge = carb::getCachedInterface<Ros2Bridge>();
        Ros2Factory* factory = state.m_ros2Bridge->getFactory();
        state.m_contextHandle = factory->createContextHandle();
    }

    static bool compute(OgnROS2ContextDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2Context>();

        // If the domain id has changed, reset the context
        if (state.m_contextHandle->isValid() && state.m_cleanUp && db.inputs.domain_id() != state.m_domainId)
        {
            state.m_contextHandle->shutdown("Omnigraph ROS2 Context node resetting");
            state.m_cleanUp = false;
        }

        if (!state.m_contextHandle->isValid())
        {
            // Set the Domain ID of the context
            state.m_domainId = db.inputs.domain_id();
            const bool useDomainIDEnvVar = db.inputs.useDomainIDEnvVar();

            if (useDomainIDEnvVar)
            {
                char* domainIdStr = nullptr;

#ifdef _MSC_VER

                size_t sz = 0;
                _dupenv_s(&domainIdStr, &sz, "ROS_DOMAIN_ID");
#else
                domainIdStr = getenv("ROS_DOMAIN_ID");
#endif

                if (domainIdStr != nullptr)
                {
                    state.m_domainId = strtoul(domainIdStr, nullptr, 0);

                    if (state.m_domainId == (std::numeric_limits<uint32_t>::max)())
                    {
                        CARB_LOG_INFO("ROS_DOMAIN_ID: %s could not be interpreted as a legal number", domainIdStr);
#ifdef _MSC_VER
                        free(domainIdStr);
#endif
                        return false;
                    }
#ifdef _MSC_VER
                    free(domainIdStr);
#endif
                }
                else
                {
                    CARB_LOG_INFO("ROS_DOMAIN_ID not found. Using input value of %zd", state.m_domainId);
                }
            }

            state.m_contextHandle->init(0, nullptr, true, state.m_domainId);
            // We cast the shared ptr directly (and not the pointer inside of it)
            // This allows us to keep track of the shared pointer properly.
            state.m_contextHandleAddr = state.m_ros2Bridge->addHandle(&state.m_contextHandle);
            db.outputs.context() = state.m_contextHandleAddr;
            return true;
        }
        return true;
    }
    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2ContextDatabase::sPerInstanceState<OgnROS2Context>(nodeObj, instanceId);
        state.reset();
        state.m_contextHandle.reset();
        state.m_ros2Bridge->removeHandle(state.m_contextHandleAddr);
    }

    void reset() override
    {
        // We cannot actually destroy the context here because downstream nodes would fail.
        // Instead perform cleanup on next frame
        m_cleanUp = true;
    }

    static bool updateNodeVersion(const GraphContextObj& context, const NodeObj& nodeObj, int oldVersion, int newVersion)
    {
        if (oldVersion < newVersion)
        {
            if (oldVersion == 1)
            {
                // We added inputs:useDomainIDEnvVar, to maintain previous behavior we should set this to false
                const bool val{ false };
                nodeObj.iNode->createAttribute(nodeObj, "inputs:useDomainIDEnvVar", Type(BaseDataType::eBool), &val,
                                               nullptr, kAttributePortType_Input, kExtendedAttributeType_Regular,
                                               nullptr);
            }
            return true;
        }
        return false;
    }

private:
    std::shared_ptr<Ros2ContextHandle> m_contextHandle = nullptr;
    bool m_cleanUp = false;
    size_t m_domainId = 0;
    uint64_t m_contextHandleAddr = 0;
    Ros2Bridge* m_ros2Bridge = nullptr;
};

REGISTER_OGN_NODE()

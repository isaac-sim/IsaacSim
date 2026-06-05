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

// Replace MyTransportHandle with your transport type.
// Replace OgnMyIpcNodeCpp / OgnMyIpcNodeCppDatabase with your node class name.

// TEMPLATE-START
#include <isaacsim/core/includes/BaseResetNode.h>

#include <OgnMyIpcNodeCppDatabase.h>
#include <memory>
#include <string>

using isaacsim::core::includes::BaseResetNode;

class OgnMyIpcNodeCpp : public BaseResetNode
{
public:
    static bool compute(OgnMyIpcNodeCppDatabase& db)
    {
        auto& state = db.perInstanceState<OgnMyIpcNodeCpp>();

        // Detect input changes (e.g. URI) and reset transport.
        const std::string uriIn(db.inputs.uri());
        if (state.m_handle && state.m_handle->getUri() != uriIn)
        {
            state.reset();
        }

        const bool success = state.ensureOpenAndTransfer(db);
        // Fire execOut unconditionally (send nodes). For receive nodes, fire only when
        // a complete message arrives (i.e. only when success == true).
        db.outputs.execOut() = omni::graph::core::kExecutionAttributeStateEnabled;
        return success;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnMyIpcNodeCppDatabase::sPerInstanceState<OgnMyIpcNodeCpp>(nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        if (m_handle)
        {
            m_handle->close();
            m_handle.reset();
        }
    }

private:
    bool isOpen() const
    {
        return m_handle && m_handle->isOpen();
    }

    void tryOpen(OgnMyIpcNodeCppDatabase& db)
    {
        // Open transport from db.inputs (e.g. URI, config).
        // m_handle = std::make_unique<MyTransportHandle>(std::string(db.inputs.uri()));
    }

    bool transfer(OgnMyIpcNodeCppDatabase& db)
    {
        // Non-blocking send or try-receive; write db.outputs on success.
        // See Performance Considerations for time-budget guidance.
        return false; // replace with actual transfer
    }

    bool ensureOpenAndTransfer(OgnMyIpcNodeCppDatabase& db)
    {
        if (!isOpen())
        {
            tryOpen(db);
            if (!isOpen())
                return false;
        }
        return transfer(db);
    }

    std::unique_ptr<MyTransportHandle> m_handle; // replace with your transport type
};

REGISTER_OGN_NODE()
// TEMPLATE-END

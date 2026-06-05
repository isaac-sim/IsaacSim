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

#include "TcpStepServer.h"

#include <isaacsim/core/includes/BaseResetNode.h>

#include <OgnSimpleReceiveExternalStepCppDatabase.h>
#include <memory>
#include <string>

using isaacsim::core::includes::BaseResetNode;
using namespace isaacsim::examples::ipc;

class OgnSimpleReceiveExternalStepCpp : public BaseResetNode
{
public:
    static bool compute(OgnSimpleReceiveExternalStepCppDatabase& db)
    {
        auto& state = db.perInstanceState<OgnSimpleReceiveExternalStepCpp>();

        const std::string uriIn(db.inputs.uri());
        if (state.m_endpoint && state.m_endpoint->getUri() != uriIn)
        {
            state.reset();
        }

        const bool success = state.ensureListeningAndReceive(db);
        if (success)
        {
            db.outputs.execOut() = omni::graph::core::kExecutionAttributeStateEnabled;
        }
        return success;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnSimpleReceiveExternalStepCppDatabase::sPerInstanceState<OgnSimpleReceiveExternalStepCpp>(
            nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        if (m_endpoint)
        {
            m_endpoint->disconnect();
            m_endpoint.reset();
        }
    }

private:
    bool isListening() const
    {
        return m_endpoint && m_endpoint->isConnected();
    }

    void tryListen(OgnSimpleReceiveExternalStepCppDatabase& db)
    {
        m_endpoint = std::make_unique<TcpStepServer>(std::string(db.inputs.uri()));
        m_endpoint->connect();
    }

    bool receiveStep(OgnSimpleReceiveExternalStepCppDatabase& db)
    {
        if (!m_endpoint)
        {
            return false;
        }
        const auto step = m_endpoint->tryReceiveStep();
        if (!step)
        {
            return false;
        }
        db.outputs.step() = *step;
        return true;
    }

    bool ensureListeningAndReceive(OgnSimpleReceiveExternalStepCppDatabase& db)
    {
        if (!isListening())
        {
            tryListen(db);
            if (!isListening())
            {
                return false;
            }
        }
        return receiveStep(db);
    }

    std::unique_ptr<TcpStepServer> m_endpoint;
};

REGISTER_OGN_NODE()

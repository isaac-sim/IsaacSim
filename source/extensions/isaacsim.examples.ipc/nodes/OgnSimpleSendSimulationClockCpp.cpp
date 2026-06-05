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

#include "TcpClockClient.h"

#include <isaacsim/core/includes/BaseResetNode.h>

#include <OgnSimpleSendSimulationClockCppDatabase.h>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

using isaacsim::core::includes::BaseResetNode;
using namespace isaacsim::examples::ipc;

class OgnSimpleSendSimulationClockCpp : public BaseResetNode
{
public:
    static bool compute(OgnSimpleSendSimulationClockCppDatabase& db)
    {
        auto& state = db.perInstanceState<OgnSimpleSendSimulationClockCpp>();

        const std::string uriIn(db.inputs.uri());
        if (state.m_endpoint && state.m_endpoint->getUri() != uriIn)
        {
            state.reset();
        }

        const bool success = state.ensureConnectedAndSend(db);
        db.outputs.execOut() = omni::graph::core::kExecutionAttributeStateEnabled;
        return success;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnSimpleSendSimulationClockCppDatabase::sPerInstanceState<OgnSimpleSendSimulationClockCpp>(
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
    bool isConnected() const
    {
        return m_endpoint && m_endpoint->isConnected();
    }

    void tryConnect(OgnSimpleSendSimulationClockCppDatabase& db)
    {
        m_endpoint = std::make_unique<TcpClockClient>(std::string(db.inputs.uri()));
        m_endpoint->connect();
    }

    bool sendClock(OgnSimpleSendSimulationClockCppDatabase& db)
    {
        if (!m_endpoint)
        {
            return false;
        }
        const double sec = db.inputs.simulationTime();
        const int64_t base_ns = static_cast<int64_t>(std::llround(sec * 1e9));
        return m_endpoint->sendClock(base_ns);
    }

    bool ensureConnectedAndSend(OgnSimpleSendSimulationClockCppDatabase& db)
    {
        if (!isConnected())
        {
            tryConnect(db);
            if (!isConnected())
            {
                return false;
            }
        }
        return sendClock(db);
    }

    std::unique_ptr<TcpClockClient> m_endpoint;
};

REGISTER_OGN_NODE()

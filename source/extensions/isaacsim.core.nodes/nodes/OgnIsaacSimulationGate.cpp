// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


#include <isaacsim/core/nodes/ICoreNodes.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/timeline/ITimeline.h>

#include <OgnIsaacSimulationGateDatabase.h>

namespace isaacsim
{
namespace core
{
namespace nodes
{


class OgnIsaacSimulationGate
{

public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacSimulationGateDatabase::sPerInstanceState<OgnIsaacSimulationGate>(nodeObj, instanceId);

        state.m_timeline = carb::getCachedInterface<omni::timeline::ITimeline>();
        if (!state.m_timeline)
        {
            CARB_LOG_ERROR("Failed to acquire timeline interface");
            return;
        }
    }

    static bool compute(OgnIsaacSimulationGateDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacSimulationGate>();
        const auto& inputStep = db.inputs.step();
        // If the timeline is stopped or step is set to zero, skip execution
        if (state.m_timeline == nullptr)
        {
            return false;
        }
        else if (state.m_timeline->isPlaying() && inputStep > 0)
        {
            state.m_frame++;
            if (state.m_frame >= inputStep)
            {
                state.m_frame = 0;
                db.outputs.execOut() = kExecutionAttributeStateEnabled;
            }
        }
        return true;
    }

private:
    unsigned int m_frame = 0;
    omni::timeline::ITimeline* m_timeline = nullptr;
};
REGISTER_OGN_NODE()
}
}
}

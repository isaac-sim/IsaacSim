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

#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/core/nodes/ICoreNodes.h>

#include <OgnIsaacRunOneSimulationFrameDatabase.h>

namespace isaacsim
{
namespace core
{
namespace nodes
{

class OgnIsaacRunOneSimulationFrame : public isaacsim::core::includes::BaseResetNode
{
public:
    static bool compute(OgnIsaacRunOneSimulationFrameDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacRunOneSimulationFrame>();

        if (state.m_firstFrame)
        {
            state.m_firstFrame = false;
            db.outputs.step() = kExecutionAttributeStateEnabled;
        }
        return true;
    }

    void reset() override
    {
        m_firstFrame = true;
    }


private:
    bool m_firstFrame = true;
};

REGISTER_OGN_NODE()
}
}
}

// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <carb/Defines.h>

#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/UsdContextIncludes.h>

#include <OgnIsaacRealTimeFactorDatabase.h>

namespace isaacsim
{
namespace core
{
namespace nodes
{

class OgnIsaacRealTimeFactor : public isaacsim::core::includes::BaseResetNode
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacRealTimeFactorDatabase::sPerInstanceState<OgnIsaacRealTimeFactor>(nodeObj, instanceId);
        state.m_simulationManagerFramework =
            carb::getCachedInterface<isaacsim::core::simulation_manager::ISimulationManager>();
    }

    static bool compute(OgnIsaacRealTimeFactorDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacRealTimeFactor>();

        // Return immediately after first frame to get accurate measurement next frame
        if (state.m_resetTimes)
        {
            state.m_realStartTime = std::chrono::steady_clock::now();
            state.m_simStartTime = state.m_simulationManagerFramework->getSimulationTimeMonotonic();
            state.m_resetTimes = false;
            return false;
        }

        double realTimeElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                                     std::chrono::steady_clock::now() - state.m_realStartTime)
                                     .count();

        double simTimeElapsed = state.m_simulationManagerFramework->getSimulationTimeMonotonic() - state.m_simStartTime;

        if (simTimeElapsed == 0.0)
        {
            return false;
        }

        float rtf = static_cast<float>(simTimeElapsed / realTimeElapsed);

        db.outputs.rtf() = rtf;
        state.m_realStartTime = std::chrono::steady_clock::now();
        state.m_simStartTime = state.m_simulationManagerFramework->getSimulationTimeMonotonic();

        return true;
    }

    void reset() override
    {
        m_resetTimes = true;
    }

private:
    std::chrono::steady_clock::time_point m_realStartTime;
    double m_simStartTime = 0.0;
    bool m_resetTimes = true;
    uint64_t m_frames = 0;
    uint64_t m_step = 1;

    isaacsim::core::simulation_manager::ISimulationManager* m_simulationManagerFramework = nullptr;
};

REGISTER_OGN_NODE()
} // nodes
} // graph
} // omni

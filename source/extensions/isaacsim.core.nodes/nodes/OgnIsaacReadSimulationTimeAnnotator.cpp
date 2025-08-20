// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <carb/Types.h>
#include <carb/events/EventsUtils.h>
#include <carb/logging/Logger.h>

#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <omni/usd/UsdContextIncludes.h>
//
#include <omni/usd/UsdContext.h>

#include <OgnIsaacReadSimulationTimeAnnotatorDatabase.h>

namespace isaacsim
{
namespace core
{
namespace nodes
{

class OgnIsaacReadSimulationTimeAnnotator
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacReadSimulationTimeAnnotatorDatabase::sPerInstanceState<OgnIsaacReadSimulationTimeAnnotator>(
            nodeObj, instanceId);
        state.m_simulationManagerFramework =
            carb::getCachedInterface<isaacsim::core::simulation_manager::ISimulationManager>();
    }

    static bool compute(OgnIsaacReadSimulationTimeAnnotatorDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacReadSimulationTimeAnnotator>();

        state.m_resetOnStop = db.inputs.resetOnStop();
        if (db.inputs.referenceTimeNumerator() > 0 || db.inputs.referenceTimeDenominator() > 0)
        {
            if (state.m_resetOnStop)
            {
                db.outputs.simulationTime() = state.m_simulationManagerFramework->getSimulationTimeAtTime(
                    omni::fabric::RationalTime(db.inputs.referenceTimeNumerator(), db.inputs.referenceTimeDenominator()));
            }
            else
            {
                db.outputs.simulationTime() = state.m_simulationManagerFramework->getSimulationTimeMonotonicAtTime(
                    omni::fabric::RationalTime(db.inputs.referenceTimeNumerator(), db.inputs.referenceTimeDenominator()));
            }
        }
        else
        {
            if (state.m_resetOnStop)
            {
                db.outputs.simulationTime() = state.m_simulationManagerFramework->getSimulationTime();
            }
            else
            {
                db.outputs.simulationTime() = state.m_simulationManagerFramework->getSimulationTimeMonotonic();
            }
        }
        db.outputs.execOut() = ExecutionAttributeState::kExecutionAttributeStateEnabled;

        return true;
    }


private:
    bool m_resetOnStop = true;
    isaacsim::core::simulation_manager::ISimulationManager* m_simulationManagerFramework;
};

REGISTER_OGN_NODE()
} // nodes
} // graph
} // omni

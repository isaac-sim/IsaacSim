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
#include <omni/usd/UsdContextIncludes.h>
//
#include <omni/usd/UsdContext.h>

#include <OgnIsaacReadSystemTimeDatabase.h>

namespace isaacsim
{
namespace core
{
namespace nodes
{

class OgnIsaacReadSystemTime
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacReadSystemTimeDatabase::sPerInstanceState<OgnIsaacReadSystemTime>(nodeObj, instanceId);
        state.m_simulationManagerFramework =
            carb::getCachedInterface<isaacsim::core::simulation_manager::ISimulationManager>();
    }

    static bool compute(OgnIsaacReadSystemTimeDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacReadSystemTime>();

        if (db.inputs.referenceTimeNumerator() > 0 || db.inputs.referenceTimeDenominator() > 0)
        {
            db.outputs.systemTime() = state.m_simulationManagerFramework->getSystemTimeAtTime(
                omni::fabric::RationalTime(db.inputs.referenceTimeNumerator(), db.inputs.referenceTimeDenominator()));
        }
        else
        {
            db.outputs.systemTime() = state.m_simulationManagerFramework->getSystemTime();
        }
        return true;
    }


private:
    isaacsim::core::simulation_manager::ISimulationManager* m_simulationManagerFramework = nullptr;
};

REGISTER_OGN_NODE()
} // core_nodes
} // isaac
} // omni

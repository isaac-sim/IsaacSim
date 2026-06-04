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

#include "SimulationBackend.h"

#include "base/BaseSimulationView.h"
#include "cpu/CpuSimulationView.h"
#include "gpu/GpuSimulationView.h"

#include <carb/logging/Log.h>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using namespace omni::physics::tensors;

SimulationBackend::SimulationBackend()
{
    CARB_LOG_INFO("SimulationBackend initialized");
}

SimulationBackend::~SimulationBackend()
{
}

ISimulationView* SimulationBackend::createSimulationView(long stageId)
{
    try
    {
        auto init = BaseSimulationView::initNewton(stageId);
        if (!init.valid)
            return nullptr;

        if (init.simDeviceOrdinal >= 0)
            return new GpuSimulationView(std::move(init));
        else
            return new CpuSimulationView(std::move(init));
    }
    catch (std::exception& e)
    {
        CARB_LOG_ERROR("Failed to create simulation view: %s", e.what());
        return nullptr;
    }
}

void SimulationBackend::reset()
{
    CARB_LOG_INFO("SimulationBackend reset");
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim

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

#include "CpuSimulationView.h"

#include "CpuArticulationView.h"
#include "CpuRigidBodyView.h"
#include "CpuRigidContactView.h"

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using namespace omni::physics::tensors;

CpuSimulationView::CpuSimulationView(SimViewInit&& init) : BaseSimulationView(std::move(init))
{
}

int CpuSimulationView::getDeviceOrdinal() const
{
    return kCpuDeviceOrdinal;
}

int CpuSimulationView::getParamDeviceOrdinal() const
{
    return kCpuDeviceOrdinal;
}

void* CpuSimulationView::getCudaContext() const
{
    return nullptr;
}

IArticulationView* CpuSimulationView::_makeArticulationView(py::object newtonStage, const std::vector<pxr::SdfPath>& paths)
{
    return new CpuArticulationView(newtonStage, paths);
}

IRigidBodyView* CpuSimulationView::_makeRigidBodyView(py::object newtonStage, const std::vector<pxr::SdfPath>& paths)
{
    return new CpuRigidBodyView(newtonStage, paths);
}

IRigidContactView* CpuSimulationView::_makeRigidContactView(py::object newtonStage,
                                                            const std::vector<std::string>& sensorPaths,
                                                            const std::vector<std::vector<std::string>>& filterPaths,
                                                            uint32_t maxContactDataCount)
{
    return new CpuRigidContactView(newtonStage, sensorPaths, filterPaths, maxContactDataCount);
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim

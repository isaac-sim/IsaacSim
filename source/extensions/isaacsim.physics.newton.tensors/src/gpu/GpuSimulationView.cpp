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

#include "GpuSimulationView.h"

#include "GpuArticulationView.h"
#include "GpuRigidBodyView.h"
#include "GpuRigidContactView.h"

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using namespace omni::physics::tensors;

GpuSimulationView::GpuSimulationView(SimViewInit&& init) : BaseSimulationView(std::move(init))
{
}

int GpuSimulationView::getDeviceOrdinal() const
{
    return m_simDeviceOrdinal;
}

int GpuSimulationView::getParamDeviceOrdinal() const
{
    return m_simDeviceOrdinal;
}

void* GpuSimulationView::getCudaContext() const
{
    return nullptr;
}

IArticulationView* GpuSimulationView::_makeArticulationView(py::object newtonStage, const std::vector<pxr::SdfPath>& paths)
{
    return new GpuArticulationView(newtonStage, paths, m_simDeviceOrdinal);
}

IRigidBodyView* GpuSimulationView::_makeRigidBodyView(py::object newtonStage, const std::vector<pxr::SdfPath>& paths)
{
    return new GpuRigidBodyView(newtonStage, paths, m_simDeviceOrdinal);
}

IRigidContactView* GpuSimulationView::_makeRigidContactView(py::object newtonStage,
                                                            const std::vector<std::string>& sensorPaths,
                                                            const std::vector<std::vector<std::string>>& filterPaths,
                                                            uint32_t maxContactDataCount)
{
    return new GpuRigidContactView(newtonStage, sensorPaths, filterPaths, maxContactDataCount, m_simDeviceOrdinal);
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim

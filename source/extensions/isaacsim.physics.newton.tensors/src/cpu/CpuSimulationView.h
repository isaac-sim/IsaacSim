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

#pragma once

#include "base/BaseSimulationView.h"

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using omni::physics::tensors::IArticulationView;
using omni::physics::tensors::IRigidBodyView;
using omni::physics::tensors::IRigidContactView;

/// CPU simulation view. Reports device ordinal -1 and creates Cpu* child views.
class CpuSimulationView : public BaseSimulationView
{
public:
    /**
     * @brief Constructs a CpuSimulationView.
     * @param[in] init Initialization parameters for the simulation view.
     */
    explicit CpuSimulationView(SimViewInit&& init);
    ~CpuSimulationView() override = default;

    /**
     * @brief Returns the device ordinal that the view operates on.
     * @return The requested value.
     */
    int getDeviceOrdinal() const override;
    /**
     * @brief Returns the device ordinal used for parameter tensors.
     * @return The requested value.
     */
    int getParamDeviceOrdinal() const override;
    /**
     * @brief Returns the CUDA context associated with the view.
     * @return Pointer to the requested object, or nullptr if unavailable.
     */
    void* getCudaContext() const override;

protected:
    IArticulationView* _makeArticulationView(py::object newtonStage, const std::vector<pxr::SdfPath>& paths) override;
    /**
     * @brief Creates the rigid body view.
     * @param[in] newtonStage Newton stage object backing the view.
     * @param[in] paths USD prim paths backing the view.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IRigidBodyView* _makeRigidBodyView(py::object newtonStage, const std::vector<pxr::SdfPath>& paths) override;
    /**
     * @brief Creates the rigid contact view.
     * @param[in] newtonStage Newton stage object backing the view.
     * @param[in] sensorPaths USD prim paths of the contact sensors.
     * @param[in] filterPaths Filter paths.
     * @param[in] maxContactDataCount Maximum number of contact data entries to report.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IRigidContactView* _makeRigidContactView(py::object newtonStage,
                                             const std::vector<std::string>& sensorPaths,
                                             const std::vector<std::vector<std::string>>& filterPaths,
                                             uint32_t maxContactDataCount) override;

private:
    static constexpr int kCpuDeviceOrdinal = -1;
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim

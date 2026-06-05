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

#include <carb/BindingsPythonUtils.h>

#include <isaacsim/sensors/rtx/nodes/ISensorsRtxNodes.h>

CARB_BINDINGS("isaacsim.sensors.rtx.nodes.python")

namespace
{
namespace py = pybind11;

PYBIND11_MODULE(_isaacsim_sensors_rtx_nodes, m)
{
    using namespace isaacsim::sensors::rtx::nodes;

    m.doc() = "RTX Sensor nodes bindings — provides point cloud extraction from GenericModelOutput.";

    carb::defineInterfaceClass<ISensorsRtxNodes>(m, "ISensorsRtxNodes", "acquire_interface", "release_interface");
}
}

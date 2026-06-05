// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/core/includes/BindingsPythonUtils.h>
#include <isaacsim/sensors/physics/nodes/IPhysicsSensorNodes.h>

CARB_BINDINGS("isaacsim.sensors.physics.nodes.python")

namespace
{

PYBIND11_MODULE(_physics_sensor_nodes, m)
{
    using namespace carb;
    using namespace isaacsim::sensors::physics::nodes;

    m.doc() = R"pbdoc(
        Internal interface used by isaacsim.sensors.physics.nodes to acquire/release
        the native plugin interface and trigger OmniGraph node registration.
    )pbdoc";

    defineInterfaceClass<IPhysicsSensorNodes>(m, "IPhysicsSensorNodes", "acquire_interface", "release_interface");
}
} // namespace

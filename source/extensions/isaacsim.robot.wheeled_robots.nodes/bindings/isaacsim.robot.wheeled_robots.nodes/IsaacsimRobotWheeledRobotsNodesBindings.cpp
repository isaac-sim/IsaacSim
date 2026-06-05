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

#include <carb/BindingsPythonUtils.h>

#include <isaacsim/robot/wheeled_robots/nodes/IWheeledRobotsNodes.h>

CARB_BINDINGS("isaacsim.robot.wheeled_robots.nodes.python")


namespace
{

PYBIND11_MODULE(_wheeled_robots_nodes, m)
{
    // clang-format off
    using namespace carb;
    using namespace isaacsim::robot::wheeled_robots::nodes;

    m.doc() = R"pbdoc(
        Internal interface that is automatically called when the extension is loaded so that OmniGraph nodes are registered.

        Example:

            # import isaacsim.robot.wheeled_robots.nodes.bindings._wheeled_robots_nodes as _wheeled_robots_nodes

            # Acquire the interface
            interface = _wheeled_robots_nodes.acquire_interface()

            # Use the interface
            # ...

            # Release the interface
            _wheeled_robots_nodes.release_interface(interface)
    )pbdoc";

    defineInterfaceClass<IWheeledRobotsNodes>(
        m,
        "IWheeledRobotsNodes",
        "acquire_interface",
        "release_interface"
    );
}
} // namespace anonymous

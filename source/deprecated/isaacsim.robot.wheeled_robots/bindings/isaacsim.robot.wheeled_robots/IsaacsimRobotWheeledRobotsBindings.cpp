// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/robot/wheeled_robots/IWheeledRobots.h>

CARB_BINDINGS("isaacsim.robot.wheeled_robots.python")


namespace
{

/**
 * @brief Python bindings for the Wheeled Robots module
 *
 * Provides Python interface access to the wheeled robots functionality
 * through pybind11 bindings.
 */
PYBIND11_MODULE(_isaacsim_robot_wheeled_robots, m)
{
    // clang-format off
    using namespace carb;
    using namespace isaacsim::robot::wheeled_robots;

    m.doc() = R"pbdoc(
        Internal interface that is automatically called when the extension is loaded so that Omnigraph nodes are registered.

        Example:

            # import  isaacsim.robot.wheeled_robots.bindings._isaacsim_robot_wheeled_robots as _isaacsim_robot_wheeled_robots

            # Acquire the interface
            interface = _isaacsim_robot_wheeled_robots.acquire_wheeled_robots_interface()

            # Use the interface
            # ...

            # Release the interface
            _isaacsim_robot_wheeled_robots.release_wheeled_robots_interface(interface)
    )pbdoc";

    defineInterfaceClass<IWheeledRobots>(
        m,
        "IWheeledRobots",
        "acquire_wheeled_robots_interface",
        "release_wheeled_robots_interface"
    );
}
} // namespace anonymous

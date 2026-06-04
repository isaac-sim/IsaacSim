# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run selectable Franka cortex behaviors with a block-stacking scene."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser("franka_examples")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument(
    "--behavior",
    type=str,
    default="block_stacking_behavior",
    help="Which behavior to run. See behavior/franka for available behavior files.",
)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import numpy as np
from behaviors.franka.franka_behaviors import ContextStateMonitor, behaviors
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.cortex.framework.cortex_utils import load_behavior_module
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.robot import add_franka_to_stage


class CubeSpec:
    """Store the name and color specification for a cube obstacle."""

    def __init__(self, name, color):
        self.name = name
        self.color = np.array(color)


def main():
    """Set up the scene and run the selected Franka behavior."""
    world = CortexWorld()
    context_monitor = ContextStateMonitor(print_dt=0.25)
    robot = world.add_robot(add_franka_to_stage(name="franka", prim_path="/World/Franka"))

    obs_specs = [
        CubeSpec("RedCube", [0.7, 0.0, 0.0]),
        CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
        CubeSpec("YellowCube", [0.7, 0.7, 0.0]),
        CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
    ]
    width = 0.0515
    for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.7, len(obs_specs)), obs_specs)):
        obj = world.scene.add(
            DynamicCuboid(
                prim_path="/World/Obs/{}".format(spec.name),
                name=spec.name,
                size=width,
                color=spec.color,
                position=np.array([x, -0.4, width / 2]),
            )
        )
        robot.register_obstacle(obj)
    world.scene.add_default_ground_plane()

    print()
    print("loading behavior: {}".format(args.behavior))
    print()
    if args.behavior in behaviors:
        decider_network = behaviors[args.behavior].make_decider_network(robot)
    else:
        decider_network = load_behavior_module(args.behavior).make_decider_network(robot)
    decider_network.context.add_monitor(context_monitor.monitor)
    world.add_decider_network(decider_network)

    if args.test:
        _test_frames = {"count": 0}

        def _test_done_cb():
            _test_frames["count"] += 1
            return _test_frames["count"] >= 10

        world.run(simulation_app, is_done_cb=_test_done_cb)
    else:
        world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()

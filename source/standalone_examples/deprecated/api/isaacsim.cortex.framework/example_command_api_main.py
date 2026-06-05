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

"""Demonstrate the Cortex command API with nullspace posture shifting."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import time

import numpy as np
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider, DfStateSequence
from isaacsim.cortex.framework.dfb import DfBasicContext
from isaacsim.cortex.framework.robot import add_franka_to_stage


class NullspaceShiftState(DfState):
    """Cycle through random nullspace posture configurations at a fixed end-effector target."""

    def __init__(self):
        super().__init__()
        self.config_mean = np.array([0.00, -1.3, 0.00, -2.87, 0.00, 2.00, 0.75])
        self.target_p = np.array([0.7, 0.0, 0.5])
        self.construction_time = time.time()

    def enter(self):
        """Sample a new posture configuration and toggle the gripper."""
        # Change the posture configuration while maintaining a consistent target.
        posture_config = self.config_mean + np.random.randn(7)
        self.context.robot.arm.send_end_effector(target_position=self.target_p, posture_config=posture_config)

        self.entry_time = time.time()

        # Close the gripper if open and open the gripper if closed. It closes more quickly than it
        # opens.
        gripper = self.context.robot.gripper
        if gripper.get_width() > 0.05:
            gripper.close(speed=0.5)
        else:
            gripper.open(speed=0.1)

        print("[%f] <enter> sampling posture config" % (self.entry_time - self.construction_time))

    def step(self):
        """Wait for two seconds before transitioning to the next state."""
        if time.time() - self.entry_time < 2.0:
            return self
        return None


def main():
    """Set up and run the nullspace posture shifting example."""
    world = CortexWorld()
    robot = world.add_robot(add_franka_to_stage(name="franka", prim_path="/World/franka"))
    world.scene.add_default_ground_plane()

    decider_network = DfNetwork(
        DfStateMachineDecider(DfStateSequence([NullspaceShiftState()], loop=True)), context=DfBasicContext(robot)
    )
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

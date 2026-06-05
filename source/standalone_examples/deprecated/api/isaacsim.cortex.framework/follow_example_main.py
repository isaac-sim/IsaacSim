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

"""Demonstrate a Franka robot following a movable sphere using a decider network."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api.objects import VisualSphere
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider
from isaacsim.cortex.framework.dfb import DfBasicContext
from isaacsim.cortex.framework.robot import add_franka_to_stage


class FollowState(DfState):
    """Access the context object as self.context.

    We have access to everything in the context object, which in this case is everything in the
    robot object (the command API and the follow sphere).
    """

    @property
    def robot(self):
        """Return the robot from the context."""
        return self.context.robot

    @property
    def follow_sphere(self):
        """Return the follow sphere from the robot."""
        return self.context.robot.follow_sphere

    def enter(self):
        """Close the gripper and initialize the follow sphere pose."""
        self.robot.gripper.close()
        self.follow_sphere.set_world_pose(*self.robot.arm.get_fk_pq().as_tuple())

    def step(self):
        """Send the end effector toward the follow sphere position."""
        target_position, _ = self.follow_sphere.get_world_pose()
        self.robot.arm.send_end_effector(target_position=target_position)
        return self  # Always transition back to this state.


def main():
    """Set up and run the Franka follow sphere example."""
    world = CortexWorld()
    robot = world.add_robot(add_franka_to_stage(name="franka", prim_path="/World/Franka"))

    # Add a sphere to the scene to follow, and store it off in a new member as part of the robot.
    robot.follow_sphere = world.scene.add(
        VisualSphere(
            name="follow_sphere", prim_path="/World/FollowSphere", radius=0.02, color=np.array([0.7, 0.0, 0.7])
        )
    )
    world.scene.add_default_ground_plane()

    # Add a simple state machine decider network with the single state defined above. This state
    # will be persistently stepped because it always returns itself.
    world.add_decider_network(DfNetwork(DfStateMachineDecider(FollowState()), context=DfBasicContext(robot)))

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

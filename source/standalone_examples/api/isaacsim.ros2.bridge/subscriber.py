# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate ROS 2 subscriber controlling a cube in simulation."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"renderer": "RealTimePathTracing", "headless": False})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.core.simulation_manager import SimulationManager

# enable ROS2 bridge extension
app_utils.enable_extension("isaacsim.ros2.bridge")

simulation_app.update()


# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class Subscriber(Node):
    """ROS 2 subscriber node that moves a cube on message receipt."""

    def __init__(self) -> None:
        super().__init__("tutorial_subscriber")

        # setting up the world with a cube
        stage_utils.set_stage_units(meters_per_unit=1.0)
        GroundPlane("/World/GroundPlane")
        DistantLight("/World/DistantLight").set_intensities(intensities=[3000])

        # add a cube in the world
        self.cube = Cube(
            paths="/cube",
            positions=np.array([[0, 0, 0.10]]),
            sizes=0.2,
            colors="red",
        )
        self._cube_position = np.array([[0, 0, 0.10]])

        # setup the ROS2 subscriber here
        self.ros_sub = self.create_subscription(Empty, "move_cube", self.move_cube_callback, 10)

        SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")

    def move_cube_callback(self, data: Empty) -> None:
        """Set a new random cube position on message receipt."""
        # callback function to set the cube position to a new one upon receiving a (empty) ROS2 message
        if app_utils.is_playing():
            self._cube_position = np.array([[np.random.rand() * 0.40, np.random.rand() * 0.40, 0.10]])

    def run_simulation(self) -> None:
        """Run the simulation loop with ROS 2 message handling."""
        app_utils.play()
        simulation_app.update()
        reset_needed = False
        frame_count = 0
        while simulation_app.is_running():
            simulation_app.update()
            rclpy.spin_once(self, timeout_sec=0.0)
            if not app_utils.is_playing() and not reset_needed:
                reset_needed = True
            if app_utils.is_playing():
                if reset_needed:
                    app_utils.stop()
                    app_utils.update_app(steps=5)
                    app_utils.play()
                    app_utils.update_app(steps=5)
                    reset_needed = False
                # the actual setting the cube pose is done here
                self.cube.set_world_poses(positions=self._cube_position)
            frame_count += 1
            if args.test and frame_count >= 10:
                break

        # Cleanup
        app_utils.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    subscriber = Subscriber()
    subscriber.run_simulation()

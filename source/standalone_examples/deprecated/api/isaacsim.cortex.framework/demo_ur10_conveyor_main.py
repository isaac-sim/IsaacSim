# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate UR10 bin stacking on a conveyor belt."""

import argparse
from typing import Any

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import random

import isaacsim.cortex.behaviors.ur10.bin_stacking_behavior as behavior
import isaacsim.cortex.framework.math_util as math_util
import numpy as np
from isaacsim.core.api.objects import VisualCapsule, VisualSphere
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.cortex.framework.cortex_rigid_prim import CortexRigidPrim
from isaacsim.cortex.framework.cortex_utils import get_assets_root_path_or_die
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.robot import CortexUr10


class Ur10Assets:
    """Store USD asset paths for the UR10 bin stacking scene."""

    def __init__(self) -> None:
        self.assets_root_path = get_assets_root_path_or_die()

        self.ur10_table_usd = (
            self.assets_root_path + "/Isaac/Samples/Leonardo/Stage/ur10_bin_stacking_short_suction.usd"
        )
        self.small_klt_usd = self.assets_root_path + "/Isaac/Props/KLT_Bin/small_KLT.usd"
        self.background_usd = self.assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        self.rubiks_cube_usd = self.assets_root_path + "/Isaac/Props/Rubiks_Cube/rubiks_cube.usd"


def print_diagnostics(diagnostic: Any) -> None:
    """Print the current logical state of the bin stacking behavior."""
    print("=========== logical state ==========")
    if diagnostic.bin_name:
        print("active bin info:")
        print(f"- bin_obj.name: {diagnostic.bin_name}")
        print(f"- bin_base: {diagnostic.bin_base}")
        print(f"- grasp_T:\n{diagnostic.grasp}")
        print(f"- is_grasp_reached: {diagnostic.grasp_reached}")
        print(f"- is_attached:  {diagnostic.attached}")
        print(f"- needs_flip:  {diagnostic.needs_flip}")
    else:
        print("<no active bin>")

    print("------------------------------------")


def random_bin_spawn_transform() -> tuple[np.ndarray, Any]:
    """Generate a random position and orientation for spawning a bin."""
    x = random.uniform(-0.15, 0.15)
    y = 1.5
    z = -0.15
    position = np.array([x, y, z])

    z = random.random() * 0.02 - 0.01
    w = random.random() * 0.02 - 0.01
    norm = np.sqrt(z**2 + w**2)
    quat = math_util.Quaternion([w / norm, 0, 0, z / norm])
    if random.random() > 0.5:
        print("<flip>")
        # flip the bin so it's upside down
        quat = quat * math_util.Quaternion([0, 0, 1, 0])
    else:
        print("<no flip>")

    return position, quat.vals


class BinStackingTask(BaseTask):
    """Manage bin spawning and tracking for the stacking task."""

    def __init__(self, env_path: str, assets: Ur10Assets) -> None:
        super().__init__("bin_stacking")
        self.assets = assets

        self.env_path = "/World/Ur10Table"
        self.bins = []
        self.stashed_bins = []
        self.on_conveyor = None

    def _spawn_bin(self, rigid_bin: Any) -> None:
        x, q = random_bin_spawn_transform()
        rigid_bin.set_world_pose(position=x, orientation=q)
        rigid_bin.set_linear_velocity(np.array([0, -0.30, 0]))
        rigid_bin.set_visibility(True)

    def post_reset(self) -> None:
        """Clear all bins and reset state after a world reset."""
        if len(self.bins) > 0:
            for rigid_bin in self.bins:
                self.scene.remove_object(rigid_bin.name)
            self.bins.clear()

        self.on_conveyor = None

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """Spawn a new randomly oriented bin if the previous bin has been placed."""
        spawn_new = False
        if self.on_conveyor is None:
            spawn_new = True
        else:
            (x, y, z), _ = self.on_conveyor.get_world_pose()
            is_on_conveyor = y > 0.0 and x > -0.4 and x < 0.4
            if not is_on_conveyor:
                spawn_new = True

        if spawn_new:
            name = f"bin_{len(self.bins)}"
            prim_path = self.env_path + f"/bins/{name}"
            add_reference_to_stage(usd_path=self.assets.small_klt_usd, prim_path=prim_path)
            self.on_conveyor = self.scene.add(CortexRigidPrim(name=name, prim_path=prim_path))

            self._spawn_bin(self.on_conveyor)
            self.bins.append(self.on_conveyor)


def main() -> None:
    """Set up and run the UR10 conveyor bin stacking demo."""
    world = CortexWorld()

    env_path = "/World/Ur10Table"
    ur10_assets = Ur10Assets()
    add_reference_to_stage(usd_path=ur10_assets.ur10_table_usd, prim_path=env_path)
    add_reference_to_stage(usd_path=ur10_assets.background_usd, prim_path="/World/Background")
    background_prim = XFormPrim(
        "/World/Background",
        positions=np.array([[10.00, 2.00, -1.18180]]),
        orientations=np.array([[0.7071, 0, 0, 0.7071]]),
    )
    robot = world.add_robot(CortexUr10(name="robot", prim_path=f"{env_path}/ur10"))

    obs = world.scene.add(
        VisualSphere(
            "/World/Ur10Table/Obstacles/FlipStationSphere",
            name="flip_station_sphere",
            position=np.array([0.73, 0.76, -0.13]),
            radius=0.2,
            visible=False,
        )
    )
    robot.register_obstacle(obs)
    obs = world.scene.add(
        VisualSphere(
            "/World/Ur10Table/Obstacles/NavigationDome",
            name="navigation_dome_obs",
            position=[-0.031, -0.018, -1.086],
            radius=1.1,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    az = np.array([1.0, 0.0, -0.3])
    ax = np.array([0.0, 1.0, 0.0])
    ay = np.cross(az, ax)
    R = math_util.pack_R(ax, ay, az)
    quat = math_util.matrix_to_quat(R)
    obs = world.scene.add(
        VisualCapsule(
            "/World/Ur10Table/Obstacles/NavigationBarrier",
            name="navigation_barrier_obs",
            position=[0.471, 0.276, -0.463 - 0.1],
            orientation=quat,
            radius=0.5,
            height=0.9,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    obs = world.scene.add(
        VisualCapsule(
            "/World/Ur10Table/Obstacles/NavigationFlipStation",
            name="navigation_flip_station_obs",
            position=np.array([0.766, 0.755, -0.5]),
            radius=0.5,
            height=0.5,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    world.add_task(BinStackingTask(env_path, ur10_assets))
    world.add_decider_network(behavior.make_decider_network(robot, print_diagnostics))

    if args.test:
        _test_frames = {"count": 0}

        def _test_done_cb() -> bool:
            _test_frames["count"] += 1
            return _test_frames["count"] >= 10

        world.run(simulation_app, is_done_cb=_test_done_cb)
    else:
        world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()

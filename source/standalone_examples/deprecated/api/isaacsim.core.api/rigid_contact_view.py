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

"""Demonstrate rigid body contact force queries using RigidPrim and GeometryPrim views."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.prims import GeometryPrim, RigidPrim

torch = import_module("torch")

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


class RigidViewExample:
    """Demonstrate contact force queries between rigid body views."""

    def __init__(self) -> None:
        self._array_container = torch.Tensor
        self.my_world = World(stage_units_in_meters=1.0, backend="torch")
        self.stage = simulation_app.context.get_stage()

    def make_env(self) -> None:
        """Create the environment with bottom boxes, top boxes, and ground plane views."""
        self.cube_height = 1.0
        self.top_cube_height = self.cube_height + 3.0
        self.cube_dx = 5.0
        self.cube_y = 2.0
        self.top_cube_y = self.cube_y + 0.0

        self.my_world._physics_context.set_gravity(-10)
        self.my_world.scene.add_default_ground_plane()

        for i in range(3):
            DynamicCuboid(
                prim_path=f"/World/Box_{i+1}", name=f"box_{i}", size=1.0, color=np.array([0.5, 0, 0]), mass=1.0
            )
            DynamicCuboid(
                prim_path=f"/World/TopBox_{i+1}",
                name=f"top_box_{i}",
                size=1.0,
                color=np.array([0.0, 0.0, 0.5]),
                mass=1.0,
            )

        # add top box as filters to the view to receive contacts between the bottom boxes and top boxes
        self._box_view = RigidPrim(
            prim_paths_expr="/World/Box_*",
            name="box_view",
            positions=self._array_container(
                [
                    [0, self.cube_y, self.cube_height],
                    [-self.cube_dx, self.cube_y, self.cube_height],
                    [self.cube_dx, self.cube_y, self.cube_height],
                ]
            ),
            contact_filter_prim_paths_expr=["/World/TopBox_*"],
        )
        # a view just to manipulate the top boxes
        self._top_box_view = RigidPrim(
            prim_paths_expr="/World/TopBox_*",
            name="top_box_view",
            positions=self._array_container(
                [
                    [0.0, self.top_cube_y, self.top_cube_height],
                    [-self.cube_dx, self.top_cube_y, self.top_cube_height],
                    [self.cube_dx, self.top_cube_y, self.top_cube_height],
                ]
            ),
            track_contact_forces=True,
        )

        # can get contact forces with non-rigid body prims such as geometry prims
        self._geom_view = GeometryPrim(
            prim_paths_expr="/World/defaultGroundPlane*",
            name="groundPlaneView",
            collisions=self._array_container([True]),
            track_contact_forces=True,
            prepare_contact_sensors=True,
            contact_filter_prim_paths_expr=["/World/Box_1", "/World/Box_2", "/World/Box_3"],
        )

        self.my_world.scene.add(self._box_view)
        self.my_world.scene.add(self._top_box_view)
        self.my_world.scene.add(self._geom_view)
        self.my_world.reset(soft=False)

    def play(self) -> None:
        """Run the simulation loop and periodically print contact forces."""
        self.make_env()
        reset_needed = False
        while simulation_app.is_running():
            if self.my_world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.my_world.is_playing():
                # deal with sim re-initialization after restarting sim
                if reset_needed:
                    # initialize simulation views
                    self.my_world.reset(soft=False)
                    reset_needed = False

            self.my_world.step(render=True)

            if self.my_world.current_time_step_index % 100 == 99:
                states = self._box_view.get_current_dynamic_state()
                top_states = self._top_box_view.get_current_dynamic_state()
                net_forces = self._box_view.get_net_contact_forces(None, dt=1 / 60)
                forces_matrix = self._box_view.get_contact_force_matrix(None, dt=1 / 60)
                top_net_forces = self._top_box_view.get_net_contact_forces(None, dt=1 / 60)
                print("==================================================================")
                print("Bottom box net forces: \n", net_forces)
                print("Top box net forces: \n", top_net_forces)
                print("Bottom box forces from top ones: \n", forces_matrix)
                print("Bottom box positions: \n", states.positions)
                print("Top box positions: \n", top_states.positions)
                print("Bottom box velocities: \n", states.linear_velocities)
                print("Top box velocities: \n", top_states.linear_velocities)

                print("ground net force from GeometryPrim : \n", self._geom_view.get_net_contact_forces(dt=1 / 60))
                print("ground force matrix from GeometryPrim: \n", self._geom_view.get_contact_force_matrix(dt=1 / 60))
                if args.test is True:
                    break
        simulation_app.close()


RigidViewExample().play()

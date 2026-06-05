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

"""Demonstrate retrieving detailed contact and friction data from rigid bodies."""

import argparse

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import RigidPrim

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


class RigidViewExample:
    """Manage rigid body contact data simulation."""

    def __init__(self):
        self.my_world = World(stage_units_in_meters=1.0, backend="numpy")
        self.stage = simulation_app.context.get_stage()
        self.g = 10
        self.count = 3

    def makeEnv(self):
        """Create the environment with cubes and contact filters."""
        self.cube_height = 1.0
        self.top_cube_height = self.cube_height + 3.0
        self.cube_dx = 5.0
        self.cube_y = 2.0
        self.top_cube_y = self.cube_y + 0.0

        self.my_world._physics_context.set_gravity(-10)
        self.my_world.scene.add_default_ground_plane()
        material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials",
            static_friction=0.5,
            dynamic_friction=0.5,
        )
        for i in range(self.count):
            DynamicCuboid(
                prim_path=f"/World/Box_{i+1}",
                name=f"box_{i}",
                size=1.0,
                color=np.array([0.5, 0, 0]),
                mass=1.0,
            ).apply_physics_material(material)

        # add top box as filters to the view to receive contacts between the bottom boxes and top boxes
        self._box_view = RigidPrim(
            prim_paths_expr="/World/Box_*",
            name="box_view",
            positions=np.array(
                [
                    [0, self.cube_y, self.cube_height],
                    [-self.cube_dx, self.cube_y, self.cube_height],
                    [self.cube_dx, self.cube_y, self.cube_height],
                ]
            ),
            contact_filter_prim_paths_expr=[
                "/World/defaultGroundPlane/GroundPlane/CollisionPlane",
            ],
            max_contact_count=3 * 10,
        )

        self.my_world.scene.add(self._box_view)
        self.my_world.reset(soft=False)

    def play(self):
        """Run the simulation loop and print contact data periodically."""
        self.makeEnv()
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

            forces = np.array([[self.g, 0, 0], [self.g, 0, 0], [self.g, 0, 0]])
            self._box_view.apply_forces(forces)
            self.my_world.step(render=True)
            if self.my_world.current_time_step_index % 100 == 99:
                # tangential forces
                (
                    friction_forces,
                    friction_points,
                    friction_pair_contacts_count,
                    friction_pair_contacts_start_indices,
                ) = self._box_view.get_friction_data(dt=1 / 60)

                # normal forces
                (
                    forces,  # only normal impulses
                    points,
                    normals,
                    distances,
                    pair_contacts_count,
                    pair_contacts_start_indices,
                ) = self._box_view.get_contact_force_data(dt=1 / 60)

                # pair_contacts_count, pair_contacts_start_indices, friction_pair_contacts_count, friction_pair_contacts_start_indices are tensors of size count x num_filters = (3x1)
                force_aggregate = np.zeros(
                    (
                        self._box_view._contact_view.num_shapes,
                        self._box_view._contact_view.num_filters,
                        3,
                    )
                )  # shape is count x num_filters x 3 = 3 x 1 x 1
                friction_force_aggregate = np.zeros(
                    (
                        self._box_view._contact_view.num_shapes,
                        self._box_view._contact_view.num_filters,
                        3,
                    )
                )  # shape is count x num_filters x 3 = 3 x 1 x 1
                effective_position = np.zeros(
                    (
                        self._box_view._contact_view.num_shapes,
                        self._box_view._contact_view.num_filters,
                        3,
                    )
                )
                friction_effective_position = np.zeros(
                    (
                        self._box_view._contact_view.num_shapes,
                        self._box_view._contact_view.num_filters,
                        3,
                    )
                )
                # process contacts for each pair i, j
                for i in range(pair_contacts_count.shape[0]):
                    for j in range(pair_contacts_count.shape[1]):
                        start_idx = pair_contacts_start_indices[i, j]
                        friction_start_idx = friction_pair_contacts_start_indices[i, j]
                        count = pair_contacts_count[i, j]
                        friction_count = friction_pair_contacts_count[i, j]
                        # sum/average across all the contact pairs
                        pair_forces = forces[start_idx : start_idx + count]  # all the pair forces, shape [count, 3]
                        pair_normals = normals[start_idx : start_idx + count]  # all the pair forces, shape [count, 3]

                        force_aggregate[i, j] = np.sum(pair_forces * pair_normals, axis=0)
                        effective_position[i, j] = np.sum(points[start_idx : start_idx + count], axis=0) / count

                        # sum/average across all the friction pairs
                        pair_forces = friction_forces[
                            friction_start_idx : friction_start_idx + friction_count
                        ]  # all the pair forces, shape [count, 3]
                        friction_force_aggregate[i, j] = np.sum(pair_forces, axis=0)
                        friction_effective_position[i, j] = (
                            np.sum(
                                friction_points[friction_start_idx : friction_start_idx + friction_count],
                                axis=0,
                            )
                            / friction_count
                        )

                print("==================================================")
                print("friction forces: \n", friction_force_aggregate)
                # applied tangential forces (m*g) is larger than the maximum dynamic friction force of mu * mg so the friction forces will be capped to that
                print("contact forces: \n", force_aggregate)
                # boxes will start sliding so the effective position of the friction/contact points will change over time
                print("friction point: \n", friction_effective_position)
                print("contact point: \n", effective_position)
                if args.test is True:
                    break

        simulation_app.close()


RigidViewExample().play()

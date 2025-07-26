# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os
import unittest

import carb
import numpy as np
import omni.kit.test
import torch
import warp as wp
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.core.prims import RigidPrim, XFormPrim
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats as euler_angles_to_quats_numpy
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.stage import create_new_stage_async, get_current_stage, update_stage_async
from isaacsim.core.utils.torch.rotations import euler_angles_to_quats as euler_angles_to_quats_torch

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from isaacsim.core.utils.types import DynamicsViewState
from isaacsim.core.utils.warp.rotations import euler_angles_to_quats as euler_angles_to_quats_warp
from omni.physx.scripts import physicsUtils

from .common import CoreTestCase

default_physics_material = {"static_friction": 1.0, "dynamic_friction": 1.0, "restitution": 0.0}

default_sim_params = {
    ### Per-scene settings
    "use_gpu": False,
    "worker_thread_count": 4,
    "solver_type": 1,  # 0: PGS, 1:TGS
    "bounce_threshold_velocity": 0.2,
    "friction_offset_threshold": 0.04,  # A threshold of contact separation distance used to decide if a contact
    # point will experience friction forces.
    "friction_correlation_distance": 0.025,  # Contact points can be merged into a single friction anchor if the
    # distance between the contacts is smaller than correlation distance.
    # disabling these can be useful for debugging
    "enable_sleeping": True,
    "enable_stabilization": False,
    # GPU buffers
    "gpu_max_rigid_contact_count": 512 * 1024,
    "gpu_max_rigid_patch_count": 80 * 1024,
    "gpu_found_lost_pairs_capacity": 1024,
    "gpu_found_lost_aggregate_pairs_capacity": 1024,
    "gpu_total_aggregate_pairs_capacity": 1024,
    "gpu_max_soft_body_contacts": 1024 * 1024,
    "gpu_max_particle_contacts": 1024 * 1024,
    "gpu_heap_capacity": 64 * 1024 * 1024,
    "gpu_temp_buffer_capacity": 16 * 1024 * 1024,
    "gpu_max_num_partitions": 8,
    ### Per-actor settings ( can override in actor_options )
    "solver_position_iteration_count": 4,
    "solver_velocity_iteration_count": 1,
    "sleep_threshold": 0.0,  # Mass-normalized kinetic energy threshold below which an actor may go to sleep.
    # Allowed range [0, max_float).
    "stabilization_threshold": 0.0,  # Mass-normalized kinetic energy threshold below which an actor may
    # participate in stabilization. Allowed range [0, max_float).
    ### Per-body settings ( can override in actor_options )
    "enable_gyroscopic_forces": False,
    "density": 1000.0,  # density to be used for bodies that do not specify mass or density
    "max_depenetration_velocity": 100.0,
    ### Per-shape settings ( can override in actor_options )
    "contact_offset": 0.02,
    "rest_offset": 0.001,
    "gravity": [0.0, 0.0, 0.0],
    "dt": 1.0 / 60.0,
    "substeps": 1,
    "use_gpu_pipeline": False,
    "add_ground_plane": False,
    "default_physics_material": default_physics_material,
}


class TestRigidPrimView(CoreTestCase):
    async def setUp(self):
        await super().setUp()
        World.clear_instance()
        self._sim_params = default_sim_params
        self._test_cfg = dict()

    async def tearDown(self):
        carb.settings.get_settings().set_bool("/physics/suppressReadback", False)
        await super().tearDown()

    @unittest.skipIf(os.getenv("ETM_ACTIVE"), "skipped in ETM")
    async def test_rigid_prim_view_gpu_pipeline(self):
        test_configs = {"use_gpu": True, "use_gpu_pipeline": True, "device": "gpu"}
        for backend in ["torch", "warp"]:
            test_configs["backend"] = backend

            self._sim_params["use_gpu"] = test_configs["use_gpu"]
            self._sim_params["use_gpu_pipeline"] = test_configs["use_gpu_pipeline"]
            self._test_cfg["use_gpu"] = test_configs["use_gpu"]
            self._test_cfg["use_gpu_pipeline"] = test_configs["use_gpu_pipeline"]
            self._test_cfg["backend"] = test_configs["backend"]
            self._test_cfg["device"] = test_configs["device"]

            if backend == "torch":
                self.euler_angles_to_quats = euler_angles_to_quats_torch
                self.isclose = torch.isclose
                if self._test_cfg["device"] == "gpu":
                    self._array_container = lambda x: torch.tensor(x, device=self._device, dtype=torch.float32)
                    self._device = "cuda:0"

            elif backend == "warp":
                self.euler_angles_to_quats = euler_angles_to_quats_warp
                self.isclose = np.isclose
                if self._test_cfg["device"] == "gpu":
                    self._device = "cuda:0"
                self._array_container = lambda x: wp.array(x, device=self._device, dtype=wp.float32)

            await self._runner()

    @unittest.skipIf(os.getenv("ETM_ACTIVE"), "skipped in ETM")
    async def test_rigid_prim_view_cpu_pipeline(self):
        test_configs = {"use_gpu_pipeline": False, "device": "cpu"}

        for backend in ["numpy", "torch", "warp"]:
            for gpu_sim in [False, True]:
                self._sim_params["use_gpu"] = gpu_sim
                self._sim_params["use_gpu_pipeline"] = test_configs["use_gpu_pipeline"]
                self._test_cfg["use_gpu"] = gpu_sim
                self._test_cfg["use_gpu_pipeline"] = test_configs["use_gpu_pipeline"]
                self._test_cfg["backend"] = backend
                self._test_cfg["device"] = test_configs["device"]

                if self._test_cfg["backend"] == "numpy":
                    self._array_container = np.array
                    self.euler_angles_to_quats = euler_angles_to_quats_numpy
                    self.isclose = np.isclose
                    self._device = "cpu"
                elif self._test_cfg["backend"] == "torch":
                    self.euler_angles_to_quats = euler_angles_to_quats_torch
                    self.isclose = torch.isclose
                    self._array_container = lambda x: torch.tensor(x, device=self._device, dtype=torch.float32)
                    self._device = "cpu"
                elif self._test_cfg["backend"] == "warp":
                    self.euler_angles_to_quats = euler_angles_to_quats_warp
                    self.isclose = np.isclose
                    self._array_container = wp.array
                    self._device = "cpu"
                    self._array_container = lambda x: wp.array(x, device=self._device, dtype=wp.float32)

                await self._runner()

    async def _setup_scene(self):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World(sim_params=self._sim_params, backend=self._test_cfg["backend"], device=self._device)
        await self._my_world.initialize_simulation_context_async()
        self._my_world._physics_context.set_gravity(0)

        num_cubes = 3
        for i in range(num_cubes):
            define_prim(prim_path=f"/World/Frame_{i+1}")
            DynamicCuboid(
                prim_path=f"/World/Frame_{i+1}/Cube", name=f"cube_{i}", size=1.0, color=np.array([0.5, 0, 0]), mass=0.0
            )
        await update_stage_async()
        self._cubes_view = RigidPrim(
            prim_paths_expr="/World/Frame_[1-3]/Cube",
            name="cubes_view",
            positions=self._array_container([[0.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, -10.0, 0.0]]),
        )
        self._my_world.scene.add(self._cubes_view)

    async def _setup_contacts_scene(self):
        self.cube_height = 0.502
        self.top_cube_height = self.cube_height + 1.02
        World.clear_instance()
        await create_new_stage_async()
        self.g = -10.0
        self._my_world = World(sim_params=self._sim_params, backend=self._test_cfg["backend"], device=self._device)
        await self._my_world.initialize_simulation_context_async()
        self._my_world._physics_context.set_gravity(self.g)
        self._my_world.scene.add_default_ground_plane()

        for i in range(3):
            DynamicCuboid(
                prim_path=f"/World/Box_{i+1}", name=f"box_{i}", size=1.0, color=np.array([0.5, 0, 0]), mass=1.0
            )
            DynamicCuboid(
                prim_path=f"/World/TopBox_{i+1}", name=f"top_box_{i}", size=1.0, color=np.array([0.5, 0, 0]), mass=1.0
            )
        # a view to receive contacts between the bottom boxes and top boxes
        self._box_view = RigidPrim(
            prim_paths_expr="/World/Box_*",
            name="box_view",
            positions=self._array_container(
                [[10.0, 10.0, self.cube_height], [10.0, 20.0, self.cube_height], [10.0, 30.0, self.cube_height]]
            ),
            contact_filter_prim_paths_expr=["/World/TopBox_*"],
            max_contact_count=3 * 10,  # 3 box each with maximum of 20 potential contacts
        )
        # a view just to manipulate the top boxes
        self._top_box_view = RigidPrim(
            prim_paths_expr="/World/TopBox_*",
            name="top_box_view",
            positions=self._array_container(
                [
                    [10.0, 10.0, self.top_cube_height],
                    [10.0, 20.0, self.top_cube_height],
                    [10.0, 30.0, self.top_cube_height],
                ]
            ),
            track_contact_forces=True,
        )
        # contact reporter don't get data for sleeping bodies
        self._box_view.set_sleep_thresholds([0, 0, 0])
        self._top_box_view.set_sleep_thresholds([0, 0, 0])
        self._my_world.scene.add(self._box_view)
        self._my_world.scene.add(self._top_box_view)

    async def _setup_friction_scene(self):
        self.cube_height = 0.502
        self.top_cube_height = self.cube_height + 1.02
        World.clear_instance()
        await create_new_stage_async()
        self.g = -10.0
        self._sim_params["dt"] = 1.0 / 60
        self._sim_params["gravity"] = [0.0, 0.0, self.g]
        self._sim_params["default_physics_material"] = {
            "static_friction": 0.5,
            "dynamic_friction": 0.5,
            "restitution": 0.0,
        }
        self._my_world = World(sim_params=self._sim_params, backend=self._test_cfg["backend"], device=self._device)
        await self._my_world.initialize_simulation_context_async()
        self._my_world.scene.add_default_ground_plane()
        stage = get_current_stage()
        for i in range(3):
            path = f"/World/Box_{i+1}"
            DynamicCuboid(prim_path=path, name=f"box_{i}", size=1.0, color=np.array([0.5, 0, 0]), mass=1.0)
            physicsUtils.add_physics_material_to_prim(
                stage,
                stage.GetPrimAtPath(path),
                str(self._my_world.get_physics_context().get_current_physics_scene_prim().GetPath())
                + "/defaultMaterial",
            )

        # a view to receive contacts between the bottom boxes and top boxes
        self._box_view = RigidPrim(
            prim_paths_expr="/World/Box_*",
            name="box_view",
            positions=self._array_container(
                [[10.0, 10.0, self.cube_height], [10.0, 20.0, self.cube_height], [10.0, 30.0, self.cube_height]]
            ),
            contact_filter_prim_paths_expr=["/World/defaultGroundPlane/GroundPlane/CollisionPlane"],
            max_contact_count=3 * 10,  # 3 box each with maximum of 20 potential contacts
            prepare_contact_sensors=True,
            disable_stablization=True,
        )
        self._my_world.scene.add(self._box_view)

    async def _runner(self):
        await self._setup_scene()
        for indexed in [False, True]:
            self._test_cfg["indexed"] = indexed
            print(indexed, self._test_cfg)
            await self._setup_scene()
            await self.com_test()
            await self.inertias_test()
            await self.world_poses_test()
            await self.linear_velocities_test()
            await self.angular_velocities_test()
            await self.transforms_test()
            await self.apply_forces_test()
            await self.apply_forces_and_torques_at_pos_test(is_global=True, apply_at_pos=False)
            await self.apply_forces_and_torques_at_pos_test(is_global=True, apply_at_pos=True)
            await self.apply_forces_and_torques_at_pos_test(is_global=False, apply_at_pos=False)
            await self.apply_forces_and_torques_at_pos_test(is_global=False, apply_at_pos=True)
            await self.velocities_test()
            await self.enable_disable_physics_test()
            await self.enable_disable_gravity_test()
            await self.default_state_post_reset_test()
            await self.default_state_before_reset_test()
            await self.masses_test()
            await self.densities_test()
            await self.sleep_thresholds_test()

            await self._setup_contacts_scene()
            await self.contact_force_test()

            await self._setup_friction_scene()
            await self.friction_force_test(force_multiplier=0.5)
            await self._setup_friction_scene()
            await self.friction_force_test(force_multiplier=1.0)
            await self._setup_friction_scene()
            await self.friction_force_test(force_multiplier=2.0)

        # test USD paths
        if self._device == "cpu":
            for indexed in [False, True]:
                print("USD", indexed, self._test_cfg)
                self._test_cfg["indexed"] = indexed
                await self._setup_scene()
                await self.local_poses_test()  # this is a cpu only test since it uses XFormPrim which doesn't use physics tensors with gpu pipeline
                await self._setup_scene()
                await self.world_poses_test(usd=True)
                await self.local_poses_test(usd=True)
                await self.linear_velocities_test(usd=True)
                await self.angular_velocities_test(usd=True)
                await self.velocities_test(usd=True)
                await self.enable_disable_physics_test(usd=True)
                await self._setup_scene()
                await self.enable_disable_gravity_test(usd=True)
                await self.masses_test(usd=True)
                await self.densities_test(usd=True)
                await self.sleep_thresholds_test(usd=True)
                await self.default_state_post_reset_test(usd=True)

        self._my_world.clear_instance()

    async def apply_forces_and_torques_at_pos_test(self, is_global, apply_at_pos):
        print(
            "Apply %s forces and torques %s test"
            % ("global" if is_global else "local", "at pos" if apply_at_pos else "at COM")
        )
        await self._my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else [0, 1, 2]

        if self._test_cfg["backend"] == "warp":
            new_positions = np.array([[20.0, -20.0, 10.0], [30.0, 30.0, 0], [-40, -40, 0]])[indices]
            new_orientations = euler_angles_to_quats_numpy(
                euler_angles=np.array([[np.pi / 2, 0, 0], [0, np.pi / 2, 0], [0, 0, np.pi / 2]])[indices],
                device=self._device,
            )  # [y->z, z->x, x->y] rotation
            new_positions = wp.from_numpy(new_positions, dtype=wp.float32, device=self._device)
            new_orientations = wp.from_numpy(new_orientations, dtype=wp.float32, device=self._device)
            self._cubes_view.set_world_poses(positions=new_positions, orientations=new_orientations, indices=indices)
        else:
            new_positions = self._array_container([[20.0, -20.0, 10.0], [30.0, 30.0, 0], [-40, -40, 0]])[indices]
            new_orientations = self.euler_angles_to_quats(
                euler_angles=self._array_container([[np.pi / 2, 0, 0], [0, np.pi / 2, 0], [0, 0, np.pi / 2]])[indices],
                device=self._device,
            )  # [y->z, z->x, x->y] rotation
            self._cubes_view.set_world_poses(positions=new_positions, orientations=new_orientations, indices=indices)

        position = None
        if self._test_cfg["backend"] == "warp":
            if is_global:
                force = np.array([[3000, 0, 0], [-3000, 0, 0], [3000, 0, 0]])[indices]
                torque = np.array([[0, 0, 3000], [0, 0, -3000], [0, 0, 3000]])[indices]
                if apply_at_pos:
                    position = (np.array([[20.0, -19.0, 10.0], [30.0, 31.0, 0], [-40, -39, 0]]))[indices]
            else:
                force = (np.array([[3000, 0, 0], [0, 0, 3000], [0, 3000, 0]]))[indices]  # global x forces
                torque = (np.array([[0, 3000, 0], [3000, 0, 0], [0, 0, 3000]]))[indices]  # global z torques
                if apply_at_pos:
                    position = (np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]]))[indices]  # cancel the torques
            force = wp.from_numpy(force, dtype=wp.float32, device=self._device)
            torque = wp.from_numpy(torque, dtype=wp.float32, device=self._device)
            position = wp.from_numpy(position, dtype=wp.float32, device=self._device) if position is not None else None
        else:
            if is_global:
                force = self._array_container([[3000, 0, 0], [-3000, 0, 0], [3000, 0, 0]])[indices]
                torque = self._array_container([[0, 0, 3000], [0, 0, -3000], [0, 0, 3000]])[indices]
                if apply_at_pos:
                    position = (self._array_container([[20.0, -19.0, 10.0], [30.0, 31.0, 0], [-40, -39, 0]]))[indices]
            else:
                force = (self._array_container([[3000, 0, 0], [0, 0, 3000], [0, 3000, 0]]))[indices]  # global x forces
                torque = (self._array_container([[0, 3000, 0], [3000, 0, 0], [0, 0, 3000]]))[
                    indices
                ]  # global z torques
                if apply_at_pos:
                    position = (self._array_container([[0, 0, -1], [0, -1, 0], [-1, 0, 0]]))[
                        indices
                    ]  # cancel the torques

        self._cubes_view.apply_forces_and_torques_at_pos(force, torque, position, indices, is_global)

        self._my_world.step_async()
        await update_stage_async()

        current_linear_velocities = self._cubes_view.get_linear_velocities(indices)
        current_angular_velocities = self._cubes_view.get_angular_velocities(indices)
        # print("linear velocities ", current_linear_velocities )
        # print("angular velocities ", current_angular_velocities )

        if self._test_cfg["backend"] == "torch":
            current_linear_velocities = current_linear_velocities.cpu().numpy()
            current_angular_velocities = current_angular_velocities.cpu().numpy()
        elif self._test_cfg["backend"] == "warp":
            current_linear_velocities = current_linear_velocities.numpy()
            current_angular_velocities = current_angular_velocities.numpy()

        # print(current_linear_velocities)
        # print(current_angular_velocities)

        # linear velocity test
        self.assertTrue(np.isclose(current_linear_velocities[:, 1:], np.array([[0, 0], [0, 0], [0, 0]])[indices]).all())
        self.assertTrue(np.logical_not(np.isclose(current_linear_velocities[:, 0], np.array([0, 0, 0])[indices])).all())
        # angular velocity test
        if apply_at_pos:
            self.assertTrue(
                np.isclose(
                    current_angular_velocities[:, :], np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices], atol=1e-06
                ).all()
            )
        else:
            self.assertTrue(
                np.isclose(
                    current_angular_velocities[:, :-1],
                    np.array([[0, 0], [0, 0], [0, 0]])[indices].squeeze(),
                    atol=1e-06,
                ).all()
            )
            self.assertTrue(
                np.logical_not(
                    np.isclose(current_angular_velocities[:, -1], np.array([0, 0, 0])[indices].squeeze())
                ).all()
            )

    async def friction_force_test(self, force_multiplier):
        print("friction force test with external force f= %.3f  * (mu * m * g)" % force_multiplier)
        await self._my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        view_size = 2 if self._test_cfg["indexed"] else 3
        for i in range(5):
            self._my_world.step_async()
            await update_stage_async()

        # print("masses = \n", self._top_box_view.get_masses())
        forces = np.array([[self.g, 0, 0], [self.g, 0, 0], [self.g, 0, 0]]) * force_multiplier
        forces = self._array_container(forces.tolist() if indices is None else forces[indices].tolist())
        self._box_view.apply_forces(forces, indices)

        self._my_world.step_async()
        await update_stage_async()

        states = self._box_view.get_current_dynamic_state()
        (
            forces,
            points,
            pair_contacts_count,
            pair_contacts_start_indices,
        ) = self._box_view.get_friction_data(indices, dt=self._sim_params["dt"])
        # print(
        #     "forces= \n",
        #     forces,
        #     "\npair_contacts_count= \n",
        #     pair_contacts_count,
        #     "\npair_contacts_start_indices= \n",
        #     pair_contacts_start_indices,
        # )
        if self._test_cfg["backend"] == "torch":
            pair_contacts_count = pair_contacts_count.cpu()
            pair_contacts_start_indices = pair_contacts_start_indices.cpu()
            forces = forces.cpu()
            points = points.cpu()

        if self._test_cfg["backend"] != "numpy":
            pair_contacts_count = pair_contacts_count.numpy()
            pair_contacts_start_indices = pair_contacts_start_indices.numpy()
            forces = forces.numpy()
            points = points.numpy()
        force_aggregate = np.zeros((view_size, pair_contacts_count.shape[1], 3))  # shape # view_size x num_filters x 3
        effective_position = np.zeros((view_size, pair_contacts_count.shape[1], 3))
        for i in range(pair_contacts_count.shape[0]):
            for j in range(pair_contacts_count.shape[1]):
                start_idx = pair_contacts_start_indices[i, j]
                count = pair_contacts_count[i, j]
                if count > 0:
                    pair_forces = forces[start_idx : start_idx + count]  # all the pair forces, shape [count, 3]
                    force_aggregate[i, j] = np.sum(pair_forces, axis=0)  # sum across anchor points and contact points
                    effective_position[i, j] = np.sum(points[start_idx : start_idx + count], axis=0) / count
        m = 1
        F_ext = -force_multiplier * self.g * m
        F_t_max = -self._sim_params["default_physics_material"]["static_friction"] * self.g * m
        friction_force = min(F_ext, F_t_max)
        # print(F_ext,F_t_max,friction_force)
        if self._test_cfg["backend"] == "warp":
            # position test
            self.assertTrue(
                self.isclose(
                    states.positions.numpy()[indices],
                    np.array([[10.0, 10, 0.5], [10.0, 20.0, 0.5], [10.0, 30.0, 0.5]])[indices],
                    atol=1.0e-2,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    effective_position.squeeze(),
                    np.array([[10.0, 10, 0.0], [10.0, 20.0, 0.0], [10.0, 30.0, 0.0]])[indices],
                    atol=1.0e-2,
                ).all()
            )
            # similar for the force aggregate
            self.assertTrue(
                self.isclose(
                    force_aggregate,  # tangential compoenent
                    np.array([[friction_force, 0, 0], [friction_force, 0, 0], [friction_force, 0, 0]])[indices],
                    atol=2.0e-1,
                ).all()
            )
        else:
            # position test
            self.assertTrue(
                self.isclose(
                    states.positions[indices],
                    self._array_container([[10.0, 10, 0.5], [10.0, 20.0, 0.5], [10.0, 30.0, 0.5]])[indices].squeeze(),
                    atol=1.0e-2,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    self._array_container(effective_position).squeeze(),
                    self._array_container([[10.0, 10, 0.0], [10.0, 20.0, 0.0], [10.0, 30.0, 0.0]])[indices].squeeze(),
                    atol=1.0e-2,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    self._array_container(force_aggregate).squeeze(),  # tangential compoenent
                    self._array_container([[friction_force, 0, 0], [friction_force, 0, 0], [friction_force, 0, 0]])[
                        indices
                    ].squeeze(),
                    atol=2.0e-1,
                ).all()
            )

    async def contact_force_test(self):
        print("contact force test")
        wp.config.verify_cuda = True
        await self._my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        view_size = 2 if self._test_cfg["indexed"] else 3
        for i in range(5):
            self._my_world.step_async()
            await update_stage_async()

        states = self._box_view.get_current_dynamic_state()
        top_states = self._top_box_view.get_current_dynamic_state()
        top_net_forces = self._top_box_view.get_net_contact_forces(indices, dt=self._sim_params["dt"])
        net_forces = self._box_view.get_net_contact_forces(indices, dt=self._sim_params["dt"])
        forces_matrix = self._box_view.get_contact_force_matrix(indices, dt=self._sim_params["dt"])
        (
            forces,
            points,
            normals,
            distances,
            pair_contacts_count,
            pair_contacts_start_indices,
        ) = self._box_view.get_contact_force_data(indices, dt=self._sim_params["dt"])
        # print("final forces: \n", net_forces)
        # print("matrix forces: \n", forces_matrix)
        # print("final positions: \n", states.positions)
        # print("final top positions: \n", top_states.positions)
        # print("final linear_velocities: \n", states.linear_velocities)
        # print("final top linear_velocities: \n", top_states.linear_velocities)
        # print("forces= \n" ,forces, "pair_contacts_count= \n" ,pair_contacts_count, "pair_contacts_start_indices= \n" ,pair_contacts_start_indices)
        # print("points= \n" ,points, "\nnormal= \n" , normals, "\ndistances= \n" , distances)
        if self._test_cfg["backend"] == "torch":
            pair_contacts_count = pair_contacts_count.cpu()
            pair_contacts_start_indices = pair_contacts_start_indices.cpu()
            normals = normals.cpu()
            forces = forces.cpu()
            points = points.cpu()

        if self._test_cfg["backend"] != "numpy":
            pair_contacts_count = pair_contacts_count.numpy()
            pair_contacts_start_indices = pair_contacts_start_indices.numpy()
            normals = normals.numpy()
            forces = forces.numpy()
            points = points.numpy()

        force_aggregate = np.zeros((view_size, 1, 3))
        effective_position = np.zeros((view_size, 1, 3))
        for i in range(pair_contacts_count.shape[0]):
            for j in range(pair_contacts_count.shape[1]):
                start_idx = pair_contacts_start_indices[i, j]
                count = pair_contacts_count[i, j]
                if count > 0:
                    pair_forces = forces[start_idx : start_idx + count] * normals[start_idx : start_idx + count]
                    force_aggregate[i, j] = np.sum(pair_forces, axis=0)
                    effective_position[i, j] = np.sum(points[start_idx : start_idx + count], axis=0) / count

        if self._test_cfg["backend"] == "warp":
            # position test
            self.assertTrue(
                self.isclose(
                    states.positions.numpy()[indices],
                    np.array([[10.0, 10, 0.5], [10.0, 20.0, 0.5], [10.0, 30.0, 0.5]])[indices],
                    atol=1.0e-3,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    effective_position.squeeze(),
                    np.array([[10.0, 10, 1.0], [10.0, 20.0, 1.0], [10.0, 30.0, 1.0]])[indices],
                    atol=1.0e-2,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_states.positions.numpy()[indices],
                    np.array([[10.0, 10, 1.5], [10.0, 20.0, 1.5], [10.0, 30.0, 1.5]])[indices],
                    atol=1.0e-2,
                ).all()
            )
            # velocity test
            self.assertTrue(
                self.isclose(
                    states.linear_velocities.numpy()[indices],
                    np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices],
                    atol=1.0e-1,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_states.linear_velocities.numpy()[indices],
                    np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices],
                    atol=2.0e-1,
                ).all()
            )

            # force test for filter 0
            self.assertTrue(
                self.isclose(
                    forces_matrix.numpy()[:, 0, :],
                    np.array([[0, 0, self.g], [0, 0, self.g], [0, 0, self.g]])[indices],
                    atol=2.0e-1,
                ).all()
            )
            # similar for the force aggregate
            self.assertTrue(
                self.isclose(
                    force_aggregate[:, 0, :],
                    np.array([[0, 0, self.g], [0, 0, self.g], [0, 0, self.g]])[indices],
                    atol=2.0e-1,
                ).all()
            )

            # force test : net forces on box
            self.assertTrue(
                self.isclose(
                    net_forces.numpy(),
                    np.array([[0, 0, -self.g], [0, 0, -self.g], [0, 0, -self.g]])[indices],
                    atol=2.0e-1,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_net_forces.numpy(), np.array([[0, 0, 10], [0, 0, 10], [0, 0, 10]])[indices], atol=2.0e-1
                ).all()
            )
        else:
            # print(states.positions[indices], "\n", self._array_container([[10.0, 10, 0.5], [10.0, 20.0, 0.5], [10.0, 30.0, 0.5]])[indices].squeeze())
            # position test
            self.assertTrue(
                self.isclose(
                    states.positions[indices],
                    self._array_container([[10.0, 10, 0.5], [10.0, 20.0, 0.5], [10.0, 30.0, 0.5]])[indices].squeeze(),
                    atol=1.0e-2,
                ).all()
            )
            # print(self._array_container(effective_position).squeeze(), " \n", self._array_container([[10.0, 10, 1.0], [10.0, 20.0, 1.0], [10.0, 30.0, 1.0]])[indices].squeeze())
            self.assertTrue(
                self.isclose(
                    self._array_container(effective_position).squeeze(),
                    self._array_container([[10.0, 10, 1.0], [10.0, 20.0, 1.0], [10.0, 30.0, 1.0]])[indices].squeeze(),
                    atol=1.0e-2,
                ).all()
            )
            # print(top_states.positions[indices], "\n", self._array_container([[10.0, 10, 1.5], [10.0, 20.0, 1.5], [10.0, 30.0, 1.5]])[indices].squeeze())
            self.assertTrue(
                self.isclose(
                    top_states.positions[indices],
                    self._array_container([[10.0, 10, 1.5], [10.0, 20.0, 1.5], [10.0, 30.0, 1.5]])[indices].squeeze(),
                    atol=1.0e-2,
                ).all()
            )
            # velocity test
            self.assertTrue(
                self.isclose(
                    states.linear_velocities[indices],
                    self._array_container([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices].squeeze(),
                    atol=1.0e-1,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_states.linear_velocities[indices],
                    self._array_container([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices].squeeze(),
                    atol=2.0e-1,
                ).all()
            )

            # force test for filter 0
            self.assertTrue(
                self.isclose(
                    forces_matrix[:, 0, :].squeeze(),
                    self._array_container([[0, 0, self.g], [0, 0, self.g], [0, 0, self.g]])[indices].squeeze(),
                    atol=2.0e-1,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    self._array_container(force_aggregate[:, 0, :]).squeeze(),
                    self._array_container([[0, 0, self.g], [0, 0, self.g], [0, 0, self.g]])[indices].squeeze(),
                    atol=2.0e-1,
                ).all()
            )

            # force test : net forces on box
            self.assertTrue(
                self.isclose(
                    net_forces.squeeze(),
                    self._array_container([[0, 0, -self.g], [0, 0, -self.g], [0, 0, -self.g]])[indices].squeeze(),
                    atol=2.0e-1,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_net_forces.squeeze(),
                    self._array_container([[0, 0, -self.g], [0, 0, -self.g], [0, 0, -self.g]])[indices].squeeze(),
                    atol=2.0e-1,
                ).all()
            )

    async def com_test(self):
        if self._device == "cpu":
            await self._my_world.reset_async()
            indices = [1, 2] if self._test_cfg["indexed"] else [0, 1, 2]

            cur_pos, cur_ori = self._cubes_view.get_coms()
            if self._test_cfg["backend"] == "warp":
                new_pos = wp.from_numpy(cur_pos.numpy()[indices] + 0.1, dtype=wp.float32, device=self._device)
                new_ori = wp.from_numpy(cur_ori.numpy()[indices], dtype=wp.float32, device=self._device)
            else:
                new_pos = cur_pos[indices] + 0.1
                new_ori = cur_ori[indices]
            self._cubes_view.set_coms(new_pos, new_ori, indices)
            pos, ori = self._cubes_view.get_coms(indices)
            if self._test_cfg["backend"] == "warp":
                new_pos = new_pos.numpy()
                pos = pos.numpy()
                new_ori = new_ori.numpy()
                ori = ori.numpy()
            self.assertTrue(self.isclose(new_pos, pos).all())
            self.assertTrue(self.isclose(new_ori, ori).all())

    async def inertias_test(self):
        await self._my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else [0, 1, 2]

        prev_values = self._cubes_view.get_inertias()
        if self._test_cfg["backend"] == "warp":
            values = prev_values.numpy()[indices]
            values[:, [0, 4, 8]] += 0.01
            new_values = wp.from_numpy(values, dtype=wp.float32, device=self._device)
        else:
            new_values = prev_values[indices]
            new_values[:, [0, 4, 8]] += 0.01
        self._cubes_view.set_inertias(new_values, indices)
        cur_values = self._cubes_view.get_inertias(indices)
        if self._test_cfg["backend"] == "warp":
            new_values = new_values.numpy()
            cur_values = cur_values.numpy()
        self.assertTrue(self.isclose(new_values, cur_values).all())

        inv_masses = self._cubes_view.get_inv_inertias()
        self.assertTrue(inv_masses.shape == (self._cubes_view.count, 9))

    async def world_poses_test(self, usd=False):
        await self._my_world.reset_async()
        if usd:
            await self._my_world.stop_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        new_positions = np.array([[25.0, -20.0, 10.0], [15.0, 10.0, 0.0], [-40.0, -40.0, 0.0]])
        new_orientations = euler_angles_to_quats_numpy(
            euler_angles=np.array([[0, np.pi / 4.0, 0], [0, 0, np.pi / 4.0], [0, 0, -np.pi / 8.0]])
        )
        new_positions = self._array_container(
            new_positions.tolist() if indices is None else new_positions[indices].tolist()
        )
        new_orientations = self._array_container(
            new_orientations.tolist() if indices is None else new_orientations[indices].tolist()
        )

        self._cubes_view.set_world_poses(positions=new_positions, orientations=new_orientations, indices=indices)
        self._my_world.step_async(0)
        await update_stage_async()
        current_positions, current_orientations = self._cubes_view.get_world_poses(indices=indices)

        if self._test_cfg["backend"] == "warp":
            current_positions = current_positions.numpy()
            new_positions = new_positions.numpy()
            current_orientations = current_orientations.numpy()
            new_orientations = new_orientations.numpy()

        self.assertTrue(self.isclose(current_positions, new_positions, atol=1e-4).all())
        self.assertTrue(self.isclose(current_orientations, new_orientations, atol=1e-4).all())

        return

    async def local_poses_test(self, usd=False):
        await self._my_world.reset_async()
        if usd:
            await self._my_world.stop_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        new_positions = np.array([[25.0, -20.0, 10.0], [15.0, 10.0, 0.0], [-40.0, -40.0, 0.0]])
        new_orientations = euler_angles_to_quats_numpy(
            euler_angles=np.array([[0, np.pi / 4.0, 0], [0, 0, np.pi / 4.0], [0, 0, -np.pi / 8.0]])
        )
        new_positions = self._array_container(
            new_positions.tolist() if indices is None else new_positions[indices].tolist()
        )
        new_orientations = self._array_container(
            new_orientations.tolist() if indices is None else new_orientations[indices].tolist()
        )

        self._cubes_view.set_local_poses(translations=new_positions, orientations=new_orientations, indices=indices)
        self._my_world.step_async(0)
        await update_stage_async()
        current_positions, current_orientations = self._cubes_view.get_local_poses(indices=indices)
        if self._test_cfg["backend"] == "warp":
            current_positions = current_positions.numpy()
            new_positions = new_positions.numpy()
            current_orientations = current_orientations.numpy()
            new_orientations = new_orientations.numpy()
        self.assertTrue(self.isclose(current_positions, new_positions, atol=1e-4).all())
        self.assertTrue(self.isclose(current_orientations, new_orientations, atol=1e-4).all())
        frames_view = XFormPrim(prim_paths_expr="/World/Frame_[1-3]", name="frames_view")
        translations = np.array([[0, 0, 0], [0, 10, 5], [0, 3, 5]])
        translations = self._array_container(
            translations.tolist() if indices is None else translations[indices].tolist()
        )
        new_orientations = euler_angles_to_quats_numpy(
            euler_angles=np.array([[0, np.pi, 0], [np.pi, 0, 0], [0, 0, np.pi]])
        )
        new_orientations = self._array_container(
            new_orientations.tolist() if indices is None else new_orientations[indices].tolist()
        )
        frames_view.set_local_poses(translations=translations, orientations=new_orientations, indices=indices)
        translations = np.array([[0, 20, 10], [0, 30, 20], [0, 50, 10]])
        translations = self._array_container(
            translations.tolist() if indices is None else translations[indices].tolist()
        )
        new_orientations = euler_angles_to_quats_numpy(
            euler_angles=np.array([[0, -np.pi, 0], [-np.pi, 0, 0], [0, 0, -np.pi]])
        )
        new_orientations = self._array_container(
            new_orientations.tolist() if indices is None else new_orientations[indices].tolist()
        )
        self._cubes_view.set_local_poses(translations=translations, orientations=new_orientations, indices=indices)
        current_positions, current_orientations = self._cubes_view.get_world_poses(indices=indices)
        expected_positions = np.array([[0, 20, -10], [0, -20, -15], [0, -47, 15]])
        expected_positions = self._array_container(
            expected_positions.tolist() if indices is None else expected_positions[indices].tolist()
        )
        expected_orientations = euler_angles_to_quats_numpy(euler_angles=np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]))
        expected_orientations = self._array_container(
            expected_orientations.tolist() if indices is None else expected_orientations[indices].tolist()
        )
        if self._test_cfg["backend"] == "warp":
            current_positions = current_positions.numpy()
            expected_positions = expected_positions.numpy()
            current_orientations = current_orientations.numpy()
            expected_orientations = expected_orientations.numpy()
        self.assertTrue(self.isclose(current_positions, expected_positions, atol=1e-4).all())
        self.assertTrue(self.isclose(current_orientations, expected_orientations, atol=1e-4).all())

    async def linear_velocities_test(self, usd=False):
        if self._device == "cpu":
            await self._my_world.reset_async()
            if usd:
                await self._my_world.stop_async()
            indices = [1, 2] if self._test_cfg["indexed"] else None

            linear_velocities = np.array([[10.0, 0.0, 0.0], [20.0, 0.0, 0.0], [-10, 0, 0]])
            linear_velocities = self._array_container(
                linear_velocities.tolist() if indices is None else linear_velocities[indices].tolist()
            )
            self._cubes_view.set_linear_velocities(linear_velocities, indices)
            self._my_world.step_async()
            await update_stage_async()

            current_linear_velocities = self._cubes_view.get_linear_velocities(indices)
            if self._test_cfg["backend"] == "warp":
                current_linear_velocities = current_linear_velocities.numpy()
                linear_velocities = linear_velocities.numpy()
            # print(current_linear_velocities)
            # print(linear_velocities)
            self.assertTrue(self.isclose(current_linear_velocities, linear_velocities, atol=1e-4).all())

        return

    async def angular_velocities_test(self, usd=False):
        if self._device == "cpu":
            await self._my_world.reset_async()
            if usd:
                await self._my_world.stop_async()
            indices = [1, 2] if self._test_cfg["indexed"] else None
            angular_velocities = np.array([[20.0, 0, 0], [0, -20, 0], [0, 0, 20]])
            angular_velocities = self._array_container(
                angular_velocities.tolist() if indices is None else angular_velocities[indices].tolist()
            )
            self._cubes_view.set_angular_velocities(angular_velocities, indices)
            self._my_world.step_async()
            await update_stage_async()
            current_angular_velocities = self._cubes_view.get_angular_velocities(indices)
            if self._test_cfg["backend"] == "warp":
                current_angular_velocities = current_angular_velocities.numpy()
                angular_velocities = angular_velocities.numpy()
            # print(current_angular_velocities)
            # print(angular_velocities)
            self.assertTrue(self.isclose(current_angular_velocities, angular_velocities, atol=1e-1).all())
        return

    async def masses_test(self, usd=False):
        await self._my_world.reset_async()
        if usd:
            await self._my_world.stop_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        masses = np.array([10, 20, 30])
        masses = self._array_container(masses.tolist() if indices is None else masses[indices].tolist())
        self._cubes_view.set_masses(masses, indices)
        current_masses = self._cubes_view.get_masses(indices)
        if self._test_cfg["backend"] == "warp":
            masses = masses.numpy()
            current_masses = current_masses.numpy()
        self.assertTrue(self.isclose(masses, current_masses, atol=1e-4).all())
        return

    async def densities_test(self, usd=False):
        await self._my_world.reset_async()
        if usd:
            await self._my_world.stop_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        densities = np.array([10, 20, 30])
        densities = self._array_container(densities.tolist() if indices is None else densities[indices].tolist())
        self._cubes_view.set_densities(densities, indices)
        current_densities = self._cubes_view.get_densities(indices)
        if self._test_cfg["backend"] == "warp":
            densities = densities.numpy()
            current_densities = current_densities.numpy()
        self.assertTrue(self.isclose(densities, current_densities, atol=1e-4).all())
        return

    async def sleep_thresholds_test(self, usd=False):
        await self._my_world.reset_async()
        if usd:
            await self._my_world.stop_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        thresholds = np.array([0.0, 0.2, 0.1])
        thresholds = self._array_container(thresholds.tolist() if indices is None else thresholds[indices].tolist())
        self._cubes_view.set_sleep_thresholds(thresholds, indices)
        current_thresholds = self._cubes_view.get_sleep_thresholds(indices)
        if self._test_cfg["backend"] == "warp":
            thresholds = thresholds.numpy()
            current_thresholds = current_thresholds.numpy()
        self.assertTrue(self.isclose(thresholds, current_thresholds, atol=1e-4).all())
        return

    async def enable_disable_physics_test(self, usd=False):
        await self._my_world.reset_async()
        if usd:
            await self._my_world.stop_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        self._cubes_view.enable_rigid_body_physics(indices)
        self._my_world.step_async()
        await update_stage_async()
        self._cubes_view.disable_rigid_body_physics(indices)

        return

    async def enable_disable_gravity_test(self, usd=False):
        await self._my_world.reset_async()
        if usd:
            await self._my_world.stop_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        self._cubes_view.enable_gravities(indices)
        self._my_world.step_async()
        await update_stage_async()
        self._cubes_view.disable_gravities(indices)
        return

    async def default_state_post_reset_test(self, usd=False):
        await self._my_world.reset_async()
        if usd:
            await self._my_world.stop_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        positions = np.array([[10.0, 10.0, 0.0], [0.0, 10.0, 0.0], [0.0, -10.0, 0.0]])
        orientations = euler_angles_to_quats_numpy(
            np.array([[0.0, np.pi / 4.0, 0], [0, 0, np.pi / 4.0], [0, 0, -np.pi / 8.0]])
        )
        linear_velocities = np.array([[0.0, 10, 0], [10, 0, 0], [-10, 0, 0]])
        angular_velocities = np.array([[0.0, 360, 0], [360, 0, 0], [-360, 0, 0]])

        positions = self._array_container(positions.tolist() if indices is None else positions[indices].tolist())
        orientations = self._array_container(
            orientations.tolist() if indices is None else orientations[indices].tolist()
        )
        linear_velocities = self._array_container(
            linear_velocities.tolist() if indices is None else linear_velocities[indices].tolist()
        )
        angular_velocities = self._array_container(
            angular_velocities.tolist() if indices is None else angular_velocities[indices].tolist()
        )

        desired_default_state = DynamicsViewState(
            positions=positions,
            orientations=orientations,
            linear_velocities=linear_velocities,
            angular_velocities=angular_velocities,
        )

        self._cubes_view.set_default_state(
            positions=desired_default_state.positions,
            orientations=desired_default_state.orientations,
            linear_velocities=desired_default_state.linear_velocities,
            angular_velocities=desired_default_state.angular_velocities,
            indices=indices,
        )

        default_state = self._cubes_view.get_default_state()
        if self._test_cfg["backend"] == "warp":
            self.assertTrue(
                self.isclose(
                    desired_default_state.positions.numpy(),
                    default_state.positions.numpy() if indices is None else default_state.positions.numpy()[indices],
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.orientations.numpy(),
                    (
                        default_state.orientations.numpy()
                        if indices is None
                        else default_state.orientations.numpy()[indices]
                    ),
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.linear_velocities.numpy(),
                    (
                        default_state.linear_velocities.numpy()
                        if indices is None
                        else default_state.linear_velocities.numpy()[indices]
                    ),
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.angular_velocities.numpy(),
                    (
                        default_state.angular_velocities.numpy()
                        if indices is None
                        else default_state.angular_velocities.numpy()[indices]
                    ),
                    atol=1e-4,
                ).all()
            )
        else:
            self.assertTrue(
                self.isclose(
                    desired_default_state.positions,
                    default_state.positions if indices is None else default_state.positions[indices],
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.orientations,
                    default_state.orientations if indices is None else default_state.orientations[indices],
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.linear_velocities,
                    default_state.linear_velocities if indices is None else default_state.linear_velocities[indices],
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.angular_velocities,
                    default_state.angular_velocities if indices is None else default_state.angular_velocities[indices],
                    atol=1e-4,
                ).all()
            )

        if not self._test_cfg["indexed"]:
            # resets to default state
            self._cubes_view.post_reset()
            current_state = self._cubes_view.get_current_dynamic_state()
            if self._test_cfg["backend"] == "warp":
                self.assertTrue(
                    self.isclose(
                        desired_default_state.positions.numpy(), current_state.positions.numpy(), atol=1e-4
                    ).all()
                )
                self.assertTrue(
                    self.isclose(
                        desired_default_state.orientations.numpy(), current_state.orientations.numpy(), atol=1e-4
                    ).all()
                )
                self.assertTrue(
                    self.isclose(
                        desired_default_state.linear_velocities.numpy(),
                        current_state.linear_velocities.numpy(),
                        atol=1e-4,
                    ).all()
                )
                self.assertTrue(
                    self.isclose(
                        desired_default_state.angular_velocities.numpy(),
                        current_state.angular_velocities.numpy(),
                        atol=1e-4,
                    ).all()
                )
            else:
                self.assertTrue(self.isclose(desired_default_state.positions, current_state.positions, atol=1e-4).all())
                self.assertTrue(
                    self.isclose(desired_default_state.orientations, current_state.orientations, atol=1e-4).all()
                )
                self.assertTrue(
                    self.isclose(
                        desired_default_state.linear_velocities, current_state.linear_velocities, atol=1e-4
                    ).all()
                )
                self.assertTrue(
                    self.isclose(
                        desired_default_state.angular_velocities, current_state.angular_velocities, atol=1e-4
                    ).all()
                )

    async def default_state_before_reset_test(self):
        # test set state before reset
        indices = None

        positions = np.array([[10.0, 10.0, 0.0], [0.0, 10.0, 0.0], [0.0, -10.0, 0.0]])
        orientations = euler_angles_to_quats_numpy(
            np.array([[0.0, np.pi / 4.0, 0], [0, 0, np.pi / 4.0], [0, 0, -np.pi / 8.0]])
        )
        linear_velocities = np.array([[0.0, 10, 0], [10, 0, 0], [-10, 0, 0]])
        angular_velocities = np.array([[0.0, 360, 0], [360, 0, 0], [-360, 0, 0]])

        positions = self._array_container(positions.tolist() if indices is None else positions[indices].tolist())
        orientations = self._array_container(
            orientations.tolist() if indices is None else orientations[indices].tolist()
        )
        linear_velocities = self._array_container(
            linear_velocities.tolist() if indices is None else linear_velocities[indices].tolist()
        )
        angular_velocities = self._array_container(
            angular_velocities.tolist() if indices is None else angular_velocities[indices].tolist()
        )
        desired_default_state = DynamicsViewState(
            positions=positions,
            orientations=orientations,
            linear_velocities=linear_velocities,
            angular_velocities=angular_velocities,
        )
        self._cubes_view.set_default_state(
            positions=desired_default_state.positions,
            orientations=desired_default_state.orientations,
            linear_velocities=desired_default_state.linear_velocities,
            angular_velocities=desired_default_state.angular_velocities,
            indices=indices,
        )
        await self._my_world.reset_async()
        default_state = self._cubes_view.get_default_state()
        if self._test_cfg["backend"] == "warp":
            self.assertTrue(
                self.isclose(
                    desired_default_state.positions.numpy(),
                    default_state.positions.numpy() if indices is None else default_state.positions.numpy()[indices],
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.orientations.numpy(),
                    (
                        default_state.orientations.numpy()
                        if indices is None
                        else default_state.orientations.numpy()[indices]
                    ),
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.linear_velocities.numpy(),
                    (
                        default_state.linear_velocities.numpy()
                        if indices is None
                        else default_state.linear_velocities.numpy()[indices]
                    ),
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.angular_velocities.numpy(),
                    (
                        default_state.angular_velocities.numpy()
                        if indices is None
                        else default_state.angular_velocities.numpy()[indices]
                    ),
                    atol=1e-4,
                ).all()
            )
        else:
            self.assertTrue(
                self.isclose(
                    desired_default_state.positions,
                    default_state.positions if indices is None else default_state.positions[indices],
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.orientations,
                    default_state.orientations if indices is None else default_state.orientations[indices],
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.linear_velocities,
                    default_state.linear_velocities if indices is None else default_state.linear_velocities[indices],
                    atol=1e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.angular_velocities,
                    default_state.angular_velocities if indices is None else default_state.angular_velocities[indices],
                    atol=1e-4,
                ).all()
            )

    async def transforms_test(self):
        await self._my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else 3

        new_positions = np.array([[25.0, -20.0, 10.0], [15.0, 10.0, 0.0], [-45.0, -40.0, 0.0]])
        new_orientations = np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
        new_positions = self._array_container(
            new_positions.tolist() if indices is None else new_positions[indices].tolist()
        )
        new_orientations = self._array_container(
            new_orientations.tolist() if indices is None else new_orientations[indices].tolist()
        )
        self._cubes_view.set_world_poses(positions=new_positions, orientations=new_orientations, indices=indices)
        self._my_world.step_async()
        await update_stage_async()
        current_positions, current_orientations = self._cubes_view.get_world_poses(indices=indices)
        if self._test_cfg["backend"] == "warp":
            current_positions = current_positions.numpy()
            new_positions = new_positions.numpy()
            current_orientations = current_orientations.numpy()
            new_orientations = new_orientations.numpy()
        self.assertTrue(self.isclose(current_positions, new_positions, atol=1e-4).all())
        self.assertTrue(self.isclose(current_orientations, new_orientations, atol=1e-4).all())
        return

    async def velocities_test(self, usd=False):
        await self._my_world.reset_async()
        if usd:
            await self._my_world.stop_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        velocities = np.array(
            [[10.0, 0.0, 0.0, 20.0, 0.0, 0.0], [20.0, 0.0, 0.0, 0.0, -20.0, 0.0], [-10.0, 0.0, 0.0, 0.0, 0.0, 20.0]]
        )
        velocities = self._array_container(velocities.tolist() if indices is None else velocities[indices].tolist())
        self._cubes_view.set_velocities(velocities, indices)
        self._my_world.step_async()
        await update_stage_async()
        current_velocities = self._cubes_view.get_velocities(indices)
        if self._test_cfg["backend"] == "warp":
            current_velocities = current_velocities.numpy()
            velocities = velocities.numpy()
        self.assertTrue(self.isclose(current_velocities, velocities, atol=1e-1).all())
        return

    async def apply_forces_test(self):
        await self._my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        new_positions = np.array([[20.0, -20.0, 10.0], [30.0, 30.0, 0], [-40, -40, 0]])
        new_orientations = euler_angles_to_quats_numpy(euler_angles=np.array([[0.0, 0, 0], [0, 0, 0], [0, 0, 0]]))
        new_positions = self._array_container(
            new_positions.tolist() if indices is None else new_positions[indices].tolist()
        )
        new_orientations = self._array_container(
            new_orientations.tolist() if indices is None else new_orientations[indices].tolist()
        )
        self._cubes_view.set_world_poses(positions=new_positions, orientations=new_orientations, indices=indices)

        forces = np.array([[3000, 0, 0], [-3000, 0, 0], [3000, 0, 0]])
        forces = self._array_container(forces.tolist() if indices is None else forces[indices].tolist())
        self._cubes_view.apply_forces(forces, indices)
        self._my_world.step_async()
        await update_stage_async()

        current_linear_velocities = self._cubes_view.get_linear_velocities(indices)

        if self._test_cfg["backend"] == "torch":
            current_linear_velocities = current_linear_velocities.cpu().numpy()
        elif self._test_cfg["backend"] == "warp":
            current_linear_velocities = current_linear_velocities.numpy()

        self.assertTrue(
            np.isclose(current_linear_velocities[:, 1:], np.array([[0, 0], [0, 0], [0, 0]])[indices], atol=1e-4).all()
        )

        self.assertTrue(np.logical_not(np.isclose(current_linear_velocities[:, 0], np.array([0, 0, 0])[indices])).all())

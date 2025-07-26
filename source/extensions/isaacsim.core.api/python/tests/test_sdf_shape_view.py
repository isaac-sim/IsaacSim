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

import asyncio

import carb
import numpy as np
import omni.kit.test
import torch
import warp as wp
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SdfShapePrim
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from isaacsim.core.utils.torch.rotations import euler_angles_to_quats as euler_angles_to_quats_torch
from isaacsim.core.utils.warp.rotations import euler_angles_to_quats as euler_angles_to_quats_warp
from omni.physx.scripts import physicsUtils

from .common import CoreTestCase

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

    async def test_sdf_shape_view_gpu_pipeline(self):
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
                    self._array_container = lambda x: torch.tensor(x, dtype=torch.float32, device=self._device)
                    self._device = "cuda:0"

            elif backend == "warp":
                self.euler_angles_to_quats = euler_angles_to_quats_warp
                self.isclose = np.isclose
                if self._test_cfg["device"] == "gpu":
                    self._device = "cuda:0"
                self._array_container = lambda x: wp.array(x, device=self._device, dtype=wp.float32)

            await self._runner()

    async def _setup_sdf_scene(self, num_query_points=10, prepare_sdf_schemas=True):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World(sim_params=self._sim_params, backend=self._test_cfg["backend"], device="cuda")
        await self._my_world.initialize_simulation_context_async()
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self.length = 0.5
        self.num_points = num_query_points
        self.num_envs = 3
        for i in range(self.num_envs):
            physicsUtils.create_mesh_cube(self.stage, f"/World/Cube_{i+1}", self.length)

        await update_stage_async()
        self._cubes_view = SdfShapePrim(
            prim_paths_expr="/World/Cube_[1-3]",
            name="cubes_view",
            positions=self._array_container([[0.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, -10.0, 0.0]]),
            num_query_points=2 * num_query_points,
            prepare_sdf_schemas=prepare_sdf_schemas,
        )
        self._my_world.scene.add(self._cubes_view)

    async def _runner(self):
        await self._setup_sdf_scene()
        for indexed in [False, True]:
            self._test_cfg["indexed"] = indexed
            print(indexed, self._test_cfg)
            await self._setup_sdf_scene(num_query_points=102, prepare_sdf_schemas=True)
            await self.signed_distance_test()

        self._my_world.clear_instance()

    async def signed_distance_test(self):
        await self._my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else [0, 1, 2]
        self._my_world.step_async()
        await update_stage_async()

        sdf_view = self._cubes_view
        margins = sdf_view.get_sdf_margins()
        thickness = sdf_view.get_sdf_narrow_band_thickness()
        subgrid_resolution = sdf_view.get_sdf_subgrid_resolution()
        sdf_resolution = sdf_view.get_sdf_resolution()
        # print(margins, thickness, subgrid_resolution, sdf_resolution)
        if self._test_cfg["backend"] == "warp":
            margins = margins.numpy()
            thickness = thickness.numpy()
            subgrid_resolution = subgrid_resolution.numpy()
            sdf_resolution = sdf_resolution.numpy()
        sdf_view.set_sdf_margins(2 * margins)
        sdf_view.set_sdf_narrow_band_thickness(2 * thickness)
        sdf_view.set_sdf_subgrid_resolution(2 * subgrid_resolution)
        sdf_view.set_sdf_resolution(2 * sdf_resolution)

        new_margins = sdf_view.get_sdf_margins()
        new_thickness = sdf_view.get_sdf_narrow_band_thickness()
        new_subgrid_resolution = sdf_view.get_sdf_subgrid_resolution()
        new_sdf_resolution = sdf_view.get_sdf_resolution()
        # print(margins, thickness, subgrid_resolution, sdf_resolution)
        if self._test_cfg["backend"] == "torch":
            margins = margins.cpu()
            thickness = thickness.cpu()
            subgrid_resolution = subgrid_resolution.cpu()
            sdf_resolution = sdf_resolution.cpu()
            new_margins = new_margins.cpu()
            new_thickness = new_thickness.cpu()
            new_subgrid_resolution = new_subgrid_resolution.cpu()
            new_sdf_resolution = new_sdf_resolution.cpu()

        self.assertTrue(np.isclose(new_margins.numpy(), 2 * margins, rtol=1e-3).all(), "expected margins")
        self.assertTrue(np.isclose(new_thickness.numpy(), 2 * thickness, rtol=1e-3).all(), "expected thickness")
        self.assertTrue(
            np.isclose(new_subgrid_resolution.numpy(), 2 * subgrid_resolution, rtol=1e-3).all(), "expected subgrid res"
        )
        self.assertTrue(
            np.isclose(new_sdf_resolution.numpy(), 2 * sdf_resolution, rtol=1e-3).all(), "expected resolution"
        )
        sdf_api_margin = new_margins.numpy().mean().tolist()
        points = np.zeros((self.num_envs, 2 * self.num_points, 3))
        points[:, : self.num_points, 0] = self.length - sdf_api_margin / 2
        points[:, self.num_points :, 0] = self.length + sdf_api_margin / 2
        num_points_row = int(np.sqrt(self.num_points))
        delta = self.length / num_points_row
        for i in range(num_points_row):
            for j in range(num_points_row):
                # start from some small distance away to make sure points don't fall on the surface
                points[:, i * num_points_row + j, 1] = -self.length + 2.0 * i * delta - delta / 10
                points[:, i * num_points_row + j, 2] = -self.length + 2.0 * j * delta - delta / 10
                points[:, self.num_points + i * num_points_row + j, 1] = -self.length + 2.0 * i * delta - delta / 10
                points[:, self.num_points + i * num_points_row + j, 2] = -self.length + 2.0 * j * delta - delta / 10

        sdf_data = self._cubes_view.get_sdf_and_gradients(self._array_container(points), indices)
        if self._test_cfg["backend"] == "torch":
            sdf_data = sdf_data.cpu()
        sdfs_np = sdf_data.numpy().reshape(len(indices), 2 * self.num_points, 4)
        d = np.abs(points[indices]) - self.length
        expected = np.linalg.norm(np.maximum(d, 0.0), axis=2) + np.minimum(np.max(d, axis=2), 0.0)
        # print("sdf= \n", np.dstack((sdfs_np[0, :, -1] , expected[0, :])))
        # inside grad
        g1 = np.zeros((len(indices), 2 * self.num_points, 3))
        # outside grad
        g2 = np.zeros((len(indices), 2 * self.num_points, 3))
        distance = points - self.length
        for i in range(len(indices)):
            for j in range(self.num_points * 2):
                is_inside = np.max(d[i, j]) < 0
                if is_inside:
                    c = np.argmax(d[i, j])
                    # inside sdf gradient direction is from the sample point to surface
                    g1[i, j, c] = 1
                else:
                    # outside sdf gradient direction is from the surface to the sample point
                    grad = np.sign(distance[i, j]) * np.maximum(d[i, j], 0.0) / np.linalg.norm(np.maximum(d[i, j], 0.0))
                    g2[i, j, :] = grad

        exptected_gradient = g1 + g2
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self._cubes_view.count, 2 * self.num_points, 4]
        )
        # print(sdf_data.shape, expected_shape)
        self.assertTrue(np.isclose(sdfs_np[:, :, -1], expected, rtol=0.1, atol=0.1).all(), "expected sdf values")
        self.assertTrue(
            np.isclose(sdfs_np[:, :, :-1], exptected_gradient, atol=0.1).all(), "expected sdf gradient values"
        )
        if self._test_cfg["backend"] == "torch":
            sdf_data = sdf_data.cpu()
        self.assertTrue(sdf_data.shape == expected_shape)

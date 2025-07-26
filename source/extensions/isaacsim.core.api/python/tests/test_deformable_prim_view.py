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

import isaacsim.core.utils.deformable_mesh_utils as DeformableMeshUtils
import numpy as np
import omni.kit.test
import torch
from isaacsim.core.api import World
from isaacsim.core.api.materials.deformable_material import DeformableMaterial
from isaacsim.core.prims import DeformablePrim, SingleDeformablePrim
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from isaacsim.core.utils.types import DynamicsViewState
from omni.physx.scripts import deformableUtils, physicsUtils
from pxr import Gf, Usd, UsdGeom

from .common import CoreTestCase


class TestDeformablePrim(CoreTestCase):
    async def setUp(self):
        await super().setUp()
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch", device="cuda")
        await self.my_world.initialize_simulation_context_async()
        self._test_cfg = dict()

    async def tearDown(self):
        await super().tearDown()

    async def test_deformable_prim_view_gpu_pipeline(self):
        self.isclose = torch.isclose
        self._array_container = lambda x: torch.tensor(x, device=self._device, dtype=torch.float32)
        await update_stage_async()
        await self._runner()

    async def _setup_scene(self):
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch", device="cuda")
        await self.my_world.initialize_simulation_context_async()
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self.num_envs = 5
        self.dimx = 5
        self.dimy = 5

        for i in range(self.num_envs):
            env_path = "/World/Env" + str(i)
            env = UsdGeom.Xform.Define(self.stage, env_path)
            # set up the geometry
            deformable_path = env.GetPrim().GetPath().AppendChild("deformable").pathString
            self.plane_mesh = UsdGeom.Mesh.Define(self.stage, deformable_path)
            tri_points, tri_indices = DeformableMeshUtils.createTriangleMeshCube(8)
            self.plane_mesh.GetPointsAttr().Set(tri_points)
            self.plane_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
            self.plane_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
            physicsUtils.setup_transform_as_scale_orient_translate(self.plane_mesh)
            physicsUtils.set_or_add_translate_op(self.plane_mesh, Gf.Vec3f(i * 2, 0.0, 2.0))
            # physicsUtils.set_or_add_orient_op(self.plane_mesh, Gf.Rotation(Gf.Vec3d([1, 0, 0]), 15 * i).GetQuat())
            deformable_material_path = env.GetPrim().GetPath().AppendChild("deformableMaterial").pathString
            self.deformable_material = DeformableMaterial(
                prim_path=deformable_material_path,
                dynamic_friction=0.5,
                youngs_modulus=5e4,
                poissons_ratio=0.4,
                damping_scale=0.1,
                elasticity_damping=0.1,
            )
            deformable = SingleDeformablePrim(
                prim_path=deformable_path,
                deformable_material=self.deformable_material,
                simulation_hexahedral_resolution=1,
            )
            self.mesh_prim = self.stage.GetPrimAtPath(deformable_path)

        # create a view to deal with all the deformables
        self.deformable_view = DeformablePrim(prim_paths_expr="/World/Env*/deformable", name="deformableView1")
        self.my_world.scene.add(self.deformable_view)

    async def _step(self):
        self.my_world.step_async()
        await update_stage_async()

    async def _runner(self):
        for indexed in [False, True]:
            self._test_cfg["indexed"] = indexed
            print(self._test_cfg)
            await self._setup_scene()
            await self.sim_position_test()
            await self._setup_scene()
            await self.sim_velocity_test()
            await self._setup_scene()
            await self.sim_position_target_test()
            await self._setup_scene()
            await self.sim_mesh_indices()
            await self._setup_scene()
            await self.sim_rest_position_test()
            await self._setup_scene()
            await self.sim_deformation_gradient_test()
            await self._setup_scene()
            await self.sim_rotation_test()
            await self._setup_scene()
            await self.sim_element_pose_test()
            await self._setup_scene()
            await self.sim_stress_test()

            await self._setup_scene()
            await self.collision_deformation_gradient_test()
            await self._setup_scene()
            await self.collision_rotation_test()
            await self._setup_scene()
            await self.collision_element_pose_test()
            await self._setup_scene()
            await self.collision_stress_test()

        await self.my_world.stop_async()
        self.my_world.clear_instance()

    async def sim_position_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.deformable_view.get_simulation_mesh_nodal_positions(indices)
        new_values = prev_values + 1
        # print(new_values)
        # print(new_values.shape, prev_values.shape)
        self.deformable_view.set_simulation_mesh_nodal_positions(new_values, indices)
        curr_values = self.deformable_view.get_simulation_mesh_nodal_positions(indices)
        self.assertTrue(self.isclose(new_values, curr_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.deformable_view.count, prev_values.shape[1], 3]
        )
        # print(expected_shape, curr_values.shape)
        self.assertTrue(curr_values.shape == expected_shape)

    async def sim_velocity_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.deformable_view.get_simulation_mesh_nodal_velocities(indices)
        new_values = prev_values + 1
        self.deformable_view.set_simulation_mesh_nodal_velocities(new_values, indices)
        curr_values = self.deformable_view.get_simulation_mesh_nodal_velocities(indices)
        self.assertTrue(self.isclose(new_values, curr_values).all())
        expected_shape = torch.Size(
            [
                len(indices) if self._test_cfg["indexed"] else self.deformable_view.count,
                self.deformable_view.max_simulation_mesh_vertices_per_body,
                3,
            ]
        )
        self.assertTrue(curr_values.shape == expected_shape)

    async def sim_position_target_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        curr_positions = self.deformable_view.get_simulation_mesh_nodal_positions(indices)
        new_values = curr_positions + torch.tensor([0, 0, 2], device="cuda:0")
        targets = torch.zeros((new_values.shape[0], new_values.shape[1], 4), device="cuda:0")
        targets[..., :3] = new_values
        targets[..., -1] = torch.where(targets[:, :, 2] < 4, 1, 0)
        self.deformable_view.set_simulation_mesh_kinematic_targets(targets, indices)
        for i in range(5):
            await self._step()
            curr_values = self.deformable_view.get_simulation_mesh_nodal_positions(indices)

        curr_values = self.deformable_view.get_simulation_mesh_nodal_positions(indices)
        kinematic_nodes = torch.where(1 - targets[..., -1])
        # for kinematically driven nodes the expected values is the target values
        self.assertTrue(
            self.isclose(
                targets[kinematic_nodes[0], kinematic_nodes[1], :3], curr_values[kinematic_nodes[0], kinematic_nodes[1]]
            ).all()
        )
        expected_shape = torch.Size(
            [
                len(indices) if self._test_cfg["indexed"] else self.deformable_view.count,
                self.deformable_view.max_simulation_mesh_vertices_per_body,
                3,
            ]
        )
        self.assertTrue(curr_values.shape == expected_shape)

    async def sim_mesh_indices(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        sim_mesh_indices = self.deformable_view.get_simulation_mesh_indices(indices)
        sim_indices = self.mesh_prim.GetAttribute("physxDeformable:simulationIndices").Get()
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.deformable_view.count, len(sim_indices) // 4, 4]
        )
        # print(sim_mesh_indices.shape, expected_shape)
        self.assertTrue(sim_mesh_indices.shape == expected_shape)
        self.assertTrue(
            self.isclose(
                sim_mesh_indices[0].flatten(), torch.tensor(sim_indices, device="cuda:0", dtype=torch.int32)
            ).all()
        )

    async def sim_rest_position_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        stress_vals = self.deformable_view.get_simulation_mesh_element_stresses(indices)
        nodal_positions = self.deformable_view.get_simulation_mesh_nodal_positions(indices)
        rest_point = self.deformable_view.get_simulation_mesh_rest_points(indices)
        xforms = self.deformable_view.get_world_poses(indices)
        num_nodes = rest_point.shape[1]
        translate = np.repeat(xforms[0].cpu()[:, np.newaxis, :], num_nodes, axis=1)
        translate = translate.clone().detach().to("cuda:0")
        self.assertTrue(
            self.isclose(
                rest_point + translate,
                nodal_positions,
                atol=1.0e-2,
            ).all()
        )

        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.deformable_view.count, nodal_positions.shape[1], 3]
        )
        self.assertTrue(rest_point.shape == expected_shape)

    async def sim_deformation_gradient_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else self.deformable_view.count
        values = self.deformable_view.get_simulation_mesh_element_deformation_gradients(indices)
        identity = torch.zeros(
            (num_indices, self.deformable_view.max_simulation_mesh_elements_per_body, 3, 3), device="cuda:0"
        )
        identity[:, :, 0, 0] = 1
        identity[:, :, 1, 1] = 1
        identity[:, :, 2, 2] = 1
        self.assertTrue(
            self.isclose(
                identity,
                values,
                atol=1.0e-3,
            ).all()
        )

    async def collision_deformation_gradient_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else self.deformable_view.count
        values = self.deformable_view.get_collision_mesh_element_deformation_gradients(indices)
        identity = torch.zeros(
            (num_indices, self.deformable_view.max_collision_mesh_elements_per_body, 3, 3), device="cuda:0"
        )
        identity[:, :, 0, 0] = 1
        identity[:, :, 1, 1] = 1
        identity[:, :, 2, 2] = 1
        self.assertTrue(
            self.isclose(
                identity,
                values,
                atol=1.0e-2,
            ).all()
        )

    async def sim_stress_test(self):
        await self.my_world.reset_async()
        await update_stage_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else self.deformable_view.count
        values = self.deformable_view.get_simulation_mesh_element_stresses(indices)
        zero_tensor = torch.zeros(
            (num_indices, self.deformable_view.max_simulation_mesh_elements_per_body, 3, 3), device="cuda:0"
        )
        # print(values)
        self.assertTrue(
            self.isclose(
                zero_tensor,
                values,
                atol=10.0,
            ).all()
        )

    async def collision_stress_test(self):
        await self.my_world.reset_async()
        await update_stage_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else self.deformable_view.count
        values = self.deformable_view.get_collision_mesh_element_stresses(indices)
        zero_tensor = torch.zeros(
            (num_indices, self.deformable_view.max_collision_mesh_elements_per_body, 3, 3), device="cuda:0"
        )
        # print(values)
        self.assertTrue(
            self.isclose(
                zero_tensor,
                values,
                atol=10.0,
            ).all()
        )

    async def sim_rotation_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else self.deformable_view.count
        values = self.deformable_view.get_simulation_mesh_element_rotations(indices)
        identity = torch.zeros(
            (num_indices, self.deformable_view.max_simulation_mesh_elements_per_body, 4), device="cuda:0"
        )
        identity[:, :, -1] = 1
        self.assertTrue(
            self.isclose(
                identity,
                values,
                atol=1.0e-3,
            ).all()
        )

    async def collision_rotation_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else self.deformable_view.count
        values = self.deformable_view.get_collision_mesh_element_rotations(indices)
        identity = torch.zeros(
            (num_indices, self.deformable_view.max_collision_mesh_elements_per_body, 4), device="cuda:0"
        )
        identity[:, :, -1] = 1
        self.assertTrue(
            self.isclose(
                identity,
                values,
                atol=1.0e-3,
            ).all()
        )

    async def sim_element_pose_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else self.deformable_view.count
        sim_mesh_indices = self.deformable_view.get_simulation_mesh_indices(indices)
        positions = self.deformable_view.get_simulation_mesh_nodal_positions(indices)
        values = self.deformable_view.get_simulation_mesh_element_rest_poses(indices)
        expected = torch.zeros(
            (num_indices, self.deformable_view.max_simulation_mesh_elements_per_body, 3, 3), device="cuda:0"
        )
        # similar connectivity between elements across the env so use the first body
        x0_indices = sim_mesh_indices[0, :, 0]
        x1_indices = sim_mesh_indices[0, :, 1]
        x2_indices = sim_mesh_indices[0, :, 2]
        x3_indices = sim_mesh_indices[0, :, 3]
        x0 = positions[:, x0_indices.tolist(), :]
        x1 = positions[:, x1_indices.tolist(), :]
        x2 = positions[:, x2_indices.tolist(), :]
        x3 = positions[:, x3_indices.tolist(), :]

        expected[:, :, 0, :] = x1 - x0
        expected[:, :, 1, :] = x2 - x0
        expected[:, :, 2, :] = x3 - x0

        for i in range(expected.shape[0]):
            for j in range(expected.shape[1]):
                expected[i, j] = torch.linalg.inv(expected[i, j])

        self.assertTrue(
            self.isclose(
                expected,
                values,
                atol=1.0e-3,
            ).all()
        )

    async def collision_element_pose_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else self.deformable_view.count
        sim_mesh_indices = self.deformable_view.get_collision_mesh_indices(indices)
        positions = self.deformable_view.get_collision_mesh_nodal_positions(indices)
        values = self.deformable_view.get_collision_mesh_element_rest_poses(indices)
        expected = torch.zeros(
            (num_indices, self.deformable_view.max_collision_mesh_elements_per_body, 3, 3), device="cuda:0"
        )
        # similar connectivity between elements across the env so use the first body
        x0_indices = sim_mesh_indices[0, :, 0]
        x1_indices = sim_mesh_indices[0, :, 1]
        x2_indices = sim_mesh_indices[0, :, 2]
        x3_indices = sim_mesh_indices[0, :, 3]
        x0 = positions[:, x0_indices.tolist(), :]
        x1 = positions[:, x1_indices.tolist(), :]
        x2 = positions[:, x2_indices.tolist(), :]
        x3 = positions[:, x3_indices.tolist(), :]

        expected[:, :, 0, :] = x1 - x0
        expected[:, :, 1, :] = x2 - x0
        expected[:, :, 2, :] = x3 - x0

        for i in range(expected.shape[0]):
            for j in range(expected.shape[1]):
                expected[i, j] = torch.linalg.inv(expected[i, j])

        self.assertTrue(
            self.isclose(
                expected,
                values,
                atol=1.0e-3,
            ).all()
        )

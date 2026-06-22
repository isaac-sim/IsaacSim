# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Validate Newton rigid contact tensor views and filter handling.

The tests build simple rigid-body contact scenes and check net contact forces,
per-filter force matrices, raw contact buffers, actor-id to path lookup, and a
multi-sensor/multi-filter regression case for contact count indexing.
"""

from __future__ import annotations

import math

import numpy as np
import warp as wp
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from .test_helpers import (
    NewtonTensorTestBase,
    create_ground_plane,
    create_rigid_ball,
    create_rigid_box,
    run_on_device_configs,
)


@run_on_device_configs()
class TestRigidContactNetForces(NewtonTensorTestBase):
    """Box on ground: verify net contact forces ~ [0, 0, mg] after settling."""

    NUM_ENVS = 4
    BOX_MASS = 1.0
    GRAVITY = 9.81

    async def test_net_contact_forces(self) -> None:
        """Verify settled box sensors report upward net contact force near ``mass * gravity``."""
        self.setup_box_on_ground(num_envs=self.NUM_ENVS, mass=self.BOX_MASS, half_extent=0.3, height=0.3)

        sim = await self.create_sim()
        self.start_playing()
        contacts = sim.create_rigid_contact_view(
            ["/envs/env*/box"],
        )
        self.assertIsNotNone(contacts)
        self.assertEqual(contacts.sensor_count, self.NUM_ENVS)

        dt = self.get_sim_dt()
        self.step(n=120, dt=dt)

        net_forces = contacts.get_net_contact_forces(dt)
        net_forces_np = net_forces.numpy().reshape(contacts.sensor_count, 3)

        expected_z = self.BOX_MASS * self.GRAVITY
        for i in range(contacts.sensor_count):
            self.assertAlmostEqual(net_forces_np[i, 0], 0.0, delta=0.5, msg=f"Sensor {i}: X force should be ~0")
            self.assertAlmostEqual(net_forces_np[i, 1], 0.0, delta=0.5, msg=f"Sensor {i}: Y force should be ~0")
            self.assertAlmostEqual(
                net_forces_np[i, 2],
                expected_z,
                delta=expected_z * 0.3,
                msg=f"Sensor {i}: Z force should be ~{expected_z}",
            )


@run_on_device_configs()
class TestRigidContactForceMatrix(NewtonTensorTestBase):
    """Ball + box with filters: verify force matrix decomposition."""

    NUM_ENVS = 4
    GRAVITY = 9.81
    BOX_MASS = 1.0
    BALL_MASS = 1.0

    async def test_force_matrix(self) -> None:
        """Verify box contact filters decompose net force into ground and ball contributions."""
        create_ground_plane(self.stage)

        env_template_path = Sdf.Path("/envTemplate")
        env_template_xform = UsdGeom.Xform.Define(self.stage, env_template_path)
        env_template_xform.GetPrim().SetSpecifier(Sdf.SpecifierClass)

        box_path = env_template_path.AppendChild("box")
        box = create_rigid_box(self.stage, box_path, position=Gf.Vec3f(0.0, 0.0, 0.3), half_extent=0.3)
        mass_api = UsdPhysics.MassAPI(box)
        mass_api.GetMassAttr().Set(float(self.BOX_MASS))

        ball_path = env_template_path.AppendChild("ball")
        ball = create_rigid_ball(self.stage, ball_path, position=Gf.Vec3f(0.0, 0.0, 1.0), radius=0.15)
        ball_mass_api = UsdPhysics.MassAPI(ball)
        ball_mass_api.GetMassAttr().Set(float(self.BALL_MASS))

        env_scope_path = Sdf.Path("/envs")
        UsdGeom.Scope.Define(self.stage, env_scope_path)

        num_rows = int(math.ceil(math.sqrt(self.NUM_ENVS)))
        num_cols = int(math.ceil(float(self.NUM_ENVS) / float(num_rows)))
        for i in range(self.NUM_ENVS):
            row = i // num_cols
            col = i % num_cols
            env_path = env_scope_path.AppendChild("env" + str(i))
            env_xform = UsdGeom.Xform.Define(self.stage, env_path)
            env_xform.GetPrim().GetInherits().AddInherit(env_template_path)
            env_xform.AddTranslateOp().Set(Gf.Vec3f(row * 2.0, col * 2.0, 0.0))

        sim = await self.create_sim()
        self.start_playing()

        box_filter_patterns = ["/groundPlane", "/envs/*/ball"]
        box_contacts = sim.create_rigid_contact_view(
            "/envs/*/box",
            filter_patterns=box_filter_patterns,
            max_contact_data_count=self.NUM_ENVS * 6,
        )
        self.check_rigid_contact_view(box_contacts, self.NUM_ENVS, len(box_filter_patterns))

        dt = self.get_sim_dt()
        self.step(n=120, dt=dt)

        force_matrix = box_contacts.get_contact_force_matrix(dt)
        fm_np = force_matrix.numpy().reshape(box_contacts.sensor_count, box_contacts.filter_count, 3)

        expected_ground_z = (self.BOX_MASS + self.BALL_MASS) * self.GRAVITY
        for i in range(box_contacts.sensor_count):
            ground_fz = fm_np[i, 0, 2]
            self.assertGreater(ground_fz, 0.0, msg=f"Sensor {i}: ground filter Z force should be > 0")

        net_forces = box_contacts.get_net_contact_forces(dt)
        nf_np = net_forces.numpy().reshape(box_contacts.sensor_count, 3)
        fm_sum = np.sum(fm_np, axis=1)
        np.testing.assert_allclose(
            nf_np, fm_sum, rtol=0.05, atol=0.5, err_msg="Net forces should match sum of force matrix"
        )


@run_on_device_configs()
class TestRawContactData(NewtonTensorTestBase):
    """Stacked boxes: verify raw contact data counts, forces, and actor IDs."""

    NUM_ENVS = 2
    GRAVITY = 9.81
    BOX_MASS = 1.0

    async def test_raw_contact_data(self) -> None:
        """Verify raw contact counts, normals, forces, and actor IDs for stacked boxes."""
        create_ground_plane(self.stage)

        env_template_path = Sdf.Path("/envTemplate")
        env_template_xform = UsdGeom.Xform.Define(self.stage, env_template_path)
        env_template_xform.GetPrim().SetSpecifier(Sdf.SpecifierClass)

        bottom_box_path = env_template_path.AppendChild("bottom_box")
        bottom_box = create_rigid_box(self.stage, bottom_box_path, position=Gf.Vec3f(0.0, 0.0, 0.3), half_extent=0.3)
        mass_api = UsdPhysics.MassAPI(bottom_box)
        mass_api.GetMassAttr().Set(float(self.BOX_MASS))

        top_box_path = env_template_path.AppendChild("top_box")
        top_box = create_rigid_box(self.stage, top_box_path, position=Gf.Vec3f(0.0, 0.0, 0.9), half_extent=0.3)
        mass_api2 = UsdPhysics.MassAPI(top_box)
        mass_api2.GetMassAttr().Set(float(self.BOX_MASS))

        env_scope_path = Sdf.Path("/envs")
        UsdGeom.Scope.Define(self.stage, env_scope_path)

        for i in range(self.NUM_ENVS):
            env_path = env_scope_path.AppendChild("env" + str(i))
            env_xform = UsdGeom.Xform.Define(self.stage, env_path)
            env_xform.GetPrim().GetInherits().AddInherit(env_template_path)
            env_xform.AddTranslateOp().Set(Gf.Vec3f(i * 3.0, 0.0, 0.0))

        sim = await self.create_sim()
        self.start_playing()

        top_contacts = sim.create_rigid_contact_view(
            "/envs/*/top_box",
            max_contact_data_count=self.NUM_ENVS * 10,
        )
        self.assertIsNotNone(top_contacts)
        self.assertEqual(top_contacts.sensor_count, self.NUM_ENVS)
        self.assertEqual(top_contacts.filter_count, 0)

        dt = self.get_sim_dt()
        self.step(n=120, dt=dt)

        forces, points, normals, separations, counts, start_indices, other_actor_ids = (
            top_contacts.get_raw_contact_data(dt)
        )

        counts_np = counts.numpy().flatten()
        start_indices_np = start_indices.numpy().flatten()

        for i in range(self.NUM_ENVS):
            self.assertGreater(counts_np[i], 0, msg=f"Sensor {i} should have contacts")

        start0 = int(start_indices_np[0])
        count0 = int(counts_np[0])
        if count0 > 0:
            ids_slice = other_actor_ids.numpy().flatten()[start0 : start0 + count0]
            ids_cpu = wp.array(ids_slice.astype(np.uint64), dtype=wp.uint64, device="cpu")
            paths = top_contacts.get_other_actor_paths_from_ids(ids_cpu)
            has_bottom = any("bottom_box" in p for p in paths if p)
            self.assertTrue(has_bottom, f"Top box should contact bottom_box. Paths: {paths}")

            forces_np = forces.numpy().flatten()
            normals_np = normals.numpy().reshape(-1, 3)
            slc = slice(start0, start0 + count0)
            force_vectors = forces_np[slc, np.newaxis] * normals_np[slc]
            net_force = np.sum(force_vectors, axis=0)
            expected_z = self.BOX_MASS * self.GRAVITY
            self.assertAlmostEqual(
                net_force[2],
                expected_z,
                delta=expected_z * 0.5,
                msg=f"Net Z force should be ~{expected_z}, got {net_force[2]}",
            )


@run_on_device_configs()
class TestRigidContactMultiSensorMultiFilter(NewtonTensorTestBase):
    """Multi-sensor + multi-filter stacked-box scene.

    Regression test for the ``cpuCountContactsPerPair`` / ``contactDataKernel``
    buffer overflow where ``counts`` was indexed with a ``numBodies`` stride
    instead of a ``filterCount`` stride. The scene is sized so that
    ``sensorCount * numBodies`` is larger than ``sensorCount * filterCount``,
    which is the exact condition that overflows the pre-fix indexing.

    The sensor set is the four bottom boxes. Filters are a broadcast ground
    filter and a per-sensor ``top_box`` filter (cardinality 1 and
    ``sensor_count`` respectively, which the view requires). The full model
    contains the ground plane plus eight boxes, so ``numBodies (9) >
    filterCount (2)`` — the precondition for the old indexing to overflow.
    """

    NUM_ENVS = 4
    BOX_MASS = 1.0
    GRAVITY = 9.81

    async def test_multi_sensor_multi_filter(self) -> None:
        """Verify multi-sensor contact filters keep per-pair counts finite and in range."""
        create_ground_plane(self.stage)

        env_template_path = Sdf.Path("/envTemplate")
        env_template_xform = UsdGeom.Xform.Define(self.stage, env_template_path)
        env_template_xform.GetPrim().SetSpecifier(Sdf.SpecifierClass)

        bottom_path = env_template_path.AppendChild("bottom_box")
        bottom = create_rigid_box(self.stage, bottom_path, position=Gf.Vec3f(0.0, 0.0, 0.3), half_extent=0.3)
        UsdPhysics.MassAPI(bottom).GetMassAttr().Set(float(self.BOX_MASS))

        top_path = env_template_path.AppendChild("top_box")
        top = create_rigid_box(self.stage, top_path, position=Gf.Vec3f(0.0, 0.0, 0.9), half_extent=0.3)
        UsdPhysics.MassAPI(top).GetMassAttr().Set(float(self.BOX_MASS))

        env_scope_path = Sdf.Path("/envs")
        UsdGeom.Scope.Define(self.stage, env_scope_path)
        for i in range(self.NUM_ENVS):
            env_path = env_scope_path.AppendChild("env" + str(i))
            env_xform = UsdGeom.Xform.Define(self.stage, env_path)
            env_xform.GetPrim().GetInherits().AddInherit(env_template_path)
            env_xform.AddTranslateOp().Set(Gf.Vec3f(i * 3.0, 0.0, 0.0))

        sim = await self.create_sim()
        self.start_playing()

        filter_patterns = ["/groundPlane", "/envs/*/top_box"]
        contacts = sim.create_rigid_contact_view(
            "/envs/*/bottom_box",
            filter_patterns=filter_patterns,
            max_contact_data_count=self.NUM_ENVS * 16,
        )
        self.check_rigid_contact_view(contacts, self.NUM_ENVS, len(filter_patterns))

        dt = self.get_sim_dt()
        self.step(n=120, dt=dt)

        force_matrix = contacts.get_contact_force_matrix(dt)
        fm_np = force_matrix.numpy().reshape(contacts.sensor_count, contacts.filter_count, 3)
        self.assertTrue(np.all(np.isfinite(fm_np)), "Force matrix contains NaN/Inf values")

        net_forces = contacts.get_net_contact_forces(dt)
        nf_np = net_forces.numpy().reshape(contacts.sensor_count, 3)
        self.assertTrue(np.all(np.isfinite(nf_np)), "Net forces contain NaN/Inf values")

        _forces, _points, _normals, _separations, counts, start_indices, _ids = contacts.get_raw_contact_data(dt)
        counts_np = counts.numpy().flatten()
        starts_np = start_indices.numpy().flatten()
        expected_slots = contacts.sensor_count
        self.assertEqual(counts_np.size, expected_slots)
        self.assertEqual(starts_np.size, expected_slots)
        self.assertTrue(
            np.all(counts_np <= self.NUM_ENVS * 16),
            "Per-pair counts exceed max_contact_data_count",
        )
        self.assertTrue(np.all(counts_np >= 0))
        self.assertTrue(np.all(starts_np >= 0))

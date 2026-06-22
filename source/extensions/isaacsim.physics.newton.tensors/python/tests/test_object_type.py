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

"""Validate Newton object-type classification exposed by SimulationView.

The tests cover standalone rigid bodies, articulation root and child links,
articulation joints, non-physics xforms, and invalid paths so Newton matches
the ``omni.physics.tensors`` object type contract.
"""

from __future__ import annotations

import omni.physics.tensors as tensors

from .test_helpers import NewtonTensorTestBase, run_on_device_configs


@run_on_device_configs()
class TestObjectTypeRigidBody(NewtonTensorTestBase):
    """Standalone rigid bodies should be classified as RigidBody."""

    async def test_rigid_body(self) -> None:
        """Classify a standalone rigid body prim as ``ObjectType.RigidBody``."""
        self.setup_ball_grid(num_envs=2)
        sim = await self.create_sim()

        otype = sim.get_object_type("/envs/env0/ball")
        self.assertEqual(otype, tensors.ObjectType.RigidBody, "Standalone rigid body should return RigidBody")

    async def test_invalid_path(self) -> None:
        """Classify an unknown path as ``ObjectType.Invalid``."""
        self.setup_ball_grid(num_envs=2)
        sim = await self.create_sim()

        otype = sim.get_object_type("/nonexistent/path")
        self.assertEqual(otype, tensors.ObjectType.Invalid, "Unknown path should return Invalid")


@run_on_device_configs()
class TestObjectTypeArticulation(NewtonTensorTestBase):
    """Articulation links and joints should be classified correctly."""

    async def test_articulation_root_link(self) -> None:
        """Classify the articulation root body as ``ArticulationRootLink``."""
        self.setup_ant_grid(num_envs=2)
        sim = await self.create_sim()

        otype = sim.get_object_type("/envs/env0/ant/torso")
        self.assertEqual(
            otype, tensors.ObjectType.ArticulationRootLink, "Articulation root body should return ArticulationRootLink"
        )

    async def test_articulation_link(self) -> None:
        """Classify a non-root articulation body as ``ArticulationLink``."""
        self.setup_ant_grid(num_envs=2)
        sim = await self.create_sim()

        otype = sim.get_object_type("/envs/env0/ant/front_left_leg")
        self.assertEqual(
            otype, tensors.ObjectType.ArticulationLink, "Non-root articulation body should return ArticulationLink"
        )

    async def test_articulation_joint(self) -> None:
        """Classify a joint prim under the articulation as ``ArticulationJoint``."""
        self.setup_ant_grid(num_envs=2)
        sim = await self.create_sim()

        otype = sim.get_object_type("/envs/env0/ant/joints/front_left_leg")
        self.assertEqual(
            otype, tensors.ObjectType.ArticulationJoint, "Articulation joint should return ArticulationJoint"
        )

    async def test_nonphysics_xform(self) -> None:
        """Classify the parent xform without physics APIs as invalid."""
        self.setup_ant_grid(num_envs=2)
        sim = await self.create_sim()

        otype = sim.get_object_type("/envs/env0/ant")
        self.assertEqual(otype, tensors.ObjectType.Invalid, "Non-physics xform should return Invalid")

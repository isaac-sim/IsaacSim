# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression tests for articulation gravity compensation DOF indexing."""

import omni.kit.test
import warp as wp
from isaacsim.core.experimental.prims import Articulation


class _FakePhysicsArticulationView:
    def __init__(self, values: list[list[float]]) -> None:
        self._values = wp.array(values, dtype=wp.float32, device="cpu")

    def get_gravity_compensation_forces(self) -> wp.array:
        return self._values


class _FakeArticulation:
    valid = True
    _device = "cpu"

    def __init__(self, values: list[list[float]], num_dofs: int) -> None:
        self.num_dofs = num_dofs
        self._physics_articulation_view = _FakePhysicsArticulationView(values)

    def __len__(self) -> int:
        return 1

    def is_physics_tensor_entity_valid(self) -> bool:
        return True


class TestArticulationGravityCompensation(omni.kit.test.AsyncTestCase):
    """Verify gravity compensation maps public DOF indices to joint forces."""

    async def test_floating_base_root_wrench_is_excluded_before_dof_selection(self) -> None:
        """Skip the six root-wrench entries before applying requested DOF indices."""
        prim = _FakeArticulation(
            [[100.0, 101.0, 102.0, 103.0, 104.0, 105.0, 11.0, 22.0, 33.0]],
            num_dofs=3,
        )
        result = Articulation.get_dof_gravity_compensation_forces(prim, dof_indices=[2, 0])
        self.assertEqual(result.numpy().tolist(), [[33.0, 11.0]])

    async def test_fixed_base_layout_is_unchanged(self) -> None:
        """Keep direct DOF indexing when the lower-level tensor has no root prefix."""
        prim = _FakeArticulation([[11.0, 22.0, 33.0]], num_dofs=3)
        result = Articulation.get_dof_gravity_compensation_forces(prim, dof_indices=[1])
        self.assertEqual(result.numpy().tolist(), [[22.0]])

    async def test_unexpected_tensor_width_is_rejected(self) -> None:
        """Fail explicitly if a backend returns an undocumented compensation layout."""
        prim = _FakeArticulation([[1.0, 2.0, 3.0, 4.0]], num_dofs=3)
        with self.assertRaisesRegex(RuntimeError, "Unexpected gravity compensation force shape"):
            Articulation.get_dof_gravity_compensation_forces(prim)

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide engine-neutral deformable-body tensor scenarios."""

from __future__ import annotations

import os
import sys

import isaacsim.physics.manager.impl.tensors as tensors
import numpy as np
import warp as wp

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    SimulationEntities,
    Transform,
    get_asset_root,
)
from isaacsim.physics.manager.impl.tensors import EntityView  # noqa: E402


class _DeformableBodyCommon(GridTestBase):
    """Build one referenced deformable-body asset on a single-environment grid.

    Args:
        test_case: Test case that owns the scenario.
        device_params: Simulation device parameters.

    """

    ASSET_NAME = ""
    BODY_PATH = "/envs/*/Asset/DeformableBody"

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        sim_params = SimParams()
        sim_params.gravity_mag = 0.0
        sim_params.add_default_ground = False
        super().__init__(test_case, GridParams(num_envs=1, env_spacing=2.5), sim_params, device_params)
        self.maxsteps = 3

        asset_path = os.path.join(get_asset_root(), self.ASSET_NAME)
        actor_path = self.env_template_path.AppendChild("Asset")
        self.create_actor_from_asset(actor_path, Transform(), asset_path)
        self.view: EntityView

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Validate tensor data on the first physics step.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step.
            dt: Simulation step duration.

        """
        if stepno != 0:
            return
        self.check_data()
        self.finish()

    def check_data(self) -> None:
        """Validate the tensors exposed by the concrete deformable body."""
        raise NotImplementedError

    def _assert_float_data(self, impl: str, expected: np.ndarray) -> np.ndarray:
        """Read and validate one floating-point tensor implementation.

        Args:
            impl: Tensor implementation name.
            expected: Expected tensor values and shape.

        Returns:
            Read-back tensor converted to NumPy.

        """
        assert self.view.has_impl(impl, tensors.ImplKind.Get), f"missing GET impl {impl!r}"
        value = self.view.get_data(impl)
        assert value is not None, f"{impl} returned no data"
        raw = value.numpy()
        assert raw.size == expected.size, f"{impl} element count mismatch: got {raw.size}, expected {expected.size}"
        actual = raw.reshape(expected.shape)
        assert np.allclose(
            actual, expected, atol=1.0e-4
        ), f"{impl} mismatch: got {actual.tolist()}, expected {expected.tolist()}"
        return actual

    def _assert_float_round_trip(self, impl: str, target: np.ndarray) -> None:
        """Write and read back one indexed floating-point tensor.

        Args:
            impl: Tensor implementation name.
            target: Values to write.

        """
        assert self.view.has_impl(impl, tensors.ImplKind.Set), f"missing SET impl {impl!r}"
        target = target.astype(np.float32)
        source = self.to_warp(target)
        indices = wp.array([0], dtype=wp.int32, device=self.wp_device)
        self.view.set_data(impl, source, indices)
        raw = self.view.get_data(impl).numpy()
        assert raw.size == target.size, f"{impl} element count mismatch: got {raw.size}, expected {target.size}"
        result = raw.reshape(target.shape)
        assert np.allclose(
            result, target, atol=1.0e-4
        ), f"{impl} indexed write did not round-trip: got {result.tolist()}, expected {target.tolist()}"

    def _assert_indices(self, impl: str, expected: np.ndarray | None = None) -> None:
        """Validate a read-only connectivity tensor.

        Args:
            impl: Tensor implementation name.
            expected: Optional expected connectivity and shape.

        """
        assert self.view.has_impl(impl, tensors.ImplKind.Get), f"missing GET impl {impl!r}"
        value = self.view.get_data(impl)
        assert value is not None, f"{impl} returned no data"
        assert value.dtype == wp.int32, f"{impl} must use int32 indices"
        actual = value.numpy()
        assert np.all(actual >= 0), f"{impl} contains negative node indices"
        if expected is not None:
            assert actual.shape == expected.shape, f"{impl} shape mismatch"
            assert np.array_equal(actual, expected), f"{impl} connectivity mismatch"
        assert not self.view.has_impl(impl, tensors.ImplKind.Set), f"{impl} must remain read-only"


class VolumeDeformableBodyCommon(_DeformableBodyCommon):
    """Validate every supported volume-deformable-body tensor."""

    ASSET_NAME = "VolumeDeformable.usda"

    def on_start(self, sim: SimulationEntities) -> None:
        """Create and validate the volume-deformable-body view.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.view = sim.create_volume_deformable_body_view(self.BODY_PATH)
        self.check_deformable_body_view(self.view, 1)

    def check_data(self) -> None:
        """Validate volume state, kinematic targets, and connectivity."""
        positions = np.array(
            [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]]],
            dtype=np.float32,
        )
        zeros = np.zeros_like(positions)
        sim_positions = self._assert_float_data("sim-nodal-positions", positions)
        sim_velocities = self._assert_float_data("sim-nodal-velocities", zeros)
        self._assert_float_data("rest-nodal-positions", positions)
        self._assert_float_round_trip(
            "sim-nodal-positions",
            sim_positions + np.array([[[0.0, 0.05, 0.0]]], dtype=np.float32),
        )
        self._assert_float_round_trip(
            "sim-nodal-velocities",
            sim_velocities + np.array([[[0.25, 0.0, 0.0]]], dtype=np.float32),
        )

        targets = np.zeros((1, 5, 4), dtype=np.float32)
        targets[:, :, :3] = positions
        targets[:, :, 3] = 1.0
        self._assert_float_round_trip("sim-kinematic-targets", targets)

        expected_elements = np.array([[[0, 1, 2, 3], [1, 2, 3, 4]]], dtype=np.int32)
        self._assert_indices("sim-element-indices", expected_elements)
        self._assert_indices("collision-element-indices")


class SurfaceDeformableBodyCommon(_DeformableBodyCommon):
    """Validate every supported surface-deformable-body tensor."""

    ASSET_NAME = "SurfaceDeformable.usda"

    def on_start(self, sim: SimulationEntities) -> None:
        """Create and validate the surface-deformable-body view.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.view = sim.create_surface_deformable_body_view(self.BODY_PATH)
        self.check_deformable_body_view(self.view, 1)

    def check_data(self) -> None:
        """Validate surface state and connectivity."""
        positions = np.array(
            [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]],
            dtype=np.float32,
        )
        zeros = np.zeros_like(positions)
        sim_positions = self._assert_float_data("sim-positions", positions)
        sim_velocities = self._assert_float_data("sim-velocities", zeros)
        self._assert_float_data("rest-positions", positions)
        self._assert_float_round_trip(
            "sim-positions",
            sim_positions + np.array([[[0.0, 0.05, 0.0]]], dtype=np.float32),
        )
        self._assert_float_round_trip(
            "sim-velocities",
            sim_velocities + np.array([[[0.1, 0.0, 0.0]]], dtype=np.float32),
        )

        expected_elements = np.array([[[0, 1, 2], [1, 3, 2]]], dtype=np.int32)
        self._assert_indices("sim-element-indices", expected_elements)
        assert not self.view.has_impl(
            "sim-kinematic-targets", tensors.ImplKind.Get
        ), "surface kinematic targets are unsupported and must not be advertised"

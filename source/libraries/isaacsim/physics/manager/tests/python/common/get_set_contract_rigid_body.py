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

"""Verify the ``get_data`` and ``set_data`` contracts for rigid bodies.

The rigid-body version of the access-shape checks in ``get_set_contract`` (full /
external-buffer / indexed get, full / subset / index-size set, device on every
read). The shared check functions are reused as-is; this module only supplies the
rigid-body scene and the impls to run them against:

* runtime state -> ``velocities`` (6 components, all independently settable;
  unlike ``transforms``, whose quaternion is re-normalized on write and so would
  not round-trip exactly).
* parameter -> ``masses`` (1 component).

It also adds two shapes that only make sense here:

* multi-buffer set - ``apply-forces-and-torques-at-position``, fed as a list
  ``[forces, torques, positions]`` (the set-side counterpart of the contact
  multi-get), full and indexed.
* write-only rejection - ``apply-forces`` rejects ``get_data``.

The scene is one rigid ball per env with gravity off, so set/get round-trips are
exact. Scenario fixture and warmup come from ``get_set_contract``.
"""

from __future__ import annotations

import os
import sys

import isaacsim.physics.manager.impl.tensors as t
import pytest
import warp as wp

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    Transform,
)
from common.get_set_contract import (  # noqa: E402
    _INDICES,
    _ContractMixin,
    check_external_get,
    check_full_get,
    check_full_set,
    check_index_size_set,
    check_indexed_external_get,
    check_indexed_get,
    check_subset_set,
)

_MULTI_SET_IMPL = "apply-forces-and-torques-at-position"


class _RigidBodyContractBase(_ContractMixin, GridTestBase):
    """Rigid-body contract scene with one gravity-free ball per environment.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8, env_spacing=1.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)
        actor_path = self.env_template_path.AppendChild("ball")
        self.create_rigid_ball(actor_path, Transform((0.0, 0.0, 0.5)), 0.15)
        self.is_gpu = bool(device_params.use_gpu_pipeline)

    def on_start(self, sim: object) -> None:
        """Create the rigid-body view used by the contract.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.sim = sim
        self.view = sim.create_rigid_body_view("/envs/*/ball")


# --- velocities (runtime) + masses (parameter), one class per access shape ---


class VelocitiesFullGetCommon(_RigidBodyContractBase):
    """Verify a full engine-allocated velocity read."""

    def check_cell(self) -> None:
        """Run the full velocity-read contract."""
        check_full_get(self, "velocities", 6)


class VelocitiesExternalGetCommon(_RigidBodyContractBase):
    """Verify a full velocity read into caller storage."""

    def check_cell(self) -> None:
        """Run the external-buffer velocity-read contract."""
        check_external_get(self, "velocities", 6)


class VelocitiesIndexedGetCommon(_RigidBodyContractBase):
    """Verify an indexed engine-allocated velocity read."""

    def check_cell(self) -> None:
        """Run the indexed velocity-read contract."""
        check_indexed_get(self, "velocities", 6)


class VelocitiesIndexedExternalGetCommon(_RigidBodyContractBase):
    """Verify an indexed velocity read into caller storage."""

    def check_cell(self) -> None:
        """Run the indexed external-buffer velocity-read contract."""
        check_indexed_external_get(self, "velocities", 6)


class VelocitiesFullSetCommon(_RigidBodyContractBase):
    """Verify a full velocity write."""

    def check_cell(self) -> None:
        """Run the full velocity-write contract."""
        check_full_set(self, "velocities", 6)


class VelocitiesSubsetSetCommon(_RigidBodyContractBase):
    """Verify a full-size velocity write with selected indices."""

    def check_cell(self) -> None:
        """Run the subset velocity-write contract."""
        check_subset_set(self, "velocities", 6)


class VelocitiesIndexSizeSetCommon(_RigidBodyContractBase):
    """Verify a compact velocity write for selected indices."""

    def check_cell(self) -> None:
        """Run the compact indexed velocity-write contract."""
        check_index_size_set(self, "velocities", 6)


class MassesFullGetCommon(_RigidBodyContractBase):
    """Verify a full engine-allocated mass read."""

    def check_cell(self) -> None:
        """Run the full mass-read contract."""
        check_full_get(self, "masses", 1)


class MassesExternalGetCommon(_RigidBodyContractBase):
    """Verify a full mass read into caller storage."""

    def check_cell(self) -> None:
        """Run the external-buffer mass-read contract."""
        check_external_get(self, "masses", 1)


class MassesIndexedGetCommon(_RigidBodyContractBase):
    """Verify an indexed engine-allocated mass read."""

    def check_cell(self) -> None:
        """Run the indexed mass-read contract."""
        check_indexed_get(self, "masses", 1)


class MassesIndexedExternalGetCommon(_RigidBodyContractBase):
    """Verify an indexed mass read into caller storage."""

    def check_cell(self) -> None:
        """Run the indexed external-buffer mass-read contract."""
        check_indexed_external_get(self, "masses", 1)


class MassesFullSetCommon(_RigidBodyContractBase):
    """Verify a full mass write."""

    def check_cell(self) -> None:
        """Run the full mass-write contract."""
        check_full_set(self, "masses", 1)


class MassesSubsetSetCommon(_RigidBodyContractBase):
    """Verify a full-size mass write with selected indices."""

    def check_cell(self) -> None:
        """Run the subset mass-write contract."""
        check_subset_set(self, "masses", 1)


class MassesIndexSizeSetCommon(_RigidBodyContractBase):
    """Verify a compact mass write for selected indices."""

    def check_cell(self) -> None:
        """Run the compact indexed mass-write contract."""
        check_index_size_set(self, "masses", 1)


class ApplyMultiSetCommon(_RigidBodyContractBase):
    """A full multi-buffer force write changes every body's velocity."""

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Apply full force buffers and verify every body accelerates.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == self._WARMUP_STEPS:
            forces = wp.array([[10.0, 0.0, 0.0]] * self.num_envs, dtype=wp.float32, device=self.wp_device)
            zeros = wp.zeros((self.num_envs, 3), dtype=wp.float32, device=self.wp_device)
            self.view.set_data_multi(_MULTI_SET_IMPL, [forces, zeros, zeros])
            return
        if stepno == self._WARMUP_STEPS + 1:
            linear_x = self.view.get_data("velocities").numpy()[:, 0]
            assert (linear_x > 1.0e-5).all(), f"full multi-set did not accelerate every body: {linear_x}"
            self.finish()


class ApplyIndexedMultiSetCommon(_RigidBodyContractBase):
    """An indexed multi-buffer force write changes selected bodies only."""

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Apply indexed force buffers and verify only selected bodies accelerate.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == self._WARMUP_STEPS:
            forces = wp.array([[10.0, 0.0, 0.0]] * self.num_envs, dtype=wp.float32, device=self.wp_device)
            zeros = wp.zeros((self.num_envs, 3), dtype=wp.float32, device=self.wp_device)
            indices = wp.array(_INDICES, dtype=wp.int32, device=self.wp_device)
            self.view.set_data_multi(_MULTI_SET_IMPL, [forces, zeros, zeros], indices)
            return
        if stepno == self._WARMUP_STEPS + 1:
            linear_x = self.view.get_data("velocities").numpy()[:, 0]
            others = [index for index in range(self.num_envs) if index not in _INDICES]
            assert (
                linear_x[_INDICES] > 1.0e-5
            ).all(), f"indexed multi-set did not accelerate selected bodies: {linear_x}"
            assert (abs(linear_x[others]) < 1.0e-6).all(), f"indexed multi-set changed non-selected bodies: {linear_x}"
            self.finish()


class WriteOnlyRejectsGetCommon(_RigidBodyContractBase):
    """The backed write-only force operation rejects get_data."""

    def check_cell(self) -> None:
        """Verify write support and rejection of reading the force operation."""
        tc = self.test_case
        assert list(self.view.paths) == ["/envs/*/ball"]
        empty = self.sim.create_rigid_body_view("/does/not/match")
        assert empty.count == 0
        assert list(empty.paths) == ["/does/not/match"]
        assert self.view.has_impl("apply-forces", t.ImplKind.Set)
        assert not (self.view.has_impl("apply-forces", t.ImplKind.Get))
        data = wp.zeros((self.num_envs, 3), dtype=wp.float32, device=self.wp_device)
        self.view.set_data("apply-forces", data)
        with pytest.raises(Exception, match="get-implementation 'apply-forces' not registered"):
            self.view.get_data("apply-forces")

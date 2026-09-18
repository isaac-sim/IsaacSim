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

"""Validate the Newton ``get_data`` and ``set_data`` contracts.

One test per (access shape, entity, device). See the ``common.get_set_contract*``
modules for what each scenario asserts.

The indexed DOF-position cells additionally require contiguous ``int32``
indices and preserve selection order. Index *values* are not validated -- an
out-of-range read yields the dropped-row marker and an out-of-range write is
skipped, because rejecting either would require a host readback.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Callable
from typing import Protocol

import _physics_setup  # noqa: F401
import numpy as np
import pytest
import warp as wp

_TENSORS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TENSORS_DIR not in sys.path:
    sys.path.append(_TENSORS_DIR)

from _legacy_runner import (  # noqa: E402
    cpu_device,
    gpu_device,
    gpu_only,
    run_scenario,
)
from common.get_set_contract import (  # noqa: E402
    DofArmaturesExternalGetCommon,
    DofArmaturesFullGetCommon,
    DofArmaturesFullSetCommon,
    DofArmaturesIndexedExternalGetCommon,
    DofArmaturesIndexedGetCommon,
    DofArmaturesIndexSizeSetCommon,
    DofArmaturesSubsetSetCommon,
    DofDampingsExternalGetCommon,
    DofDampingsFullGetCommon,
    DofDampingsFullSetCommon,
    DofDampingsIndexedExternalGetCommon,
    DofDampingsIndexedGetCommon,
    DofDampingsIndexSizeSetCommon,
    DofDampingsSubsetSetCommon,
    DofMaxForcesExternalGetCommon,
    DofMaxForcesFullGetCommon,
    DofMaxForcesFullSetCommon,
    DofMaxForcesIndexedExternalGetCommon,
    DofMaxForcesIndexedGetCommon,
    DofMaxForcesIndexSizeSetCommon,
    DofMaxForcesSubsetSetCommon,
    DofPositionsExternalGetCommon,
    DofPositionsFullGetCommon,
    DofPositionsFullSetCommon,
    DofPositionsIndexedExternalGetCommon,
    DofPositionsIndexedGetCommon,
    DofPositionsIndexSizeSetCommon,
    DofPositionsSubsetSetCommon,
    DofStiffnessesExternalGetCommon,
    DofStiffnessesFullGetCommon,
    DofStiffnessesFullSetCommon,
    DofStiffnessesIndexedExternalGetCommon,
    DofStiffnessesIndexedGetCommon,
    DofStiffnessesIndexSizeSetCommon,
    DofStiffnessesSubsetSetCommon,
    ExtensibilityCommon,
    MetadataCommon,
    ReadOnlyRejectsSetCommon,
)
from common.get_set_contract_contact import (  # noqa: E402
    RawContactExternalMultiGetCommon,
    RawContactIndexedExternalMultiGetCommon,
    RawContactIndexedMultiGetCommon,
    RawContactMultiGetCommon,
)
from common.get_set_contract_rigid_body import (  # noqa: E402
    ApplyIndexedMultiSetCommon,
    ApplyMultiSetCommon,
    MassesExternalGetCommon,
    MassesFullGetCommon,
    MassesFullSetCommon,
    MassesIndexedExternalGetCommon,
    MassesIndexedGetCommon,
    MassesIndexSizeSetCommon,
    MassesSubsetSetCommon,
    VelocitiesExternalGetCommon,
    VelocitiesFullGetCommon,
    VelocitiesFullSetCommon,
    VelocitiesIndexedExternalGetCommon,
    VelocitiesIndexedGetCommon,
    VelocitiesIndexSizeSetCommon,
    VelocitiesSubsetSetCommon,
    WriteOnlyRejectsGetCommon,
)

_GAP_MULTI_SET = (
    "apply-forces-and-torques-at-position is wired as a single packed [K,9] buffer "
    "(matching the ovphysx contract); the multi-buffer set_data_multi path is not wired "
    "on Newton -- the C++ registry rejects the same op name as both single and multi."
)


class _CountedView(Protocol):
    count: int


class _IndexedScenario(Protocol):
    view: _CountedView
    wp_device: str


class _NewtonRawContactExternalMultiGet(RawContactExternalMultiGetCommon):
    EXPECT_EXTERNAL_ALIAS = True
    EXPECT_EXTERNAL_VALIDATION = True


class _NewtonRawContactIndexedExternalMultiGet(RawContactIndexedExternalMultiGetCommon):
    EXPECT_EXTERNAL_ALIAS = True
    EXPECT_EXTERNAL_VALIDATION = True


class _NewtonRawContactIndexedMultiGet(RawContactIndexedMultiGetCommon):
    EXPECT_EMPTY_SELECTION = True
    EXPECT_TRUNCATION_CLAMP = True


class _NewtonRawContactMultiGet(RawContactMultiGetCommon):
    pass


def _assert_out_of_range_read_is_dropped(
    scenario: _IndexedScenario,
    operation: Callable[[wp.array], object],
) -> None:
    """Verify that an out-of-range index yields the dropped-row marker.

    Index values live in device memory, so rejecting them would need a readback and
    the data path never synchronizes. The row carries the all-bits-set marker --
    ``NaN`` for the float bindings here -- so it cannot be mistaken for data.

    Args:
        scenario: Scenario supplying the view and device.
        operation: Indexed read to exercise.

    """
    for value in (-1, scenario.view.count):
        indices = wp.array([value], dtype=wp.int32, device=scenario.wp_device)
        row = operation(indices).numpy()
        assert np.all(np.isnan(row)), f"out-of-range index {value} produced {row!r}, expected the NaN marker"


def _assert_out_of_range_write_is_dropped(
    scenario: _IndexedScenario,
    operation: Callable[[wp.array], object],
    readback: Callable[[], object] | None = None,
) -> None:
    """Verify that an out-of-range index writes nothing and raises nothing.

    ``readback`` covers the half that matters. An out-of-range index is clamped so
    every dereference stays in bounds, and slot zero is the clamp target, so without
    the active-row mask the dropped write lands on entity zero. Snapshotting before
    and comparing after is what catches that; the operation raising nothing does not.

    The caller must supply data that differs from the current state, or writing it
    back is a no-op in value terms and the comparison passes regardless.

    Write-only operations have no readback and can only cover the "raises nothing"
    half.

    Args:
        scenario: Scenario supplying the view and device.
        operation: Indexed write to exercise.
        readback: Reads the state the write would have modified.

    """
    for value in (-1, scenario.view.count):
        before = None if readback is None else readback().numpy().copy()
        indices = wp.array([value], dtype=wp.int32, device=scenario.wp_device)
        operation(indices)
        if before is not None:
            np.testing.assert_array_equal(
                readback().numpy(),
                before,
                err_msg=f"out-of-range index {value} modified engine state",
            )


class _NewtonDofPositionsIndexedGet(DofPositionsIndexedGetCommon):
    def check_cell(self) -> None:
        # int64 indices are rejected by the framework int32-index guard (now uniform
        # across both backends -- ValueError, ahead of the Newton adapter's own check).
        wrong_dtype = wp.array([0], dtype=wp.int64, device=self.wp_device)
        with pytest.raises(ValueError, match="int32"):
            self.view.get_data("dof-positions", wrong_dtype)

        non_contiguous = wp.array([0, 0], dtype=wp.int32, device=self.wp_device)[::2]
        with pytest.raises(ValueError, match="indices must be contiguous"):
            self.view.get_data("dof-positions", non_contiguous)

        super().check_cell()
        full = self.view.get_data("dof-positions").numpy()
        for values in ([2, 0], [0, 0]):
            indices = wp.array(values, dtype=wp.int32, device=self.wp_device)
            selected = self.view.get_data("dof-positions", indices).numpy()
            assert (selected == full[values]).all(), f"indexed read did not preserve selection order {values}"

        _assert_out_of_range_read_is_dropped(
            self,
            lambda indices: self.view.get_data("dof-positions", indices),
        )


class _NewtonDofPositionsSubsetSet(DofPositionsSubsetSetCommon):
    def check_cell(self) -> None:
        super().check_cell()
        # Offset from the live state: writing the state back to itself would be
        # invisible to the readback comparison.
        data = wp.array(self.view.get_data("dof-positions").numpy() + 0.5, dtype=wp.float32, device=self.wp_device)
        _assert_out_of_range_write_is_dropped(
            self,
            lambda indices: self.view.set_data("dof-positions", data, indices),
            readback=lambda: self.view.get_data("dof-positions"),
        )


class _NewtonDofPositionsIndexSizeSet(DofPositionsIndexSizeSetCommon):
    def check_cell(self) -> None:
        """Verify that a dropped index does not disturb a write sharing its clamp slot.

        An out-of-range index is clamped so every dereference stays in bounds, and
        slot zero is the clamp target. A compact write selecting both entity 0 and an
        out-of-range entity therefore stages two rows at the same destination; only
        the active-row mask stops the dropped one from overwriting the real one.
        """
        super().check_cell()
        entity, dropped = 0, self.view.count  # `dropped` clamps onto `entity`
        wanted, junk = 0.25, -0.75
        # Distinct rows: were the dropped row allowed to race, entity 0 would end up
        # holding `junk`. Identical rows would make the race undetectable.
        compact = wp.array(
            np.array([[wanted] * 8, [junk] * 8], dtype=np.float32), dtype=wp.float32, device=self.wp_device
        )
        indices = wp.array([entity, dropped], dtype=wp.int32, device=self.wp_device)
        self.view.set_data("dof-positions", compact, indices)

        written = self.view.get_data("dof-positions").numpy()[entity]
        assert np.allclose(written, wanted), f"the dropped index corrupted entity {entity}: {written!r}"


class _NewtonApplyIndexedMultiSet(ApplyIndexedMultiSetCommon):
    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        if stepno == self._WARMUP_STEPS:
            data = [wp.zeros((self.num_envs, 3), dtype=wp.float32, device=self.wp_device) for _ in range(3)]
            _assert_out_of_range_write_is_dropped(
                self,
                lambda indices: self.view.set_data_multi(
                    "apply-forces-and-torques-at-position",
                    data,
                    indices,
                ),
            )
        super().on_physics_step(sim, stepno, dt)


class _NewtonWriteOnlyRejectsGet(WriteOnlyRejectsGetCommon):
    def check_cell(self) -> None:
        super().check_cell()
        # apply-forces supports indexed writes: full-count data with a selecting
        # index array (here a single-body subset) must be accepted.
        data = wp.zeros((self.num_envs, 3), dtype=wp.float32, device=self.wp_device)
        indices = wp.array([0], dtype=wp.int32, device=self.wp_device)
        self.view.set_data("apply-forces", data, indices)

        # The force-application paths reach the legacy Newton kernels directly rather
        # than through the shared row writer, so they get their own out-of-range cover.
        # This pins the visible half of the contract -- the call is accepted and drops
        # the entity. It cannot prove the memory-safety half: an unsanitized index is
        # dereferenced inside the kernel, which is undefined rather than reliably
        # fatal, so this assertion still passes against the unfixed code.
        _assert_out_of_range_write_is_dropped(
            self,
            lambda oob: self.view.set_data("apply-forces", data, oob),
        )


class TestGetSetContract:
    # --- articulation: dof-positions (runtime) -------------------------
    """Validate entity-view get/set contracts with Newton.

    The matrix runs CPU and GPU variants for supported single- and
    multi-buffer operations. Multi-buffer wrench writes remain explicitly
    skipped because Newton registers that operation in packed single-buffer
    form.
    """

    def test_dof_positions_full_get_newton_cc(self) -> None:
        """Verify full DOF-position reads on the Newton CPU pipeline."""
        run_scenario(self, DofPositionsFullGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_positions_full_get_newton_gg(self) -> None:
        """Verify full DOF-position reads on the Newton GPU pipeline."""
        run_scenario(self, DofPositionsFullGetCommon, "newton", gpu_device())

    def test_dof_positions_external_get_newton_cc(self) -> None:
        """Verify external-buffer DOF-position reads on the Newton CPU pipeline."""
        run_scenario(self, DofPositionsExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_positions_external_get_newton_gg(self) -> None:
        """Verify external-buffer DOF-position reads on the Newton GPU pipeline."""
        run_scenario(self, DofPositionsExternalGetCommon, "newton", gpu_device())

    def test_dof_positions_indexed_get_newton_cc(self) -> None:
        """Verify indexed DOF-position reads on the Newton CPU pipeline."""
        run_scenario(self, _NewtonDofPositionsIndexedGet, "newton", cpu_device())

    @gpu_only
    def test_dof_positions_indexed_get_newton_gg(self) -> None:
        """Verify indexed DOF-position reads on the Newton GPU pipeline."""
        run_scenario(self, _NewtonDofPositionsIndexedGet, "newton", gpu_device())

    def test_dof_positions_indexed_external_get_newton_cc(self) -> None:
        """Verify indexed external-buffer DOF-position reads on the Newton CPU pipeline."""
        run_scenario(self, DofPositionsIndexedExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_positions_indexed_external_get_newton_gg(self) -> None:
        """Verify indexed external-buffer DOF-position reads on the Newton GPU pipeline."""
        run_scenario(self, DofPositionsIndexedExternalGetCommon, "newton", gpu_device())

    def test_dof_positions_full_set_newton_cc(self) -> None:
        """Verify full DOF-position writes on the Newton CPU pipeline."""
        run_scenario(self, DofPositionsFullSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_positions_full_set_newton_gg(self) -> None:
        """Verify full DOF-position writes on the Newton GPU pipeline."""
        run_scenario(self, DofPositionsFullSetCommon, "newton", gpu_device())

    def test_dof_positions_subset_set_newton_cc(self) -> None:
        """Verify subset DOF-position writes on the Newton CPU pipeline."""
        run_scenario(self, _NewtonDofPositionsSubsetSet, "newton", cpu_device())

    @gpu_only
    def test_dof_positions_subset_set_newton_gg(self) -> None:
        """Verify subset DOF-position writes on the Newton GPU pipeline."""
        run_scenario(self, _NewtonDofPositionsSubsetSet, "newton", gpu_device())

    def test_dof_positions_index_size_set_newton_cc(self) -> None:
        """Verify compact indexed DOF-position writes on the Newton CPU pipeline."""
        run_scenario(self, _NewtonDofPositionsIndexSizeSet, "newton", cpu_device())

    @gpu_only
    def test_dof_positions_index_size_set_newton_gg(self) -> None:
        """Verify compact indexed DOF-position writes on the Newton GPU pipeline."""
        run_scenario(self, _NewtonDofPositionsIndexSizeSet, "newton", gpu_device())

    # --- articulation: dof-stiffnesses (parameter) ---------------------
    def test_dof_stiffnesses_full_get_newton_cc(self) -> None:
        """Verify full DOF-stiffness reads on the Newton CPU pipeline."""
        run_scenario(self, DofStiffnessesFullGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_full_get_newton_gg(self) -> None:
        """Verify full DOF-stiffness reads on the Newton GPU pipeline."""
        run_scenario(self, DofStiffnessesFullGetCommon, "newton", gpu_device())

    def test_dof_stiffnesses_external_get_newton_cc(self) -> None:
        """Verify external-buffer DOF-stiffness reads on the Newton CPU pipeline."""
        run_scenario(self, DofStiffnessesExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_external_get_newton_gg(self) -> None:
        """Verify external-buffer DOF-stiffness reads on the Newton GPU pipeline."""
        run_scenario(self, DofStiffnessesExternalGetCommon, "newton", gpu_device())

    def test_dof_stiffnesses_indexed_get_newton_cc(self) -> None:
        """Verify indexed DOF-stiffness reads on the Newton CPU pipeline."""
        run_scenario(self, DofStiffnessesIndexedGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_indexed_get_newton_gg(self) -> None:
        """Verify indexed DOF-stiffness reads on the Newton GPU pipeline."""
        run_scenario(self, DofStiffnessesIndexedGetCommon, "newton", gpu_device())

    def test_dof_stiffnesses_indexed_external_get_newton_cc(self) -> None:
        """Verify indexed external-buffer DOF-stiffness reads on the Newton CPU pipeline."""
        run_scenario(self, DofStiffnessesIndexedExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_indexed_external_get_newton_gg(self) -> None:
        """Verify indexed external-buffer DOF-stiffness reads on the Newton GPU pipeline."""
        run_scenario(self, DofStiffnessesIndexedExternalGetCommon, "newton", gpu_device())

    def test_dof_stiffnesses_full_set_newton_cc(self) -> None:
        """Verify full DOF-stiffness writes on the Newton CPU pipeline."""
        run_scenario(self, DofStiffnessesFullSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_full_set_newton_gg(self) -> None:
        """Verify full DOF-stiffness writes on the Newton GPU pipeline."""
        run_scenario(self, DofStiffnessesFullSetCommon, "newton", gpu_device())

    def test_dof_stiffnesses_subset_set_newton_cc(self) -> None:
        """Verify subset DOF-stiffness writes on the Newton CPU pipeline."""
        run_scenario(self, DofStiffnessesSubsetSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_subset_set_newton_gg(self) -> None:
        """Verify subset DOF-stiffness writes on the Newton GPU pipeline."""
        run_scenario(self, DofStiffnessesSubsetSetCommon, "newton", gpu_device())

    def test_dof_stiffnesses_index_size_set_newton_cc(self) -> None:
        """Verify compact indexed DOF-stiffness writes on the Newton CPU pipeline."""
        run_scenario(self, DofStiffnessesIndexSizeSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_index_size_set_newton_gg(self) -> None:
        """Verify compact indexed DOF-stiffness writes on the Newton GPU pipeline."""
        run_scenario(self, DofStiffnessesIndexSizeSetCommon, "newton", gpu_device())

    # --- articulation: dof-dampings (parameter) -----------------------
    def test_dof_dampings_full_get_newton_cc(self) -> None:
        """Verify full DOF-damping reads on the Newton CPU pipeline."""
        run_scenario(self, DofDampingsFullGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_dampings_full_get_newton_gg(self) -> None:
        """Verify full DOF-damping reads on the Newton GPU pipeline."""
        run_scenario(self, DofDampingsFullGetCommon, "newton", gpu_device())

    def test_dof_dampings_external_get_newton_cc(self) -> None:
        """Verify external-buffer DOF-damping reads on the Newton CPU pipeline."""
        run_scenario(self, DofDampingsExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_dampings_external_get_newton_gg(self) -> None:
        """Verify external-buffer DOF-damping reads on the Newton GPU pipeline."""
        run_scenario(self, DofDampingsExternalGetCommon, "newton", gpu_device())

    def test_dof_dampings_indexed_get_newton_cc(self) -> None:
        """Verify indexed DOF-damping reads on the Newton CPU pipeline."""
        run_scenario(self, DofDampingsIndexedGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_dampings_indexed_get_newton_gg(self) -> None:
        """Verify indexed DOF-damping reads on the Newton GPU pipeline."""
        run_scenario(self, DofDampingsIndexedGetCommon, "newton", gpu_device())

    def test_dof_dampings_indexed_external_get_newton_cc(self) -> None:
        """Verify indexed external-buffer DOF-damping reads on the Newton CPU pipeline."""
        run_scenario(self, DofDampingsIndexedExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_dampings_indexed_external_get_newton_gg(self) -> None:
        """Verify indexed external-buffer DOF-damping reads on the Newton GPU pipeline."""
        run_scenario(self, DofDampingsIndexedExternalGetCommon, "newton", gpu_device())

    def test_dof_dampings_full_set_newton_cc(self) -> None:
        """Verify full DOF-damping writes on the Newton CPU pipeline."""
        run_scenario(self, DofDampingsFullSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_dampings_full_set_newton_gg(self) -> None:
        """Verify full DOF-damping writes on the Newton GPU pipeline."""
        run_scenario(self, DofDampingsFullSetCommon, "newton", gpu_device())

    def test_dof_dampings_subset_set_newton_cc(self) -> None:
        """Verify subset DOF-damping writes on the Newton CPU pipeline."""
        run_scenario(self, DofDampingsSubsetSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_dampings_subset_set_newton_gg(self) -> None:
        """Verify subset DOF-damping writes on the Newton GPU pipeline."""
        run_scenario(self, DofDampingsSubsetSetCommon, "newton", gpu_device())

    def test_dof_dampings_index_size_set_newton_cc(self) -> None:
        """Verify compact indexed DOF-damping writes on the Newton CPU pipeline."""
        run_scenario(self, DofDampingsIndexSizeSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_dampings_index_size_set_newton_gg(self) -> None:
        """Verify compact indexed DOF-damping writes on the Newton GPU pipeline."""
        run_scenario(self, DofDampingsIndexSizeSetCommon, "newton", gpu_device())

    # --- articulation: dof-armatures (parameter) ----------------------
    def test_dof_armatures_full_get_newton_cc(self) -> None:
        """Verify full DOF-armature reads on the Newton CPU pipeline."""
        run_scenario(self, DofArmaturesFullGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_armatures_full_get_newton_gg(self) -> None:
        """Verify full DOF-armature reads on the Newton GPU pipeline."""
        run_scenario(self, DofArmaturesFullGetCommon, "newton", gpu_device())

    def test_dof_armatures_external_get_newton_cc(self) -> None:
        """Verify external-buffer DOF-armature reads on the Newton CPU pipeline."""
        run_scenario(self, DofArmaturesExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_armatures_external_get_newton_gg(self) -> None:
        """Verify external-buffer DOF-armature reads on the Newton GPU pipeline."""
        run_scenario(self, DofArmaturesExternalGetCommon, "newton", gpu_device())

    def test_dof_armatures_indexed_get_newton_cc(self) -> None:
        """Verify indexed DOF-armature reads on the Newton CPU pipeline."""
        run_scenario(self, DofArmaturesIndexedGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_armatures_indexed_get_newton_gg(self) -> None:
        """Verify indexed DOF-armature reads on the Newton GPU pipeline."""
        run_scenario(self, DofArmaturesIndexedGetCommon, "newton", gpu_device())

    def test_dof_armatures_indexed_external_get_newton_cc(self) -> None:
        """Verify indexed external-buffer DOF-armature reads on the Newton CPU pipeline."""
        run_scenario(self, DofArmaturesIndexedExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_armatures_indexed_external_get_newton_gg(self) -> None:
        """Verify indexed external-buffer DOF-armature reads on the Newton GPU pipeline."""
        run_scenario(self, DofArmaturesIndexedExternalGetCommon, "newton", gpu_device())

    def test_dof_armatures_full_set_newton_cc(self) -> None:
        """Verify full DOF-armature writes on the Newton CPU pipeline."""
        run_scenario(self, DofArmaturesFullSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_armatures_full_set_newton_gg(self) -> None:
        """Verify full DOF-armature writes on the Newton GPU pipeline."""
        run_scenario(self, DofArmaturesFullSetCommon, "newton", gpu_device())

    def test_dof_armatures_subset_set_newton_cc(self) -> None:
        """Verify subset DOF-armature writes on the Newton CPU pipeline."""
        run_scenario(self, DofArmaturesSubsetSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_armatures_subset_set_newton_gg(self) -> None:
        """Verify subset DOF-armature writes on the Newton GPU pipeline."""
        run_scenario(self, DofArmaturesSubsetSetCommon, "newton", gpu_device())

    def test_dof_armatures_index_size_set_newton_cc(self) -> None:
        """Verify compact indexed DOF-armature writes on the Newton CPU pipeline."""
        run_scenario(self, DofArmaturesIndexSizeSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_armatures_index_size_set_newton_gg(self) -> None:
        """Verify compact indexed DOF-armature writes on the Newton GPU pipeline."""
        run_scenario(self, DofArmaturesIndexSizeSetCommon, "newton", gpu_device())

    # --- articulation: dof-max-forces (parameter) ---------------------
    def test_dof_max_forces_full_get_newton_cc(self) -> None:
        """Verify full DOF maximum-force reads on the Newton CPU pipeline."""
        run_scenario(self, DofMaxForcesFullGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_max_forces_full_get_newton_gg(self) -> None:
        """Verify full DOF maximum-force reads on the Newton GPU pipeline."""
        run_scenario(self, DofMaxForcesFullGetCommon, "newton", gpu_device())

    def test_dof_max_forces_external_get_newton_cc(self) -> None:
        """Verify external-buffer DOF maximum-force reads on the Newton CPU pipeline."""
        run_scenario(self, DofMaxForcesExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_max_forces_external_get_newton_gg(self) -> None:
        """Verify external-buffer DOF maximum-force reads on the Newton GPU pipeline."""
        run_scenario(self, DofMaxForcesExternalGetCommon, "newton", gpu_device())

    def test_dof_max_forces_indexed_get_newton_cc(self) -> None:
        """Verify indexed DOF maximum-force reads on the Newton CPU pipeline."""
        run_scenario(self, DofMaxForcesIndexedGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_max_forces_indexed_get_newton_gg(self) -> None:
        """Verify indexed DOF maximum-force reads on the Newton GPU pipeline."""
        run_scenario(self, DofMaxForcesIndexedGetCommon, "newton", gpu_device())

    def test_dof_max_forces_indexed_external_get_newton_cc(self) -> None:
        """Verify indexed external-buffer DOF maximum-force reads on the Newton CPU pipeline."""
        run_scenario(self, DofMaxForcesIndexedExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_max_forces_indexed_external_get_newton_gg(self) -> None:
        """Verify indexed external-buffer DOF maximum-force reads on the Newton GPU pipeline."""
        run_scenario(self, DofMaxForcesIndexedExternalGetCommon, "newton", gpu_device())

    def test_dof_max_forces_full_set_newton_cc(self) -> None:
        """Verify full DOF maximum-force writes on the Newton CPU pipeline."""
        run_scenario(self, DofMaxForcesFullSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_max_forces_full_set_newton_gg(self) -> None:
        """Verify full DOF maximum-force writes on the Newton GPU pipeline."""
        run_scenario(self, DofMaxForcesFullSetCommon, "newton", gpu_device())

    def test_dof_max_forces_subset_set_newton_cc(self) -> None:
        """Verify subset DOF maximum-force writes on the Newton CPU pipeline."""
        run_scenario(self, DofMaxForcesSubsetSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_max_forces_subset_set_newton_gg(self) -> None:
        """Verify subset DOF maximum-force writes on the Newton GPU pipeline."""
        run_scenario(self, DofMaxForcesSubsetSetCommon, "newton", gpu_device())

    def test_dof_max_forces_index_size_set_newton_cc(self) -> None:
        """Verify compact indexed DOF maximum-force writes on the Newton CPU pipeline."""
        run_scenario(self, DofMaxForcesIndexSizeSetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_max_forces_index_size_set_newton_gg(self) -> None:
        """Verify compact indexed DOF maximum-force writes on the Newton GPU pipeline."""
        run_scenario(self, DofMaxForcesIndexSizeSetCommon, "newton", gpu_device())

    # --- articulation: extensibility / metadata / read-only -----------
    def test_extensibility_newton_cc(self) -> None:
        """Verify extensibility on the Newton CPU pipeline."""
        run_scenario(self, ExtensibilityCommon, "newton", cpu_device())

    @gpu_only
    def test_extensibility_newton_gg(self) -> None:
        """Verify extensibility on the Newton GPU pipeline."""
        run_scenario(self, ExtensibilityCommon, "newton", gpu_device())

    def test_metadata_newton_cc(self) -> None:
        """Verify metadata on the Newton CPU pipeline."""
        run_scenario(self, MetadataCommon, "newton", cpu_device())

    @gpu_only
    def test_metadata_newton_gg(self) -> None:
        """Verify metadata on the Newton GPU pipeline."""
        run_scenario(self, MetadataCommon, "newton", gpu_device())

    def test_read_only_rejects_set_newton_cc(self) -> None:
        """Verify read-only operations reject writes on the Newton CPU pipeline."""
        run_scenario(self, ReadOnlyRejectsSetCommon, "newton", cpu_device())

    @gpu_only
    def test_read_only_rejects_set_newton_gg(self) -> None:
        """Verify read-only operations reject writes on the Newton GPU pipeline."""
        run_scenario(self, ReadOnlyRejectsSetCommon, "newton", gpu_device())

    # --- rigid-body: velocities (runtime) -----------------------------
    def test_velocities_full_get_newton_cc(self) -> None:
        """Verify full velocity reads on the Newton CPU pipeline."""
        run_scenario(self, VelocitiesFullGetCommon, "newton", cpu_device())

    @gpu_only
    def test_velocities_full_get_newton_gg(self) -> None:
        """Verify full velocity reads on the Newton GPU pipeline."""
        run_scenario(self, VelocitiesFullGetCommon, "newton", gpu_device())

    def test_velocities_external_get_newton_cc(self) -> None:
        """Verify external-buffer velocity reads on the Newton CPU pipeline."""
        run_scenario(self, VelocitiesExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_velocities_external_get_newton_gg(self) -> None:
        """Verify external-buffer velocity reads on the Newton GPU pipeline."""
        run_scenario(self, VelocitiesExternalGetCommon, "newton", gpu_device())

    def test_velocities_indexed_get_newton_cc(self) -> None:
        """Verify indexed velocity reads on the Newton CPU pipeline."""
        run_scenario(self, VelocitiesIndexedGetCommon, "newton", cpu_device())

    @gpu_only
    def test_velocities_indexed_get_newton_gg(self) -> None:
        """Verify indexed velocity reads on the Newton GPU pipeline."""
        run_scenario(self, VelocitiesIndexedGetCommon, "newton", gpu_device())

    def test_velocities_indexed_external_get_newton_cc(self) -> None:
        """Verify indexed external-buffer velocity reads on the Newton CPU pipeline."""
        run_scenario(self, VelocitiesIndexedExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_velocities_indexed_external_get_newton_gg(self) -> None:
        """Verify indexed external-buffer velocity reads on the Newton GPU pipeline."""
        run_scenario(self, VelocitiesIndexedExternalGetCommon, "newton", gpu_device())

    def test_velocities_full_set_newton_cc(self) -> None:
        """Verify full velocity writes on the Newton CPU pipeline."""
        run_scenario(self, VelocitiesFullSetCommon, "newton", cpu_device())

    @gpu_only
    def test_velocities_full_set_newton_gg(self) -> None:
        """Verify full velocity writes on the Newton GPU pipeline."""
        run_scenario(self, VelocitiesFullSetCommon, "newton", gpu_device())

    def test_velocities_subset_set_newton_cc(self) -> None:
        """Verify subset velocity writes on the Newton CPU pipeline."""
        run_scenario(self, VelocitiesSubsetSetCommon, "newton", cpu_device())

    @gpu_only
    def test_velocities_subset_set_newton_gg(self) -> None:
        """Verify subset velocity writes on the Newton GPU pipeline."""
        run_scenario(self, VelocitiesSubsetSetCommon, "newton", gpu_device())

    def test_velocities_index_size_set_newton_cc(self) -> None:
        """Verify compact indexed velocity writes on the Newton CPU pipeline."""
        run_scenario(self, VelocitiesIndexSizeSetCommon, "newton", cpu_device())

    @gpu_only
    def test_velocities_index_size_set_newton_gg(self) -> None:
        """Verify compact indexed velocity writes on the Newton GPU pipeline."""
        run_scenario(self, VelocitiesIndexSizeSetCommon, "newton", gpu_device())

    # --- rigid-body: masses (parameter) -------------------------------
    def test_masses_full_get_newton_cc(self) -> None:
        """Verify full mass reads on the Newton CPU pipeline."""
        run_scenario(self, MassesFullGetCommon, "newton", cpu_device())

    @gpu_only
    def test_masses_full_get_newton_gg(self) -> None:
        """Verify full mass reads on the Newton GPU pipeline."""
        run_scenario(self, MassesFullGetCommon, "newton", gpu_device())

    def test_masses_external_get_newton_cc(self) -> None:
        """Verify external-buffer mass reads on the Newton CPU pipeline."""
        run_scenario(self, MassesExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_masses_external_get_newton_gg(self) -> None:
        """Verify external-buffer mass reads on the Newton GPU pipeline."""
        run_scenario(self, MassesExternalGetCommon, "newton", gpu_device())

    def test_masses_indexed_get_newton_cc(self) -> None:
        """Verify indexed mass reads on the Newton CPU pipeline."""
        run_scenario(self, MassesIndexedGetCommon, "newton", cpu_device())

    @gpu_only
    def test_masses_indexed_get_newton_gg(self) -> None:
        """Verify indexed mass reads on the Newton GPU pipeline."""
        run_scenario(self, MassesIndexedGetCommon, "newton", gpu_device())

    def test_masses_indexed_external_get_newton_cc(self) -> None:
        """Verify indexed external-buffer mass reads on the Newton CPU pipeline."""
        run_scenario(self, MassesIndexedExternalGetCommon, "newton", cpu_device())

    @gpu_only
    def test_masses_indexed_external_get_newton_gg(self) -> None:
        """Verify indexed external-buffer mass reads on the Newton GPU pipeline."""
        run_scenario(self, MassesIndexedExternalGetCommon, "newton", gpu_device())

    def test_masses_full_set_newton_cc(self) -> None:
        """Verify full mass writes on the Newton CPU pipeline."""
        run_scenario(self, MassesFullSetCommon, "newton", cpu_device())

    @gpu_only
    def test_masses_full_set_newton_gg(self) -> None:
        """Verify full mass writes on the Newton GPU pipeline."""
        run_scenario(self, MassesFullSetCommon, "newton", gpu_device())

    def test_masses_subset_set_newton_cc(self) -> None:
        """Verify subset mass writes on the Newton CPU pipeline."""
        run_scenario(self, MassesSubsetSetCommon, "newton", cpu_device())

    @gpu_only
    def test_masses_subset_set_newton_gg(self) -> None:
        """Verify subset mass writes on the Newton GPU pipeline."""
        run_scenario(self, MassesSubsetSetCommon, "newton", gpu_device())

    def test_masses_index_size_set_newton_cc(self) -> None:
        """Verify compact indexed mass writes on the Newton CPU pipeline."""
        run_scenario(self, MassesIndexSizeSetCommon, "newton", cpu_device())

    @gpu_only
    def test_masses_index_size_set_newton_gg(self) -> None:
        """Verify compact indexed mass writes on the Newton GPU pipeline."""
        run_scenario(self, MassesIndexSizeSetCommon, "newton", gpu_device())

    # --- rigid-body: multi-buffer set + write-only --------------------
    @pytest.mark.skip(reason=_GAP_MULTI_SET)
    def test_apply_multi_set_newton_cc(self) -> None:
        """Verify multi-buffer wrench writes on the Newton CPU pipeline."""
        run_scenario(self, ApplyMultiSetCommon, "newton", cpu_device())

    @gpu_only
    @pytest.mark.skip(reason=_GAP_MULTI_SET)
    def test_apply_multi_set_newton_gg(self) -> None:
        """Verify multi-buffer wrench writes on the Newton GPU pipeline."""
        run_scenario(self, ApplyMultiSetCommon, "newton", gpu_device())

    @pytest.mark.skip(reason=_GAP_MULTI_SET)
    def test_apply_indexed_multi_set_newton_cc(self) -> None:
        """Verify indexed multi-buffer wrench writes on the Newton CPU pipeline."""
        run_scenario(self, _NewtonApplyIndexedMultiSet, "newton", cpu_device())

    @gpu_only
    @pytest.mark.skip(reason=_GAP_MULTI_SET)
    def test_apply_indexed_multi_set_newton_gg(self) -> None:
        """Verify indexed multi-buffer wrench writes on the Newton GPU pipeline."""
        run_scenario(self, _NewtonApplyIndexedMultiSet, "newton", gpu_device())

    def test_write_only_rejects_get_newton_cc(self) -> None:
        """Verify write-only operations reject reads on the Newton CPU pipeline."""
        run_scenario(self, _NewtonWriteOnlyRejectsGet, "newton", cpu_device())

    @gpu_only
    def test_write_only_rejects_get_newton_gg(self) -> None:
        """Verify write-only operations reject reads on the Newton GPU pipeline."""
        run_scenario(self, _NewtonWriteOnlyRejectsGet, "newton", gpu_device())

    # --- rigid-contact: multi-buffer get ------------------------------
    def test_raw_contact_multi_get_newton_cc(self) -> None:
        """Verify raw-contact multi-buffer reads on the Newton CPU pipeline."""
        run_scenario(self, _NewtonRawContactMultiGet, "newton", cpu_device())

    @gpu_only
    def test_raw_contact_multi_get_newton_gg(self) -> None:
        """Verify raw-contact multi-buffer reads on the Newton GPU pipeline."""
        run_scenario(self, _NewtonRawContactMultiGet, "newton", gpu_device())

    def test_raw_contact_indexed_multi_get_newton_cc(self) -> None:
        """Verify indexed raw-contact multi-buffer reads on the Newton CPU pipeline."""
        run_scenario(self, _NewtonRawContactIndexedMultiGet, "newton", cpu_device())

    @gpu_only
    def test_raw_contact_indexed_multi_get_newton_gg(self) -> None:
        """Verify indexed raw-contact multi-buffer reads on the Newton GPU pipeline."""
        run_scenario(self, _NewtonRawContactIndexedMultiGet, "newton", gpu_device())

    def test_raw_contact_external_multi_get_newton_cc(self) -> None:
        """Verify external-buffer raw-contact reads on the Newton CPU pipeline."""
        run_scenario(self, _NewtonRawContactExternalMultiGet, "newton", cpu_device())

    @gpu_only
    def test_raw_contact_external_multi_get_newton_gg(self) -> None:
        """Verify external-buffer raw-contact reads on the Newton GPU pipeline."""
        run_scenario(self, _NewtonRawContactExternalMultiGet, "newton", gpu_device())

    def test_raw_contact_indexed_external_multi_get_newton_cc(self) -> None:
        """Verify indexed external-buffer raw-contact reads on the Newton CPU pipeline."""
        run_scenario(self, _NewtonRawContactIndexedExternalMultiGet, "newton", cpu_device())

    @gpu_only
    def test_raw_contact_indexed_external_multi_get_newton_gg(self) -> None:
        """Verify indexed external-buffer raw-contact reads on the Newton GPU pipeline."""
        run_scenario(self, _NewtonRawContactIndexedExternalMultiGet, "newton", gpu_device())

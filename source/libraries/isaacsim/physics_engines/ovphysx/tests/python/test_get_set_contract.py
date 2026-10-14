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

"""Validate the OvPhysX ``get_data`` and ``set_data`` contracts.

Each test selects an access shape, entity type, and device. The
``common.get_set_contract*`` modules define the assertions for each scenario.
Skip markers identify backend-contract gaps; enabled cases validate the
supported access shapes.
"""

from __future__ import annotations

import os
import sys

import _physics_setup  # noqa: F401
import pytest
import warp as wp

_TENSORS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TENSORS_DIR not in sys.path:
    sys.path.insert(0, _TENSORS_DIR)

from _legacy_runner import (  # noqa: E402
    cpu_device,
    gpu_device,
    gpu_only,
    register_scenario_batches,
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
from physx_usd_schemas import PhysxSchema  # noqa: E402

# Gaps in the current ovphysx tensor backend. Each guards the cases
# that fail until the capability lands; delete the decorator to re-enable.
_GAP_INDEXED_MULTI_GET = "ovphysx get_data_multi (contact views) has no indexed-read path"
_GAP_MULTI_SET = "ovphysx multi-buffer set (apply-forces-and-torques-at-position) is not wired"


class _PhysxContactReportMixin:
    """Configure each scenario body for OvPhysX contact reporting.

    The mixin disables sleeping and uses a zero contact-report threshold so
    every replicated contact body can be resolved by a contact view.
    """

    def _apply_engine_specifics(self) -> None:
        super()._apply_engine_specifics()
        prefix = str(self.env_template_path)
        for tmpl_path in self._contact_body_paths:
            rel = str(tmpl_path)[len(prefix) :].lstrip("/") if str(tmpl_path).startswith(prefix) else tmpl_path.name
            for i in range(self.num_envs):
                prim = self.stage.GetPrimAtPath(f"/envs/env{i}/{rel}")
                if not prim:
                    continue
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(0)
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0)


# Contact scenarios need the engine-specific contact-report schemas mixed in.
class PhysxRawContactMultiGet(_PhysxContactReportMixin, RawContactMultiGetCommon):
    """Configure raw-contact multi-buffer reads for OvPhysX."""


class PhysxRawContactIndexedMultiGet(_PhysxContactReportMixin, RawContactIndexedMultiGetCommon):
    """Configure indexed raw-contact multi-buffer reads for OvPhysX."""


class PhysxRawContactExternalMultiGet(_PhysxContactReportMixin, RawContactExternalMultiGetCommon):
    """Configure external-buffer raw-contact reads for OvPhysX."""

    EXPECT_EXTERNAL_ALIAS = True
    EXPECT_EXTERNAL_VALIDATION = True


class PhysxRawContactIndexedExternalMultiGet(_PhysxContactReportMixin, RawContactIndexedExternalMultiGetCommon):
    """Configure indexed external-buffer raw-contact reads for OvPhysX."""


class _PhysxDofPositionsOutOfRangeSet(DofPositionsSubsetSetCommon):
    """An out-of-range indexed write must drop the entity rather than raise.

    The tensor backend already behaves this way -- the CPU and GPU views skip an
    index outside their entry range -- but the ovphysx C API adds a validation pass
    over the index buffer and turns the drop into an error. That pass is gated on
    CPU-resident indices, so ovphysx also disagrees with itself across lanes.

    Marked expected-to-fail rather than skipped so it reports XPASS once the wheel
    stops rejecting and the pin moves, instead of sitting unnoticed. That also makes
    it the check for whether the pinned wheel carries the defect at all.
    """

    def check_cell(self) -> None:
        super().check_cell()
        data = self.view.get_data("dof-positions")
        for value in (-1, self.view.count):
            indices = wp.array([value], dtype=wp.int32, device=self.wp_device)
            self.view.set_data("dof-positions", data, indices)


_ARTICULATION_CONTRACT_SCENARIOS = (
    DofPositionsFullGetCommon,
    DofPositionsExternalGetCommon,
    DofPositionsIndexedGetCommon,
    DofPositionsIndexedExternalGetCommon,
    DofPositionsFullSetCommon,
    DofPositionsSubsetSetCommon,
    DofPositionsIndexSizeSetCommon,
    DofStiffnessesFullGetCommon,
    DofStiffnessesExternalGetCommon,
    DofStiffnessesIndexedGetCommon,
    DofStiffnessesIndexedExternalGetCommon,
    DofStiffnessesFullSetCommon,
    DofStiffnessesSubsetSetCommon,
    DofStiffnessesIndexSizeSetCommon,
    DofDampingsFullGetCommon,
    DofDampingsExternalGetCommon,
    DofDampingsIndexedGetCommon,
    DofDampingsIndexedExternalGetCommon,
    DofDampingsFullSetCommon,
    DofDampingsSubsetSetCommon,
    DofDampingsIndexSizeSetCommon,
    DofArmaturesFullGetCommon,
    DofArmaturesExternalGetCommon,
    DofArmaturesIndexedGetCommon,
    DofArmaturesIndexedExternalGetCommon,
    DofArmaturesFullSetCommon,
    DofArmaturesSubsetSetCommon,
    DofArmaturesIndexSizeSetCommon,
    DofMaxForcesFullGetCommon,
    DofMaxForcesExternalGetCommon,
    DofMaxForcesIndexedGetCommon,
    DofMaxForcesIndexedExternalGetCommon,
    DofMaxForcesFullSetCommon,
    DofMaxForcesSubsetSetCommon,
    DofMaxForcesIndexSizeSetCommon,
    ExtensibilityCommon,
    MetadataCommon,
    ReadOnlyRejectsSetCommon,
)
_RIGID_BODY_CONTRACT_SCENARIOS = (
    VelocitiesFullGetCommon,
    VelocitiesExternalGetCommon,
    VelocitiesIndexedGetCommon,
    VelocitiesIndexedExternalGetCommon,
    VelocitiesFullSetCommon,
    VelocitiesSubsetSetCommon,
    VelocitiesIndexSizeSetCommon,
    MassesFullGetCommon,
    MassesExternalGetCommon,
    MassesIndexedGetCommon,
    MassesIndexedExternalGetCommon,
    MassesFullSetCommon,
    MassesSubsetSetCommon,
    MassesIndexSizeSetCommon,
    WriteOnlyRejectsGetCommon,
)
register_scenario_batches(
    "ovphysx",
    (
        _ARTICULATION_CONTRACT_SCENARIOS,
        _RIGID_BODY_CONTRACT_SCENARIOS,
    ),
)


class TestGetSetContract:
    # --- articulation: dof-positions (runtime) -------------------------
    """Validate OvPhysX view reads, writes, metadata, and extension hooks."""

    def test_dof_positions_full_get_ovphysx_cc(self) -> None:
        """Verify full DOF-position reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofPositionsFullGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_positions_full_get_ovphysx_gg(self) -> None:
        """Verify full DOF-position reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofPositionsFullGetCommon, "ovphysx", gpu_device())

    def test_dof_positions_external_get_ovphysx_cc(self) -> None:
        """Verify external-buffer DOF-position reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofPositionsExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_positions_external_get_ovphysx_gg(self) -> None:
        """Verify external-buffer DOF-position reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofPositionsExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_positions_indexed_get_ovphysx_cc(self) -> None:
        """Verify indexed DOF-position reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofPositionsIndexedGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_positions_indexed_get_ovphysx_gg(self) -> None:
        """Verify indexed DOF-position reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofPositionsIndexedGetCommon, "ovphysx", gpu_device())

    def test_dof_positions_indexed_external_get_ovphysx_cc(self) -> None:
        """Verify indexed external-buffer DOF-position reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofPositionsIndexedExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_positions_indexed_external_get_ovphysx_gg(self) -> None:
        """Verify indexed external-buffer DOF-position reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofPositionsIndexedExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_positions_full_set_ovphysx_cc(self) -> None:
        """Verify full DOF-position writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofPositionsFullSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_positions_full_set_ovphysx_gg(self) -> None:
        """Verify full DOF-position writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofPositionsFullSetCommon, "ovphysx", gpu_device())

    def test_dof_positions_subset_set_ovphysx_cc(self) -> None:
        """Verify subset DOF-position writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofPositionsSubsetSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_positions_subset_set_ovphysx_gg(self) -> None:
        """Verify subset DOF-position writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofPositionsSubsetSetCommon, "ovphysx", gpu_device())

    @pytest.mark.xfail(
        reason="ovphysx_write_tensor_binding rejects CPU-resident out-of-range indices; the GPU lane and the "
        "tensor backend both drop them",
        strict=True,
    )
    def test_dof_positions_out_of_range_set_ovphysx_cc(self) -> None:
        """Verify an out-of-range DOF-position write is dropped, CPU sim and tensors."""
        run_scenario(self, _PhysxDofPositionsOutOfRangeSet, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_positions_out_of_range_set_ovphysx_gg(self) -> None:
        """Verify an out-of-range DOF-position write is dropped, GPU sim and tensors."""
        run_scenario(self, _PhysxDofPositionsOutOfRangeSet, "ovphysx", gpu_device())

    def test_dof_positions_index_size_set_ovphysx_cc(self) -> None:
        """Verify compact indexed DOF-position writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofPositionsIndexSizeSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_positions_index_size_set_ovphysx_gg(self) -> None:
        """Verify compact indexed DOF-position writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofPositionsIndexSizeSetCommon, "ovphysx", gpu_device())

    # --- articulation: dof-stiffnesses (parameter) ---------------------
    def test_dof_stiffnesses_full_get_ovphysx_cc(self) -> None:
        """Verify full DOF-stiffness reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofStiffnessesFullGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_full_get_ovphysx_gg(self) -> None:
        """Verify full DOF-stiffness reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofStiffnessesFullGetCommon, "ovphysx", gpu_device())

    def test_dof_stiffnesses_external_get_ovphysx_cc(self) -> None:
        """Verify external-buffer DOF-stiffness reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofStiffnessesExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_external_get_ovphysx_gg(self) -> None:
        """Verify external-buffer DOF-stiffness reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofStiffnessesExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_stiffnesses_indexed_get_ovphysx_cc(self) -> None:
        """Verify indexed DOF-stiffness reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofStiffnessesIndexedGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_indexed_get_ovphysx_gg(self) -> None:
        """Verify indexed DOF-stiffness reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofStiffnessesIndexedGetCommon, "ovphysx", gpu_device())

    def test_dof_stiffnesses_indexed_external_get_ovphysx_cc(self) -> None:
        """Verify indexed external-buffer DOF-stiffness reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofStiffnessesIndexedExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_indexed_external_get_ovphysx_gg(self) -> None:
        """Verify indexed external-buffer DOF-stiffness reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofStiffnessesIndexedExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_stiffnesses_full_set_ovphysx_cc(self) -> None:
        """Verify full DOF-stiffness writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofStiffnessesFullSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_full_set_ovphysx_gg(self) -> None:
        """Verify full DOF-stiffness writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofStiffnessesFullSetCommon, "ovphysx", gpu_device())

    def test_dof_stiffnesses_subset_set_ovphysx_cc(self) -> None:
        """Verify subset DOF-stiffness writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofStiffnessesSubsetSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_subset_set_ovphysx_gg(self) -> None:
        """Verify subset DOF-stiffness writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofStiffnessesSubsetSetCommon, "ovphysx", gpu_device())

    def test_dof_stiffnesses_index_size_set_ovphysx_cc(self) -> None:
        """Verify compact indexed DOF-stiffness writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofStiffnessesIndexSizeSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_stiffnesses_index_size_set_ovphysx_gg(self) -> None:
        """Verify compact indexed DOF-stiffness writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofStiffnessesIndexSizeSetCommon, "ovphysx", gpu_device())

    # --- articulation: dof-dampings (parameter) -----------------------
    def test_dof_dampings_full_get_ovphysx_cc(self) -> None:
        """Verify full DOF-damping reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofDampingsFullGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_dampings_full_get_ovphysx_gg(self) -> None:
        """Verify full DOF-damping reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofDampingsFullGetCommon, "ovphysx", gpu_device())

    def test_dof_dampings_external_get_ovphysx_cc(self) -> None:
        """Verify external-buffer DOF-damping reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofDampingsExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_dampings_external_get_ovphysx_gg(self) -> None:
        """Verify external-buffer DOF-damping reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofDampingsExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_dampings_indexed_get_ovphysx_cc(self) -> None:
        """Verify indexed DOF-damping reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofDampingsIndexedGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_dampings_indexed_get_ovphysx_gg(self) -> None:
        """Verify indexed DOF-damping reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofDampingsIndexedGetCommon, "ovphysx", gpu_device())

    def test_dof_dampings_indexed_external_get_ovphysx_cc(self) -> None:
        """Verify indexed external-buffer DOF-damping reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofDampingsIndexedExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_dampings_indexed_external_get_ovphysx_gg(self) -> None:
        """Verify indexed external-buffer DOF-damping reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofDampingsIndexedExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_dampings_full_set_ovphysx_cc(self) -> None:
        """Verify full DOF-damping writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofDampingsFullSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_dampings_full_set_ovphysx_gg(self) -> None:
        """Verify full DOF-damping writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofDampingsFullSetCommon, "ovphysx", gpu_device())

    def test_dof_dampings_subset_set_ovphysx_cc(self) -> None:
        """Verify subset DOF-damping writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofDampingsSubsetSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_dampings_subset_set_ovphysx_gg(self) -> None:
        """Verify subset DOF-damping writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofDampingsSubsetSetCommon, "ovphysx", gpu_device())

    def test_dof_dampings_index_size_set_ovphysx_cc(self) -> None:
        """Verify compact indexed DOF-damping writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofDampingsIndexSizeSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_dampings_index_size_set_ovphysx_gg(self) -> None:
        """Verify compact indexed DOF-damping writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofDampingsIndexSizeSetCommon, "ovphysx", gpu_device())

    # --- articulation: dof-armatures (parameter) ----------------------
    def test_dof_armatures_full_get_ovphysx_cc(self) -> None:
        """Verify full DOF-armature reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofArmaturesFullGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_armatures_full_get_ovphysx_gg(self) -> None:
        """Verify full DOF-armature reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofArmaturesFullGetCommon, "ovphysx", gpu_device())

    def test_dof_armatures_external_get_ovphysx_cc(self) -> None:
        """Verify external-buffer DOF-armature reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofArmaturesExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_armatures_external_get_ovphysx_gg(self) -> None:
        """Verify external-buffer DOF-armature reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofArmaturesExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_armatures_indexed_get_ovphysx_cc(self) -> None:
        """Verify indexed DOF-armature reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofArmaturesIndexedGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_armatures_indexed_get_ovphysx_gg(self) -> None:
        """Verify indexed DOF-armature reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofArmaturesIndexedGetCommon, "ovphysx", gpu_device())

    def test_dof_armatures_indexed_external_get_ovphysx_cc(self) -> None:
        """Verify indexed external-buffer DOF-armature reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofArmaturesIndexedExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_armatures_indexed_external_get_ovphysx_gg(self) -> None:
        """Verify indexed external-buffer DOF-armature reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofArmaturesIndexedExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_armatures_full_set_ovphysx_cc(self) -> None:
        """Verify full DOF-armature writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofArmaturesFullSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_armatures_full_set_ovphysx_gg(self) -> None:
        """Verify full DOF-armature writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofArmaturesFullSetCommon, "ovphysx", gpu_device())

    def test_dof_armatures_subset_set_ovphysx_cc(self) -> None:
        """Verify subset DOF-armature writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofArmaturesSubsetSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_armatures_subset_set_ovphysx_gg(self) -> None:
        """Verify subset DOF-armature writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofArmaturesSubsetSetCommon, "ovphysx", gpu_device())

    def test_dof_armatures_index_size_set_ovphysx_cc(self) -> None:
        """Verify compact indexed DOF-armature writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofArmaturesIndexSizeSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_armatures_index_size_set_ovphysx_gg(self) -> None:
        """Verify compact indexed DOF-armature writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofArmaturesIndexSizeSetCommon, "ovphysx", gpu_device())

    # --- articulation: dof-max-forces (parameter) ---------------------
    def test_dof_max_forces_full_get_ovphysx_cc(self) -> None:
        """Verify full DOF maximum-force reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofMaxForcesFullGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_max_forces_full_get_ovphysx_gg(self) -> None:
        """Verify full DOF maximum-force reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofMaxForcesFullGetCommon, "ovphysx", gpu_device())

    def test_dof_max_forces_external_get_ovphysx_cc(self) -> None:
        """Verify external-buffer DOF maximum-force reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofMaxForcesExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_max_forces_external_get_ovphysx_gg(self) -> None:
        """Verify external-buffer DOF maximum-force reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofMaxForcesExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_max_forces_indexed_get_ovphysx_cc(self) -> None:
        """Verify indexed DOF maximum-force reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofMaxForcesIndexedGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_max_forces_indexed_get_ovphysx_gg(self) -> None:
        """Verify indexed DOF maximum-force reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofMaxForcesIndexedGetCommon, "ovphysx", gpu_device())

    def test_dof_max_forces_indexed_external_get_ovphysx_cc(self) -> None:
        """Verify indexed external-buffer DOF maximum-force reads with CPU simulation and CPU tensors."""
        run_scenario(self, DofMaxForcesIndexedExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_max_forces_indexed_external_get_ovphysx_gg(self) -> None:
        """Verify indexed external-buffer DOF maximum-force reads with GPU simulation and GPU tensors."""
        run_scenario(self, DofMaxForcesIndexedExternalGetCommon, "ovphysx", gpu_device())

    def test_dof_max_forces_full_set_ovphysx_cc(self) -> None:
        """Verify full DOF maximum-force writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofMaxForcesFullSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_max_forces_full_set_ovphysx_gg(self) -> None:
        """Verify full DOF maximum-force writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofMaxForcesFullSetCommon, "ovphysx", gpu_device())

    def test_dof_max_forces_subset_set_ovphysx_cc(self) -> None:
        """Verify subset DOF maximum-force writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofMaxForcesSubsetSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_max_forces_subset_set_ovphysx_gg(self) -> None:
        """Verify subset DOF maximum-force writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofMaxForcesSubsetSetCommon, "ovphysx", gpu_device())

    def test_dof_max_forces_index_size_set_ovphysx_cc(self) -> None:
        """Verify compact indexed DOF maximum-force writes with CPU simulation and CPU tensors."""
        run_scenario(self, DofMaxForcesIndexSizeSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_dof_max_forces_index_size_set_ovphysx_gg(self) -> None:
        """Verify compact indexed DOF maximum-force writes with GPU simulation and GPU tensors."""
        run_scenario(self, DofMaxForcesIndexSizeSetCommon, "ovphysx", gpu_device())

    # --- articulation: extensibility / metadata / read-only -----------
    def test_extensibility_ovphysx_cc(self) -> None:
        """Check pattern lists and a custom read implementation with CPU tensors."""
        run_scenario(self, ExtensibilityCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_extensibility_ovphysx_gg(self) -> None:
        """Check pattern lists and a custom read implementation with GPU tensors."""
        run_scenario(self, ExtensibilityCommon, "ovphysx", gpu_device())

    def test_metadata_ovphysx_cc(self) -> None:
        """Check Ant path, topology, name, and root metadata with CPU tensors."""
        run_scenario(self, MetadataCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_metadata_ovphysx_gg(self) -> None:
        """Check Ant path, topology, name, and root metadata with GPU tensors."""
        run_scenario(self, MetadataCommon, "ovphysx", gpu_device())

    def test_read_only_rejects_set_ovphysx_cc(self) -> None:
        """Read inverse masses and reject their write with CPU simulation and CPU tensors."""
        run_scenario(self, ReadOnlyRejectsSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_read_only_rejects_set_ovphysx_gg(self) -> None:
        """Read inverse masses and reject their write with GPU simulation and GPU tensors."""
        run_scenario(self, ReadOnlyRejectsSetCommon, "ovphysx", gpu_device())

    # --- rigid-body: velocities (runtime) -----------------------------
    def test_velocities_full_get_ovphysx_cc(self) -> None:
        """Verify full velocity reads with CPU simulation and CPU tensors."""
        run_scenario(self, VelocitiesFullGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_velocities_full_get_ovphysx_gg(self) -> None:
        """Verify full velocity reads with GPU simulation and GPU tensors."""
        run_scenario(self, VelocitiesFullGetCommon, "ovphysx", gpu_device())

    def test_velocities_external_get_ovphysx_cc(self) -> None:
        """Verify external-buffer velocity reads with CPU simulation and CPU tensors."""
        run_scenario(self, VelocitiesExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_velocities_external_get_ovphysx_gg(self) -> None:
        """Verify external-buffer velocity reads with GPU simulation and GPU tensors."""
        run_scenario(self, VelocitiesExternalGetCommon, "ovphysx", gpu_device())

    def test_velocities_indexed_get_ovphysx_cc(self) -> None:
        """Verify indexed velocity reads with CPU simulation and CPU tensors."""
        run_scenario(self, VelocitiesIndexedGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_velocities_indexed_get_ovphysx_gg(self) -> None:
        """Verify indexed velocity reads with GPU simulation and GPU tensors."""
        run_scenario(self, VelocitiesIndexedGetCommon, "ovphysx", gpu_device())

    def test_velocities_indexed_external_get_ovphysx_cc(self) -> None:
        """Verify indexed external-buffer velocity reads with CPU simulation and CPU tensors."""
        run_scenario(self, VelocitiesIndexedExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_velocities_indexed_external_get_ovphysx_gg(self) -> None:
        """Verify indexed external-buffer velocity reads with GPU simulation and GPU tensors."""
        run_scenario(self, VelocitiesIndexedExternalGetCommon, "ovphysx", gpu_device())

    def test_velocities_full_set_ovphysx_cc(self) -> None:
        """Verify full velocity writes with CPU simulation and CPU tensors."""
        run_scenario(self, VelocitiesFullSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_velocities_full_set_ovphysx_gg(self) -> None:
        """Verify full velocity writes with GPU simulation and GPU tensors."""
        run_scenario(self, VelocitiesFullSetCommon, "ovphysx", gpu_device())

    def test_velocities_subset_set_ovphysx_cc(self) -> None:
        """Verify subset velocity writes with CPU simulation and CPU tensors."""
        run_scenario(self, VelocitiesSubsetSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_velocities_subset_set_ovphysx_gg(self) -> None:
        """Verify subset velocity writes with GPU simulation and GPU tensors."""
        run_scenario(self, VelocitiesSubsetSetCommon, "ovphysx", gpu_device())

    def test_velocities_index_size_set_ovphysx_cc(self) -> None:
        """Verify compact indexed velocity writes with CPU simulation and CPU tensors."""
        run_scenario(self, VelocitiesIndexSizeSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_velocities_index_size_set_ovphysx_gg(self) -> None:
        """Verify compact indexed velocity writes with GPU simulation and GPU tensors."""
        run_scenario(self, VelocitiesIndexSizeSetCommon, "ovphysx", gpu_device())

    # --- rigid-body: masses (parameter) -------------------------------
    def test_masses_full_get_ovphysx_cc(self) -> None:
        """Verify full mass reads with CPU simulation and CPU tensors."""
        run_scenario(self, MassesFullGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_masses_full_get_ovphysx_gg(self) -> None:
        """Verify full mass reads with GPU simulation and GPU tensors."""
        run_scenario(self, MassesFullGetCommon, "ovphysx", gpu_device())

    def test_masses_external_get_ovphysx_cc(self) -> None:
        """Verify external-buffer mass reads with CPU simulation and CPU tensors."""
        run_scenario(self, MassesExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_masses_external_get_ovphysx_gg(self) -> None:
        """Verify external-buffer mass reads with GPU simulation and GPU tensors."""
        run_scenario(self, MassesExternalGetCommon, "ovphysx", gpu_device())

    def test_masses_indexed_get_ovphysx_cc(self) -> None:
        """Verify indexed mass reads with CPU simulation and CPU tensors."""
        run_scenario(self, MassesIndexedGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_masses_indexed_get_ovphysx_gg(self) -> None:
        """Verify indexed mass reads with GPU simulation and GPU tensors."""
        run_scenario(self, MassesIndexedGetCommon, "ovphysx", gpu_device())

    def test_masses_indexed_external_get_ovphysx_cc(self) -> None:
        """Verify indexed external-buffer mass reads with CPU simulation and CPU tensors."""
        run_scenario(self, MassesIndexedExternalGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_masses_indexed_external_get_ovphysx_gg(self) -> None:
        """Verify indexed external-buffer mass reads with GPU simulation and GPU tensors."""
        run_scenario(self, MassesIndexedExternalGetCommon, "ovphysx", gpu_device())

    def test_masses_full_set_ovphysx_cc(self) -> None:
        """Verify full mass writes with CPU simulation and CPU tensors."""
        run_scenario(self, MassesFullSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_masses_full_set_ovphysx_gg(self) -> None:
        """Verify full mass writes with GPU simulation and GPU tensors."""
        run_scenario(self, MassesFullSetCommon, "ovphysx", gpu_device())

    def test_masses_subset_set_ovphysx_cc(self) -> None:
        """Verify subset mass writes with CPU simulation and CPU tensors."""
        run_scenario(self, MassesSubsetSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_masses_subset_set_ovphysx_gg(self) -> None:
        """Verify subset mass writes with GPU simulation and GPU tensors."""
        run_scenario(self, MassesSubsetSetCommon, "ovphysx", gpu_device())

    def test_masses_index_size_set_ovphysx_cc(self) -> None:
        """Verify compact indexed mass writes with CPU simulation and CPU tensors."""
        run_scenario(self, MassesIndexSizeSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_masses_index_size_set_ovphysx_gg(self) -> None:
        """Verify compact indexed mass writes with GPU simulation and GPU tensors."""
        run_scenario(self, MassesIndexSizeSetCommon, "ovphysx", gpu_device())

    # --- rigid-body: multi-buffer set + write-only --------------------
    @pytest.mark.skip(reason=_GAP_MULTI_SET)
    def test_apply_multi_set_ovphysx_cc(self) -> None:
        """Apply force buffers to every body with CPU simulation and CPU tensors."""
        run_scenario(self, ApplyMultiSetCommon, "ovphysx", cpu_device())

    @pytest.mark.skip(reason=_GAP_MULTI_SET)
    @gpu_only
    def test_apply_multi_set_ovphysx_gg(self) -> None:
        """Apply force buffers to every body with GPU simulation and GPU tensors."""
        run_scenario(self, ApplyMultiSetCommon, "ovphysx", gpu_device())

    @pytest.mark.skip(reason=_GAP_MULTI_SET)
    def test_apply_indexed_multi_set_ovphysx_cc(self) -> None:
        """Apply force buffers only to selected bodies with CPU simulation and CPU tensors."""
        run_scenario(self, ApplyIndexedMultiSetCommon, "ovphysx", cpu_device())

    @pytest.mark.skip(reason=_GAP_MULTI_SET)
    @gpu_only
    def test_apply_indexed_multi_set_ovphysx_gg(self) -> None:
        """Apply force buffers only to selected bodies with GPU simulation and GPU tensors."""
        run_scenario(self, ApplyIndexedMultiSetCommon, "ovphysx", gpu_device())

    def test_write_only_rejects_get_ovphysx_cc(self) -> None:
        """Write force buffers and reject their read with CPU simulation and CPU tensors."""
        run_scenario(self, WriteOnlyRejectsGetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_write_only_rejects_get_ovphysx_gg(self) -> None:
        """Write force buffers and reject their read with GPU simulation and GPU tensors."""
        run_scenario(self, WriteOnlyRejectsGetCommon, "ovphysx", gpu_device())

    # --- rigid-contact: multi-buffer get ------------------------------
    def test_raw_contact_multi_get_ovphysx_cc(self) -> None:
        """Check full raw-contact buffers and empty-view metadata with CPU tensors."""
        run_scenario(self, PhysxRawContactMultiGet, "ovphysx", cpu_device())

    @gpu_only
    def test_raw_contact_multi_get_ovphysx_gg(self) -> None:
        """Check full raw-contact buffers and empty-view metadata with GPU tensors."""
        run_scenario(self, PhysxRawContactMultiGet, "ovphysx", gpu_device())

    @pytest.mark.skip(reason=_GAP_INDEXED_MULTI_GET)
    def test_raw_contact_indexed_multi_get_ovphysx_cc(self) -> None:
        """Read selected raw-contact sensors into allocated CPU tensor buffers."""
        run_scenario(self, PhysxRawContactIndexedMultiGet, "ovphysx", cpu_device())

    @pytest.mark.skip(reason=_GAP_INDEXED_MULTI_GET)
    @gpu_only
    def test_raw_contact_indexed_multi_get_ovphysx_gg(self) -> None:
        """Read selected raw-contact sensors into allocated GPU tensor buffers."""
        run_scenario(self, PhysxRawContactIndexedMultiGet, "ovphysx", gpu_device())

    def test_raw_contact_external_multi_get_ovphysx_cc(self) -> None:
        """Read all raw-contact data into caller-provided CPU tensor buffers."""
        run_scenario(self, PhysxRawContactExternalMultiGet, "ovphysx", cpu_device())

    @gpu_only
    def test_raw_contact_external_multi_get_ovphysx_gg(self) -> None:
        """Read all raw-contact data into caller-provided GPU tensor buffers."""
        run_scenario(self, PhysxRawContactExternalMultiGet, "ovphysx", gpu_device())

    @pytest.mark.skip(reason=_GAP_INDEXED_MULTI_GET)
    def test_raw_contact_indexed_external_multi_get_ovphysx_cc(self) -> None:
        """Read selected contact sensors into caller-provided CPU tensor buffers."""
        run_scenario(self, PhysxRawContactIndexedExternalMultiGet, "ovphysx", cpu_device())

    @pytest.mark.skip(reason=_GAP_INDEXED_MULTI_GET)
    @gpu_only
    def test_raw_contact_indexed_external_multi_get_ovphysx_gg(self) -> None:
        """Read selected contact sensors into caller-provided GPU tensor buffers."""
        run_scenario(self, PhysxRawContactIndexedExternalMultiGet, "ovphysx", gpu_device())

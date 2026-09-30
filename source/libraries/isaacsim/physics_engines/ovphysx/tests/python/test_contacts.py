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

"""Validate contact-tensor behavior with the OvPhysX engine.

These tests extend the engine-neutral contact scenarios with the PhysX schemas
required to emit contact data. Each scenario body receives
``PhysxRigidBodyAPI`` with sleeping disabled and ``PhysxContactReportAPI`` with
a zero reporting threshold.
"""

from __future__ import annotations

import gc
import os
import sys
from typing import Any
from unittest.mock import patch

import _physics_setup  # noqa: F401
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
from common.contacts import (  # noqa: E402
    ArticulationContactsCommon,
    ArticulationContactsFullCommon,
    ContactCapacityResizeCommon,
    ContactDataCommon,
    RawContactDataCommon,
    RigidContactMatrixCommon,
    RigidContactsCommon,
)
from physx_usd_schemas import PhysxSchema  # noqa: E402

# ---------------------------------------------------------------------------
# ovphysx engine-specific extension.
# ---------------------------------------------------------------------------


class _PhysxContactReportMixin:
    """Configure PhysX contact reporting for each scenario body.

    The engine-neutral scenarios provide template paths in
    ``self._contact_body_paths``. This mixin resolves those paths in every
    replicated environment, disables sleeping, and enables contact reports
    with a zero threshold.
    """

    def _apply_engine_specifics(self) -> None:
        super()._apply_engine_specifics()
        for tmpl_path in self._contact_body_paths:
            # Strip the env_template prefix and re-attach under each
            # env_<i> root.
            tmpl_str = str(tmpl_path)
            tmpl_prefix = str(self.env_template_path)
            if tmpl_str.startswith(tmpl_prefix):
                rel = tmpl_str[len(tmpl_prefix) :].lstrip("/")
            else:
                rel = tmpl_path.name
            for i in range(self.num_envs):
                prim = self.stage.GetPrimAtPath(f"/envs/env{i}/{rel}")
                if not prim:
                    continue
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                rb.CreateSleepThresholdAttr().Set(0)
                cs = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                cs.CreateThresholdAttr().Set(0)


class _RigidContactsScenario(_PhysxContactReportMixin, RigidContactsCommon):
    pass


class _RigidContactMatrixScenario(_PhysxContactReportMixin, RigidContactMatrixCommon):
    pass


class _ArticulationContactsScenario(_PhysxContactReportMixin, ArticulationContactsCommon):
    pass


class _RawContactDataScenario(_PhysxContactReportMixin, RawContactDataCommon):
    pass


class _ContactCapacityResizeScenario(_PhysxContactReportMixin, ContactCapacityResizeCommon):
    pass


class _ContactDataScenario(_PhysxContactReportMixin, ContactDataCommon):
    pass


class _ArticulationContactsFullScenario(_PhysxContactReportMixin, ArticulationContactsFullCommon):
    pass


# ---------------------------------------------------------------------------
# Test classes.
# ---------------------------------------------------------------------------


class TestRigidContacts:
    """Verify resting boxes report a net ground reaction equal to their weight."""

    def test_rigid_contacts_ovphysx_cc(self) -> None:
        """Check resting-box contact forces with CPU simulation and CPU tensors."""
        run_scenario(self, _RigidContactsScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_contacts_ovphysx_gc(self) -> None:
        """Check resting-box contacts with GPU simulation and CPU tensors."""
        from _scenario import DeviceParams

        run_scenario(self, _RigidContactsScenario, "ovphysx", DeviceParams(True, False))

    @gpu_only
    def test_rigid_contacts_ovphysx_gg(self) -> None:
        """Check resting-box contact forces with GPU simulation and GPU tensors."""
        run_scenario(self, _RigidContactsScenario, "ovphysx", gpu_device())


class TestRigidContactMatrix:
    """Validate filtered contact-force matrices for hovering boxes."""

    def test_rigid_contact_matrix_ovphysx_cc(self) -> None:
        """Check filtered contact matrices with CPU simulation and CPU tensors."""
        run_scenario(self, _RigidContactMatrixScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_contact_matrix_ovphysx_gg(self) -> None:
        """Check filtered contact matrices with GPU simulation and GPU tensors."""
        run_scenario(self, _RigidContactMatrixScenario, "ovphysx", gpu_device())


class TestArticulationContacts:
    """Validate net contact forces for boxes resting on the ground.

    The scenario verifies that every box sensor reports a steady-state normal
    force equal to the box weight.
    """

    def test_articulation_contacts_ovphysx_cc(self) -> None:
        """Check companion-box ground reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _ArticulationContactsScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_contacts_ovphysx_gg(self) -> None:
        """Check companion-box ground reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _ArticulationContactsScenario, "ovphysx", gpu_device())


class TestRawContactData:
    """Validate the seven-buffer raw-contact read with overlapping boxes.

    An absent ``raw-contact-data`` registration remains a visible failure.
    """

    def test_raw_contact_data_ovphysx_cc(self) -> None:
        """Check overlapping-box raw records with CPU simulation and CPU tensors."""
        run_scenario(self, _RawContactDataScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_raw_contact_data_ovphysx_gc(self) -> None:
        """Check overlapping-box raw records with GPU simulation and CPU tensors."""
        from _scenario import DeviceParams

        run_scenario(self, _RawContactDataScenario, "ovphysx", DeviceParams(True, False))

    @gpu_only
    def test_raw_contact_data_ovphysx_gg(self) -> None:
        """Check overlapping-box raw records with GPU simulation and GPU tensors."""
        run_scenario(self, _RawContactDataScenario, "ovphysx", gpu_device())


class TestContactData:
    """Validate filtered multi-buffer contact and friction reads.

    The scenario checks tensor counts, shapes, finite values, populated record
    ranges, and execution-device placement.
    """

    def test_contact_data_ovphysx_cc(self) -> None:
        """Check filtered contact buffers with CPU simulation and CPU tensors."""
        run_scenario(self, _ContactDataScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_contact_data_ovphysx_gg(self) -> None:
        """Check filtered contact buffers with GPU simulation and GPU tensors."""
        run_scenario(self, _ContactDataScenario, "ovphysx", gpu_device())


class TestArticulationContactsFull:
    """Validate metadata for contact sensors on articulation links.

    The scenario checks the exact sensor path, ordering, and count for each
    replicated articulation torso.
    """

    def test_articulation_contacts_full_with_link_indices_ovphysx_cc(self) -> None:
        """Verify articulation-link sensor metadata with CPU simulation and CPU tensors."""
        run_scenario(self, _ArticulationContactsFullScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_contacts_full_with_link_indices_ovphysx_gg(self) -> None:
        """Verify articulation-link sensor metadata with GPU simulation and GPU tensors."""
        run_scenario(self, _ArticulationContactsFullScenario, "ovphysx", gpu_device())


class _ResultLifetimeScenario(_RigidContactsScenario):
    """Validate tensor-result ownership after the producing view is collected.

    Both single-buffer and multi-buffer reads must retain valid backing storage
    while their source views are deleted and the allocator is placed under
    pressure.
    """

    def _hammer(self) -> list[wp.array]:
        # Reuse freed storage so a dangling result reads clobbered data.
        return [wp.zeros((self.num_envs, 8), dtype=wp.float32, device=self.wp_device) for _ in range(256)]

    def on_physics_step(self, sim: Any, stepno: int, dt: float) -> None:
        if stepno < 2:
            return

        # get_data internal result: no `out`, so it aliases a buffer owned by
        # the view's impl closure -> the returned array must retain the view.
        rbv = sim.create_rigid_body_view("/envs/*/box")
        masses = rbv.get_data("masses")
        snap = masses.numpy().copy()
        assert (snap != 0).any(), "masses unexpectedly all-zero"
        del rbv
        gc.collect()
        _pressure = self._hammer()
        assert (masses.numpy() == snap).all(), "get_data internal result dangled after its view was dropped + GC'd"

        # get_data_multi result: on the DirectGPU lane each returned array owns a
        # framework-allocated device buffer (the C++ binding pins it to the matching
        # supplied `out`); on the CPU-staged lane it aliases a view-owned buffer.
        # Either way the result must outlive the view. Repeat the read/drop/GC/hammer
        # cycle so repeated get_raw_contact_data() calls are exercised, not just one.
        for _ in range(8):
            # Capacity has to cover the step's real contact count: an undersized view
            # is an error, not a truncation, and this scenario is about result
            # ownership rather than the overflow contract.
            cv = sim.create_rigid_contact_view("/envs/*/box", max_contact_data_count=self.num_envs * 10)
            multi = cv.get_data_multi("raw-contact-data")
            msnap = [m.numpy().copy() for m in multi]
            del cv
            gc.collect()
            _pressure2 = self._hammer()
            for i, (m, ms) in enumerate(zip(multi, msnap)):
                assert (m.numpy() == ms).all(), f"get_data_multi result[{i}] dangled after its view was dropped + GC'd"
        self.finish()


class _MultiGetAllocFailureScenario(_RigidContactsScenario):
    """Validate error propagation from a GPU multi-get allocation failure.

    A failure while allocating an output tensor must reach the caller instead
    of triggering host staging or producing a partial output list.
    """

    def on_physics_step(self, sim: Any, stepno: int, dt: float) -> None:
        if stepno < 2:
            return
        cv = sim.create_rigid_contact_view("/envs/*/box", max_contact_data_count=self.num_envs * 4)
        # The DirectGPU alloc path routes every output through create_tensor; a real
        # OOM there is an error, not a reason to fall back to the CPU-staged read.
        with patch(
            "isaacsim.physics.manager.impl.tensors.frontends.create_tensor", side_effect=RuntimeError("simulated OOM")
        ):
            with pytest.raises(RuntimeError, match="simulated OOM"):
                cv.get_data_multi("raw-contact-data")
        self.finish()


class TestResultLifetime:
    """Verify that tensor results outlive the entity views that produced them.

    The scenarios exercise both ``get_data`` and ``get_data_multi`` results
    under forced garbage collection and allocator pressure.
    """

    def test_result_lifetime_ovphysx_cc(self) -> None:
        """Check result ownership after view collection with CPU simulation and CPU tensors."""
        run_scenario(self, _ResultLifetimeScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_result_lifetime_ovphysx_gg(self) -> None:
        # DirectGPU lane: get_data_multi reads into framework-allocated device
        # buffers, so the returned arrays own device storage (C++ pins each to the
        # matching supplied `out`). Confirms that owner chain survives dropping +
        # GC'ing the view under memory pressure -- a UAF here would read clobbered
        # device memory, not fail cleanly.
        """Check result ownership after view collection with GPU simulation and GPU tensors."""
        run_scenario(self, _ResultLifetimeScenario, "ovphysx", gpu_device())


class TestMultiGetAllocFailure:
    """Verify that GPU multi-get output allocation failures propagate.

    The scenario replaces output allocation with a deterministic failure and
    requires that failure to reach the caller.
    """

    @gpu_only
    def test_multi_get_alloc_failure_ovphysx_gg(self) -> None:
        """Check allocation-error propagation with GPU simulation and GPU tensors."""
        run_scenario(self, _MultiGetAllocFailureScenario, "ovphysx", gpu_device())


class TestContactCapacityResize:
    """Validate that a contact view adopts the extent the caller's buffers declare."""

    def test_contact_capacity_resize_ovphysx_cc(self) -> None:
        """Check grow and shrink with CPU simulation and CPU tensors."""
        run_scenario(self, _ContactCapacityResizeScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_contact_capacity_resize_ovphysx_gg(self) -> None:
        """Check grow and shrink with GPU simulation and GPU tensors."""
        run_scenario(self, _ContactCapacityResizeScenario, "ovphysx", gpu_device())

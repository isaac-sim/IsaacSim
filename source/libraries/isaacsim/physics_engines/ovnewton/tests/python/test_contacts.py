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

"""Validate contact-tensor behavior with the Newton engine.

The engine-neutral scenarios run directly because Newton controls its contact
stream without the PhysX contact-report schemas used by OvPhysX.
"""

from __future__ import annotations

import os
import sys

import _physics_setup  # noqa: F401

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
    RawContactDataCommon,
    RawContactSignConventionCommon,
    RigidContactMatrixCommon,
    RigidContactsCommon,
)


class TestRigidContacts:
    """Validate rigid contacts with Newton."""

    def test_rigid_contacts_newton_cc(self) -> None:
        """Verify rigid contacts on the Newton CPU pipeline."""
        run_scenario(self, RigidContactsCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_contacts_newton_gc(self) -> None:
        """Verify rigid contacts with Newton GPU simulation and CPU tensors."""
        from _scenario import DeviceParams

        run_scenario(self, RigidContactsCommon, "newton", DeviceParams(True, False))

    @gpu_only
    def test_rigid_contacts_newton_gg(self) -> None:
        """Verify rigid contacts on the Newton GPU pipeline."""
        run_scenario(self, RigidContactsCommon, "newton", gpu_device())


class TestRigidContactMatrix:
    """Validate rigid contact matrix with Newton."""

    def test_rigid_contact_matrix_newton_cc(self) -> None:
        """Verify rigid contact matrix on the Newton CPU pipeline."""
        run_scenario(self, RigidContactMatrixCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_contact_matrix_newton_gg(self) -> None:
        """Verify rigid contact matrix on the Newton GPU pipeline."""
        run_scenario(self, RigidContactMatrixCommon, "newton", gpu_device())


class TestArticulationContacts:
    """Validate net contact forces for boxes resting on the ground with Newton."""

    def test_articulation_contacts_newton_cc(self) -> None:
        """Verify articulation contacts on the Newton CPU pipeline."""
        run_scenario(self, ArticulationContactsCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_contacts_newton_gg(self) -> None:
        """Verify articulation contacts on the Newton GPU pipeline."""
        run_scenario(self, ArticulationContactsCommon, "newton", gpu_device())


class TestRawContactData:
    """Validate the seven-buffer raw-contact read with overlapping boxes."""

    def test_raw_contact_data_newton_cc(self) -> None:
        """Verify raw contact data on the Newton CPU pipeline."""
        run_scenario(self, RawContactDataCommon, "newton", cpu_device())

    @gpu_only
    def test_raw_contact_data_newton_gg(self) -> None:
        """Verify raw contact data on the Newton GPU pipeline."""
        run_scenario(self, RawContactDataCommon, "newton", gpu_device())


class TestRawContactSignConvention:
    """Validate opposite raw-contact signs for the two participating sensors."""

    def test_raw_contact_sign_convention_newton_cc(self) -> None:
        """Verify raw contact sign convention on the Newton CPU pipeline."""
        run_scenario(self, RawContactSignConventionCommon, "newton", cpu_device())

    @gpu_only
    def test_raw_contact_sign_convention_newton_gg(self) -> None:
        """Verify raw contact sign convention on the Newton GPU pipeline."""
        run_scenario(self, RawContactSignConventionCommon, "newton", gpu_device())


class TestArticulationContactsFull:
    """Validate articulation-link contact sensor metadata with Newton."""

    def test_articulation_contacts_full_with_link_indices_newton_cc(self) -> None:
        """Verify articulation-link sensor metadata with the Newton CPU pipeline."""
        run_scenario(self, ArticulationContactsFullCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_contacts_full_with_link_indices_newton_gg(self) -> None:
        """Verify articulation-link sensor metadata with the Newton GPU pipeline."""
        run_scenario(self, ArticulationContactsFullCommon, "newton", gpu_device())

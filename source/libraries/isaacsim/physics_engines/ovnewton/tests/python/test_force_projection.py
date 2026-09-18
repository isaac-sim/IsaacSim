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

"""Validate joint-force projection with the Newton engine.

Single-link cases that read projected joint forces are skipped because Newton
does not provide joint-force reporting. The two-link, torsional, and
spherical-labeled scenarios run on each available device.
"""

from __future__ import annotations

import os
import sys

import _physics_setup  # noqa: F401
import pytest

_TENSORS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TENSORS_DIR not in sys.path:
    sys.path.append(_TENSORS_DIR)

from _legacy_runner import (  # noqa: E402
    cpu_device,
    gpu_device,
    gpu_only,
    run_scenario,
)
from common.force_projection import (  # noqa: E402
    JFP_Spherical_Y_Common,
    JFP_Torsional_Y_Common,
    JFP_TwoLinks_Y_DofContactCommon,
    JFP_TwoLinks_Y_ZeroContactCommon,
    JFP_TwoLinks_Y_ZeroDofCommon,
    JFP_X_ZeroContactCommon,
    JFP_X_ZeroDofCommon,
    JFP_Y_DofContactCommon,
    JFP_Y_ZeroContactCommon,
    JFP_Y_ZeroDofCommon,
    JFP_Z_ZeroContactCommon,
    JFP_Z_ZeroDofCommon,
)

_GAP_JOINT_FORCE_REPORTING = (
    "Newton joint-force reporting (link-incoming-joint-force / "
    "dof-projected-joint-forces) is deferred pending a design decision: computing them "
    "requires requesting State.body_parent_f at model-build time, an always-on per-step "
    "cost on every solver (a full RNE post-constraint pass under MuJoCo) whether or not "
    "the tensor is read. Unconditional vs. opt-in gating is undecided; the ops stay "
    "unregistered until then."
)


@pytest.mark.skip(reason=_GAP_JOINT_FORCE_REPORTING)
class TestForceProjectionSingleLinkYZeroContact:
    """Skip the Y-axis zero-contact case because Newton does not report joint forces."""

    def test_force_projection_single_link_y_zero_contact_newton_cc(self) -> None:
        """Verify zero-contact force projection for a single Y-axis link on the Newton CPU pipeline."""
        run_scenario(self, JFP_Y_ZeroContactCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_single_link_y_zero_contact_newton_gg(self) -> None:
        """Verify zero-contact force projection for a single Y-axis link on the Newton GPU pipeline."""
        run_scenario(self, JFP_Y_ZeroContactCommon, "newton", gpu_device())


@pytest.mark.skip(reason=_GAP_JOINT_FORCE_REPORTING)
class TestForceProjectionSingleLinkXZeroContact:
    """Skip the X-axis zero-contact case because Newton does not report joint forces."""

    def test_force_projection_single_link_x_zero_contact_newton_cc(self) -> None:
        """Verify zero-contact force projection for a single X-axis link on the Newton CPU pipeline."""
        run_scenario(self, JFP_X_ZeroContactCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_single_link_x_zero_contact_newton_gg(self) -> None:
        """Verify zero-contact force projection for a single X-axis link on the Newton GPU pipeline."""
        run_scenario(self, JFP_X_ZeroContactCommon, "newton", gpu_device())


@pytest.mark.skip(reason=_GAP_JOINT_FORCE_REPORTING)
class TestForceProjectionSingleLinkZZeroContact:
    """Skip the Z-axis zero-contact case because Newton does not report joint forces."""

    def test_force_projection_single_link_z_zero_contact_newton_cc(self) -> None:
        """Verify zero-contact force projection for a single Z-axis link on the Newton CPU pipeline."""
        run_scenario(self, JFP_Z_ZeroContactCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_single_link_z_zero_contact_newton_gg(self) -> None:
        """Verify zero-contact force projection for a single Z-axis link on the Newton GPU pipeline."""
        run_scenario(self, JFP_Z_ZeroContactCommon, "newton", gpu_device())


@pytest.mark.skip(reason=_GAP_JOINT_FORCE_REPORTING)
class TestForceProjectionSingleLinkYZeroDof:
    """Skip the Y-axis zero-motor case because Newton does not report joint forces."""

    def test_force_projection_single_link_y_zero_dof_newton_cc(self) -> None:
        """Verify zero-motor force projection for a single Y-axis link on the Newton CPU pipeline."""
        run_scenario(self, JFP_Y_ZeroDofCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_single_link_y_zero_dof_newton_gg(self) -> None:
        """Verify zero-motor force projection for a single Y-axis link on the Newton GPU pipeline."""
        run_scenario(self, JFP_Y_ZeroDofCommon, "newton", gpu_device())


@pytest.mark.skip(reason=_GAP_JOINT_FORCE_REPORTING)
class TestForceProjectionSingleLinkXZeroDof:
    """Skip the X-axis zero-motor case because Newton does not report joint forces."""

    def test_force_projection_single_link_x_zero_dof_newton_cc(self) -> None:
        """Verify zero-motor force projection for a single X-axis link on the Newton CPU pipeline."""
        run_scenario(self, JFP_X_ZeroDofCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_single_link_x_zero_dof_newton_gg(self) -> None:
        """Verify zero-motor force projection for a single X-axis link on the Newton GPU pipeline."""
        run_scenario(self, JFP_X_ZeroDofCommon, "newton", gpu_device())


@pytest.mark.skip(reason=_GAP_JOINT_FORCE_REPORTING)
class TestForceProjectionSingleLinkZZeroDof:
    """Skip the Z-axis zero-motor case because Newton does not report joint forces."""

    def test_force_projection_single_link_z_zero_dof_newton_cc(self) -> None:
        """Verify zero-motor force projection for a single Z-axis link on the Newton CPU pipeline."""
        run_scenario(self, JFP_Z_ZeroDofCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_single_link_z_zero_dof_newton_gg(self) -> None:
        """Verify zero-motor force projection for a single Z-axis link on the Newton GPU pipeline."""
        run_scenario(self, JFP_Z_ZeroDofCommon, "newton", gpu_device())


@pytest.mark.skip(reason=_GAP_JOINT_FORCE_REPORTING)
class TestForceProjectionSingleLinkYDofContact:
    """Skip the Y-axis DOF-contact case because Newton does not report joint forces."""

    def test_force_projection_single_link_y_dof_contact_newton_cc(self) -> None:
        """Verify contact-force projection for a single Y-axis DOF on the Newton CPU pipeline."""
        run_scenario(self, JFP_Y_DofContactCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_single_link_y_dof_contact_newton_gg(self) -> None:
        """Verify contact-force projection for a single Y-axis DOF on the Newton GPU pipeline."""
        run_scenario(self, JFP_Y_DofContactCommon, "newton", gpu_device())


class TestForceProjectionTwoLinksYZeroContact:
    """Validate negative unit motor projection and a nonzero ground reaction."""

    def test_force_projection_two_links_y_zero_contact_newton_cc(self) -> None:
        """Check negative-motor projection on the Newton CPU pipeline."""
        run_scenario(self, JFP_TwoLinks_Y_ZeroContactCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_two_links_y_zero_contact_newton_gg(self) -> None:
        """Check negative-motor projection on the Newton GPU pipeline."""
        run_scenario(self, JFP_TwoLinks_Y_ZeroContactCommon, "newton", gpu_device())


class TestForceProjectionTwoLinksYZeroDof:
    """Validate a two-link Y-axis pendulum with zero motor torque."""

    def test_force_projection_two_links_y_zero_dof_newton_cc(self) -> None:
        """Check zero-motor projection on the Newton CPU pipeline."""
        run_scenario(self, JFP_TwoLinks_Y_ZeroDofCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_two_links_y_zero_dof_newton_gg(self) -> None:
        """Check zero-motor projection on the Newton GPU pipeline."""
        run_scenario(self, JFP_TwoLinks_Y_ZeroDofCommon, "newton", gpu_device())


class TestForceProjectionTwoLinksYDofContact:
    """Validate contact-force projection for two Y-axis links with Newton."""

    def test_force_projection_two_links_y_dof_contact_newton_cc(self) -> None:
        """Verify contact-force projection for two Y-axis links on the Newton CPU pipeline."""
        run_scenario(self, JFP_TwoLinks_Y_DofContactCommon, "newton", cpu_device())

    @gpu_only
    def test_force_projection_two_links_y_dof_contact_newton_gg(self) -> None:
        """Verify contact-force projection for two Y-axis links on the Newton GPU pipeline."""
        run_scenario(self, JFP_TwoLinks_Y_DofContactCommon, "newton", gpu_device())


class TestForceProjectionTorsionalY:
    """Validate torsional Y-axis force projection with Newton."""

    def test_force_projection_torsional_y_newton_cc(self) -> None:
        """Verify torsional Y-axis force projection on the Newton CPU pipeline."""
        run_scenario(self, JFP_Torsional_Y_Common, "newton", cpu_device())

    @gpu_only
    def test_force_projection_torsional_y_newton_gg(self) -> None:
        """Verify torsional Y-axis force projection on the Newton GPU pipeline."""
        run_scenario(self, JFP_Torsional_Y_Common, "newton", gpu_device())


class TestForceProjectionSphericalY:
    """Validate force projection with the Y and Z rotation axes free."""

    def test_force_projection_spherical_y_newton_cc(self) -> None:
        """Check the spherical-labeled configuration on the Newton CPU pipeline."""
        run_scenario(self, JFP_Spherical_Y_Common, "newton", cpu_device())

    @gpu_only
    def test_force_projection_spherical_y_newton_gg(self) -> None:
        """Check the spherical-labeled configuration on the Newton GPU pipeline."""
        run_scenario(self, JFP_Spherical_Y_Common, "newton", gpu_device())

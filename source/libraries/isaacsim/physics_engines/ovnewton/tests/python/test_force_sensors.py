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

"""Validate force-sensor tensors with the Newton engine."""

from __future__ import annotations

import os
import sys
from typing import Any

import _physics_setup  # noqa: F401
from isaacsim.physics_engines.ovnewton import SolverType

_TENSORS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TENSORS_DIR not in sys.path:
    sys.path.append(_TENSORS_DIR)

from _legacy_runner import (  # noqa: E402
    cpu_device,
    gpu_device,
    gpu_only,
    run_scenario,
)
from common.force_sensors import (  # noqa: E402
    ForceTorqueSensorXCommon,
    ForceTorqueSensorYCommon,
    ForceTorqueSensorZCommon,
    JointFrictionCommon,
    LinkIncomingJointForceChildXCommon,
    LinkIncomingJointForceChildYCommon,
    LinkIncomingJointForceChildZCommon,
    LinkIncomingJointForceFixedXCommon,
    LinkIncomingJointForceFixedYCommon,
    LinkIncomingJointForceFixedZCommon,
)


class _NewtonJointForceOutputMixin:
    """Enable optional Newton joint wrench computation."""

    def configure_newton(self, config: Any) -> None:
        """Request parent-joint wrench storage before Newton creates its states."""
        config.solver_outputs.link_incoming_joint_force = True
        config.solver.solver_type = SolverType.FEATHERSTONE


class _NewtonLinkIncomingJointForceMixin(_NewtonJointForceOutputMixin):
    """Adapt analytical incoming-link-force scenarios to Newton reporting."""

    def on_start(self, sim: Any) -> None:
        """Create tensor views and apply the force before the first Featherstone step."""
        super().on_start(sim)
        self._apply_ext_force()

    def on_physics_step(self, sim: Any, stepno: int, dt: float) -> None:
        """Validate the first Featherstone step, which directly reports inverse-dynamics reactions."""
        if stepno == 0:
            self._validate_link_incoming_joint_forces()
            self.finish()


class _ForceTorqueSensorY(_NewtonJointForceOutputMixin, ForceTorqueSensorYCommon):
    pass


class _ForceTorqueSensorX(_NewtonJointForceOutputMixin, ForceTorqueSensorXCommon):
    pass


class _ForceTorqueSensorZ(_NewtonJointForceOutputMixin, ForceTorqueSensorZCommon):
    pass


class _LinkIncomingJointActuationForce(_NewtonJointForceOutputMixin, ForceTorqueSensorYCommon):
    """Validate the active wrench component away from equilibrium."""

    def __init__(self, test_case: object, device_params: Any) -> None:
        super().__init__(test_case, device_params)
        self.motor_torque *= 0.5

    def on_physics_step(self, sim: Any, stepno: int, dt: float) -> None:
        """Compare the active incoming-link torque with the applied actuation."""
        if stepno >= 1:
            joint_forces = self.pendulums.get_data("link-incoming-joint-force").numpy().reshape(self.num_envs, 3, 6)
            assert (
                abs(joint_forces[:, 1, 4] - self.motor_torque) <= 0.05
            ).all(), "active incoming-link torque matches applied actuation"
            self.finish()


class TestForceTorqueSensorY:
    """Validate Y-axis force/torque sensing."""

    def test_force_torque_sensor_y_newton_cc(self) -> None:
        """Verify Y-axis force/torque sensing on the Newton CPU pipeline."""
        run_scenario(self, _ForceTorqueSensorY, "newton", cpu_device())

    @gpu_only
    def test_force_torque_sensor_y_newton_gg(self) -> None:
        """Verify Y-axis force/torque sensing on the Newton GPU pipeline."""
        run_scenario(self, _ForceTorqueSensorY, "newton", gpu_device())


class TestForceTorqueSensorX:
    """Validate X-axis force/torque sensing."""

    def test_force_torque_sensor_x_newton_cc(self) -> None:
        """Verify X-axis force/torque sensing on the Newton CPU pipeline."""
        run_scenario(self, _ForceTorqueSensorX, "newton", cpu_device())

    @gpu_only
    def test_force_torque_sensor_x_newton_gg(self) -> None:
        """Verify X-axis force/torque sensing on the Newton GPU pipeline."""
        run_scenario(self, _ForceTorqueSensorX, "newton", gpu_device())


class TestForceTorqueSensorZ:
    """Validate Z-axis force/torque sensing."""

    def test_force_torque_sensor_z_newton_cc(self) -> None:
        """Verify Z-axis force/torque sensing on the Newton CPU pipeline."""
        run_scenario(self, _ForceTorqueSensorZ, "newton", cpu_device())

    @gpu_only
    def test_force_torque_sensor_z_newton_gg(self) -> None:
        """Verify Z-axis force/torque sensing on the Newton GPU pipeline."""
        run_scenario(self, _ForceTorqueSensorZ, "newton", gpu_device())


class TestLinkIncomingJointActuationForce:
    """Validate incoming-link actuation reporting away from equilibrium."""

    def test_link_incoming_joint_actuation_force_newton_cc(self) -> None:
        """Verify active link torque with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointActuationForce, "newton", cpu_device())

    @gpu_only
    def test_link_incoming_joint_actuation_force_newton_gg(self) -> None:
        """Verify active link torque with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointActuationForce, "newton", gpu_device())


class _LinkIncomingJointForceChildY(_NewtonLinkIncomingJointForceMixin, LinkIncomingJointForceChildYCommon):
    pass


class _LinkIncomingJointForceFixedY(_NewtonLinkIncomingJointForceMixin, LinkIncomingJointForceFixedYCommon):
    pass


class _LinkIncomingJointForceChildX(_NewtonLinkIncomingJointForceMixin, LinkIncomingJointForceChildXCommon):
    pass


class _LinkIncomingJointForceFixedX(_NewtonLinkIncomingJointForceMixin, LinkIncomingJointForceFixedXCommon):
    pass


class _LinkIncomingJointForceChildZ(_NewtonLinkIncomingJointForceMixin, LinkIncomingJointForceChildZCommon):
    pass


class _LinkIncomingJointForceFixedZ(_NewtonLinkIncomingJointForceMixin, LinkIncomingJointForceFixedZCommon):
    pass


class TestLinkIncomingJointForceChildY:
    """Validate both joint reactions when a Y-axis pendulum child is forced."""

    def test_link_incoming_joint_force_child_y_newton_cc(self) -> None:
        """Check child-forced Y-axis reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildY, "newton", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_child_y_newton_gg(self) -> None:
        """Check child-forced Y-axis reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildY, "newton", gpu_device())


class TestLinkIncomingJointForceFixedY:
    """Validate both joint reactions when a Y-axis pendulum fixed link is forced."""

    def test_link_incoming_joint_force_fixed_y_newton_cc(self) -> None:
        """Check fixed-link-forced Y reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedY, "newton", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_fixed_y_newton_gg(self) -> None:
        """Check fixed-link-forced Y reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedY, "newton", gpu_device())


class TestLinkIncomingJointForceChildX:
    """Validate both joint reactions when an X-axis pendulum child is forced."""

    def test_link_incoming_joint_force_child_x_newton_cc(self) -> None:
        """Check child-forced X-axis reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildX, "newton", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_child_x_newton_gg(self) -> None:
        """Check child-forced X-axis reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildX, "newton", gpu_device())


class TestLinkIncomingJointForceFixedX:
    """Validate both joint reactions when an X-axis pendulum fixed link is forced."""

    def test_link_incoming_joint_force_fixed_x_newton_cc(self) -> None:
        """Check fixed-link-forced X reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedX, "newton", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_fixed_x_newton_gg(self) -> None:
        """Check fixed-link-forced X reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedX, "newton", gpu_device())


class TestLinkIncomingJointForceChildZ:
    """Validate both joint reactions when a Z-axis pendulum child is forced."""

    def test_link_incoming_joint_force_child_z_newton_cc(self) -> None:
        """Check child-forced Z-axis reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildZ, "newton", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_child_z_newton_gg(self) -> None:
        """Check child-forced Z-axis reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildZ, "newton", gpu_device())


class TestLinkIncomingJointForceFixedZ:
    """Validate both joint reactions when a Z-axis pendulum fixed link is forced."""

    def test_link_incoming_joint_force_fixed_z_newton_cc(self) -> None:
        """Check fixed-link-forced Z reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedZ, "newton", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_fixed_z_newton_gg(self) -> None:
        """Check fixed-link-forced Z reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedZ, "newton", gpu_device())


class TestJointFriction:
    """Validate joint friction with Newton."""

    def test_joint_friction_newton_cc(self) -> None:
        """Verify joint friction on the Newton CPU pipeline."""
        run_scenario(self, JointFrictionCommon, "newton", cpu_device())

    @gpu_only
    def test_joint_friction_newton_gg(self) -> None:
        """Verify joint friction on the Newton GPU pipeline."""
        run_scenario(self, JointFrictionCommon, "newton", gpu_device())

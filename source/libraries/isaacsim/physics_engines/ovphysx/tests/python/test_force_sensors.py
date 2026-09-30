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

"""Validate force-sensor tensors with the OvPhysX engine.

These tests disable pendulum sleeping through ``PhysxArticulationAPI``.
Incoming-joint-force fixtures also author matching ``PhysxForceAPI`` metadata;
the common scenario controls the measured force step through rigid-body
tensors.
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
from physx_usd_schemas import PhysxSchema  # noqa: E402
from pxr import Gf, Sdf, Usd  # noqa: E402


def _add_force_torque(
    stage: Usd.Stage,
    path: str | Sdf.Path,
    force: Gf.Vec3f,
    torque: Gf.Vec3f,
    mode: str,
    is_enabled: bool,
    is_world_space: bool,
) -> PhysxSchema.PhysxForceAPI:
    """Apply and configure a PhysX force API on a prim.

    Args:
        stage: Stage containing the target prim.
        path: Path of the prim that receives the force API.
        force: Force vector to author.
        torque: Torque vector to author.
        mode: PhysX force mode token.
        is_enabled: Whether the authored force is active.
        is_world_space: Whether the vectors are expressed in world space.

    Returns:
        Configured force API for the target prim.

    """
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    api = PhysxSchema.PhysxForceAPI.Apply(prim)
    api.CreateForceAttr().Set(force)
    api.CreateTorqueAttr().Set(torque)
    api.CreateModeAttr().Set(mode)
    api.CreateForceEnabledAttr().Set(is_enabled)
    api.CreateWorldFrameEnabledAttr().Set(is_world_space)
    return api


class _PhysxArticulationApiMixin:
    """Configure each pendulum articulation for force-sensor tests.

    The mixin applies ``PhysxArticulationAPI`` to every replicated pendulum and
    disables sleeping so force observations remain active.
    """

    def _apply_engine_specifics(self) -> None:
        super()._apply_engine_specifics()
        for i in range(self.num_envs):
            prim = self.stage.GetPrimAtPath(f"/envs/env{i}/pendulum")
            if prim:
                api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                api.CreateSleepThresholdAttr(0.0)


class _PhysxForceApiMixin(_PhysxArticulationApiMixin):
    """Author PhysX force metadata matching the tensor-driven scenarios.

    Each environment receives one world-space force on the pendulum child link
    and one on its fixed-joint link.
    """

    def _apply_engine_specifics(self) -> None:
        super()._apply_engine_specifics()
        zero = Gf.Vec3f(0.0)
        for env_path in self.env_paths:
            child_link_path = env_path.AppendChild("pendulum").AppendChild("ChildLink")
            fixed_link_path = env_path.AppendChild("pendulum").AppendChild("FixedJointLink")
            _add_force_torque(
                self.stage,
                child_link_path,
                force=self.child_force,
                torque=zero,
                mode="force",
                is_enabled=True,
                is_world_space=True,
            )
            _add_force_torque(
                self.stage,
                fixed_link_path,
                force=self.fixed_force,
                torque=zero,
                mode="force",
                is_enabled=True,
                is_world_space=True,
            )


class _JointFrictionScenario(_PhysxArticulationApiMixin, JointFrictionCommon):
    pass


class _ForceTorqueSensorXScenario(_PhysxArticulationApiMixin, ForceTorqueSensorXCommon):
    pass


class _ForceTorqueSensorYScenario(_PhysxArticulationApiMixin, ForceTorqueSensorYCommon):
    pass


class _ForceTorqueSensorZScenario(_PhysxArticulationApiMixin, ForceTorqueSensorZCommon):
    pass


class _LinkIncomingJointForceChildY(_PhysxForceApiMixin, LinkIncomingJointForceChildYCommon):
    pass


class _LinkIncomingJointForceFixedY(_PhysxForceApiMixin, LinkIncomingJointForceFixedYCommon):
    pass


class _LinkIncomingJointForceChildX(_PhysxForceApiMixin, LinkIncomingJointForceChildXCommon):
    pass


class _LinkIncomingJointForceFixedX(_PhysxForceApiMixin, LinkIncomingJointForceFixedXCommon):
    pass


class _LinkIncomingJointForceChildZ(_PhysxForceApiMixin, LinkIncomingJointForceChildZCommon):
    pass


class _LinkIncomingJointForceFixedZ(_PhysxForceApiMixin, LinkIncomingJointForceFixedZCommon):
    pass


class TestForceTorqueSensorY:
    """Compare Y-axis pendulum joint wrenches and projected motor force."""

    def test_force_torque_sensor_y_ovphysx_cc(self) -> None:
        """Check Y-axis equilibrium forces with CPU simulation and CPU tensors."""
        run_scenario(self, _ForceTorqueSensorYScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_force_torque_sensor_y_ovphysx_gg(self) -> None:
        """Check Y-axis equilibrium forces with GPU simulation and GPU tensors."""
        run_scenario(self, _ForceTorqueSensorYScenario, "ovphysx", gpu_device())


class TestForceTorqueSensorX:
    """Compare X-axis pendulum joint wrenches and projected motor force."""

    def test_force_torque_sensor_x_ovphysx_cc(self) -> None:
        """Check X-axis equilibrium forces with CPU simulation and CPU tensors."""
        run_scenario(self, _ForceTorqueSensorXScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_force_torque_sensor_x_ovphysx_gg(self) -> None:
        """Check X-axis equilibrium forces with GPU simulation and GPU tensors."""
        run_scenario(self, _ForceTorqueSensorXScenario, "ovphysx", gpu_device())


class TestForceTorqueSensorZ:
    """Compare Z-axis pendulum joint wrenches and projected motor force."""

    def test_force_torque_sensor_z_ovphysx_cc(self) -> None:
        """Check Z-axis equilibrium forces with CPU simulation and CPU tensors."""
        run_scenario(self, _ForceTorqueSensorZScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_force_torque_sensor_z_ovphysx_gg(self) -> None:
        """Check Z-axis equilibrium forces with GPU simulation and GPU tensors."""
        run_scenario(self, _ForceTorqueSensorZScenario, "ovphysx", gpu_device())


class TestLinkIncomingJointForceChildY:
    """Validate both joint reactions when a Y-axis pendulum child is forced."""

    def test_link_incoming_joint_force_child_y_ovphysx_cc(self) -> None:
        """Check child-forced Y-axis reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildY, "ovphysx", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_child_y_ovphysx_gg(self) -> None:
        """Check child-forced Y-axis reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildY, "ovphysx", gpu_device())


class TestLinkIncomingJointForceFixedY:
    """Validate both joint reactions when a Y-axis pendulum fixed link is forced."""

    def test_link_incoming_joint_force_fixed_y_ovphysx_cc(self) -> None:
        """Check fixed-link-forced Y reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedY, "ovphysx", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_fixed_y_ovphysx_gg(self) -> None:
        """Check fixed-link-forced Y reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedY, "ovphysx", gpu_device())


class TestLinkIncomingJointForceChildX:
    """Validate both joint reactions when an X-axis pendulum child is forced."""

    def test_link_incoming_joint_force_child_x_ovphysx_cc(self) -> None:
        """Check child-forced X-axis reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildX, "ovphysx", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_child_x_ovphysx_gg(self) -> None:
        """Check child-forced X-axis reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildX, "ovphysx", gpu_device())


class TestLinkIncomingJointForceFixedX:
    """Validate both joint reactions when an X-axis pendulum fixed link is forced."""

    def test_link_incoming_joint_force_fixed_x_ovphysx_cc(self) -> None:
        """Check fixed-link-forced X reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedX, "ovphysx", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_fixed_x_ovphysx_gg(self) -> None:
        """Check fixed-link-forced X reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedX, "ovphysx", gpu_device())


class TestLinkIncomingJointForceChildZ:
    """Validate both joint reactions when a Z-axis pendulum child is forced."""

    def test_link_incoming_joint_force_child_z_ovphysx_cc(self) -> None:
        """Check child-forced Z-axis reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildZ, "ovphysx", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_child_z_ovphysx_gg(self) -> None:
        """Check child-forced Z-axis reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceChildZ, "ovphysx", gpu_device())


class TestLinkIncomingJointForceFixedZ:
    """Validate both joint reactions when a Z-axis pendulum fixed link is forced."""

    def test_link_incoming_joint_force_fixed_z_ovphysx_cc(self) -> None:
        """Check fixed-link-forced Z reactions with CPU simulation and CPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedZ, "ovphysx", cpu_device())

    @gpu_only
    def test_link_incoming_joint_force_fixed_z_ovphysx_gg(self) -> None:
        """Check fixed-link-forced Z reactions with GPU simulation and GPU tensors."""
        run_scenario(self, _LinkIncomingJointForceFixedZ, "ovphysx", gpu_device())


class TestJointFriction:
    """Verify high static friction holds a displaced pendulum joint."""

    def test_joint_friction_ovphysx_cc(self) -> None:
        """Check one-step friction holding with CPU simulation and CPU tensors."""
        run_scenario(self, _JointFrictionScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_joint_friction_ovphysx_gg(self) -> None:
        """Check one-step friction holding with GPU simulation and GPU tensors."""
        run_scenario(self, _JointFrictionScenario, "ovphysx", gpu_device())

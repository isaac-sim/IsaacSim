# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Xpbd scene module."""

from __future__ import annotations

from pxr import Usd

from .physics_scene import PhysicsScene


class NewtonXpbdScene(PhysicsScene):
    """Newton XPBD solver-specific wrapper for manipulating a USD Physics Scene prim.

    This class extends PhysicsScene to provide XPBD solver-specific functionality including
    relaxation parameters, compliance settings, and damping configuration.

    Args:
        prim: USD Physics Scene prim path or prim instance.
            If the input is a path, a new USD Physics Scene prim is created if it does not exist.

    Raises:
        ValueError: If the input prim exists and is not a USD Physics Scene prim.
    """

    def __init__(self, prim: str | Usd.Prim) -> None:
        super().__init__(prim)
        if not self._prim.HasAPI("NewtonXpbdSceneAPI"):
            self._prim.ApplyAPI("NewtonXpbdSceneAPI")

    def get_soft_body_relaxation(self) -> float:
        """Get the soft body relaxation parameter.

        Returns:
            Relaxation multiplier for tetrahedral FEM constraint corrections.
        """
        attr = self._prim.GetAttribute("newton:xpbd:softBodyRelaxation")
        return attr.Get() if attr else 0.9

    def set_soft_body_relaxation(self, relaxation: float) -> None:
        """Set the soft body relaxation parameter.

        Args:
            relaxation: Relaxation multiplier in range [0, 1].
        """
        attr = self._prim.GetAttribute("newton:xpbd:softBodyRelaxation")
        if attr:
            attr.Set(float(relaxation))

    def get_soft_contact_relaxation(self) -> float:
        """Get the soft contact relaxation parameter.

        Returns:
            Relaxation multiplier for soft contact constraint corrections.
        """
        attr = self._prim.GetAttribute("newton:xpbd:softContactRelaxation")
        return attr.Get() if attr else 0.9

    def set_soft_contact_relaxation(self, relaxation: float) -> None:
        """Set the soft contact relaxation parameter.

        Args:
            relaxation: Relaxation multiplier in range [0, 1].
        """
        attr = self._prim.GetAttribute("newton:xpbd:softContactRelaxation")
        if attr:
            attr.Set(float(relaxation))

    def get_joint_linear_relaxation(self) -> float:
        """Get the joint linear relaxation parameter.

        Returns:
            Relaxation multiplier for joint linear constraint corrections.
        """
        attr = self._prim.GetAttribute("newton:xpbd:jointLinearRelaxation")
        return attr.Get() if attr else 0.7

    def set_joint_linear_relaxation(self, relaxation: float) -> None:
        """Set the joint linear relaxation parameter.

        Args:
            relaxation: Relaxation multiplier in range [0, 1].
        """
        attr = self._prim.GetAttribute("newton:xpbd:jointLinearRelaxation")
        if attr:
            attr.Set(float(relaxation))

    def get_joint_angular_relaxation(self) -> float:
        """Get the joint angular relaxation parameter.

        Returns:
            Relaxation multiplier for joint angular constraint corrections.
        """
        attr = self._prim.GetAttribute("newton:xpbd:jointAngularRelaxation")
        return attr.Get() if attr else 0.4

    def set_joint_angular_relaxation(self, relaxation: float) -> None:
        """Set the joint angular relaxation parameter.

        Args:
            relaxation: Relaxation multiplier in range [0, 1].
        """
        attr = self._prim.GetAttribute("newton:xpbd:jointAngularRelaxation")
        if attr:
            attr.Set(float(relaxation))

    def get_joint_linear_compliance(self) -> float:
        """Get the joint linear compliance parameter.

        Returns:
            Compliance for joint linear constraints.
        """
        attr = self._prim.GetAttribute("newton:xpbd:jointLinearCompliance")
        return attr.Get() if attr else 0.0

    def set_joint_linear_compliance(self, compliance: float) -> None:
        """Set the joint linear compliance parameter.

        Args:
            compliance: Compliance value (0 = rigid constraints).
        """
        attr = self._prim.GetAttribute("newton:xpbd:jointLinearCompliance")
        if attr:
            attr.Set(float(compliance))

    def get_joint_angular_compliance(self) -> float:
        """Get the joint angular compliance parameter.

        Returns:
            Compliance for joint angular constraints.
        """
        attr = self._prim.GetAttribute("newton:xpbd:jointAngularCompliance")
        return attr.Get() if attr else 0.0

    def set_joint_angular_compliance(self, compliance: float) -> None:
        """Set the joint angular compliance parameter.

        Args:
            compliance: Compliance value (0 = rigid constraints).
        """
        attr = self._prim.GetAttribute("newton:xpbd:jointAngularCompliance")
        if attr:
            attr.Set(float(compliance))

    def get_rigid_contact_relaxation(self) -> float:
        """Get the rigid contact relaxation parameter.

        Returns:
            Relaxation multiplier for rigid body contact constraint corrections.
        """
        attr = self._prim.GetAttribute("newton:xpbd:rigidContactRelaxation")
        return attr.Get() if attr else 0.8

    def set_rigid_contact_relaxation(self, relaxation: float) -> None:
        """Set the rigid contact relaxation parameter.

        Args:
            relaxation: Relaxation multiplier in range [0, 1].
        """
        attr = self._prim.GetAttribute("newton:xpbd:rigidContactRelaxation")
        if attr:
            attr.Set(float(relaxation))

    def get_rigid_contact_con_weighting(self) -> bool:
        """Get whether contact constraint weighting is enabled.

        Returns:
            True if contact constraint weighting is enabled.
        """
        attr = self._prim.GetAttribute("newton:xpbd:rigidContactConWeighting")
        return attr.Get() if attr else True

    def set_rigid_contact_con_weighting(self, enabled: bool) -> None:
        """Enable or disable contact constraint weighting.

        Args:
            enabled: True to enable contact constraint weighting.
        """
        attr = self._prim.GetAttribute("newton:xpbd:rigidContactConWeighting")
        if attr:
            attr.Set(bool(enabled))

    def get_angular_damping(self) -> float:
        """Get the angular damping coefficient.

        Returns:
            Angular velocity damping coefficient.
        """
        attr = self._prim.GetAttribute("newton:xpbd:angularDamping")
        return attr.Get() if attr else 0.0

    def set_angular_damping(self, damping: float) -> None:
        """Set the angular damping coefficient.

        Args:
            damping: Angular velocity damping coefficient.
        """
        attr = self._prim.GetAttribute("newton:xpbd:angularDamping")
        if attr:
            attr.Set(float(damping))

    def get_enabled_restitution(self) -> bool:
        """Get whether restitution is enabled.

        Returns:
            True if restitution is enabled for contacts.
        """
        attr = self._prim.GetAttribute("newton:xpbd:restitutionEnabled")
        return attr.Get() if attr else False

    def set_enabled_restitution(self, enabled: bool) -> None:
        """Enable or disable restitution.

        Args:
            enabled: True to enable restitution for contacts.
        """
        attr = self._prim.GetAttribute("newton:xpbd:restitutionEnabled")
        if attr:
            attr.Set(bool(enabled))

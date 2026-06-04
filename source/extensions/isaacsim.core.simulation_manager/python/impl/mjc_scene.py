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

"""Mjc scene module."""

from __future__ import annotations

from typing import Literal

from pxr import Gf, Usd

from .physics_scene import PhysicsScene


class NewtonMjcScene(PhysicsScene):
    """Newton MuJoCo solver-specific wrapper for manipulating a USD Physics Scene prim.

    This class extends PhysicsScene to provide MuJoCo solver-specific functionality including
    integrator selection, solver parameters, and contact settings.

    Args:
        prim: USD Physics Scene prim path or prim instance.
            If the input is a path, a new USD Physics Scene prim is created if it does not exist.

    Raises:
        ValueError: If the input prim exists and is not a USD Physics Scene prim.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.simulation_manager import NewtonMjcScene
        >>>
        >>> mjc_scene = NewtonMjcScene("/World/PhysicsScene")
        >>> mjc_scene.set_dt(0.002)
        >>> mjc_scene.get_dt()
        0.002
    """

    def __init__(self, prim: str | Usd.Prim) -> None:
        super().__init__(prim)
        if not self._prim.HasAPI("MjcSceneAPI"):
            self._prim.ApplyAPI("MjcSceneAPI")

    def get_dt(self) -> float:
        """Get the MuJoCo Scene's delta time (timestep).

        Returns:
            MuJoCo Scene's delta time in seconds.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_dt()
            0.002
        """
        attr = self._prim.GetAttribute("mjc:option:timestep")
        return attr.Get() if attr else 0.002

    def set_dt(self, dt: float) -> None:
        """Set the MuJoCo Scene's delta time (timestep).

        Args:
            dt: MuJoCo Scene's delta time in seconds.

        Raises:
            ValueError: If the delta time is less than or equal to 0.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_dt(0.002)
        """
        if dt <= 0.0:
            raise ValueError(f"The delta time (DT) must be greater than 0, got {dt}")
        attr = self._prim.GetAttribute("mjc:option:timestep")
        if attr:
            attr.Set(float(dt))

    def get_integrator(self) -> Literal["euler", "rk4", "implicit", "implicitfast"]:
        """Get the MuJoCo Scene's integrator type.

        Returns:
            Numerical integrator type.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_integrator()
            'euler'
        """
        attr = self._prim.GetAttribute("mjc:option:integrator")
        return attr.Get() if attr else "euler"

    def set_integrator(self, integrator: Literal["euler", "rk4", "implicit", "implicitfast"]) -> None:
        """Set the MuJoCo Scene's integrator type.

        Args:
            integrator: Numerical integrator type.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_integrator("implicit")
        """
        attr = self._prim.GetAttribute("mjc:option:integrator")
        if attr:
            attr.Set(integrator)

    def get_solver(self) -> Literal["pgs", "cg", "newton"]:
        """Get the MuJoCo Scene's constraint solver algorithm.

        Returns:
            Constraint solver algorithm.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_solver()
            'newton'
        """
        attr = self._prim.GetAttribute("mjc:option:solver")
        return attr.Get() if attr else "newton"

    def set_solver(self, solver: Literal["pgs", "cg", "newton"]) -> None:
        """Set the MuJoCo Scene's constraint solver algorithm.

        Args:
            solver: Constraint solver algorithm.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_solver("cg")
        """
        attr = self._prim.GetAttribute("mjc:option:solver")
        if attr:
            attr.Set(solver)

    def get_iterations(self) -> int:
        """Get the maximum number of constraint solver iterations.

        Returns:
            Maximum number of constraint solver iterations.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_iterations()
            100
        """
        attr = self._prim.GetAttribute("mjc:option:iterations")
        return attr.Get() if attr else 100

    def set_iterations(self, iterations: int) -> None:
        """Set the maximum number of constraint solver iterations.

        Args:
            iterations: Maximum number of constraint solver iterations.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_iterations(200)
        """
        attr = self._prim.GetAttribute("mjc:option:iterations")
        if attr:
            attr.Set(int(iterations))

    def get_tolerance(self) -> float:
        """Get the solver tolerance for early termination.

        Returns:
            Tolerance threshold for early termination.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_tolerance()
            1e-08
        """
        attr = self._prim.GetAttribute("mjc:option:tolerance")
        return attr.Get() if attr else 1e-08

    def set_tolerance(self, tolerance: float) -> None:
        """Set the solver tolerance for early termination.

        Args:
            tolerance: Tolerance threshold for early termination.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_tolerance(1e-06)
        """
        attr = self._prim.GetAttribute("mjc:option:tolerance")
        if attr:
            attr.Set(float(tolerance))

    def get_cone(self) -> Literal["pyramidal", "elliptic"]:
        """Get the friction cone type.

        Returns:
            Friction cone type.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_cone()
            'pyramidal'
        """
        attr = self._prim.GetAttribute("mjc:option:cone")
        return attr.Get() if attr else "pyramidal"

    def set_cone(self, cone: Literal["pyramidal", "elliptic"]) -> None:
        """Set the friction cone type.

        Args:
            cone: Friction cone type.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_cone("elliptic")
        """
        attr = self._prim.GetAttribute("mjc:option:cone")
        if attr:
            attr.Set(cone)

    def get_jacobian(self) -> Literal["auto", "dense", "sparse"]:
        """Get the constraint Jacobian type.

        Returns:
            Constraint Jacobian type.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_jacobian()
            'auto'
        """
        attr = self._prim.GetAttribute("mjc:option:jacobian")
        return attr.Get() if attr else "auto"

    def set_jacobian(self, jacobian: Literal["auto", "dense", "sparse"]) -> None:
        """Set the constraint Jacobian type.

        Args:
            jacobian: Constraint Jacobian type.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_jacobian("sparse")
        """
        attr = self._prim.GetAttribute("mjc:option:jacobian")
        if attr:
            attr.Set(jacobian)

    def get_impratio(self) -> float:
        """Get the impedance ratio for elliptic friction cones.

        Returns:
            Ratio of frictional-to-normal constraint impedance.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_impratio()
            1.0
        """
        attr = self._prim.GetAttribute("mjc:option:impratio")
        return attr.Get() if attr else 1.0

    def set_impratio(self, impratio: float) -> None:
        """Set the impedance ratio for elliptic friction cones.

        Args:
            impratio: Ratio of frictional-to-normal constraint impedance.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_impratio(2.0)
        """
        attr = self._prim.GetAttribute("mjc:option:impratio")
        if attr:
            attr.Set(float(impratio))

    def get_wind(self) -> Gf.Vec3d:
        """Get the wind velocity vector.

        Returns:
            Velocity vector of medium (wind).

        Example:

        .. code-block:: python

            >>> mjc_scene.get_wind()
            Gf.Vec3d(0.0, 0.0, 0.0)
        """
        attr = self._prim.GetAttribute("mjc:option:wind")
        return Gf.Vec3d(attr.Get()) if attr else Gf.Vec3d(0.0, 0.0, 0.0)

    def set_wind(self, wind: Gf.Vec3d | tuple[float, float, float] | list[float]) -> None:
        """Set the wind velocity vector.

        Args:
            wind: Velocity vector of medium (wind).

        Example:

        .. code-block:: python

            >>> mjc_scene.set_wind(Gf.Vec3d(1.0, 0.0, 0.0))
        """
        if not isinstance(wind, Gf.Vec3d):
            wind = Gf.Vec3d(*wind)
        attr = self._prim.GetAttribute("mjc:option:wind")
        if attr:
            attr.Set(wind)

    def get_density(self) -> float:
        """Get the medium density.

        Returns:
            Density of medium.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_density()
            0.0
        """
        attr = self._prim.GetAttribute("mjc:option:density")
        return attr.Get() if attr else 0.0

    def set_density(self, density: float) -> None:
        """Set the medium density.

        Args:
            density: Density of medium.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_density(1.225)
        """
        attr = self._prim.GetAttribute("mjc:option:density")
        if attr:
            attr.Set(float(density))

    def get_viscosity(self) -> float:
        """Get the medium viscosity.

        Returns:
            Viscosity of medium.

        Example:

        .. code-block:: python

            >>> mjc_scene.get_viscosity()
            0.0
        """
        attr = self._prim.GetAttribute("mjc:option:viscosity")
        return attr.Get() if attr else 0.0

    def set_viscosity(self, viscosity: float) -> None:
        """Set the medium viscosity.

        Args:
            viscosity: Viscosity of medium.

        Example:

        .. code-block:: python

            >>> mjc_scene.set_viscosity(0.001)
        """
        attr = self._prim.GetAttribute("mjc:option:viscosity")
        if attr:
            attr.Set(float(viscosity))

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

"""Implementation for manipulating USD Physics Scene prims and their attributes."""

from __future__ import annotations

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from .. import _simulation_manager


class PhysicsScene:
    """Base class for manipulating a USD Physics Scene prim and its attributes.

    This class provides common functionality for working with USD Physics Scene prims,
    including Newton-common attributes that are applied by default to all physics scenes.

    Args:
        prim: USD Physics Scene prim path or prim instance.
            If the input is a path, a new USD Physics Scene prim is created if it does not exist.

    Raises:
        ValueError: If the input prim exists and is not a USD Physics Scene prim.
    """

    def __init__(self, prim: str | Usd.Prim) -> None:
        physics_scene = _simulation_manager.PhysicsScene(prim_utils.get_prim_path(prim))
        self._path = physics_scene.path
        self._prim = prim_utils.get_prim_at_path(prim)
        # Apply NewtonSceneAPI by default to all physics scenes
        if not self._prim.HasAPI("NewtonSceneAPI"):
            self._prim.ApplyAPI("NewtonSceneAPI")

    @property
    def path(self) -> str:
        """USD Physics Scene prim path.

        Returns:
            Prim path encapsulated by the wrapper.

        Example:

        .. code-block:: python

            >>> physics_scene.path
            '/World/physicsScene'
        """
        return self._path

    @property
    def prim(self) -> Usd.Prim:
        """USD Physics Scene prim instance.

        Returns:
            Prim instance encapsulated by the wrapper.

        Example:

        .. code-block:: python

            >>> physics_scene.prim
            Usd.Prim(</World/physicsScene>)
        """
        return self._prim

    @property
    def physics_scene(self) -> UsdPhysics.Scene:
        """USD Physics Scene instance.

        Returns:
            Physics Scene instance encapsulated by the wrapper.

        Example:

        .. code-block:: python

            >>> physics_scene.physics_scene
            UsdPhysics.Scene(Usd.Prim(</World/physicsScene>))
        """
        return UsdPhysics.Scene(self._prim)

    @staticmethod
    def get_physics_scene_paths(stage: Usd.Stage | None = None) -> list[str]:
        """Get the paths of all USD Physics Scene prims in the stage.

        Args:
            stage: USD stage to search for Physics Scene prims. If None, the current stage is used.

        Returns:
            List of USD Physics Scene prim paths.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import PhysicsScene
            >>>
            >>> PhysicsScene.get_physics_scene_paths()
            ['/World/physicsScene']
        """
        if stage is not None:
            return _simulation_manager.get_physics_scene_paths(stage_utils.get_stage_id(stage))
        return _simulation_manager.get_physics_scene_paths(None)

    def get_gravity(self) -> Gf.Vec3f:
        """Get the Physics Scene's gravity vector.

        Returns:
            Gravity vector in world units per second squared.

        Example:

        .. code-block:: python

            >>> physics_scene.get_gravity()
            Gf.Vec3f(0.0, 0.0, -9.81)
        """
        stage = self._prim.GetStage()
        meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
        magnitude = self.physics_scene.GetGravityMagnitudeAttr().Get()
        direction = self.physics_scene.GetGravityDirectionAttr().Get()
        return Gf.Vec3f(direction) * magnitude / meters_per_unit

    def set_gravity(self, gravity: Gf.Vec3f | tuple[float, float, float] | list[float]) -> None:
        """Set the Physics Scene's gravity vector.

        Args:
            gravity: Gravity vector in world units per second squared.

        Example:

        .. code-block:: python

            >>> physics_scene.set_gravity(Gf.Vec3f(0.0, 0.0, -9.81))
        """
        if not isinstance(gravity, Gf.Vec3f):
            gravity = Gf.Vec3f(*gravity)
        stage = self._prim.GetStage()
        meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
        magnitude = gravity.GetLength() * meters_per_unit
        direction = gravity.GetNormalized() if magnitude > 0 else Gf.Vec3f(0, 0, -1)
        self.physics_scene.GetGravityMagnitudeAttr().Set(magnitude)
        self.physics_scene.GetGravityDirectionAttr().Set(direction)

    def get_dt(self) -> float:
        """Get the Physics Scene's delta time (DT).

        Returns:
            Physics Scene's delta time (DT) for the active physics engine.

        Example:

        .. code-block:: python

            >>> physics_scene.get_dt()
            0.001
        """
        if self._prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            physx_scene_api = PhysxSchema.PhysxSceneAPI(self._prim)
            steps_per_second = physx_scene_api.GetTimeStepsPerSecondAttr().Get()
            return 1.0 / steps_per_second if steps_per_second else 0.0

        attr = self._prim.GetAttribute("newton:timeStepsPerSecond")
        steps_per_second = attr.Get() if attr else 1000
        return 1.0 / steps_per_second if steps_per_second else 0.0

    def set_dt(self, dt: float) -> None:
        """Set the Physics Scene's delta time (DT).

        Args:
            dt: Physics Scene's delta time (DT) for the active physics engine.

        Raises:
            RuntimeError: If the Physics Scene has no 'NewtonSceneAPI' applied.
            ValueError: If the delta time (DT) is less than 0 or greater than 1.0.

        Example:

        .. code-block:: python

            >>> physics_scene.set_dt(0.001)
        """
        if dt < 0.0 or dt > 1.0:
            raise ValueError(f"The delta time (DT) must be in the range [0.0, 1.0], got {dt}")
        steps_per_second = int(1.0 / dt) if dt else 0

        if self._prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            physx_scene_api = PhysxSchema.PhysxSceneAPI(self._prim)
            physx_scene_api.GetTimeStepsPerSecondAttr().Set(steps_per_second)
            return

        if not self._prim.HasAPI("NewtonSceneAPI"):
            raise RuntimeError("The Physics Scene has no 'NewtonSceneAPI' applied")
        attr = self._prim.GetAttribute("newton:timeStepsPerSecond")
        attr.Set(steps_per_second)

    def get_enabled_gravity(self) -> bool:
        """Get whether gravity is enabled for the Physics Scene.

        Returns:
            True if gravity is enabled, False otherwise.

        Example:

        .. code-block:: python

            >>> physics_scene.get_enabled_gravity()
            True
        """
        attr = self._prim.GetAttribute("newton:gravityEnabled")
        return attr.Get() if attr else True

    def set_enabled_gravity(self, enabled: bool) -> None:
        """Enable or disable gravity for the Physics Scene.

        Args:
            enabled: True to enable gravity, False to disable.

        Raises:
            RuntimeError: If the Physics Scene has no 'NewtonSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physics_scene.set_enabled_gravity(False)
        """
        if not self._prim.HasAPI("NewtonSceneAPI"):
            raise RuntimeError("The Physics Scene has no 'NewtonSceneAPI' applied")
        attr = self._prim.GetAttribute("newton:gravityEnabled")
        attr.Set(bool(enabled))

    def get_max_solver_iterations(self) -> int:
        """Get the maximum number of solver iterations for the Physics Scene.

        Returns:
            Maximum number of solver iterations. -1 means the solver chooses the default.

        Example:

        .. code-block:: python

            >>> physics_scene.get_max_solver_iterations()
            -1
        """
        attr = self._prim.GetAttribute("newton:maxSolverIterations")
        return attr.Get() if attr else -1

    def set_max_solver_iterations(self, iterations: int) -> None:
        """Set the maximum number of solver iterations for the Physics Scene.

        Args:
            iterations: Maximum number of solver iterations. Set to -1 to use solver default.

        Raises:
            RuntimeError: If the Physics Scene has no 'NewtonSceneAPI' applied.
            ValueError: If the iterations is less than -1.

        Example:

        .. code-block:: python

            >>> physics_scene.set_max_solver_iterations(100)
        """
        if iterations < -1:
            raise ValueError(f"The iterations must be greater than or equal to -1, got {iterations}")
        if not self._prim.HasAPI("NewtonSceneAPI"):
            raise RuntimeError("The Physics Scene has no 'NewtonSceneAPI' applied")
        attr = self._prim.GetAttribute("newton:maxSolverIterations")
        attr.Set(int(iterations))

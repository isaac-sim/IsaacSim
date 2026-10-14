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

"""Kit-backed debug visualization for the cuMotion collision world."""

from __future__ import annotations

from typing import Any

import numpy as np
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Capsule, Cube, Sphere
from isaacsim.core.experimental.prims import XformPrim


class KitCumotionDebugVisualizer:
    """Create and update visible USD representations of cuMotion colliders."""

    def __init__(
        self,
        enabled_rgb: list[float] | None = None,
        disabled_rgb: list[float] | None = None,
        alpha: float = 0.3,
    ) -> None:
        if disabled_rgb is None:
            disabled_rgb = [0.0, 1.0, 0.0]
        self._disabled_material = OmniPbrMaterial(paths="/CumotionDebug/DisabledMaterial")
        self._disabled_material.set_input_values("diffuse_color_constant", disabled_rgb)
        self._disabled_material.set_input_values("enable_opacity", [True])
        self._disabled_material.set_input_values("opacity_constant", [alpha])

        if enabled_rgb is None:
            enabled_rgb = [1.0, 0.0, 0.0]
        self._enabled_material = OmniPbrMaterial(paths="/CumotionDebug/EnabledMaterial")
        self._enabled_material.set_input_values("diffuse_color_constant", enabled_rgb)
        self._enabled_material.set_input_values("enable_opacity", [True])
        self._enabled_material.set_input_values("opacity_constant", [alpha])

    def create_sphere(self, source_prim_path: str, geometry_index: int, radius: float, enabled: bool) -> str:
        """Create a sphere debug prim and return its path."""
        path = self._generate_prim_path(source_prim_path, geometry_index)
        sphere = Sphere(paths=path, radii=radius)
        self._apply_material(sphere, enabled)
        return path

    def create_cube(self, source_prim_path: str, geometry_index: int, side_lengths: np.ndarray, enabled: bool) -> str:
        """Create a cube debug prim and return its path."""
        path = self._generate_prim_path(source_prim_path, geometry_index)
        cube = Cube(paths=path, sizes=1.0, scales=side_lengths)
        self._apply_material(cube, enabled)
        return path

    def create_capsule(
        self,
        source_prim_path: str,
        geometry_index: int,
        radius: float,
        height: float,
        enabled: bool,
    ) -> str:
        """Create a capsule debug prim and return its path."""
        path = self._generate_prim_path(source_prim_path, geometry_index)
        capsule = Capsule(paths=path, radii=radius, heights=height)
        self._apply_material(capsule, enabled)
        return path

    def update_poses(self, paths: list[str], positions: np.ndarray, orientations: np.ndarray) -> None:
        """Update world poses for debug prims."""
        XformPrim(paths=paths).set_local_poses(translations=positions, orientations=orientations)

    def set_enabled(self, path: str, enabled: bool) -> None:
        """Update the visual state of a debug prim."""
        self._apply_material(XformPrim(path), enabled)

    def _apply_material(self, debug_visual: Any, enabled: bool) -> None:
        """Apply the material corresponding to the collider's enabled state."""
        material = self._enabled_material if enabled else self._disabled_material
        debug_visual.apply_visual_materials(material)

    @staticmethod
    def _generate_prim_path(source_prim_path: str, geometry_index: int) -> str:
        """Generate a unique debug prim path for one collision geometry."""
        return f"/CumotionDebug/{source_prim_path.lstrip('/')}/Part{geometry_index}"

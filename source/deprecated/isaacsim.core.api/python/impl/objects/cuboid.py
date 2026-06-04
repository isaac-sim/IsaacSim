# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""High-level wrappers for creating and managing visual, fixed, and dynamic cuboid objects with collision and physics properties."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.materials.preview_surface import PreviewSurface
from isaacsim.core.api.materials.visual_material import VisualMaterial
from isaacsim.core.prims import SingleGeometryPrim, SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.string import find_unique_string_name
from pxr import Gf, UsdGeom


class VisualCuboid(SingleGeometryPrim):
    """High level wrapper to create/encapsulate a visual cuboid.

    .. note::

        Visual cuboids (Cube shape) have no collisions (Collider API) or rigid body dynamics (Rigid Body API)

    Args:
        prim_path: Prim path of the Prim to encapsulate or create.
        name: Shortname to be used as a key by Scene class. Note: needs to be unique if the object is added to the
            Scene.
        position: Position in the world frame of the prim. Shape is (3, ).
        translation: Translation in the local frame of the prim (with respect to its parent prim). Shape is (3, ).
        orientation: Quaternion orientation in the world/ local frame of the prim (depends if translation or position
            is specified). Quaternion is scalar-first (w, x, y, z). Shape is (4, ).
        scale: Local scale to be applied to the prim's dimensions. Shape is (3, ).
        visible: Set to false for an invisible prim in the stage while rendering.
        color: Color of the visual shape.
        size: Length of each cube edge.
        visual_material: Visual material to be applied to the held prim. If not specified, a default visual material
            will be added.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import VisualCuboid
        >>> import numpy as np
        >>>
        >>> # create a red visual cube at the given path
        >>> prim = VisualCuboid(prim_path="/World/Xform/Cube", color=np.array([1.0, 0.0, 0.0]))
        >>> prim
        <isaacsim.core.api.objects.cuboid.VisualCuboid object at 0x7f12e756fa00>

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "visual_cube",
        position: Sequence[float] | None = None,
        translation: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        scale: Sequence[float] | None = None,
        visible: bool | None = None,
        color: np.ndarray | None = None,
        size: float | None = None,
        visual_material: VisualMaterial | None = None,
    ) -> None:
        if is_prim_path_valid(prim_path):
            prim = get_prim_at_path(prim_path)
            if not prim.IsA(UsdGeom.Cube):
                raise Exception(f"The prim at path {prim_path} cannot be parsed as a Cube object")
            cubeGeom = UsdGeom.Cube(prim)
        else:
            cubeGeom = UsdGeom.Cube.Define(get_current_stage(), prim_path)
            if size is None:
                size = 1.0
            if visible is None:
                visible = True
            if visual_material is None:
                if color is None:
                    color = np.array([0.5, 0.5, 0.5])
                visual_prim_path = find_unique_string_name(
                    initial_name="/World/Looks/visual_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
                )
                visual_material = PreviewSurface(prim_path=visual_prim_path, color=color)
        SingleGeometryPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            collision=False,
        )
        if visual_material is not None:
            VisualCuboid.apply_visual_material(self, visual_material)
        if size is not None:
            VisualCuboid.set_size(self, size)
        size = VisualCuboid.get_size(self)
        extent_size = size * VisualCuboid.get_local_scale(self)
        cubeGeom.GetExtentAttr().Set(
            [
                Gf.Vec3f([-extent_size[0] / 2.0, -extent_size[1] / 2.0, -extent_size[2] / 2.0]),
                Gf.Vec3f([extent_size[0] / 2.0, extent_size[1] / 2.0, extent_size[2] / 2.0]),
            ]
        )
        return

    def set_size(self, size: float) -> None:
        """Set the length of each cube edge.

        Args:
            size: edge length

        Example:

        .. code-block:: python

            >>> prim.set_size(2.0)

        """
        self.geom.CreateSizeAttr(size)
        return

    def get_size(self) -> np.ndarray:
        """Get the length of each cube edge.

        Returns:
            edge length

        Example:

        .. code-block:: python

            >>> prim.get_size()
            1.0

        """
        return self.geom.GetSizeAttr().Get()


class FixedCuboid(VisualCuboid):
    """High level wrapper to create/encapsulate a fixed cuboid.

    .. note::

        Fixed cuboids (Cube shape) have collisions (Collider API) but no rigid body dynamics (Rigid Body API)

    Args:
        prim_path: prim path of the Prim to encapsulate or create
        name: shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        position: position in the world frame of the prim. shape is (3, ).
        translation: translation in the local frame of the prim
            (with respect to its parent prim). shape is (3, ).
        orientation: quaternion orientation in the world/ local frame of the prim
            (depends if translation or position is specified).
            quaternion is scalar-first (w, x, y, z). shape is (4, ).
        scale: local scale to be applied to the prim's dimensions. shape is (3, ).
        visible: set to false for an invisible prim in the stage while rendering.
        color: color of the visual shape.
        size: length of each cube edge.
        visual_material: visual material to be applied to the held prim.
            If not specified, a default visual material will be added.
        physics_material: physics material to be applied to the held prim.
            If not specified, a default physics material will be added.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import FixedCuboid
        >>> import numpy as np
        >>>
        >>> # create a red fixed cube at the given path
        >>> prim = FixedCuboid(prim_path="/World/Xform/Cube", color=np.array([1.0, 0.0, 0.0]))
        >>> prim
        <isaacsim.core.api.objects.cuboid.FixedCuboid object at 0x7f7b4d91da80>

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "fixed_cube",
        position: np.ndarray | None = None,
        translation: np.ndarray | None = None,
        orientation: np.ndarray | None = None,
        scale: np.ndarray | None = None,
        visible: bool | None = None,
        color: np.ndarray | None = None,
        size: float | None = None,
        visual_material: VisualMaterial | None = None,
        physics_material: PhysicsMaterial | None = None,
    ) -> None:
        set_offsets = False
        if not is_prim_path_valid(prim_path):
            # set default values if no physics material given
            if physics_material is None:
                static_friction = 0.2
                dynamic_friction = 1.0
                restitution = 0.0
                physics_material_path = find_unique_string_name(
                    initial_name="/World/Physics_Materials/physics_material",
                    is_unique_fn=lambda x: not is_prim_path_valid(x),
                )
                physics_material = PhysicsMaterial(
                    prim_path=physics_material_path,
                    dynamic_friction=dynamic_friction,
                    static_friction=static_friction,
                    restitution=restitution,
                )
            set_offsets = True
        VisualCuboid.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            size=size,
            visual_material=visual_material,
        )
        SingleGeometryPrim.set_collision_enabled(self, True)
        SingleGeometryPrim.set_collision_approximation(self, "boundingCube")
        if physics_material is not None:
            FixedCuboid.apply_physics_material(self, physics_material)
        if set_offsets:
            FixedCuboid.set_rest_offset(self, 0.0)
            FixedCuboid.set_contact_offset(self, 0.1)
            FixedCuboid.set_torsional_patch_radius(self, 1.0)
            FixedCuboid.set_min_torsional_patch_radius(self, 0.8)
        return


class DynamicCuboid(SingleRigidPrim, FixedCuboid):
    """High level wrapper to create/encapsulate a dynamic cuboid.

    .. note::

        Dynamic cuboids (Cube shape) have collisions (Collider API) and rigid body dynamics (Rigid Body API)

    Args:
        prim_path: Prim path of the Prim to encapsulate or create.
        name: Shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        position: Position in the world frame of the prim. Shape is (3, ).
        translation: Translation in the local frame of the prim
            (with respect to its parent prim). Shape is (3, ).
        orientation: Quaternion orientation in the world/ local frame of the prim
            (depends if translation or position is specified).
            Quaternion is scalar-first (w, x, y, z). Shape is (4, ).
        scale: Local scale to be applied to the prim's dimensions. Shape is (3, ).
        visible: Set to false for an invisible prim in the stage while rendering.
        color: Color of the visual shape.
        size: Length of each cube edge.
        visual_material: Visual material to be applied to the held prim.
            If not specified, a default visual material will be added.
        physics_material: Physics material to be applied to the held prim.
            If not specified, a default physics material will be added.
        mass: Mass in kg.
        density: Density.
        linear_velocity: Linear velocity in the world frame.
        angular_velocity: Angular velocity in the world frame.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import DynamicCuboid
        >>> import numpy as np
        >>>
        >>> # create a red dynamic cube of mass 1kg at the given path
        >>> prim = DynamicCuboid(prim_path="/World/Xform/Cube", color=np.array([1.0, 0.0, 0.0]), mass=1.0)
        >>> prim
        <isaacsim.core.api.objects.cuboid.DynamicCuboid object at 0x7ff14c04d990>

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "dynamic_cube",
        position: np.ndarray | None = None,
        translation: np.ndarray | None = None,
        orientation: np.ndarray | None = None,
        scale: np.ndarray | None = None,
        visible: bool | None = None,
        color: np.ndarray | None = None,
        size: float | None = None,
        visual_material: VisualMaterial | None = None,
        physics_material: PhysicsMaterial | None = None,
        mass: float | None = None,
        density: float | None = None,
        linear_velocity: Sequence[float] | None = None,
        angular_velocity: Sequence[float] | None = None,
    ) -> None:
        if not is_prim_path_valid(prim_path):
            if mass is None:
                mass = 0.02
        FixedCuboid.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            size=size,
            visual_material=visual_material,
            physics_material=physics_material,
        )
        SingleRigidPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            mass=mass,
            density=density,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )

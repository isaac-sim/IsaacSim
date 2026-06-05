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

"""High level wrappers for creating and manipulating capsule geometry prims with visual, collision, and physics properties."""

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


class VisualCapsule(SingleGeometryPrim):
    """High level wrapper to create/encapsulate a visual capsule.

    .. note::

        Visual capsules (Capsule shape) have no collisions (Collider API) or rigid body dynamics (Rigid Body API)

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
        radius: capsule radius.
        height: capsule height.
        visual_material: visual material to be applied to the held prim.
            If not specified, a default visual material will be added.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import VisualCapsule
        >>> import numpy as np
        >>>
        >>> # create a red visual capsule at the given path
        ... prim = VisualCapsule(
        ...     prim_path="/World/Xform/Capsule",
        ...     radius=0.5,
        ...     height=1.0,
        ...     color=np.array([1.0, 0.0, 0.0])
        ... )
        >>> prim
        <isaacsim.core.api.objects.capsule.VisualCapsule object at 0x7f4ff958b0d0>

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "visual_capsule",
        position: Sequence[float] | None = None,
        translation: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        scale: Sequence[float] | None = None,
        visible: bool | None = None,
        color: np.ndarray | None = None,
        radius: float | None = None,
        height: float | None = None,
        visual_material: VisualMaterial | None = None,
    ) -> None:

        if is_prim_path_valid(prim_path):
            prim = get_prim_at_path(prim_path)
            if not prim.IsA(UsdGeom.Capsule):
                raise Exception(f"The prim at path {prim_path} cannot be parsed as a Capsule object")
            capsule_geom = UsdGeom.Capsule(prim)
        else:
            capsule_geom = UsdGeom.Capsule.Define(get_current_stage(), prim_path)
            if radius is None:
                radius = 0.5
            if height is None:
                height = 1.0
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
            VisualCapsule.apply_visual_material(self, visual_material)
        if radius is not None:
            VisualCapsule.set_radius(self, radius)
        if height is not None:
            VisualCapsule.set_height(self, height)
        height = VisualCapsule.get_height(self)
        radius = VisualCapsule.get_radius(self)
        capsule_geom.GetExtentAttr().Set(
            [Gf.Vec3f([-radius, -radius, -height / 2.0]), Gf.Vec3f([radius, radius, height / 2.0])]
        )
        return

    def set_radius(self, radius: float) -> None:
        """Set the capsule radius.

        Args:
            radius: capsule radius

        Example:

        .. code-block:: python

            >>> prim.set_radius(1.0)

        """
        self.geom.GetRadiusAttr().Set(radius)
        return

    def get_radius(self) -> float:
        """Capsule radius.

        Returns:
            Capsule radius.

        Example:

        .. code-block:: python

            >>> prim.get_radius()
            0.5

        """
        return self.geom.GetRadiusAttr().Get()

    def set_height(self, height: float) -> None:
        """Set the capsule height.

        Args:
            height: capsule height

        Example:

        .. code-block:: python

            >>> prim.set_height(2.0)

        """
        self.geom.GetHeightAttr().Set(height)
        return

    def get_height(self) -> float:
        """Capsule height.

        Returns:
            Capsule height.

        Example:

        .. code-block:: python

            >>> prim.get_height()
            1.0

        """
        return self.geom.GetHeightAttr().Get()


class FixedCapsule(VisualCapsule):
    """High level wrapper to create/encapsulate a fixed capsule.

    .. note::

        Fixed capsules (Capsule shape) have collisions (Collider API) but no rigid body dynamics (Rigid Body API)

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
        radius: capsule radius.
        height: capsule height.
        visual_material: visual material to be applied to the held prim.
            If not specified, a default visual material will be added.
        physics_material: physics material to be applied to the held prim.
            If not specified, a default physics material will be added.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import FixedCapsule
        >>> import numpy as np
        >>>
        >>> # create a red fixed capsule at the given path
        >>> prim = FixedCapsule(
        ...     prim_path="/World/Xform/Capsule",
        ...     radius=0.5,
        ...     height=1.0,
        ...     color=np.array([1.0, 0.0, 0.0])
        ... )
        >>> print(prim)
        <isaacsim.core.api.objects.capsule.FixedCapsule object at 0x7f520c0d4790>

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "fixed_capsule",
        position: np.ndarray | None = None,
        translation: np.ndarray | None = None,
        orientation: np.ndarray | None = None,
        scale: np.ndarray | None = None,
        visible: bool | None = None,
        color: np.ndarray | None = None,
        radius: np.ndarray | None = None,
        height: float | None = None,
        visual_material: VisualMaterial | None = None,
        physics_material: PhysicsMaterial | None = None,
    ) -> None:
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
        VisualCapsule.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            radius=radius,
            height=height,
            visual_material=visual_material,
        )
        SingleGeometryPrim.set_collision_enabled(self, True)
        if physics_material is not None:
            FixedCapsule.apply_physics_material(self, physics_material)
        return


class DynamicCapsule(SingleRigidPrim, FixedCapsule):
    """High level wrapper to create/encapsulate a dynamic capsule.

    .. note::

        Dynamic capsules (Capsule shape) have collisions (Collider API) and rigid body dynamics (Rigid Body API)

    Args:
        prim_path: prim path of the Prim to encapsulate or create
        name: shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        position: position in the world frame of the prim. shape is (3, ).
        translation: translation in the local frame of the prim (with respect to its parent prim). shape is (3, ).
        orientation: quaternion orientation in the world/ local frame of the prim
            (depends if translation or position is specified). quaternion is scalar-first (w, x, y, z). shape is (4, ).
        scale: local scale to be applied to the prim's dimensions. shape is (3, ).
        visible: set to false for an invisible prim in the stage while rendering.
        color: color of the visual shape.
        radius: capsule radius.
        height: capsule height.
        visual_material: visual material to be applied to the held prim.
            If not specified, a default visual material will be added.
        physics_material: physics material to be applied to the held prim.
            If not specified, a default physics material will be added.
        mass: mass in kg.
        density: density.
        linear_velocity: linear velocity in the world frame.
        angular_velocity: angular velocity in the world frame.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import DynamicCapsule
        >>> import numpy as np
        >>>
        >>> # create a red dynamic capsule of mass 1kg at the given path
        >>> prim = DynamicCapsule(
        ...     prim_path="/World/Xform/Capsule",
        ...     radius=0.5,
        ...     height=1.0,
        ...     color=np.array([1.0, 0.0, 0.0]),
        ...     mass=1.0
        ... )
        >>> prim
        <isaacsim.core.api.objects.capsule.DynamicCapsule object at 0x7f4ff915f8e0>

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "dynamic_capsule",
        position: np.ndarray | None = None,
        translation: np.ndarray | None = None,
        orientation: np.ndarray | None = None,
        scale: np.ndarray | None = None,
        visible: bool | None = None,
        color: np.ndarray | None = None,
        radius: np.ndarray | None = None,
        height: np.ndarray | None = None,
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
        FixedCapsule.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            radius=radius,
            height=height,
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

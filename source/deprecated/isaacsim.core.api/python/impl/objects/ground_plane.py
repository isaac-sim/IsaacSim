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

"""Module for creating and managing ground plane objects in Isaac Sim."""

from __future__ import annotations

from collections.abc import Sequence

import carb
import numpy as np
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.materials.preview_surface import PreviewSurface
from isaacsim.core.api.materials.visual_material import VisualMaterial
from isaacsim.core.prims import SingleGeometryPrim, SingleXFormPrim
from isaacsim.core.utils.prims import (
    get_first_matching_child_prim,
    get_prim_path,
    get_prim_type_name,
    is_prim_path_valid,
)
from isaacsim.core.utils.stage import get_current_stage, get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.types import XFormPrimState
from pxr import Gf, PhysicsSchemaTools, Usd


class GroundPlane(object):
    """High level wrapper to create/encapsulate a ground plane.

    Args:
        prim_path: Prim path of the Prim to encapsulate or create.
        name: Shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        size: Length of each edge.
        z_position: Ground plane position in the z-axis.
        scale: Local scale to be applied to the prim's dimensions.
        visible: Set to false for an invisible prim in the stage while rendering.
        color: Color of the visual plane.
        physics_material: Physics material to be applied to the held prim.
            If not specified, a default physics material will be added.
        visual_material: Visual material to be applied to the held prim.
            If not specified, a default visual material will be added.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import GroundPlane
        >>> import numpy as np
        >>>
        >>> # create a ground plane placed at 0 in the z-axis
        >>> plane = GroundPlane(prim_path="/World/GroundPlane", z_position=0)
        >>> plane
        <isaacsim.core.api.objects.ground_plane.GroundPlane object at 0x7f15d003fb50>

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "ground_plane",
        size: float | None = None,
        z_position: float | None = None,
        scale: np.ndarray | None = None,
        visible: bool | None = None,
        color: np.ndarray | None = None,
        physics_material: PhysicsMaterial | None = None,
        visual_material: VisualMaterial | None = None,
    ) -> None:
        # wrap two object the xform and the collision plane
        if not is_prim_path_valid(prim_path):
            carb.log_info(f"Creating a new Ground Plane prim at path {prim_path}")
            stage = get_current_stage()
            if size is None:
                size = 50.0 / get_stage_units()
            if z_position is None:
                z_position = 0.0
            PhysicsSchemaTools.addGroundPlane(
                stage, prim_path, "Z", size, Gf.Vec3f(0, 0, z_position), Gf.Vec3f([0.0, 0.0, 0.0])
            )
            collision_prim_path = prim_path + "/geom"
            # set default values if no physics material given
            if physics_material is None:
                static_friction = 0.5
                dynamic_friction = 0.5
                restitution = 0.8
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
            if visual_material is None:
                if color is None:
                    color = np.array([0.5, 0.5, 0.5])
                visual_prim_path = find_unique_string_name(
                    initial_name="/World/Looks/visual_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
                )
                visual_material = PreviewSurface(prim_path=visual_prim_path, color=color)
        else:
            collision_prim_path = get_prim_path(
                get_first_matching_child_prim(prim_path=prim_path, predicate=lambda x: get_prim_type_name(x) == "Plane")
            )

        self._xform_prim = SingleXFormPrim(
            prim_path=prim_path, name=name, position=None, orientation=None, scale=scale, visible=visible
        )
        self._collision_prim = SingleGeometryPrim(
            prim_path=collision_prim_path,
            name=name + "_collision_plane",
            position=None,
            orientation=None,
            scale=scale,
            visible=visible,
            collision=True,
        )
        if z_position is not None:
            position = self._xform_prim._backend_utils.create_tensor_from_list(
                [0, 0, z_position], dtype="float32", device=self._xform_prim._device
            )
            self._xform_prim.set_world_pose(position=position)
            self._xform_prim.set_default_state(position=position)
            self._collision_prim.set_world_pose(position=position)
            self._collision_prim.set_default_state(position=position)
        if physics_material is not None:
            self._collision_prim.apply_physics_material(physics_material)
        if visual_material is not None:
            self._xform_prim.apply_visual_material(visual_material)
        return

    @property
    def prim_path(self) -> str:
        """Prim path in the stage.

        Returns:
            Prim path in the stage.

        Example:

        .. code-block:: python

            >>> plane.prim_path
            /World/GroundPlane

        """
        return self._xform_prim.prim_path

    @property
    def name(self) -> str | None:
        """Name given to the prim when instantiating it.

        Returns:
            Name given to the prim when instantiating it. Otherwise None.

        Example:

        .. code-block:: python

            >>> plane.name
            ground_plane

        """
        return self._xform_prim.name

    @property
    def prim(self) -> Usd.Prim:
        """USD Prim object that this object holds.

        Returns:
            USD Prim object that this object holds.

        Example:

        .. code-block:: python

            >>> plane.prim
            Usd.Prim(</World/GroundPlane>)

        """
        return self._xform_prim.prim

    @property
    def xform_prim(self) -> SingleXFormPrim:
        """Wrapped object as a SingleXFormPrim.

        Returns:
            Wrapped object as a SingleXFormPrim.

        Example:

        .. code-block:: python

            >>> plane.xform_prim
            <isaacsim.core.prims.single_xform_prim.SingleXFormPrim object at 0x7f1578d32560>

        """
        return self._xform_prim

    @property
    def collision_geometry_prim(self) -> SingleGeometryPrim:
        """Wrapped object as a SingleGeometryPrim.

        Returns:
            Wrapped object as a SingleGeometryPrim.

        Example:

        .. code-block:: python

            >>> plane.collision_geometry_prim
            <isaacsim.core.prims.single_geometry_prim.SingleGeometryPrim object at 0x7f15ff3461a0>

        """
        return self._collision_prim

    def initialize(self, physics_sim_view: object = None) -> None:
        """Create a physics simulation view if not passed and using PhysX tensor API.

        .. note::

            If the prim has been added to the world scene (e.g., ``world.scene.add(prim)``),
            it will be automatically initialized when the world is reset (e.g., ``world.reset()``).

        Args:
            physics_sim_view: Current physics simulation view.

        Example:

        .. code-block:: python

            >>> plane.initialize()

        """
        self._xform_prim.initialize(physics_sim_view=physics_sim_view)
        self._collision_prim.initialize(physics_sim_view=physics_sim_view)
        return

    def post_reset(self) -> None:
        """Reset the prim to its default state (position and orientation).

        Example:

        .. code-block:: python

            >>> plane.post_reset()

        """
        self._xform_prim.post_reset()
        self._collision_prim.post_reset()
        return

    def is_valid(self) -> bool:
        """Check if the prim path has a valid USD Prim at it.

        Returns:
            True is the current prim path corresponds to a valid prim in stage. False otherwise.

        Example:

        .. code-block:: python

            >>> # given an existing and valid prim
            >>> plane.is_valid()
            True

        """
        return self._xform_prim.is_valid()

    def apply_physics_material(self, physics_material: PhysicsMaterial, weaker_than_descendants: bool = False) -> None:
        """Used to apply physics material to the held prim and optionally its descendants.

        Args:
            physics_material: Physics material to be applied to the held prim. This where you want to
                define friction, restitution..etc. Note: if a physics material is not
                defined, the defaults will be used from PhysX.
            weaker_than_descendants: True if the material shouldn't override the descendants
                materials, otherwise False.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.api.materials import PhysicsMaterial
            >>>
            >>> # create a rigid body physical material
            >>> material = PhysicsMaterial(
            ...     prim_path="/World/physics_material/aluminum",  # path to the material prim to create
            ...     dynamic_friction=0.4,
            ...     static_friction=1.1,
            ...     restitution=0.1
            ... )
            >>> plane.apply_physics_material(material)

        """
        self._collision_prim.apply_physics_material(
            physics_material=physics_material, weaker_than_descendants=weaker_than_descendants
        )
        return

    def get_applied_physics_material(self) -> PhysicsMaterial:
        """Returns the current applied physics material in case it was applied using apply_physics_material or not.

        Returns:
            The current applied physics material.

        Example:

        .. code-block:: python

            >>> plane.get_applied_physics_material()
            <isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7f517ff62920>

        """
        return self._collision_prim.get_applied_physics_material()

    def set_world_pose(
        self, position: Sequence[float] | None = None, orientation: Sequence[float] | None = None
    ) -> None:
        """Sets prim's pose with respect to the world's frame.

        .. warning::

            This method will change (teleport) the prim pose immediately to the indicated value

        Args:
            position: Position in the world frame of the prim. Shape is (3, ).
            orientation: Quaternion orientation in the world frame of the prim.
                Quaternion is scalar-first (w, x, y, z). Shape is (4, ).

        .. hint::

            This method belongs to the methods used to set the prim state

        Example:

        .. code-block:: python

            >>> plane.set_world_pose(position=np.array([0.0, 0.0, 0.5]), orientation=np.array([1., 0., 0., 0.]))

        """
        self._collision_prim.set_world_pose(position=position, orientation=orientation)
        self._xform_prim.set_world_pose(position=position, orientation=orientation)
        return

    def get_world_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Get prim's pose with respect to the world's frame.

        Returns:
            First index is the position in the world frame (with shape (3, )).
            Second index is quaternion orientation (with shape (4, )) in the world frame

        Example:

        .. code-block:: python

            >>> # if the prim is in position (0.0, 0.0, 0.0) with respect to the world frame
            >>> position, orientation = prim.get_world_pose()
            >>> position
            [0. 0. 0.]
            >>> orientation
            [1. 0. 0. 0.]

        """
        return self._xform_prim.get_world_pose()

    def get_default_state(self) -> XFormPrimState:
        """Get the default prim states (spatial position and orientation).

        Returns:
            An object that contains the default state of the prim (position and orientation)

        Example:

        .. code-block:: python

            >>> state = plane.get_default_state()
            >>> state
            <isaacsim.core.utils.types.XFormPrimState object at 0x7f6efff41cf0>
            >>>
            >>> state.position
            [0. 0. 0.]
            >>> state.orientation
            [1. 0. 0. 0.]

        """
        return self._xform_prim.get_default_state()

    def set_default_state(
        self, position: Sequence[float] | None = None, orientation: Sequence[float] | None = None
    ) -> None:
        """Sets the default state of the prim (position and orientation), that will be used after each reset.

        Args:
            position: Position in the world frame of the prim. Shape is (3, ).
            orientation: Quaternion orientation in the world frame of the prim.
                Quaternion is scalar-first (w, x, y, z). Shape is (4, ).

        Example:

        .. code-block:: python

            >>> # configure default state
            >>> plane.set_default_state(position=np.array([0.0, 0.0, -1.0]), orientation=np.array([1, 0, 0, 0]))
            >>>
            >>> # set default states during post-reset
            >>> plane.post_reset()

        """
        self._xform_prim.set_default_state(position=position, orientation=orientation)
        self._collision_prim.set_default_state(position=position, orientation=orientation)
        return

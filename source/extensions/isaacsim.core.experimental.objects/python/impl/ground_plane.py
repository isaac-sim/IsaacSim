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

"""High level class for creating and wrapping ground plane prims with collision and rendering components."""

from __future__ import annotations

import pathlib
from typing import Literal, get_args

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.prims import GeomPrim, XformPrim
from pxr import PhysicsSchemaTools, Usd, UsdGeom

from .mesh import Mesh
from .shapes import Plane


def _check_and_get_composite_prims(stage: Usd.Stage, path: str) -> tuple[str, str]:
    """Check and retrieve the Plane and Mesh child prim paths from a ground plane prim.

    Validates that the ground plane prim has exactly 2 children (one Plane and one Mesh) and returns their paths.

    Args:
        stage: The USD stage containing the ground plane prim.
        path: Path to the ground plane prim to check.

    Returns:
        A tuple containing (plane_path, mesh_path) as strings.

    Raises:
        AssertionError: If the ground plane prim doesn't have exactly 2 children.
        AssertionError: If no Plane child prim is found.
        AssertionError: If no Mesh child prim is found.
    """
    prim = stage.GetPrimAtPath(path)
    children = prim.GetChildren()
    assert len(children) == 2 or len(children) == 3, (
        f"Ground plane (at path '{path}') must have 2 or 3 child prims: plane, mesh and visual material (optional). "
        f"Got {len(children)} child prims"
    )
    plane, mesh = None, None
    for child in children:
        if child.IsA(UsdGeom.Plane):
            plane = child.GetPath().pathString
        elif child.IsA(UsdGeom.Mesh):
            mesh = child.GetPath().pathString
    assert plane is not None, f"No Plane child prim found for ground plane prim at path {path}"
    assert mesh is not None, f"No Mesh child prim found for ground plane prim at path {path}"
    return plane, mesh


TEMPLATE = Literal["wireframe-blue"]


class GroundPlane(XformPrim):
    """High level class for creating/wrapping ground plane prims.

    Ground plane prims have the following USD structure:

    .. code-block:: cpp

        Xform           // GroundPlane instance
          |-- Plane     // for collision and physics
          |-- Mesh      // for rendering, since Plane is unsupported by Hydra rendering
          |-- Material  // template's visual material (optional)

    .. note::

        This class creates or wraps (one of both) ground plane prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the ground plane prims.
        * If the prim paths do not exist, ground plane prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to existing or non-existing (one of both) ground plane prims.
            Can include regular expressions for matching multiple prims.
        sizes: Sizes of the ground planes (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            It has effect only when new ground plane prims are created. Ignored when wrapping existing prims.
        colors: Normalized RGB display colors (shape ``(N, 3)``) or case-insensitive string representations.
            Supported string representations include hex codes and X11/CSS4 color names without spaces,
            as well as any other format supported by Matplotlib. Alpha channel is ignored for string representations.
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        templates: Templates to apply to the ground plane prims. If defined, it has precedence over the colors argument.
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        positions: Positions in the world frame (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        translations: Translations in the local frame (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        scales: Scales to be applied to the prims (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        reset_xform_op_properties: Whether to reset the transformation operation attributes of the prims to a standard set.
            See :py:meth:`reset_xform_op_properties` for more details.

    Raises:
        ValueError: If resulting paths are mixed (existing and non-existing prims) or invalid.
        AssertionError: If wrapped prims are not ground planes.
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import GroundPlane
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create a ground plane (with default template) at path: /World/ground_plane
        >>> prim = GroundPlane("/World/ground_plane")  # doctest: +NO_CHECK
        >>>
        >>> # create dark-blue ground planes at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = GroundPlane(paths, colors="darkblue", templates=None)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # GroundPlane
        sizes: float | list | np.ndarray | wp.array = 100.0,
        colors: str | list | np.ndarray | wp.array | None = None,
        templates: TEMPLATE | list[TEMPLATE] | None = "wireframe-blue",
        # XformPrim
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
    ) -> None:
        planes = []
        meshes = []
        stage = stage_utils.get_current_stage(backend="usd")
        existent_paths, nonexistent_paths = XformPrim.resolve_paths(paths)
        # get ground planes
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                plane, mesh = _check_and_get_composite_prims(stage, path)
                planes.append(plane)
                meshes.append(mesh)
        # create ground planes
        else:
            paths = nonexistent_paths
            up_axis = stage_utils.get_stage_up_axis()
            sizes = ops_utils.broadcast_to(sizes, shape=(len(paths), 1), device="cpu").numpy().flatten().tolist()
            for path, size in zip(nonexistent_paths, sizes):
                PhysicsSchemaTools.addGroundPlane(stage, path, up_axis, size / 2.0, (0.0, 0.0, 0.0), (0.5, 0.5, 0.5))
                plane, mesh = _check_and_get_composite_prims(stage, path)
                planes.append(plane)
                meshes.append(mesh)
        # initialize base class
        super().__init__(
            paths,
            resolve_paths=False,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )
        # get inner wrappers
        self._planes = Plane(planes)
        self._meshes = Mesh(meshes)
        self._geoms = GeomPrim(self._planes.paths)
        # initialize instance from arguments
        if colors is not None:
            self._planes.set_display_colors(colors)
            self._meshes.set_display_colors(colors)
        if templates is not None:
            self.apply_visual_templates(templates)

    """
    Properties.
    """

    @property
    def planes(self) -> Plane:
        """Plane instance that encapsulates the USD Plane prims that compose the ground planes.

        Returns:
            Plane instance.

        Example:

        .. code-block:: python

            >>> prims.planes
            <isaacsim.core.experimental.objects.impl.shapes.plane.Plane object at 0x...>
        """
        return self._planes

    @property
    def meshes(self) -> Mesh:
        """Mesh instance that encapsulates the USD Mesh prims that compose the ground planes.

        Returns:
            Mesh instance.

        Example:

        .. code-block:: python

            >>> prims.meshes
            <isaacsim.core.experimental.objects.impl.mesh.Mesh object at 0x...>
        """
        return self._meshes

    """
    Methods.
    """

    def set_offsets(
        self,
        contact_offsets: float | list | np.ndarray | wp.array = None,
        rest_offsets: float | list | np.ndarray | wp.array = None,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the contact and rest offsets for collision detection between prims.

        Backends: :guilabel:`usd`.

        Shapes whose distance is less than the sum of their contact offset values will generate contacts.
        The rest offset determines the distance at which two shapes will come to rest.
        Search for *Advanced Collision Detection* in |physx_docs| for more details.

        .. warning::

            The contact offset must be positive and greater than the rest offset.

        Args:
            contact_offsets: Contact offsets of the collision shapes (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            rest_offsets: Rest offsets of the collision shapes (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither contact_offsets nor rest_offsets are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same offsets (contact: 0.005, rest: 0.001) for all prims
            >>> prims.set_offsets(contact_offsets=[0.005], rest_offsets=[0.001])
            >>>
            >>> # set only the rest offsets for the second prim
            >>> prims.set_offsets(rest_offsets=[0.002], indices=[1])
        """
        self._geoms.set_offsets(contact_offsets=contact_offsets, rest_offsets=rest_offsets, indices=indices)

    def get_offsets(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> tuple[wp.array, wp.array]:
        """Get the contact and rest offsets for collision detection between prims.

        Backends: :guilabel:`usd`.

        Shapes whose distance is less than the sum of their contact offset values will generate contacts.
        The rest offset determines the distance at which two shapes will come to rest.
        Search for *Advanced Collision Detection* in |physx_docs| for more details.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The contact offsets (shape ``(N, 1)``). 2) The rest offsets (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the offsets of all prims
            >>> contact_offsets, rest_offsets = prims.get_offsets()
            >>> contact_offsets.shape, rest_offsets.shape
            ((3, 1), (3, 1))
            >>>
            >>> # get the offsets of the second prim
            >>> contact_offsets, rest_offsets = prims.get_offsets(indices=[1])
            >>> contact_offsets.shape, rest_offsets.shape
            ((1, 1), (1, 1))
        """
        return self._geoms.get_offsets(indices=indices)

    def set_torsional_patch_radii(
        self,
        radii: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
        minimum: bool = False,
    ) -> None:
        """Set the torsional patch radii of the contact patches used to apply torsional frictions.

        Backends: :guilabel:`usd`.

        Search for *Torsional Patch Radius* in |physx_docs| for more details.

        Args:
            radii: Torsional patch radii (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.
            minimum: Whether to set the minimum torsional patch radii instead of the standard ones.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the torsional patch radii for all prims
            >>> prims.set_torsional_patch_radii([0.1])
            >>>
            >>> # set the torsional patch radii for the first and last prims
            >>> prims.set_torsional_patch_radii([0.2], indices=[0, 2])
        """
        self._geoms.set_torsional_patch_radii(radii=radii, indices=indices, minimum=minimum)

    def get_torsional_patch_radii(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
        minimum: bool = False,
    ) -> wp.array:
        """Get the torsional patch radii of the contact patches used to apply torsional frictions.

        Backends: :guilabel:`usd`.

        Search for *Torsional Patch Radius* in |physx_docs| for more details.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.
            minimum: Whether to get the minimum torsional patch radii instead of the standard ones.

        Returns:
            The (minimum) torsional patch radii (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the torsional patch radii of all prims
            >>> radii = prims.get_torsional_patch_radii()
            >>> radii.shape
            (3, 1)
            >>>
            >>> # get the torsional patch radii of second prim
            >>> radii = prims.get_torsional_patch_radii(indices=[1])
            >>> radii.shape
            (1, 1)
        """
        return self._geoms.get_torsional_patch_radii(indices=indices, minimum=minimum)

    def set_enabled_collisions(
        self,
        enabled: bool | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Enable or disable the collision API of the prims.

        Backends: :guilabel:`usd`.

        When disabled, the prims will not participate in collision detection.

        Args:
            enabled: Boolean flags to enable/disable collision APIs (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # enable the collision API for all prims
            >>> prims.set_enabled_collisions([True])
            >>>
            >>> # disable the collision API for the first and last prims
            >>> prims.set_enabled_collisions([False], indices=[0, 2])
        """
        self._geoms.set_enabled_collisions(enabled=enabled, indices=indices)

    def get_enabled_collisions(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the enabled state of the collision API of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Boolean flags indicating if the collision API is enabled (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the collision enabled state of all prims after disabling it for the second prim
            >>> prims.set_enabled_collisions([False], indices=[1])
            >>> print(prims.get_enabled_collisions())
            [[ True]
             [False]
             [ True]]
        """
        return self._geoms.get_enabled_collisions(indices=indices)

    def apply_physics_materials(
        self,
        materials: type["PhysicsMaterial"] | list[type["PhysicsMaterial"]],
        *,
        weaker_than_descendants: bool | list | np.ndarray | wp.array | None = None,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Apply physics materials to the prims, and optionally, to their descendants.

        Backends: :guilabel:`usd`.

        Physics materials define properties like friction and restitution that affect how objects interact during collisions.
        If no physics material is defined, default values from Physics will be used.

        Args:
            materials: Physics materials to be applied to the prims (shape ``(N)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            weaker_than_descendants: Boolean flags to indicate whether descendant materials should be overridden (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.experimental.materials import RigidBodyMaterial
            >>>
            >>> # create a rigid body physics material
            >>> material = RigidBodyMaterial(
            ...     "/World/physics_material/aluminum",
            ...     static_frictions=[1.1],
            ...     dynamic_frictions=[0.4],
            ...     restitutions=[0.1],
            ... )
            >>>
            >>> # apply the material to all prims
            >>> prims.apply_physics_materials(material)  # or [material]
        """
        self._geoms.apply_physics_materials(
            materials=materials, weaker_than_descendants=weaker_than_descendants, indices=indices
        )

    def get_applied_physics_materials(
        self, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> list[type["PhysicsMaterial"] | None]:
        """Get the applied physics materials.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List of applied physics materials (shape ``(N,)``). If a prim does not have a material, ``None`` is returned.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the applied material path of the first prim
            >>> physics_material = prims.get_applied_physics_materials(indices=[0])[0]
            >>> physics_material.paths[0]
            '/World/physics_material/aluminum'
        """
        return self._geoms.get_applied_physics_materials(indices=indices)

    def apply_visual_templates(
        self, templates: TEMPLATE | list[TEMPLATE], *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Apply visual templates to the ground plane prims.

        Backends: :guilabel:`usd`.

        Args:
            templates: Templates to apply to the ground plane prims.
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            ValueError: If the template is not supported.

        Example:

        .. code-block:: python

            >>> # apply the blue grid template to all prims
            >>> prims.apply_visual_templates("wireframe-blue")
        """
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        templates = [templates] if isinstance(templates, str) else templates
        for template in templates:
            if template not in get_args(TEMPLATE):
                raise ValueError(f"Unsupported template: {template}. Supported templates are {get_args(TEMPLATE)}")
        templates = np.broadcast_to(np.array(templates, dtype=object), (indices.shape[0],))
        ext_path = app_utils.get_extension_path("isaacsim.core.experimental.objects")
        for i, index in enumerate(indices.numpy()):
            if templates[i] == "wireframe-blue":
                diffuse_texture = str(pathlib.Path(ext_path) / "data" / "templates" / "wireframe-blue.png")
            # apply visual material
            visual_material = OmniPbrMaterial(f"{self.paths[index]}/visualMaterial")
            visual_material.set_input_values("diffuse_texture", diffuse_texture)
            visual_material.set_input_values("project_uvw", True)
            self.meshes.apply_visual_materials(visual_material, indices=[index])

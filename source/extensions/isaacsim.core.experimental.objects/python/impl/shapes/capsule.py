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

"""Provide a high-level class for creating and manipulating USD Capsule primitives in Isaac Sim."""

from __future__ import annotations

from typing import Literal

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Gf, Usd, UsdGeom

from .shape import Shape


class Capsule(Shape):
    """High level class for creating/wrapping USD Capsule (primitive cylinder capped by two half spheres,.

    centered at the origin, whose spine is along the specified axis) prims.

    .. note::

        This class creates or wraps (one of both) USD Capsule prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the USD Capsule prims.
        * If the prim paths do not exist, USD Capsule prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to existing or non-existing (one of both) USD prims.
            Can include regular expressions for matching multiple prims.
        radii: Radii (capsule's two half-spheres radius) (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        heights: Heights (capsule's spine length along the axis excluding the size of the two half spheres) (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        axes: Axes (capsule's axis along which the spine is aligned) (shape ``(N,)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        colors: Normalized RGB display colors (shape ``(N, 3)``) or case-insensitive string representations.
            Supported string representations include hex codes and X11/CSS4 color names without spaces,
            as well as any other format supported by Matplotlib. Alpha channel is ignored for string representations.
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
        ValueError: Invalid string representation format for the colors.
        AssertionError: If wrapped prims are not USD Capsule.
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import Capsule
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create red capsules at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = Capsule(paths, colors=[1.0, 0.0, 0.0])  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # Capsule
        radii: float | list | np.ndarray | wp.array | None = None,
        heights: float | list | np.ndarray | wp.array | None = None,
        axes: Literal["X", "Y", "Z"] | list[Literal["X", "Y", "Z"]] | None = None,
        # Shape
        colors: str | list | np.ndarray | wp.array | None = None,
        # XformPrim
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
    ) -> None:
        self._geoms = []
        stage = stage_utils.get_current_stage(backend="usd")
        existent_paths, nonexistent_paths = Shape.resolve_paths(paths)
        # get capsules
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                prim = stage.GetPrimAtPath(path)
                assert prim.IsA(UsdGeom.Capsule), f"The wrapped prim at path {path} is not a USD Capsule"
                self._geoms.append(UsdGeom.Capsule(prim))
        # create capsules
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                self._geoms.append(UsdGeom.Capsule.Define(stage, path))
        # initialize base class
        super().__init__(
            paths,
            resolve_paths=False,
            colors=colors,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )
        # initialize instance from arguments
        if radii is not None:
            self.set_radii(radii)
        if heights is not None:
            self.set_heights(heights)
        if axes is not None:
            self.set_axes(axes)

    """
    Static methods.
    """

    @staticmethod
    def update_extents(geoms: list[UsdGeom.Capsule]) -> None:
        """Update the gprims' extents.

        Backends: :guilabel:`usd`.

        Args:
            geoms: Geoms to process.
        """
        # USD API
        for geom in geoms:
            axis = geom.GetAxisAttr().Get()
            radius = geom.GetRadiusAttr().Get()
            value = geom.GetHeightAttr().Get() / 2.0
            if axis == "X":
                extent = (Gf.Vec3f([-(radius + value), -radius, -radius]), Gf.Vec3f([radius + value, radius, radius]))
            elif axis == "Y":
                extent = (Gf.Vec3f([-radius, -(radius + value), -radius]), Gf.Vec3f([radius, radius + value, radius]))
            elif axis == "Z":
                extent = (Gf.Vec3f([-radius, -radius, -(radius + value)]), Gf.Vec3f([radius, radius, radius + value]))
            geom.GetExtentAttr().Set(extent)

    @staticmethod
    def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array:
        """Check if the prims at the given paths are valid for creating Capsule instances of this type.

        Backends: :guilabel:`usd`.

        .. warning::

            Since this method is static, the output is always on the CPU.

        Args:
            paths: Prim paths (or prims) to check for.

        Returns:
            Boolean flags indicating if the prims are valid for creating Capsule instances.

        Example:

        .. code-block:: python

            >>> # check if the following prims at paths are valid for creating instances
            >>> result = Capsule.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [False  True]
        """
        stage = stage_utils.get_current_stage(backend="usd")
        return ops_utils.place(
            [
                (stage.GetPrimAtPath(item) if isinstance(item, str) else item).IsA(UsdGeom.Capsule)
                for item in (paths if isinstance(paths, (list, tuple)) else [paths])
            ],
            dtype=wp.bool,
            device="cpu",
        )

    """
    Methods.
    """

    def set_radii(
        self,
        radii: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the radii (capsule's two half-spheres radius) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            radii: Radii (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same radii for all prims
            >>> prims.set_radii(radii=[0.1])
            >>>
            >>> # set only the radius for the second prim
            >>> prims.set_radii(radii=[0.2], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        radii = ops_utils.place(radii, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            geom = self.geoms[index]
            geom.GetRadiusAttr().Set(radii[0 if radii.shape[0] == 1 else i].item())
            self.update_extents([geom])

    def get_radii(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the radii (capsule's two half-spheres radius) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The radii (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the radii of all prims
            >>> radii = prims.get_radii()
            >>> radii.shape
            (3, 1)
            >>>
            >>> # get the radii of the first and last prims
            >>> radii = prims.get_radii(indices=[0, 2])
            >>> radii.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self.geoms[index].GetRadiusAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_heights(
        self,
        heights: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the heights (capsule's spine length along the axis excluding the size of the two half spheres) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            heights: Heights (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same heights for all prims
            >>> prims.set_heights(heights=[0.1])
            >>>
            >>> # set only the height for the second prim
            >>> prims.set_heights(heights=[0.2], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        heights = ops_utils.place(heights, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            geom = self.geoms[index]
            geom.GetHeightAttr().Set(heights[0 if heights.shape[0] == 1 else i].item())
            self.update_extents([geom])

    def get_heights(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the heights (capsule's spine length along the axis excluding the size of the two half spheres) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The heights (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the heights of all prims
            >>> heights = prims.get_heights()
            >>> heights.shape
            (3, 1)
            >>>
            >>> # get the heights of the first and last prims
            >>> heights = prims.get_heights(indices=[0, 2])
            >>> heights.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self.geoms[index].GetHeightAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_axes(
        self,
        axes: Literal["X", "Y", "Z"] | list[Literal["X", "Y", "Z"]],
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the axes (capsule's axis along which the spine is aligned) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            axes: Axes (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Invalid axis token.

        Example:

        .. code-block:: python

            >>> # set a different axis for each prim
            >>> prims.set_axes(axes=["X", "Y", "Z"])
            >>>
            >>> # set the axis for the second prim
            >>> prims.set_axes(axes=["X"], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        axes = [axes] if isinstance(axes, str) else axes
        for axis in set(axes):
            assert axis in ["X", "Y", "Z"], f"Invalid axis token: {axis}"
        axes = np.broadcast_to(np.array(axes, dtype=object), (indices.shape[0],))
        for i, index in enumerate(indices.numpy()):
            geom = self.geoms[index]
            geom.GetAxisAttr().Set(axes[i])
            self.update_extents([geom])

    def get_axes(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> list[Literal["X", "Y", "Z"]]:
        """Get the axes (capsule's axis along which the spine is aligned) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The axes (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the axes of all prims
            >>> axes = prims.get_axes()
            >>> axes
            ['Z', 'Z', 'Z']
            >>>
            >>> # get the axes of the first and last prims
            >>> axes = prims.get_axes(indices=[0, 2])
            >>> axes
            ['Z', 'Z']
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.empty((indices.shape[0],), dtype=object)
        for i, index in enumerate(indices.numpy()):
            data[i] = self.geoms[index].GetAxisAttr().Get()
        return data.tolist()

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

"""High level class for creating and wrapping USD Plane primitive prims centered at the origin."""

from __future__ import annotations

from typing import Literal

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Gf, Usd, UsdGeom

from .shape import Shape


class Plane(Shape):
    """High level class for creating/wrapping USD Plane (primitive plane centered at the origin) prims.

    .. warning::

        USD Plane is currently not supported by Omniverse Hydra rendering.
        Authoring operations can still be applied, but there will be no rendering in the viewport.

    .. note::

        This class creates or wraps (one of both) USD Plane prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the USD Plane prims.
        * If the prim paths do not exist, USD Plane prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to existing or non-existing (one of both) USD prims.
            Can include regular expressions for matching multiple prims.
        widths: Widths (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        lengths: Lengths (plane's length) (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        axes: Axes (plane's axis along which the surface is aligned) (shape ``(N,)``).
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
        AssertionError: If wrapped prims are not USD Plane.
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import Plane
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create dark cyan planes at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = Plane(paths, colors="DarkCyan")  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # Plane
        widths: float | list | np.ndarray | wp.array | None = None,
        lengths: float | list | np.ndarray | wp.array | None = None,
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
        # get planes
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                prim = stage.GetPrimAtPath(path)
                assert prim.IsA(UsdGeom.Plane), f"The wrapped prim at path {path} is not a USD Plane"
                self._geoms.append(UsdGeom.Plane(prim))
        # create planes
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                self._geoms.append(UsdGeom.Plane.Define(stage, path))
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
        if widths is not None:
            self.set_widths(widths)
        if lengths is not None:
            self.set_lengths(lengths)
        if axes is not None:
            self.set_axes(axes)

    """
    Static methods.
    """

    @staticmethod
    def update_extents(geoms: list[UsdGeom.Plane]) -> None:
        """Update the gprims' extents.

        Backends: :guilabel:`usd`.

        Args:
            geoms: Geoms to process.
        """
        # USD API
        for geom in geoms:
            half_width = geom.GetWidthAttr().Get() / 2.0
            half_length = geom.GetLengthAttr().Get() / 2.0
            axis = geom.GetAxisAttr().Get()
            if axis == "X":
                extent = (Gf.Vec3f([0, -half_length, -half_width]), Gf.Vec3f([0, half_length, half_width]))
            elif axis == "Y":
                extent = (Gf.Vec3f([-half_width, 0, -half_length]), Gf.Vec3f([half_width, 0, half_length]))
            elif axis == "Z":
                extent = (Gf.Vec3f([-half_width, -half_length, 0]), Gf.Vec3f([half_width, half_length, 0]))
            geom.GetExtentAttr().Set(extent)

    @staticmethod
    def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array:
        """Check if the prims at the given paths are valid for creating Plane instances of this type.

        Backends: :guilabel:`usd`.

        .. warning::

            Since this method is static, the output is always on the CPU.

        Args:
            paths: Prim paths (or prims) to check for.

        Returns:
            Boolean flags indicating if the prims are valid for creating Plane instances.

        Example:

        .. code-block:: python

            >>> # check if the following prims at paths are valid for creating instances
            >>> result = Plane.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [False  True]
        """
        stage = stage_utils.get_current_stage(backend="usd")
        return ops_utils.place(
            [
                (stage.GetPrimAtPath(item) if isinstance(item, str) else item).IsA(UsdGeom.Plane)
                for item in (paths if isinstance(paths, (list, tuple)) else [paths])
            ],
            dtype=wp.bool,
            device="cpu",
        )

    """
    Methods.
    """

    def set_widths(
        self,
        widths: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the widths of the prims.

        Backends: :guilabel:`usd`.

        The width aligns to the x-axis when axis is ``'Y'`` or ``'Z'``, or to the z-axis when axis is ``'X'``.

        Args:
            widths: Widths (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same widths for all prims
            >>> prims.set_widths(widths=[0.1])
            >>>
            >>> # set only the width for the second prim
            >>> prims.set_widths(widths=[0.2], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        widths = ops_utils.place(widths, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            geom = self.geoms[index]
            geom.GetWidthAttr().Set(widths[0 if widths.shape[0] == 1 else i].item())
            self.update_extents([geom])

    def get_widths(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the widths of the prims.

        Backends: :guilabel:`usd`.

        The width aligns to the x-axis when axis is ``'Y'`` or ``'Z'``, or to the z-axis when axis is ``'X'``.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The widths (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the widths of all prims
            >>> widths = prims.get_widths()
            >>> widths.shape
            (3, 1)
            >>>
            >>> # get the widths of the first and last prims
            >>> widths = prims.get_widths(indices=[0, 2])
            >>> widths.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self.geoms[index].GetWidthAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_lengths(
        self,
        lengths: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the lengths of the prims.

        Backends: :guilabel:`usd`.

        The length aligns to the y-axis when axis is ``'X'`` or ``'Z'``, or to the z-axis when axis is ``'Y'``.

        Args:
            lengths: Lengths (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same lengths for all prims
            >>> prims.set_lengths(lengths=[0.1])
            >>>
            >>> # set only the length for the second prim
            >>> prims.set_lengths(lengths=[0.2], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        lengths = ops_utils.place(lengths, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            geom = self.geoms[index]
            geom.GetLengthAttr().Set(lengths[0 if lengths.shape[0] == 1 else i].item())
            self.update_extents([geom])

    def get_lengths(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the lengths of the prims.

        Backends: :guilabel:`usd`.

        The length aligns to the y-axis when axis is ``'X'`` or ``'Z'``, or to the z-axis when axis is ``'Y'``.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The lengths (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the lengths of all prims
            >>> lengths = prims.get_lengths()
            >>> lengths.shape
            (3, 1)
            >>>
            >>> # get the lengths of the first and last prims
            >>> lengths = prims.get_lengths(indices=[0, 2])
            >>> lengths.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self.geoms[index].GetLengthAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_axes(
        self,
        axes: Literal["X", "Y", "Z"] | list[Literal["X", "Y", "Z"]],
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the axes (plane's axis along which the surface is aligned) of the prims.

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
        """Get the axes (plane's axis along which the surface is aligned) of the prims.

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

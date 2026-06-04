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

"""High level class for creating/wrapping USD Rect Light prims that emit light from one side of a rectangle."""

from __future__ import annotations

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Usd, UsdLux

from .light import Light


class RectLight(Light):
    """High level class for creating/wrapping USD Rect Light (light emitted from one side of a rectangle) prims.

    The rectangle, centered in the XY plane, emits light along the -Z axis.

    .. note::

        This class creates or wraps (one of both) USD Rect Light prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the USD Rect Light prims.
        * If the prim paths do not exist, USD Rect Light prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to existing or non-existing (one of both) USD prims.
            Can include regular expressions for matching multiple prims.
        widths: Widths (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        heights: Heights (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        texture_files: Color texture files (shape ``(N,)``). In the default position, the texture file's minimum
            and maximum coordinates should be at ``(+X, +Y)`` and ``(-X, -Y)`` respectively.
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
        AssertionError: If wrapped prims are not USD Rect Light.
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import RectLight
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create rect lights at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = RectLight(paths)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # RectLight
        widths: float | list | np.ndarray | wp.array | None = None,
        heights: float | list | np.ndarray | wp.array | None = None,
        texture_files: str | list[str] | None = None,
        # XformPrim
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
    ) -> None:
        self._lights = []
        stage = stage_utils.get_current_stage(backend="usd")
        existent_paths, nonexistent_paths = Light.resolve_paths(paths)
        # get lights
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                prim = stage.GetPrimAtPath(path)
                assert prim.IsA(UsdLux.RectLight), f"Prim at path {path} is not a RectLight"
                self._lights.append(UsdLux.RectLight(prim))
        # create lights
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                self._lights.append(UsdLux.RectLight.Define(stage, path))
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
        # initialize instance from arguments
        if widths is not None:
            self.set_widths(widths)
        if heights is not None:
            self.set_heights(heights)
        if texture_files is not None:
            self.set_texture_files(texture_files)

    """
    Methods.
    """

    def set_widths(
        self, widths: float | list | np.ndarray | wp.array, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the widths of the prims.

        Backends: :guilabel:`usd`.

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
            self.lights[index].GetWidthAttr().Set(widths[0 if widths.shape[0] == 1 else i].item())

    def get_widths(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the widths of the prims.

        Backends: :guilabel:`usd`.

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
            data[i][0] = self.lights[index].GetWidthAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_heights(
        self,
        heights: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the heights of the prims.

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
            self.lights[index].GetHeightAttr().Set(heights[0 if heights.shape[0] == 1 else i].item())

    def get_heights(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the heights of the prims.

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
            data[i][0] = self.lights[index].GetHeightAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_texture_files(
        self, texture_files: str | list[str], *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the color texture files (to use on the rectangle) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            texture_files: Color texture files (shape ``(N,)``). In the default position, the texture file's minimum
                and maximum coordinates should be at ``(+X, +Y)`` and ``(-X, -Y)`` respectively.
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set color texture files for all prims
            >>> prims.set_texture_files(texture_files=[
            ...     "/local/path/to/texture_0",
            ...     "/local/path/to/texture_1",
            ...     "/local/path/to/texture_2",
            ... ])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        texture_files = [texture_files] if isinstance(texture_files, str) else texture_files
        texture_files = np.broadcast_to(np.array(texture_files, dtype=object), (indices.shape[0],))
        for i, index in enumerate(indices.numpy()):
            self.lights[index].GetTextureFileAttr().Set(texture_files[i])

    def get_texture_files(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> list[str | None]:
        """Get the color texture files (to use on the rectangle) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The color texture files or ``None`` if no texture is set (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the color texture file for the second prim
            >>> prims.set_texture_files(["/local/path/to/texture_1"], indices=[1])
            >>>
            >>> # get the color texture files of all prims
            >>> prims.get_texture_files()
            [None, '/local/path/to/texture_1', None]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = []
        for index in indices.numpy():
            texture_file = self.lights[index].GetTextureFileAttr().Get()
            data.append(None if texture_file is None else texture_file.path)
        return data

    """
    Static methods.
    """

    @staticmethod
    def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array:
        """Check if the prims at the given paths are valid for creating Light instances of this type.

        Backends: :guilabel:`usd`.

        .. warning::

            Since this method is static, the output is always on the CPU.

        Args:
            paths: Prim paths (or prims) to check for.

        Returns:
            Boolean flags indicating if the prims are valid for creating Light instances.

        Example:

        .. code-block:: python

            >>> # check if the following prims at paths are valid for creating instances
            >>> result = RectLight.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [False  True]
        """
        stage = stage_utils.get_current_stage(backend="usd")
        return ops_utils.place(
            [
                (stage.GetPrimAtPath(item) if isinstance(item, str) else item).IsA(UsdLux.RectLight)
                for item in (paths if isinstance(paths, (list, tuple)) else [paths])
            ],
            dtype=wp.bool,
            device="cpu",
        )

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

"""High level class for creating and managing USD Dome Light prims for Image Based Lighting from external environments."""

from __future__ import annotations

from typing import Literal

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Usd, UsdLux

from .light import Light


class DomeLight(Light):
    """High level class for creating/wrapping USD Dome Light (light emitted inward from a distant external environment) prims.

    .. note::

        This class creates or wraps (one of both) USD Dome Light prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the USD Dome Light prims.
        * If the prim paths do not exist, USD Dome Light prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to existing or non-existing (one of both) USD prims.
            Can include regular expressions for matching multiple prims.
        radii: Guide radii (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        texture_files: Color texture files (shape ``(N,)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        texture_formats: Texture formats (shape ``(N,)``).
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
        AssertionError: If wrapped prims are not USD Dome Light.
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import DomeLight
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create dome lights at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = DomeLight(paths)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # DomeLight
        radii: float | list | np.ndarray | wp.array | None = None,
        texture_files: str | list[str] | None = None,
        texture_formats: (
            Literal["automatic", "latlong", "mirroredBall", "angular", "cubeMapVerticalCross"]
            | list[Literal["automatic", "latlong", "mirroredBall", "angular", "cubeMapVerticalCross"]]
            | None
        ) = None,
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
                assert prim.IsA(UsdLux.DomeLight), f"Prim at path {path} is not a DomeLight"
                self._lights.append(UsdLux.DomeLight(prim))
        # create lights
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                self._lights.append(UsdLux.DomeLight.Define(stage, path))
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
        if radii is not None:
            self.set_guide_radii(radii)
        if texture_files is not None:
            self.set_texture_files(texture_files)
        if texture_formats is not None:
            self.set_texture_formats(texture_formats)

    """
    Methods.
    """

    def set_guide_radii(
        self, radii: float | list | np.ndarray | wp.array, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the guide radii (use to visualize the dome light) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            radii: Guide radii (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same guide radii for all prims
            >>> prims.set_guide_radii(radii=[0.1])
            >>>
            >>> # set only the guide radius for the second prim
            >>> prims.set_guide_radii(radii=[0.2], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        radii = ops_utils.place(radii, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            self.lights[index].GetGuideRadiusAttr().Set(radii[0 if radii.shape[0] == 1 else i].item())

    def get_guide_radii(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the guide radii (use to visualize the dome light) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The guide radii (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the guide radii of all prims
            >>> radii = prims.get_guide_radii()
            >>> radii.shape
            (3, 1)
            >>>
            >>> # get the guide radii of the first and last prims
            >>> radii = prims.get_guide_radii(indices=[0, 2])
            >>> radii.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self.lights[index].GetGuideRadiusAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_texture_files(
        self, texture_files: str | list[str], *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the color texture files (e.g.: High Dynamic Range (HDR) texture intended for Image Based Lighting (IBL)) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            texture_files: Color texture files (shape ``(N,)``).
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

    def get_texture_files(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> list[str]:
        """Get the color texture files (e.g.: High Dynamic Range (HDR) texture intended for Image Based Lighting (IBL)) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The color texture files (shape ``(N,)``).

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

    def set_texture_formats(
        self,
        texture_formats: (
            Literal["automatic", "latlong", "mirroredBall", "angular", "cubeMapVerticalCross"]
            | list[Literal["automatic", "latlong", "mirroredBall", "angular", "cubeMapVerticalCross"]]
        ),
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the texture formats (parameterization of the color map file) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            texture_formats: Texture formats (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the texture formats for the second prim
            >>> prims.set_texture_formats(texture_formats=["latlong"], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        texture_formats = [texture_formats] if isinstance(texture_formats, str) else texture_formats
        broadcast = len(texture_formats) == 1
        for i, index in enumerate(indices.numpy()):
            self.lights[index].GetTextureFormatAttr().Set(texture_formats[0 if broadcast else i])

    def get_texture_formats(
        self, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> list[Literal["automatic", "latlong", "mirroredBall", "angular", "cubeMapVerticalCross"]]:
        """Get the texture formats (parameterization of the color map file) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The texture formats (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the texture formats of all prims
            >>> prims.get_texture_formats()
            ['automatic', 'automatic', 'automatic']
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = []
        for index in indices.numpy():
            data.append(self.lights[index].GetTextureFormatAttr().Get())
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
            >>> result = DomeLight.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [False  True]
        """
        stage = stage_utils.get_current_stage(backend="usd")
        return ops_utils.place(
            [
                (stage.GetPrimAtPath(item) if isinstance(item, str) else item).IsA(UsdLux.DomeLight)
                for item in (paths if isinstance(paths, (list, tuple)) else [paths])
            ],
            dtype=wp.bool,
            device="cpu",
        )

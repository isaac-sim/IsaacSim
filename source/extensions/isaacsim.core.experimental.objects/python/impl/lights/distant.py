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

"""High level class for creating/wrapping USD Distant Light prims that emit light from a distant source along the -Z axis."""

from __future__ import annotations

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Usd, UsdLux

from .light import Light


class DistantLight(Light):
    """High level class for creating/wrapping USD Distant Light (light emitted from a distant source along the -Z axis) prims.

    .. note::

        This class creates or wraps (one of both) USD Distant Light prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the USD Distant Light prims.
        * If the prim paths do not exist, USD Distant Light prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to existing or non-existing (one of both) USD prims.
            Can include regular expressions for matching multiple prims.
        angles: Angles (shape ``(N, 1)``). The values are assumed to be in the range ``[0, 2 * pi)``.
            If a value is 0, the light represents a perfectly parallel light source.
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
        AssertionError: If wrapped prims are not USD Distant Light.
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import DistantLight
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create distant lights at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = DistantLight(paths)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # DistantLight
        angles: float | list | np.ndarray | wp.array | None = None,
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
                assert prim.IsA(UsdLux.DistantLight), f"Prim at path {path} is not a DistantLight"
                self._lights.append(UsdLux.DistantLight(prim))
        # create lights
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                self._lights.append(UsdLux.DistantLight.Define(stage, path))
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
        if angles is not None:
            self.set_angles(angles)

    """
    Methods.
    """

    def set_angles(
        self, angles: float | list | np.ndarray | wp.array, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the angles (angular diameter of the light) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            angles: Angles (shape ``(N, 1)``). The values are assumed to be in the range ``[0, 2 * pi)``.
                If a value is 0, the light represents a perfectly parallel light source.
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same angles for all prims
            >>> prims.set_angles(angles=[0.5236])
            >>>
            >>> # set only the angle for the second prim
            >>> prims.set_angles(angles=[0.7854], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        angles = ops_utils.place(angles, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            self.lights[index].GetAngleAttr().Set(angles[0 if angles.shape[0] == 1 else i].item())

    def get_angles(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the angles (angular diameter of the light) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The angles (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the angles of all prims
            >>> angles = prims.get_angles()
            >>> angles.shape
            (3, 1)
            >>>
            >>> # get the angles of the first and last prims
            >>> angles = prims.get_angles(indices=[0, 2])
            >>> angles.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self.lights[index].GetAngleAttr().Get()
        return ops_utils.place(data, device=self._device)

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
            >>> result = DistantLight.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [False  True]
        """
        stage = stage_utils.get_current_stage(backend="usd")
        return ops_utils.place(
            [
                (stage.GetPrimAtPath(item) if isinstance(item, str) else item).IsA(UsdLux.DistantLight)
                for item in (paths if isinstance(paths, (list, tuple)) else [paths])
            ],
            dtype=wp.bool,
            device="cpu",
        )

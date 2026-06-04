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

"""High level class for creating/wrapping USD Cylinder Light prims that emit light outward from a cylinder."""

from __future__ import annotations

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Usd, UsdLux

from .light import Light


class CylinderLight(Light):
    """High level class for creating/wrapping USD Cylinder Light (light emitted outward from a cylinder) prims.

    The cylinder is centered at the origin and has its major axis on the X axis.
    The cylinder does not emit light from the flat end-caps.

    .. note::

        This class creates or wraps (one of both) USD Cylinder Light prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the USD Cylinder Light prims.
        * If the prim paths do not exist, USD Cylinder Light prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to existing or non-existing (one of both) USD prims.
            Can include regular expressions for matching multiple prims.
        radii: Radii (cylinder's radius) (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        lengths: Lengths (cylinder's spine length along the axis) (shape ``(N, 1)``).
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
        AssertionError: If wrapped prims are not USD Cylinder Light.
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import CylinderLight
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create cylinder lights at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = CylinderLight(paths)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # CylinderLight
        radii: float | list | np.ndarray | wp.array | None = None,
        lengths: float | list | np.ndarray | wp.array | None = None,
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
                assert prim.IsA(UsdLux.CylinderLight), f"Prim at path {path} is not a CylinderLight"
                self._lights.append(UsdLux.CylinderLight(prim))
        # create lights
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                self._lights.append(UsdLux.CylinderLight.Define(stage, path))
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
            self.set_radii(radii)
        if lengths is not None:
            self.set_lengths(lengths)

    """
    Methods.
    """

    def set_lengths(
        self,
        lengths: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the lengths (in the local X axis) of the prims.

        Backends: :guilabel:`usd`.

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
            self.lights[index].GetLengthAttr().Set(lengths[0 if lengths.shape[0] == 1 else i].item())

    def get_lengths(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the lengths (in the local X axis) of the prims.

        Backends: :guilabel:`usd`.

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
            data[i][0] = self.lights[index].GetLengthAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_radii(
        self, radii: float | list | np.ndarray | wp.array, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the radii of the prims.

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
            self.lights[index].GetRadiusAttr().Set(radii[0 if radii.shape[0] == 1 else i].item())

    def get_radii(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the radii of the prims.

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
            data[i][0] = self.lights[index].GetRadiusAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_enabled_treat_as_lines(
        self, enabled: bool | list | np.ndarray | wp.array, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Enable or disable the treat as lines (effectively, a zero-radius cylinder) of the prims.

        Backends: :guilabel:`usd`.

        .. warning::

            Renderers that do not support non-area lighting can ignore this.

        Args:
            enabled: Boolean flags to enable/disable treat as lines (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # enable the treat as lines for all prims
            >>> prims.set_enabled_treat_as_lines([True])
            >>>
            >>> # disable the treat as lines for the first and last prims
            >>> prims.set_enabled_treat_as_lines([False], indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        enabled = ops_utils.place(enabled, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            self.lights[index].GetTreatAsLineAttr().Set(bool(enabled[0 if enabled.shape[0] == 1 else i].item()))

    def get_enabled_treat_as_lines(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the enabled state of the treat as lines (effectively, a zero-radius cylinder) of the prims.

        Backends: :guilabel:`usd`.

        .. warning::

            Renderers that do not support non-area lighting can ignore this.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Boolean flags indicating if treat as lines is enabled (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the treat as lines enabled state of all prims after enabling it for the second prim
            >>> prims.set_enabled_treat_as_lines([True], indices=[1])
            >>> print(prims.get_enabled_treat_as_lines())
            [[False]
             [ True]
             [False]]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        enabled = np.zeros((indices.shape[0], 1), dtype=np.bool_)
        for i, index in enumerate(indices.numpy()):
            enabled[i] = self.lights[index].GetTreatAsLineAttr().Get()
        return ops_utils.place(enabled, device=self._device)

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
            >>> result = CylinderLight.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [False  True]
        """
        stage = stage_utils.get_current_stage(backend="usd")
        return ops_utils.place(
            [
                (stage.GetPrimAtPath(item) if isinstance(item, str) else item).IsA(UsdLux.CylinderLight)
                for item in (paths if isinstance(paths, (list, tuple)) else [paths])
            ],
            dtype=wp.bool,
            device="cpu",
        )

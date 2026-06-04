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

"""Camera module."""

from __future__ import annotations

from typing import Any, Literal

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Usd, UsdGeom


class Camera(XformPrim):
    """Create or wrap USD Camera prims with optical properties defined by common attributes.

    .. note::

        This class creates or wraps (one of both) USD Camera prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the USD Camera prims.
        * If the prim paths do not exist, USD Camera prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to existing or non-existing (one of both) USD prims.
            Can include regular expressions for matching multiple prims.
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
        AssertionError: If wrapped prims are not USD Camera.
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import Camera
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create cameras at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = Camera(paths)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # XformPrim
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
    ) -> None:
        self._geoms = []
        stage = stage_utils.get_current_stage(backend="usd")
        existent_paths, nonexistent_paths = XformPrim.resolve_paths(paths)
        # get cameras
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                prim = stage.GetPrimAtPath(path)
                assert prim.IsA(UsdGeom.Camera), f"The wrapped prim at path {path} is not a USD Camera"
                self._geoms.append(UsdGeom.Camera(prim))
        # create cameras
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                self._geoms.append(UsdGeom.Camera.Define(stage, path))
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

    @staticmethod
    def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array:
        """Check if the prims at the given paths are valid for creating Camera instances of this type.

        Backends: :guilabel:`usd`.

        .. warning::

            Since this method is static, the output is always on the CPU.

        Args:
            paths: Prim paths (or prims) to check for.

        Returns:
            Boolean flags indicating if the prims are valid for creating Camera instances.

        Example:

        .. code-block:: python

            >>> # check if the following prims at paths are valid for creating instances
            >>> result = Camera.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [False  True]
        """
        stage = stage_utils.get_current_stage(backend="usd")
        return ops_utils.place(
            [
                (stage.GetPrimAtPath(item) if isinstance(item, str) else item).IsA(UsdGeom.Camera)
                for item in (paths if isinstance(paths, (list, tuple)) else [paths])
            ],
            dtype=wp.bool,
            device="cpu",
        )

    """
    Methods.
    """

    def set_focal_lengths(
        self,
        focal_lengths: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the perspective focal lengths of the prims.

        Backends: :guilabel:`usd`.

        Args:
            focal_lengths: Focal lengths (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same focal lengths for all prims
            >>> prims.set_focal_lengths(focal_lengths=[65])
            >>>
            >>> # set only the focal length for the second prim
            >>> prims.set_focal_lengths(focal_lengths=[75], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        focal_lengths = ops_utils.place(focal_lengths, device="cpu").numpy().reshape((-1, 1)) * 10.0
        for i, index in enumerate(indices.numpy()):
            self._geoms[index].GetFocalLengthAttr().Set(focal_lengths[0 if focal_lengths.shape[0] == 1 else i].item())

    def get_focal_lengths(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the perspective focal lengths of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The focal lengths (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the focal lengths of all prims
            >>> focal_lengths = prims.get_focal_lengths()
            >>> focal_lengths.shape
            (3, 1)
            >>>
            >>> # get the focal lengths of the first and last prims
            >>> focal_lengths = prims.get_focal_lengths(indices=[0, 2])
            >>> focal_lengths.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self._geoms[index].GetFocalLengthAttr().Get()
        return ops_utils.place(data / 10.0, device=self._device)

    def set_focus_distances(
        self,
        focus_distances: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the focus distances (distance from the cameras to the focus planes) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            focus_distances: Focus distances (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same focus distances for all prims
            >>> prims.set_focus_distances(focus_distances=[65])
            >>>
            >>> # set only the focus distance for the second prim
            >>> prims.set_focus_distances(focus_distances=[75], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        focus_distances = ops_utils.place(focus_distances, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            self._geoms[index].GetFocusDistanceAttr().Set(
                focus_distances[0 if focus_distances.shape[0] == 1 else i].item()
            )

    def get_focus_distances(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the focus distances (distance from the cameras to the focus planes) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The focus distances (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the focus distances of all prims
            >>> focus_distances = prims.get_focus_distances()
            >>> focus_distances.shape
            (3, 1)
            >>>
            >>> # get the focus distances of the first and last prims
            >>> focus_distances = prims.get_focus_distances(indices=[0, 2])
            >>> focus_distances.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self._geoms[index].GetFocusDistanceAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_stereo_roles(
        self,
        roles: Literal["mono", "left", "right"] | list[Literal["mono", "left", "right"]],
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the stereo roles for the prims.

        Backends: :guilabel:`usd`.

        If different from ``"mono"``, the camera is intended to be the ``"left"`` or ``"right"`` camera of a stereo setup.

        Args:
            roles: Stereo roles (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the stereo roles for all prims to 'mono'
            >>> prims.set_stereo_roles(["mono"])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        roles = [roles] if isinstance(roles, str) else roles
        roles = np.broadcast_to(np.array(roles, dtype=object), (indices.shape[0],))
        for i, index in enumerate[Any](indices.numpy()):
            self._geoms[index].GetStereoRoleAttr().Set(roles[i])

    def get_stereo_roles(
        self, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> list[Literal["mono", "left", "right"]]:
        """Get the stereo roles for the prims.

        Backends: :guilabel:`usd`.

        If different from ``"mono"``, the camera is intended to be the ``"left"`` or ``"right"`` camera of a stereo setup.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List of stereo roles (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the stereo roles of all prims after setting them to different roles
            >>> prims.set_stereo_roles(["mono", "left", "right"])
            >>> prims.get_stereo_roles()
            ['mono', 'left', 'right']
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        roles = np.empty((indices.shape[0],), dtype=object)
        for i, index in enumerate(indices.numpy()):
            roles[i] = self._geoms[index].GetStereoRoleAttr().Get()
        return roles.tolist()

    def set_fstops(
        self,
        fstops: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the lens apertures (f-stop values that control the amount of light passing through the lens) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            fstops: Lens apertures (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same lens apertures for all prims
            >>> prims.set_fstops(fstops=[2.8])
            >>>
            >>> # set only the lens aperture for the second prim
            >>> prims.set_fstops(fstops=[4.0], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        fstops = ops_utils.place(fstops, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            self._geoms[index].GetFStopAttr().Set(fstops[0 if fstops.shape[0] == 1 else i].item())

    def get_fstops(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the lens apertures (f-stop values that control the amount of light passing through the lens) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The lens apertures (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the lens apertures of all prims
            >>> fstops = prims.get_fstops()
            >>> fstops.shape
            (3, 1)
            >>>
            >>> # get the lens apertures of the first and last prims
            >>> fstops = prims.get_fstops(indices=[0, 2])
            >>> fstops.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self._geoms[index].GetFStopAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_apertures(
        self,
        horizontal_apertures: float | list | np.ndarray | wp.array = None,
        vertical_apertures: float | list | np.ndarray | wp.array = None,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the horizontal and vertical apertures of the prims.

        Backends: :guilabel:`usd`.

        Args:
            horizontal_apertures: Horizontal apertures (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            vertical_apertures: Vertical apertures (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither horizontal_apertures nor vertical_apertures are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same apertures for all prims
            >>> prims.set_apertures(horizontal_apertures=[20.955], vertical_apertures=[15.2908])
            >>>
            >>> # set only the horizontal aperture for the second prim
            >>> prims.set_apertures(horizontal_apertures=[24.0], indices=[1])
        """
        assert (
            horizontal_apertures is not None or vertical_apertures is not None
        ), "Both 'horizontal_apertures' and 'vertical_apertures' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if horizontal_apertures is not None:
            horizontal_apertures = ops_utils.place(horizontal_apertures, device="cpu").numpy().reshape((-1, 1)) * 10.0
        if vertical_apertures is not None:
            vertical_apertures = ops_utils.place(vertical_apertures, device="cpu").numpy().reshape((-1, 1)) * 10.0
        for i, index in enumerate(indices.numpy()):
            if horizontal_apertures is not None:
                self._geoms[index].GetHorizontalApertureAttr().Set(
                    horizontal_apertures[0 if horizontal_apertures.shape[0] == 1 else i].item()
                )
            if vertical_apertures is not None:
                self._geoms[index].GetVerticalApertureAttr().Set(
                    vertical_apertures[0 if vertical_apertures.shape[0] == 1 else i].item()
                )

    def get_apertures(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> tuple[wp.array, wp.array]:
        """Get the horizontal and vertical apertures of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The horizontal apertures (shape ``(N, 1)``).
            2) The vertical apertures (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the apertures of all prims
            >>> horizontal_apertures, vertical_apertures = prims.get_apertures()
            >>> horizontal_apertures.shape, vertical_apertures.shape
            ((3, 1), (3, 1))
            >>>
            >>> # get the apertures of the first and last prims
            >>> horizontal_apertures, vertical_apertures = prims.get_apertures(indices=[0, 2])
            >>> horizontal_apertures.shape, vertical_apertures.shape
            ((2, 1), (2, 1))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        horizontal_apertures = np.zeros((indices.shape[0], 1), dtype=np.float32)
        vertical_apertures = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            horizontal_apertures[i][0] = self._geoms[index].GetHorizontalApertureAttr().Get()
            vertical_apertures[i][0] = self._geoms[index].GetVerticalApertureAttr().Get()
        return (
            ops_utils.place(horizontal_apertures / 10.0, device=self._device),
            ops_utils.place(vertical_apertures / 10.0, device=self._device),
        )

    def set_aperture_offsets(
        self,
        horizontal_offsets: float | list | np.ndarray | wp.array = None,
        vertical_offsets: float | list | np.ndarray | wp.array = None,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the horizontal and vertical aperture offsets of the prims.

        Backends: :guilabel:`usd`.

        Args:
            horizontal_offsets: Horizontal aperture offsets (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            vertical_offsets: Vertical aperture offsets (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither horizontal_offsets nor vertical_offsets are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same aperture offsets for all prims
            >>> prims.set_aperture_offsets(horizontal_offsets=[0.5], vertical_offsets=[0.3])
            >>>
            >>> # set only the horizontal aperture offset for the second prim
            >>> prims.set_aperture_offsets(horizontal_offsets=[0.2], indices=[1])
        """
        assert (
            horizontal_offsets is not None or vertical_offsets is not None
        ), "Both 'horizontal_offsets' and 'vertical_offsets' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if horizontal_offsets is not None:
            horizontal_offsets = ops_utils.place(horizontal_offsets, device="cpu").numpy().reshape((-1, 1)) * 10.0
        if vertical_offsets is not None:
            vertical_offsets = ops_utils.place(vertical_offsets, device="cpu").numpy().reshape((-1, 1)) * 10.0
        for i, index in enumerate(indices.numpy()):
            if horizontal_offsets is not None:
                self._geoms[index].GetHorizontalApertureOffsetAttr().Set(
                    horizontal_offsets[0 if horizontal_offsets.shape[0] == 1 else i].item()
                )
            if vertical_offsets is not None:
                self._geoms[index].GetVerticalApertureOffsetAttr().Set(
                    vertical_offsets[0 if vertical_offsets.shape[0] == 1 else i].item()
                )

    def get_aperture_offsets(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> tuple[wp.array, wp.array]:
        """Get the horizontal and vertical aperture offsets of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The horizontal aperture offsets (shape ``(N, 1)``).
            2) The vertical aperture offsets (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the aperture offsets of all prims
            >>> horizontal_offsets, vertical_offsets = prims.get_aperture_offsets()
            >>> horizontal_offsets.shape, vertical_offsets.shape
            ((3, 1), (3, 1))
            >>>
            >>> # get the aperture offsets of the first and last prims
            >>> horizontal_offsets, vertical_offsets = prims.get_aperture_offsets(indices=[0, 2])
            >>> horizontal_offsets.shape, vertical_offsets.shape
            ((2, 1), (2, 1))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        horizontal_offsets = np.zeros((indices.shape[0], 1), dtype=np.float32)
        vertical_offsets = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            horizontal_offsets[i][0] = self._geoms[index].GetHorizontalApertureOffsetAttr().Get()
            vertical_offsets[i][0] = self._geoms[index].GetVerticalApertureOffsetAttr().Get()
        return (
            ops_utils.place(horizontal_offsets / 10.0, device=self._device),
            ops_utils.place(vertical_offsets / 10.0, device=self._device),
        )

    def set_projections(
        self,
        projections: Literal["perspective", "orthographic"] | list[Literal["perspective", "orthographic"]],
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the projection types for the prims.

        Backends: :guilabel:`usd`.

        Args:
            projections: Projection types (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the projection types for all prims to 'perspective'
            >>> prims.set_projections(["perspective"])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        projections = [projections] if isinstance(projections, str) else projections
        projections = np.broadcast_to(np.array(projections, dtype=object), (indices.shape[0],))
        for i, index in enumerate[Any](indices.numpy()):
            self._geoms[index].GetProjectionAttr().Set(projections[i])

    def get_projections(
        self, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> list[Literal["perspective", "orthographic"]]:
        """Get the projection types for the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List of projection types (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the projection types of all prims after setting them to different types
            >>> prims.set_projections(["perspective", "orthographic", "perspective"])
            >>> prims.get_projections()
            ['perspective', 'orthographic', 'perspective']
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        projections = np.empty((indices.shape[0],), dtype=object)
        for i, index in enumerate(indices.numpy()):
            projections[i] = self._geoms[index].GetProjectionAttr().Get()
        return projections.tolist()

    def set_clipping_ranges(
        self,
        near_distances: float | list | np.ndarray | wp.array = None,
        far_distances: float | list | np.ndarray | wp.array = None,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the near and far clipping distance ranges of the prims.

        Backends: :guilabel:`usd`.

        Args:
            near_distances: Near clipping distances (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            far_distances: Far clipping distances (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither near_distances nor far_distances are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same clipping ranges for all prims
            >>> prims.set_clipping_ranges(near_distances=[0.1], far_distances=[1000.0])
            >>>
            >>> # set only the near clipping distance for the second prim
            >>> prims.set_clipping_ranges(near_distances=[0.5], indices=[1])
        """
        assert (
            near_distances is not None or far_distances is not None
        ), "Both 'near_distances' and 'far_distances' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if near_distances is not None:
            near_distances = ops_utils.place(near_distances, device="cpu").numpy().reshape((-1, 1))
        if far_distances is not None:
            far_distances = ops_utils.place(far_distances, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            clipping_range = self._geoms[index].GetClippingRangeAttr().Get()
            if near_distances is not None:
                clipping_range[0] = near_distances[0 if near_distances.shape[0] == 1 else i].item()
            if far_distances is not None:
                clipping_range[1] = far_distances[0 if far_distances.shape[0] == 1 else i].item()
            self._geoms[index].GetClippingRangeAttr().Set(clipping_range)

    def get_clipping_ranges(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> tuple[wp.array, wp.array]:
        """Get the near and far clipping distance ranges of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The near clipping distances (shape ``(N, 1)``).
            2) The far clipping distances (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the clipping ranges of all prims
            >>> near_distances, far_distances = prims.get_clipping_ranges()
            >>> near_distances.shape, far_distances.shape
            ((3, 1), (3, 1))
            >>>
            >>> # get the clipping ranges of the first and last prims
            >>> near_distances, far_distances = prims.get_clipping_ranges(indices=[0, 2])
            >>> near_distances.shape, far_distances.shape
            ((2, 1), (2, 1))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        near_distances = np.zeros((indices.shape[0], 1), dtype=np.float32)
        far_distances = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            clipping_range = self._geoms[index].GetClippingRangeAttr().Get()
            near_distances[i][0] = clipping_range[0]
            far_distances[i][0] = clipping_range[1]
        return ops_utils.place(near_distances, device=self._device), ops_utils.place(far_distances, device=self._device)

    def set_shutter_times(
        self,
        open_times: float | list | np.ndarray | wp.array = None,
        close_times: float | list | np.ndarray | wp.array = None,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the frame relative shutter open and close times of the prims.

        Backends: :guilabel:`usd`.

        Args:
            open_times: Frame relative shutter open times (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            close_times: Frame relative shutter close times (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither open_times nor close_times are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same shutter properties for all prims
            >>> prims.set_shutter_times(open_times=[0.0], close_times=[0.5])
            >>>
            >>> # set only the shutter open time for the second prim
            >>> prims.set_shutter_times(open_times=[0.1], indices=[1])
        """
        assert (
            open_times is not None or close_times is not None
        ), "Both 'open_times' and 'close_times' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if open_times is not None:
            open_times = ops_utils.place(open_times, device="cpu").numpy().reshape((-1, 1))
        if close_times is not None:
            close_times = ops_utils.place(close_times, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            if open_times is not None:
                self._geoms[index].GetShutterOpenAttr().Set(
                    float(open_times[0 if open_times.shape[0] == 1 else i].item())
                )
            if close_times is not None:
                self._geoms[index].GetShutterCloseAttr().Set(
                    float(close_times[0 if close_times.shape[0] == 1 else i].item())
                )

    def get_shutter_times(
        self,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> tuple[wp.array, wp.array]:
        """Get the frame relative shutter open and close times of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The shutter open times (shape ``(N, 1)``).
            2) The shutter close times (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the shutter properties of all prims
            >>> open_times, close_times = prims.get_shutter_times()
            >>> open_times.shape, close_times.shape
            ((3, 1), (3, 1))
            >>>
            >>> # get the shutter properties of the first and last prims
            >>> open_times, close_times = prims.get_shutter_times(indices=[0, 2])
            >>> open_times.shape, close_times.shape
            ((2, 1), (2, 1))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        open_times = np.zeros((indices.shape[0], 1), dtype=np.float32)
        close_times = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            open_times[i][0] = self._geoms[index].GetShutterOpenAttr().Get()
            close_times[i][0] = self._geoms[index].GetShutterCloseAttr().Get()
        return ops_utils.place(open_times, device=self._device), ops_utils.place(close_times, device=self._device)

    def enforce_square_pixels(
        self,
        resolutions: list | np.ndarray | wp.array,
        *,
        modes: Literal["horizontal", "vertical"] | list[Literal["horizontal", "vertical"]] = "horizontal",
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Enforce square pixels by updating the apertures to be in sync with the aspect ratio.

        Backends: :guilabel:`usd`.

        Args:
            resolutions: Resolutions to enforce square pixels (following OpenCV/NumPy convention: ``(height, width)``).
            modes: Modes to enforce square pixels.
                If ``"horizontal"``, the horizontal aperture is used as reference and the vertical aperture is updated.
                If ``"vertical"``, the vertical aperture is used as reference and the horizontal aperture is updated.
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            ValueError: Invalid mode.

        Example:

        .. code-block:: python

            >>> # enforce square pixels for all prims
            >>> prims.enforce_square_pixels(resolutions=[480, 640])
            >>>
            >>> # enforce square pixels for the first and last prims (using vertical mode)
            >>> prims.enforce_square_pixels(resolutions=[480, 640], modes=["vertical"], indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        modes = [modes] if isinstance(modes, str) else modes
        modes = np.broadcast_to(np.array(modes, dtype=object), (indices.shape[0],))
        resolutions = ops_utils.broadcast_to(resolutions, shape=(indices.shape[0], 2), device="cpu").numpy()
        horizontal_apertures, vertical_apertures = self.get_apertures(indices=indices)
        horizontal_apertures = horizontal_apertures.numpy()
        vertical_apertures = vertical_apertures.numpy()
        for i, index in enumerate(indices.numpy()):
            aspect_ratio = resolutions[i][1] / float(resolutions[i][0])
            horizontal_aperture = horizontal_apertures[i][0]
            vertical_aperture = vertical_apertures[i][0]
            if modes[i] == "horizontal":
                expected_vertical_aperture = horizontal_aperture / aspect_ratio
                if not np.isclose(vertical_aperture, expected_vertical_aperture, rtol=1e-5, atol=1e-8):
                    self._geoms[index].GetVerticalApertureAttr().Set(expected_vertical_aperture * 10)
            elif modes[i] == "vertical":
                expected_horizontal_aperture = vertical_aperture * aspect_ratio
                if not np.isclose(horizontal_aperture, expected_horizontal_aperture, rtol=1e-5, atol=1e-8):
                    self._geoms[index].GetHorizontalApertureAttr().Set(expected_horizontal_aperture * 10)
            else:
                raise ValueError(f"Invalid mode: {modes[i]}. Valid modes are 'horizontal' and 'vertical'")

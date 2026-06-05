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

"""High level wrapper for creating/encapsulating USD Mesh (points that are connected into edges and faces) prims."""

from __future__ import annotations

from typing import Literal

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import matplotlib.colors
import numpy as np
import omni.kit.commands
import warp as wp
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Gf, Usd, UsdGeom, Vt


class Mesh(XformPrim):
    """High level class for creating/wrapping USD Mesh (points that are connected into edges and faces) prims.

    .. note::

        This class creates or wraps (one of both) USD Mesh prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the USD Mesh prims.
        * If the prim paths do not exist, USD Mesh prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to existing or non-existing (one of both) USD prims.
            Can include regular expressions for matching multiple prims.
        primitives: Primitives to be created (shape ``(N,)``). If not defined, an empty mesh is created.
            Primitives are used only for *creating* operations. For *wrapping* operations, primitives are ignored.
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
        AssertionError: If wrapped prims are not USD Mesh.
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import Mesh
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create red Plane meshes at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = Mesh(paths, primitives="Plane", colors="red")  # doctest: +NO_CHECK

    Example (create mesh from external package: trimesh):

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import Mesh
        >>> import trimesh  # doctest: +SKIP
        >>>
        >>> # icosahedron mesh (20 faces)
        >>> mesh = trimesh.creation.icosahedron()  # doctest: +SKIP
        >>>
        >>> # create a yellow USD Mesh from the icosahedron defined by trimesh
        >>> mesh_prim = Mesh("/World/icosahedron", colors=(1.0, 1.0, 0.0))  # doctest: +SKIP
        >>> mesh_prim.set_points([mesh.vertices])  # doctest: +SKIP
        >>> mesh_prim.set_face_specs(
        ...    vertex_indices=[mesh.faces.flatten()],
        ...    vertex_counts=[[3] * len(mesh.faces)]
        ... )  # doctest: +SKIP
        >>> mesh_prim.set_subdivision_specs(subdivision_schemes=["bilinear"])  # doctest: +SKIP
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # Mesh
        primitives: (
            Literal["Cone", "Cube", "Cylinder", "Disk", "Plane", "Sphere", "Torus"]
            | list[Literal["Cone", "Cube", "Cylinder", "Disk", "Plane", "Sphere", "Torus"]]
            | None
        ) = None,
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
        existent_paths, nonexistent_paths = XformPrim.resolve_paths(paths)
        # get meshes
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                prim = stage.GetPrimAtPath(path)
                assert prim.IsA(UsdGeom.Mesh), f"The wrapped prim at path {path} is not a USD Mesh"
                self._geoms.append(UsdGeom.Mesh(prim))
        # create meshes
        else:
            # check primitives
            if primitives is None:
                primitives = [None] * len(nonexistent_paths)
            else:
                primitives = [primitives] if isinstance(primitives, str) else primitives
                for primitive in primitives:
                    assert primitive in [
                        "Cone",
                        "Cube",
                        "Cylinder",
                        "Disk",
                        "Plane",
                        "Sphere",
                        "Torus",
                    ], f"Invalid primitive: {primitive}"
            if len(primitives) == 1:
                primitives = primitives * len(nonexistent_paths)
            assert len(primitives) == len(
                nonexistent_paths
            ), f"The number of primitives ({len(primitives)}) and paths ({len(nonexistent_paths)}) must be equal"
            # create meshes (empty or from primitives)
            paths = nonexistent_paths
            for path, primitive in zip(nonexistent_paths, primitives):
                # - empty
                if primitive is None:
                    UsdGeom.Mesh.Define(stage, path)
                # - from primitives
                else:
                    omni.kit.commands.execute(
                        "CreateMeshPrimWithDefaultXformCommand", prim_type=primitive, prim_path=path
                    )
                self._geoms.append(UsdGeom.Mesh(stage.GetPrimAtPath(path)))
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
        if colors is not None:
            self.set_display_colors(colors)

    """
    Properties.
    """

    @property
    def geoms(self) -> list[UsdGeom.Mesh]:
        """USD Mesh prims encapsulated by the wrapper.

        Returns:
            List of USD Mesh prims.

        Example:

        .. code-block:: python

            >>> prims.geoms
            [UsdGeom.Mesh(Usd.Prim(</World/prim_0>)),
             UsdGeom.Mesh(Usd.Prim(</World/prim_1>)),
             UsdGeom.Mesh(Usd.Prim(</World/prim_2>))]
        """
        return self._geoms

    @property
    def num_faces(self) -> list[int]:
        """Number of faces of the meshes.

        Returns:
            List of number of faces as defined by the size of the ``faceVertexCounts`` array.

        Example:

        .. code-block:: python

            >>> prims.num_faces
            [1, 1, 1]
        """
        return [geom.GetFaceCount() for geom in self.geoms]

    """
    Static methods.
    """

    @staticmethod
    def update_extents(geoms: list[UsdGeom.Mesh]) -> None:
        """Update the gprims' extents.

        Backends: :guilabel:`usd`.

        Args:
            geoms: Geoms to process.
        """
        # USD API
        for geom in geoms:
            extent = UsdGeom.Boundable.ComputeExtentFromPlugins(
                UsdGeom.Boundable(geom.GetPrim()), Usd.TimeCode.Default()
            )
            if extent:
                geom.GetExtentAttr().Set(extent)

    @staticmethod
    def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array:
        """Check if the prims at the given paths are valid for creating Mesh instances of this type.

        Backends: :guilabel:`usd`.

        .. warning::

            Since this method is static, the output is always on the CPU.

        Args:
            paths: Prim paths (or prims) to check for.

        Returns:
            Boolean flags indicating if the prims are valid for creating Mesh instances.

        Example:

        .. code-block:: python

            >>> # check if the following prims at paths are valid for creating instances
            >>> result = Mesh.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [False  True]
        """
        stage = stage_utils.get_current_stage(backend="usd")
        return ops_utils.place(
            [
                (stage.GetPrimAtPath(item) if isinstance(item, str) else item).IsA(UsdGeom.Mesh)
                for item in (paths if isinstance(paths, (list, tuple)) else [paths])
            ],
            dtype=wp.bool,
            device="cpu",
        )

    @staticmethod
    def fetch_instances(paths: str | Usd.Prim | list[str | Usd.Prim]) -> list[Mesh | None]:
        """Fetch instances of Mesh from prims (or prim paths) at the given paths.

        Backends: :guilabel:`usd`.

        Args:
            paths: Prim paths (or prims) to get Mesh instances from.

        Returns:
            List of Mesh instances or ``None`` if the prim is not a supported Mesh type.

        Example:

        .. code-block:: python

            >>> import isaacsim.core.experimental.utils.stage as stage_utils
            >>> from isaacsim.core.experimental.objects import Mesh
            >>>
            >>> # given a USD stage with the prims at paths /World, /World/A (Mesh)
            >>> stage_utils.define_prim(f"/World/A", "Mesh")  # doctest: +NO_CHECK
            >>>
            >>> # fetch mesh instances
            >>> Mesh.fetch_instances(["/World", "/World/A"])
            [None, <isaacsim.core.experimental.objects.impl.mesh.Mesh object at 0x...>]
        """
        instances = []
        stage = stage_utils.get_current_stage(backend="usd")
        for item in paths if isinstance(paths, (list, tuple)) else [paths]:
            prim = stage.GetPrimAtPath(item) if isinstance(item, str) else item
            if Mesh.are_of_type(prim).numpy().item():
                instances.append(Mesh(prim.GetPath().pathString))
            else:
                instances.append(None)
        return instances

    """
    Methods.
    """

    def set_display_colors(
        self,
        colors: str | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the display colors of the prims.

        Backends: :guilabel:`usd`.

        Args:
            colors: Normalized RGB display colors (shape ``(N, 3)``) or case-insensitive string representations.
                Supported string representations include hex codes and X11/CSS4 color names without spaces,
                as well as any other format supported by Matplotlib. Alpha channel is ignored for string representations.
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            ValueError: Invalid string representation format for the colors.

        Example:

        .. code-block:: python

            >>> # set same colors (red) for all prims
            >>> prims.set_display_colors([1.0, 0.0, 0.0])
            >>>
            >>> # set only the color (green) for the second prim
            >>> prims.set_display_colors("#00ff00", indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if isinstance(colors, str):
            colors = matplotlib.colors.to_rgb(colors)
        elif isinstance(colors, (list, tuple)):
            colors = [matplotlib.colors.to_rgb(color) if isinstance(color, str) else color for color in colors]
        colors = ops_utils.broadcast_to(colors, shape=(indices.shape[0], 3), dtype=wp.float32, device="cpu").numpy()
        for i, index in enumerate(indices.numpy()):
            self.geoms[index].GetDisplayColorAttr().Set([Gf.Vec3f(*colors[i].tolist())])

    def set_points(
        self, points: list[list | np.ndarray | wp.array], *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the mesh points (in local space) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            points: List (shape ``(N,)``) of mesh points (shape ``(number of points, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the points (fold each face diagonally) for the second prim
            >>> prims.set_points([[(-0.5, -0.5, 0.0), (0.5, -0.5, 1.0), (-0.5, 0.5, 1.0), (0.5, 0.5, 0.0)]], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        for i, index in enumerate(indices.numpy()):
            data = ops_utils.place(points[0 if len(points) == 1 else i], device="cpu").numpy().reshape((-1, 3))
            self.geoms[index].GetPointsAttr().Set(Vt.Vec3fArray(data.tolist()))

    def get_points(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> list[wp.array]:
        """Get the mesh points (in local space) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List (shape ``(N,)``) of mesh points (shape ``(number of points, 3)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the points of all prims
            >>> points = prims.get_points()
            >>> points[0].shape  # points' shape of the first prim
            (4, 3)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = []
        for index in indices.numpy():
            points = self.geoms[index].GetPointsAttr().Get()
            if points is None:
                points = []
            data.append(ops_utils.place(np.array(points), device=self._device))
        return data

    def set_normals(
        self, normals: list[list | np.ndarray | wp.array], *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the mesh normals (object-space orientation for individual points) of the prims.

        Backends: :guilabel:`usd`.

        .. note::

            Normals should not be authored on any USD Mesh that is subdivided,
            since the subdivision algorithm will define its own normals.

        Args:
            normals: List (shape ``(N,)``) of mesh normals (shape ``(number of normals, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # clear the normals for the second prim
            >>> prims.set_normals([[]], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        for i, index in enumerate(indices.numpy()):
            data = ops_utils.place(normals[0 if len(normals) == 1 else i], device="cpu").numpy().reshape((-1, 3))
            self.geoms[index].GetNormalsAttr().Set(Vt.Vec3fArray(data.tolist()))

    def get_normals(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> list[wp.array]:
        """Get the mesh normals (object-space orientation for individual points) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List (shape ``(N,)``) of mesh normals (shape ``(number of normals, 3)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the normals of all prims
            >>> normals = prims.get_normals()
            >>> normals[0].shape  # normals' shape of the first prim
            (4, 3)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = []
        for index in indices.numpy():
            data.append(ops_utils.place(np.array(self.geoms[index].GetNormalsAttr().Get()), device=self._device))
        return data

    def set_face_specs(
        self,
        vertex_indices: list[list | np.ndarray | wp.array] | None = None,
        vertex_counts: list[list | np.ndarray | wp.array] | None = None,
        varying_linear_interpolations: (
            list[Literal["none", "cornersOnly", "cornersPlus1", "cornersPlus2", "boundaries", "all"]] | None
        ) = None,
        hole_indices: list[list | np.ndarray | wp.array] | None = None,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the face (3D model flat surface) specifications of the prims.

        Backends: :guilabel:`usd`.

        Args:
            vertex_indices: List (shape ``(N,)``) of indices (of points) of each vertex of each face
                (shape ``(sum of all elements of the vertexCounts,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            vertex_counts: List (shape ``(N,)``) of number of vertices in each face, which is also the number of consecutive
                indices in ``faceVertexIndices`` that define the face (shape ``(number of faces,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            varying_linear_interpolations: Face-varying interpolation rules in the interior of face-varying regions
                (smooth or linear) and at the boundaries for subdivision surfaces (shape ``(N,)``).
            hole_indices: List (shape ``(N,)``) of indices of all faces that should be treated as holes, e.g.:
                made invisible (shape ``(up to the number of faces,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither vertex_indices, vertex_counts, varying_linear_interpolations nor
                hole_indices are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the face specifications (2 triangles) for the second prim
            >>> prims.set_face_specs(vertex_indices=[[0, 1, 3, 0, 2, 3]], vertex_counts=[[3, 3]], indices=[1])
        """
        assert (
            vertex_indices is not None
            or vertex_counts is not None
            or varying_linear_interpolations is not None
            or hole_indices is not None
        ), (
            "All 'vertex_indices', 'vertex_counts', 'varying_linear_interpolations' and 'hole_indices' are not defined. "
            "Define at least one of them"
        )
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if vertex_indices is not None:
            vertex_indices = [ops_utils.place(item, device="cpu").numpy().flatten() for item in vertex_indices]
        if vertex_counts is not None:
            vertex_counts = [ops_utils.place(item, device="cpu").numpy().flatten() for item in vertex_counts]
        if varying_linear_interpolations is not None:
            varying_linear_interpolations = np.broadcast_to(
                np.array(varying_linear_interpolations, dtype=object), (indices.shape[0],)
            )
        if hole_indices is not None:
            hole_indices = [ops_utils.place(item, device="cpu").numpy().flatten() for item in hole_indices]
        for i, index in enumerate(indices.numpy()):
            geom = self.geoms[index]
            if vertex_indices is not None:
                data = vertex_indices[0 if len(vertex_indices) == 1 else i]
                geom.GetFaceVertexIndicesAttr().Set(Vt.IntArray(data.tolist()))
            if vertex_counts is not None:
                data = vertex_counts[0 if len(vertex_counts) == 1 else i]
                geom.GetFaceVertexCountsAttr().Set(Vt.IntArray(data.tolist()))
            if varying_linear_interpolations is not None:
                geom.GetFaceVaryingLinearInterpolationAttr().Set(varying_linear_interpolations[i])
            if hole_indices is not None:
                data = hole_indices[0 if len(hole_indices) == 1 else i]
                geom.GetHoleIndicesAttr().Set(Vt.IntArray(data.tolist()))

    def get_face_specs(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> tuple[
        list[wp.array],
        list[wp.array],
        list[Literal["none", "cornersOnly", "cornersPlus1", "cornersPlus2", "boundaries", "all"]],
        list[wp.array],
    ]:
        """Get the face (3D model flat surface) specifications of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Four-elements tuple. 1) List (shape ``(N,)``) of indices (of points) of each vertex
            of each face (shape ``(sum of all elements of the vertexCounts,)``).
            2) List (shape ``(N,)``) of number of vertices in each face (shape ``(number of faces,)``).
            3) List (shape ``(N,)``) of face-varying interpolation rules.
            4) List (shape ``(N,)``) of indices of all face holes (shape ``(up to the number of faces,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the face specifications of all prims
            >>> face_indices, face_counts, face_interpolations, face_holes = prims.get_face_specs()
            >>> # print the first prim's face specifications (Plane mesh composed of 4 points with 1 face)
            >>> print(face_indices[0])  # points indices of the face
            [0 1 3 2]
            >>> print(face_counts[0])  # number of points in the face
            [4]
            >>> face_interpolations[0]  # face-varying interpolation rule
            'cornersPlus1'
            >>> print(face_holes[0])  # indices of the face holes (empty: no face holes)
            []
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        vertex_indices, vertex_counts, varying_linear_interpolation, hole_indices = [], [], [], []
        for index in indices.numpy():
            geom = self.geoms[index]
            vertex_indices.append(ops_utils.place(np.array(geom.GetFaceVertexIndicesAttr().Get()), device=self._device))
            vertex_counts.append(ops_utils.place(np.array(geom.GetFaceVertexCountsAttr().Get()), device=self._device))
            varying_linear_interpolation.append(geom.GetFaceVaryingLinearInterpolationAttr().Get())
            hole_indices.append(ops_utils.place(np.array(geom.GetHoleIndicesAttr().Get()), device=self._device))
        return vertex_indices, vertex_counts, varying_linear_interpolation, hole_indices

    def set_crease_specs(
        self,
        crease_indices: list[list | np.ndarray | wp.array],
        crease_lengths: list[list | np.ndarray | wp.array],
        crease_sharpnesses: list[list | np.ndarray | wp.array],
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the crease (set of adjacent sharpened edges) specifications of the prims.

        Backends: :guilabel:`usd`.

        Args:
            crease_indices: List (shape ``(N,)``) of indices of points grouped into sets of successive pairs that identify
                edges to be creased (shape ``(sum of all elements of the creaseLengths,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            crease_lengths: List (shape ``(N,)``) of number of points of each crease, whose indices are successively laid
                out in the ``creaseIndices`` attribute (shape ``(number of creases,)``).
                Since each crease must be at least one edge long, each element of this array must be at least two.
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            crease_sharpnesses: List (shape ``(N,)``) of per-crease or per-edge sharpness values
                (shape ``(number of creases,)`` or ``(sum over all X of (creaseLengths[X] - 1),)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: If the sum of the elements of ``crease_lengths`` is not equal to the number of elements
                of ``crease_indices``.
            AssertionError: If the number of elements of ``crease_sharpnesses`` is not equal to the number of elements
                of ``crease_lengths`` or the sum over all X of ``(crease_lengths[X] - 1)``.

        Example:

        .. code-block:: python

            >>> # set the crease specifications (mesh points: 0, 3) for the second prim
            >>> prims.set_crease_specs(
            ...     crease_indices=[[0, 3]],
            ...     crease_lengths=[[2]],
            ...     crease_sharpnesses=[[10.0]],
            ...     indices=[1]
            ... )
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        crease_indices = [ops_utils.place(item, device="cpu").numpy().flatten() for item in crease_indices]
        crease_lengths = [ops_utils.place(item, device="cpu").numpy().flatten() for item in crease_lengths]
        crease_sharpnesses = [ops_utils.place(item, device="cpu").numpy().flatten() for item in crease_sharpnesses]
        for i, index in enumerate(indices.numpy()):
            crease_index = crease_indices[0 if len(crease_indices) == 1 else i]
            crease_length = crease_lengths[0 if len(crease_lengths) == 1 else i]
            crease_sharpness = crease_sharpnesses[0 if len(crease_sharpnesses) == 1 else i]
            assert np.sum(crease_length) == crease_index.shape[0], (
                f"The sum of the elements of 'crease_lengths' ({np.sum(crease_length)}) (at index {i}) "
                f"is not equal to the number of elements of 'crease_indices' ({crease_index.shape[0]})"
            )
            assert crease_sharpness.shape[0] == crease_length.shape[0] or crease_sharpness.shape[0] == np.sum(
                crease_length - 1
            ), (
                f"The number of elements of 'crease_sharpnesses' ({crease_sharpness.shape[0]}) (at index {i}) "
                f"is not equal to the number of elements of 'crease_lengths' ({crease_length.shape[0]}) or "
                f"the sum over all X of (crease_lengths[X] - 1) ({np.sum(crease_length - 1)})"
            )
            geom = self.geoms[index]
            geom.GetCreaseIndicesAttr().Set(Vt.IntArray(crease_index.tolist()))
            geom.GetCreaseLengthsAttr().Set(Vt.IntArray(crease_length.tolist()))
            geom.GetCreaseSharpnessesAttr().Set(Vt.FloatArray(crease_sharpness.tolist()))

    def get_crease_specs(
        self, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> tuple[list[wp.array], list[wp.array], list[wp.array]]:
        """Get the crease (set of adjacent sharpened edges) specifications of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Three-elements tuple.
            1) List (shape ``(N,)``) of indices of points (shape ``(sum of all elements of the creaseLengths,)``).
            2) List (shape ``(N,)``) of number of points of each crease (shape ``(number of creases,)``).
            3) List (shape ``(N,)``) of sharpness values (shape ``(number of creases,)`` or
            ``(sum over all X of (creaseLengths[X] - 1),)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the crease specifications of all prims
            >>> crease_indices, crease_lengths, crease_sharpnesses = prims.get_crease_specs()
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        crease_indices, crease_lengths, crease_sharpnesses = [], [], []
        for index in indices.numpy():
            geom = self.geoms[index]
            crease_indices.append(ops_utils.place(np.array(geom.GetCreaseIndicesAttr().Get()), device=self._device))
            crease_lengths.append(ops_utils.place(np.array(geom.GetCreaseLengthsAttr().Get()), device=self._device))
            crease_sharpnesses.append(
                ops_utils.place(np.array(geom.GetCreaseSharpnessesAttr().Get()), device=self._device)
            )
        return crease_indices, crease_lengths, crease_sharpnesses

    def set_corner_specs(
        self,
        corner_indices: list[list | np.ndarray | wp.array],
        corner_sharpnesses: list[list | np.ndarray | wp.array],
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the corner specifications of the prims.

        Backends: :guilabel:`usd`.

        Args:
            corner_indices: List (shape ``(N,)``) of indices of points (shape ``(number of target points,)``) for which
                a corresponding sharpness value is specified in ``cornerSharpnesses``.
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            corner_sharpnesses: List (shape ``(N,)``) of sharpness values (shape ``(number of target points,)``)
                associated with a corresponding set of points specified in ``cornerIndices``.
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: If each respective pair of items (from ``corner_indices`` and ``corner_sharpnesses``)
                has a different shape.

        Example:

        .. code-block:: python

            >>> # set the corner specifications (mesh points: 1, 2) for the second prim
            >>> prims.set_corner_specs(corner_indices=[[1, 2]], corner_sharpnesses=[[5.0, 10.0]], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        corner_indices = [ops_utils.place(item, device="cpu").numpy().flatten() for item in corner_indices]
        corner_sharpnesses = [ops_utils.place(item, device="cpu").numpy().flatten() for item in corner_sharpnesses]
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        for i, index in enumerate(indices.numpy()):
            corner_index = corner_indices[0 if len(corner_indices) == 1 else i]
            corner_sharpness = corner_sharpnesses[0 if len(corner_sharpnesses) == 1 else i]
            assert corner_index.shape == corner_sharpness.shape, (
                f"Items from 'corner_indices' and 'corner_sharpnesses' (at index {i}) "
                f"have different shapes ({corner_index.shape} != {corner_sharpness.shape})"
            )
            geom = self.geoms[index]
            geom.GetCornerIndicesAttr().Set(Vt.IntArray(corner_index.tolist()))
            geom.GetCornerSharpnessesAttr().Set(Vt.FloatArray(corner_sharpness.tolist()))

    def get_corner_specs(
        self, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> tuple[list[wp.array], list[wp.array]]:
        """Get the corner specifications of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) List (shape ``(N,)``) of indices of points (shape ``(number of target points,)``).
            2) List (shape ``(N,)``) of sharpness values (shape ``(number of target points,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the corner specifications of all prims
            >>> corner_indices, corner_sharpnesses = prims.get_corner_specs()
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        corner_indices, corner_sharpnesses = [], []
        for index in indices.numpy():
            geom = self.geoms[index]
            corner_indices.append(ops_utils.place(np.array(geom.GetCornerIndicesAttr().Get()), device=self._device))
            corner_sharpnesses.append(
                ops_utils.place(np.array(geom.GetCornerSharpnessesAttr().Get()), device=self._device)
            )
        return corner_indices, corner_sharpnesses

    def set_subdivision_specs(
        self,
        subdivision_schemes: list[Literal["catmullClark", "loop", "bilinear", "none"]] | None = None,
        interpolate_boundaries: list[Literal["none", "edgeOnly", "edgeAndCorner"]] | None = None,
        triangle_subdivision_rules: list[Literal["catmullClark", "smooth"]] | None = None,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the subdivision specifications of the prims.

        Backends: :guilabel:`usd`.

        Args:
            subdivision_schemes: Subdivision schemes (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            interpolate_boundaries: Boundary interpolation rules for faces adjacent to boundary edges and points (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            triangle_subdivision_rules: Subdivision rules for the *Catmull-Clark* scheme to try and improve undesirable
                artifacts when subdividing triangles (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither subdivision_schemes, interpolate_boundaries nor triangle_subdivision_rules are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the subdivision specifications for the second prim
            >>> prims.set_subdivision_specs(subdivision_schemes=["bilinear"], indices=[1])
        """
        assert (
            subdivision_schemes is not None
            or interpolate_boundaries is not None
            or triangle_subdivision_rules is not None
        ), (
            "All 'subdivision_schemes', 'interpolate_boundaries' and 'triangle_subdivision_rules' are not defined. "
            "Define at least one of them"
        )
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        for i, index in enumerate(indices.numpy()):
            geom = self.geoms[index]
            if subdivision_schemes is not None:
                data = np.broadcast_to(np.array(subdivision_schemes, dtype=object), (indices.shape[0],))
                geom.GetSubdivisionSchemeAttr().Set(data[i])
            if interpolate_boundaries is not None:
                data = np.broadcast_to(np.array(interpolate_boundaries, dtype=object), (indices.shape[0],))
                geom.GetInterpolateBoundaryAttr().Set(data[i])
            if triangle_subdivision_rules is not None:
                data = np.broadcast_to(np.array(triangle_subdivision_rules, dtype=object), (indices.shape[0],))
                geom.GetTriangleSubdivisionRuleAttr().Set(data[i])

    def get_subdivision_specs(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> tuple[
        list[Literal["catmullClark", "loop", "bilinear", "none"]],
        list[Literal["none", "edgeOnly", "edgeAndCorner"]],
        list[Literal["catmullClark", "smooth"]],
    ]:
        """Get the subdivision specifications of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Three-elements tuple. 1) Subdivision schemes (shape ``(N,)``).
            2) Boundary interpolation rules (shape ``(N,)``).
            3) Triangle subdivision rules (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the subdivision specifications of all prims
            >>> subdivision_schemes, interpolate_boundaries, triangle_subdivision_rules = prims.get_subdivision_specs()
            >>> # print the first prim's subdivision specifications
            >>> subdivision_schemes[0]  # subdivision scheme
            'none'
            >>> interpolate_boundaries[0]  # boundary interpolation rule
            'edgeAndCorner'
            >>> triangle_subdivision_rules[0]  # triangle subdivision rule
            'catmullClark'
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        subdivision_schemes, interpolate_boundaries, triangle_subdivision_rules = [], [], []
        for index in indices.numpy():
            geom = self.geoms[index]
            subdivision_schemes.append(geom.GetSubdivisionSchemeAttr().Get())
            interpolate_boundaries.append(geom.GetInterpolateBoundaryAttr().Get())
            triangle_subdivision_rules.append(geom.GetTriangleSubdivisionRuleAttr().Get())
        return subdivision_schemes, interpolate_boundaries, triangle_subdivision_rules

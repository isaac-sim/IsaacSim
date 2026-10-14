# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Mesh cleanup and merge operations for asset importers."""

from __future__ import annotations

from pxr import Usd, UsdGeom, UsdPhysics

from . import importer_utils

__all__ = [
    "clean_mesh_operation",
    "generate_mesh_uv_normals_operation",
    "merge_meshes_operation",
    "merge_mesh",
]

try:
    from usd_optimize.core import ExecutionContext, UsdOptimizeCore

    _HAS_USD_OPTIMIZE = True
except ImportError:
    _HAS_USD_OPTIMIZE = False


def _create_optimizer_context(stage: Usd.Stage) -> tuple:
    """Create a USD Optimize execution context bound to *stage*.

    Args:
        stage: USD stage to operate on.

    Returns:
        Tuple of ``(UsdOptimizeCore, ExecutionContext)``.

    Raises:
        RuntimeError: If ``usd_optimize`` is not available.
    """
    if not _HAS_USD_OPTIMIZE:
        raise RuntimeError("usd_optimize is not installed")

    context = ExecutionContext()
    context.set_stage(stage)
    context.generateReport = 0
    context.captureStats = 0

    core = UsdOptimizeCore.getInstance()
    return core, context


def _execute_optimizer_operation(
    core: UsdOptimizeCore, context: ExecutionContext, operation: str, config: dict[str, object]
) -> None:
    """Execute an optimizer operation and propagate its failure details."""
    ok, message, _ = core.executeOperation(operation, context, config)
    if not ok:
        raise RuntimeError(f"USD Optimize operation '{operation}' failed: {message}")


def clean_mesh_operation(stage: Usd.Stage) -> None:
    """Clean mesh prims.

    Args:
        stage: USD stage for executing clean operations.

    Raises:
        RuntimeError: If USD Optimize is unavailable or mesh cleanup fails.
    """
    core, context = _create_optimizer_context(stage)

    clean_config = {
        "paths": [],
        "mergeVertices": True,
        "tolerance": 0.0,
        "mergeBoundaries": True,
        "mergeNeighbors": True,
        "contractDegenerateEdges": True,
        "removeDegenerateFaces": True,
        "removeIsolatedVertices": True,
        "removeDuplicateFaces": True,
        "makeManifold": True,
    }

    _execute_optimizer_operation(core, context, "meshCleanup", clean_config)


def generate_mesh_uv_normals_operation(stage: Usd.Stage) -> None:
    """Generate mesh UV normals.

    Args:
        stage: USD stage for executing generate operations.

    Raises:
        RuntimeError: If USD Optimize is unavailable or normal or UV generation fails.
    """
    core, context = _create_optimizer_context(stage)

    generate_normal_config = {
        "paths": [],
        "binding": 0,
        "sharpnessAngle": 60.0,
        "gpuThreshold": 500000,
    }

    generate_uv_config = {
        "paths": [],
        "projectionType": 4,
        "useWorldSpaceScales": True,
        "scaleFactor": 0.01,
        "overwriteExisting": True,
    }

    _execute_optimizer_operation(core, context, "generateNormals", generate_normal_config)
    _execute_optimizer_operation(core, context, "generateProjectionUVs", generate_uv_config)


def merge_meshes_operation(stage: Usd.Stage) -> int:
    """Merge mesh prims grouped under rigid bodies.

    Args:
        stage: USD stage for executing merge operations.

    Returns:
        Number of mesh groups merged.

    Raises:
        RuntimeError: If USD Optimize is unavailable or merging a mesh group fails.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.asset.importer.utils import merge_meshes_operation
        >>>
        >>> stage = Usd.Stage.CreateInMemory()
        >>> stage_utils.use_stage(stage)
        >>> merge_meshes_operation(stage)  # doctest: +SKIP
    """
    # generate_mesh_uv_normals_operation(stage)

    rigid_bodies = [
        prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI("PhysicsRigidBodyAPI")
    ]
    mesh_groups = []

    for rigid_body in rigid_bodies:
        if not rigid_body or not rigid_body.IsValid():
            continue

        group = []
        prim_stack = list(rigid_body.GetChildren())
        while prim_stack:
            child = prim_stack.pop()
            if child.HasAPI(UsdPhysics.RigidBodyAPI) or child.HasAPI("PhysicsRigidBodyAPI"):
                continue

            if child.GetTypeName() in importer_utils.USD_GEOMETRY_TYPES:
                imageable = UsdGeom.Imageable(child)
                purpose = imageable.ComputePurpose()
                if purpose in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
                    group.append(child.GetPath().pathString)
            prim_stack.extend(child.GetChildren())

        if group:
            mesh_groups.append(group)

    for meshes in mesh_groups:
        merge_mesh(stage, meshes)

    return len(mesh_groups)


def merge_mesh(stage: Usd.Stage, meshes: list[str]) -> None:
    """Merge a list of mesh prim paths using USD Optimize.

    Args:
        stage: USD stage containing the mesh prims.
        meshes: List of mesh prim paths to merge.

    Raises:
        RuntimeError: If USD Optimize is unavailable or merging fails.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.asset.importer.utils import merge_mesh
        >>>
        >>> stage = Usd.Stage.CreateInMemory()
        >>> stage_utils.use_stage(stage)
        >>> merge_mesh(stage, ["/World/meshA", "/World/meshB"])  # doctest: +SKIP
    """
    if not meshes:
        return

    core, context = _create_optimizer_context(stage)

    merge_config = {
        "meshPrimPaths": list(meshes),
        "considerMaterials": False,
        "materialAlbedoAsVertexColors": False,
        "originalGeomOption": 1,
        "mergePoint": 1,
        "rootPath": "visual",
        "considerAllAttributes": False,
        "allowSingleMeshes": False,
        "spatialMode": 0,
        "spatialThreshold": 10.0,
        "spatialMaxSize": 0.0,
        "spatialVertexCount": 10000,
        "spatialDebug": False,
    }

    _execute_optimizer_operation(core, context, "merge", merge_config)

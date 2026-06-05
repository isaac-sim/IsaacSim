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

"""Functions for performing transform operations on Xformable USD/USDRT prims."""

from __future__ import annotations

import numpy as np
import usdrt
import warp as wp
from pxr import Usd, UsdGeom

from . import ops as ops_utils
from . import prim as prim_utils
from . import stage as stage_utils


def get_local_pose(
    prim: str | Usd.Prim | usdrt.Usd.Prim, *, device: str | wp.Device | None = None
) -> tuple[wp.array, wp.array]:
    """Get the local pose of a prim.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        prim: Prim path or prim instance.
        device: Device to place the output arrays on. If ``None``, the default device is used.

    Returns:
        Two-elements tuple. 1) The translation in the local frame (shape ``(3,)``).
        2) The orientation in the local frame (shape ``(4,)``, quaternion ``wxyz``).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.xform as xform_utils
        >>>
        >>> # given the stage with the following hierarchy:
        >>> # / ()
        >>> # ├─ A (Xform)    --> with local position: (1.0, 2.0, 3.0)
        >>> # │  ├─ B (Xform) --> with local position: (4.0, 5.0, 6.0)
        >>> translation, orientation = xform_utils.get_local_pose("/A/B")
        >>> print(translation)  # doctest: +NO_CHECK
        [4. 5. 6.]
        >>> print(orientation)
        [1. 0. 0. 0.]
    """
    prim = prim_utils.get_prim_at_path(prim)
    backend = "usd" if isinstance(prim, Usd.Prim) else "usdrt"
    if backend == "usd":
        transform = UsdGeom.Xformable(prim).GetLocalTransformation(Usd.TimeCode.Default())
        transform.Orthonormalize()
        translation = np.array(transform.ExtractTranslation())
        orientation = np.array(
            [transform.ExtractRotationQuat().GetReal(), *transform.ExtractRotationQuat().GetImaginary()]
        )
    elif backend == "usdrt":
        fabric_stage = stage_utils.get_current_stage(backend="fabric")
        fabric_hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
            fabric_stage.GetFabricId(), fabric_stage.GetStageIdAsStageId()
        )
        matrix = fabric_hierarchy.get_local_xform(usdrt.Sdf.Path(prim_utils.get_prim_path(prim)))
        quaternion = matrix.RemoveScaleShear().ExtractRotationQuat()
        translation = np.array(matrix.ExtractTranslation())
        orientation = np.array([quaternion.GetReal(), *quaternion.GetImaginary()])
    return (
        ops_utils.place(translation.flatten(), dtype=wp.float32, device=device),
        ops_utils.place(orientation.flatten(), dtype=wp.float32, device=device),
    )


def set_local_pose(
    prim: str | Usd.Prim | usdrt.Usd.Prim,
    *,
    translation: list | np.ndarray | wp.array | None = None,
    orientation: list | np.ndarray | wp.array | None = None,
) -> None:
    """Set the local pose of a prim.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    .. warning::

        This function is not implemented for the :guilabel:`usd` backend.
        Use :py:meth:`~isaacsim.core.experimental.prims.XformPrim.set_local_poses` instead of what is being implemented.

    Args:
        prim: Prim path or prim instance.
        translation: Translation in the local frame (shape ``(3,)``).
        orientation: Orientation in the local frame (shape ``(4,)``, quaternion ``wxyz``).

    Raises:
        NotImplementedError: If the backend is USD.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.xform as xform_utils
        >>>
        >>> # given the stage with the following hierarchy:
        >>> # / ()
        >>> # ├─ A (Xform)    --> with local position: (1.0, 2.0, 3.0)
        >>> # │  ├─ B (Xform) --> with local position: (4.0, 5.0, 6.0)
        >>> xform_utils.set_local_pose("/A/B", translation=[-4.0, -5.0, -6.0])  # doctest: +SKIP
    """
    prim = prim_utils.get_prim_at_path(prim)
    backend = "usd" if isinstance(prim, Usd.Prim) else "usdrt"
    if backend == "usd":
        # TODO: Implement for USD
        raise NotImplementedError("This function is not implemented for USD. Use `XformPrim.set_local_poses` instead.")
    elif backend == "usdrt":
        path = usdrt.Sdf.Path(prim_utils.get_prim_path(prim))
        fabric_stage = stage_utils.get_current_stage(backend="fabric")
        fabric_hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
            fabric_stage.GetFabricId(), fabric_stage.GetStageIdAsStageId()
        )
        matrix = fabric_hierarchy.get_local_xform(path)
        if translation is not None:
            translation = ops_utils.place(translation, device="cpu").numpy().flatten()
            matrix.SetTranslateOnly(usdrt.Gf.Vec3d(*translation))
        if orientation is not None:
            orientation = ops_utils.place(orientation, device="cpu").numpy().flatten()
            matrix.SetRotateOnly(usdrt.Gf.Quatd(*orientation))
            scaling_matrix = usdrt.Gf.Matrix4d().SetIdentity().SetScale(usdrt.Gf.Transform(matrix).GetScale())
            matrix = scaling_matrix * matrix
        fabric_hierarchy.set_local_xform(path, matrix)


def get_world_pose(
    prim: str | Usd.Prim | usdrt.Usd.Prim, *, device: str | wp.Device | None = None
) -> tuple[wp.array, wp.array]:
    """Get the world pose of a prim.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        prim: Prim path or prim instance.
        device: Device to place the output arrays on. If ``None``, the default device is used.

    Returns:
        Two-elements tuple. 1) The translation in the world frame (shape ``(3,)``).
        2) The orientation in the world frame (shape ``(4,)``, quaternion ``wxyz``).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.xform as xform_utils
        >>>
        >>> # given the stage with the following hierarchy:
        >>> # / ()
        >>> # ├─ A (Xform)    --> with local position: (1.0, 2.0, 3.0)
        >>> # │  ├─ B (Xform) --> with local position: (4.0, 5.0, 6.0)
        >>> translation, orientation = xform_utils.get_world_pose("/A/B")
        >>> print(translation)  # doctest: +NO_CHECK
        [5. 7. 9.]
        >>> print(orientation)
        [1. 0. 0. 0.]
    """
    prim = prim_utils.get_prim_at_path(prim)
    backend = "usd" if isinstance(prim, Usd.Prim) else "usdrt"
    if backend == "usd":
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        transform.Orthonormalize()
        translation = np.array(transform.ExtractTranslation())
        orientation = np.array(
            [transform.ExtractRotationQuat().GetReal(), *transform.ExtractRotationQuat().GetImaginary()]
        )
    elif backend == "usdrt":
        fabric_stage = stage_utils.get_current_stage(backend="fabric")
        fabric_hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
            fabric_stage.GetFabricId(), fabric_stage.GetStageIdAsStageId()
        )
        matrix = fabric_hierarchy.get_world_xform(usdrt.Sdf.Path(prim_utils.get_prim_path(prim)))
        quaternion = matrix.RemoveScaleShear().ExtractRotationQuat()
        translation = np.array(matrix.ExtractTranslation())
        orientation = np.array([quaternion.GetReal(), *quaternion.GetImaginary()])
    return (
        ops_utils.place(translation.flatten(), dtype=wp.float32, device=device),
        ops_utils.place(orientation.flatten(), dtype=wp.float32, device=device),
    )


def set_world_pose(
    prim: str | Usd.Prim | usdrt.Usd.Prim,
    *,
    position: list | np.ndarray | wp.array | None = None,
    orientation: list | np.ndarray | wp.array | None = None,
) -> None:
    """Set the world pose of a prim.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    .. warning::

        This function is not implemented for the :guilabel:`usd` backend.
        Use :py:meth:`~isaacsim.core.experimental.prims.XformPrim.set_world_poses` instead of what is being implemented.

    Args:
        prim: Prim path or prim instance.
        position: Position in the world frame (shape ``(3,)``).
        orientation: Orientation in the world frame (shape ``(4,)``, quaternion ``wxyz``).

    Raises:
        NotImplementedError: If the backend is USD.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.xform as xform_utils
        >>>
        >>> # given the stage with the following hierarchy:
        >>> # / ()
        >>> # ├─ A (Xform)    --> with local position: (1.0, 2.0, 3.0)
        >>> # │  ├─ B (Xform) --> with local position: (4.0, 5.0, 6.0)
        >>> xform_utils.set_world_pose("/A/B", position=[-4.0, -5.0, -6.0])  # doctest: +SKIP
    """
    prim = prim_utils.get_prim_at_path(prim)
    backend = "usd" if isinstance(prim, Usd.Prim) else "usdrt"
    if backend == "usd":
        # TODO: Implement for USD
        raise NotImplementedError("This function is not implemented for USD. Use `XformPrim.set_world_poses` instead.")
    elif backend == "usdrt":
        path = usdrt.Sdf.Path(prim_utils.get_prim_path(prim))
        fabric_stage = stage_utils.get_current_stage(backend="fabric")
        fabric_hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
            fabric_stage.GetFabricId(), fabric_stage.GetStageIdAsStageId()
        )
        matrix = fabric_hierarchy.get_world_xform(path)
        if position is not None:
            position = ops_utils.place(position, device="cpu").numpy().flatten()
            matrix.SetTranslateOnly(usdrt.Gf.Vec3d(*position))
        if orientation is not None:
            orientation = ops_utils.place(orientation, device="cpu").numpy().flatten()
            matrix.SetRotateOnly(usdrt.Gf.Quatd(*orientation))
            scaling_matrix = usdrt.Gf.Matrix4d().SetIdentity().SetScale(usdrt.Gf.Transform(matrix).GetScale())
            matrix = scaling_matrix * matrix
        fabric_hierarchy.set_world_xform(path, matrix)


def get_relative_transform(source_prim: str | Usd.Prim, target_prim: str | Usd.Prim) -> np.ndarray:
    """Compute the relative transformation matrix from a source prim to a target prim.

    Computes the column-major 4x4 matrix that transforms points from the source prim's
    local frame into the target prim's local frame.

    Backends: :guilabel:`usd`.

    Args:
        source_prim: Source prim path or prim instance.
        target_prim: Target prim path or prim instance.

    Returns:
        Column-major transformation matrix with shape (4, 4).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.xform as xform_utils
        >>> import omni.usd
        >>>
        >>> stage = omni.usd.get_context().get_stage()
        >>> source = stage.GetPrimAtPath("/World/Source")
        >>> target = stage.GetPrimAtPath("/World/Target")
        >>> relative_tf = xform_utils.get_relative_transform(source, target)  # doctest: +SKIP
        >>> relative_tf.shape  # doctest: +SKIP
        (4, 4)
    """
    from . import transform as transform_utils

    source_prim = prim_utils.get_prim_at_path(source_prim)
    target_prim = prim_utils.get_prim_at_path(target_prim)
    source_to_world = UsdGeom.Xformable(source_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    target_to_world = UsdGeom.Xformable(target_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return transform_utils.compute_relative_transform(source_to_world, target_to_world)

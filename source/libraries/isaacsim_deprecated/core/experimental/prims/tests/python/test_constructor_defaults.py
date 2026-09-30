# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test deprecated prim constructor compatibility."""

import inspect

import numpy as np
import pytest
from isaacsim.core.experimental.prims import GeomPrim, XformPrim
from isaacsim.foundation.objects import Stage
from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils


def _get_openusd_stage(stage: Stage) -> Usd.Stage:
    """Resolve the OpenUSD stage owned by a Foundation stage wrapper.

    Args:
        stage: Stage used by the test.

    Returns:
        The resulting value.
    """
    stage_id = Usd.StageCache.Id.FromLongInt(stage.get_stage_id())
    return UsdUtils.StageCache.Get().Find(stage_id)


def _get_ordered_xform_ops(prim: Usd.Prim) -> list[str]:
    """Get the authored transform operations in evaluation order.

    Args:
        prim: USD prim to process.

    Returns:
        The resulting value.
    """
    return [operation.GetOpName() for operation in UsdGeom.Xformable(prim).GetOrderedXformOps()]


def test_xform_prim_preserves_reset_default() -> None:
    """Preserve the experimental non-destructive transform-operation default."""
    signature = inspect.signature(XformPrim)

    assert signature.parameters["reset_xform_op_properties"].default is False


def test_geom_prim_preserves_construction_defaults() -> None:
    """Preserve the experimental non-destructive geometry construction defaults."""
    signature = inspect.signature(GeomPrim)

    assert signature.parameters["apply_collision_apis"].default is False
    assert signature.parameters["reset_xform_op_properties"].default is False


def test_xform_prim_construction_preserves_existing_operations() -> None:
    """Keep existing transform operations when wrapping a prim."""
    stage = Stage("openusd").create_stage()
    try:
        prim = _get_openusd_stage(stage).DefinePrim("/World/Xform", "Xform")
        UsdGeom.Xformable(prim).AddTranslateOp(opSuffix="custom")
        expected_operations = _get_ordered_xform_ops(prim)

        XformPrim("/World/Xform")

        assert _get_ordered_xform_ops(prim) == expected_operations
    finally:
        stage.close_stage()


def test_geom_prim_construction_preserves_existing_operations_and_schemas() -> None:
    """Keep existing transform operations and avoid authoring collision schemas when wrapping a geometry prim."""
    stage = Stage("openusd").create_stage()
    try:
        prim = _get_openusd_stage(stage).DefinePrim("/World/Cube", "Cube")
        UsdGeom.Xformable(prim).AddTranslateOp(opSuffix="custom")
        expected_operations = _get_ordered_xform_ops(prim)

        GeomPrim("/World/Cube")

        assert _get_ordered_xform_ops(prim) == expected_operations
        assert not prim.HasAPI(UsdPhysics.CollisionAPI)
    finally:
        stage.close_stage()


def test_xform_prim_construction_applies_local_transform() -> None:
    """Apply the local-frame constructor arguments that the base wrapper no longer accepts."""
    stage = Stage("openusd").create_stage()
    try:
        prims = XformPrim(
            "/World/Xform",
            translations=[[1.0, 2.0, 3.0]],
            scales=[[2.0, 2.0, 2.0]],
            reset_xform_op_properties=True,
        )

        translations, _ = prims.get_local_poses()

        assert np.allclose(translations.numpy(), [[1.0, 2.0, 3.0]])
        assert np.allclose(prims.get_local_scales().numpy(), [[2.0, 2.0, 2.0]])
    finally:
        stage.close_stage()


def test_xform_prim_construction_applies_world_positions() -> None:
    """Apply the world-frame constructor arguments that the base wrapper no longer accepts."""
    stage = Stage("openusd").create_stage()
    try:
        prims = XformPrim("/World/Xform", positions=[[4.0, 5.0, 6.0]], reset_xform_op_properties=True)

        positions, _ = prims.get_world_poses()

        assert np.allclose(positions.numpy(), [[4.0, 5.0, 6.0]])
    finally:
        stage.close_stage()


def test_xform_prim_construction_rejects_positions_and_translations() -> None:
    """Reject the mutually exclusive world- and local-frame constructor arguments."""
    stage = Stage("openusd").create_stage()
    try:
        with pytest.raises(ValueError, match="Specify only one of them"):
            XformPrim("/World/Xform", positions=[[0.0, 0.0, 0.0]], translations=[[0.0, 0.0, 0.0]])
    finally:
        stage.close_stage()

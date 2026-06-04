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

"""Tests for scene utility functions used by behavior scripts."""

from __future__ import annotations

import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.behavior.utils.scene_utils import decompose_rotation, set_rotation_with_ops
from pxr import Gf, UsdGeom


class TestSceneUtils(omni.kit.test.AsyncTestCase):
    """Test scene utility helpers."""

    async def setUp(self) -> None:
        """Set up a new stage before each test."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Close the stage after each test."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()

    async def test_set_rotation_with_single_axis_rotate_ops(self) -> None:
        """Test that single-axis rotate ops receive scalar values."""
        stage = omni.usd.get_context().get_stage()
        axis_vectors = {
            "X": Gf.Vec3d.XAxis(),
            "Y": Gf.Vec3d.YAxis(),
            "Z": Gf.Vec3d.ZAxis(),
        }

        for axis, axis_vector in axis_vectors.items():
            with self.subTest(axis=axis):
                prim = stage.DefinePrim(f"/World/Rotate{axis}Prim", "Xform")
                xformable = UsdGeom.Xformable(prim)
                rotate_op = getattr(xformable, f"AddRotate{axis}Op")()
                rotate_op.Set(0.0)

                rotation = Gf.Rotation(axis_vector, 45.0)
                rotation_decomp = decompose_rotation(rotation, axis)

                self.assertIsInstance(rotation_decomp, float)
                set_rotation_with_ops(prim, rotation)
                self.assertAlmostEqual(rotate_op.Get(), rotation_decomp, places=4)

    async def test_decompose_rotation_rejects_unsupported_order(self) -> None:
        """Test that unsupported rotation orders fail explicitly."""
        rotation = Gf.Rotation(Gf.Vec3d.XAxis(), 45.0)

        with self.assertRaises(ValueError):
            decompose_rotation(rotation, "ABC")

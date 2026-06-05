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

"""Tests for prim utility functions."""

from typing import Any

import carb
import omni.kit.test
from isaacsim.core.utils.prims import define_prim, get_prim_attribute_value, set_prim_attribute_value
from pxr import Gf, Sdf


class TestPrims(omni.kit.test.AsyncTestCase):
    """Test cases for Prims."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""

    def _create_attributes(self, prim: Any) -> list:
        specs = [
            (Sdf.ValueTypeNames.Asset, "./abc"),
            (Sdf.ValueTypeNames.AssetArray, ["./abc", "./def"]),
            (Sdf.ValueTypeNames.Bool, True),
            (Sdf.ValueTypeNames.BoolArray, [True, False]),
            (Sdf.ValueTypeNames.Color3d, Gf.Vec3d()),
            (Sdf.ValueTypeNames.Color3dArray, [Gf.Vec3d()]),
            (Sdf.ValueTypeNames.Color3f, Gf.Vec3f()),
            (Sdf.ValueTypeNames.Color3fArray, [Gf.Vec3f()]),
            (Sdf.ValueTypeNames.Color3h, Gf.Vec3h()),
            (Sdf.ValueTypeNames.Color3hArray, [Gf.Vec3h()]),
            (Sdf.ValueTypeNames.Color4d, Gf.Vec4d()),
            (Sdf.ValueTypeNames.Color4dArray, [Gf.Vec4d()]),
            (Sdf.ValueTypeNames.Color4f, Gf.Vec4f()),
            (Sdf.ValueTypeNames.Color4fArray, [Gf.Vec4f()]),
            (Sdf.ValueTypeNames.Color4h, Gf.Vec4h()),
            (Sdf.ValueTypeNames.Color4hArray, [Gf.Vec4h()]),
            (Sdf.ValueTypeNames.Double, 1.0),
            (Sdf.ValueTypeNames.Double2, Gf.Vec2d()),
            (Sdf.ValueTypeNames.Double2Array, [Gf.Vec2d()]),
            (Sdf.ValueTypeNames.Double3, Gf.Vec3d()),
            (Sdf.ValueTypeNames.Double3Array, [Gf.Vec3d()]),
            (Sdf.ValueTypeNames.Double4, Gf.Vec4d()),
            (Sdf.ValueTypeNames.Double4Array, [Gf.Vec4d()]),
            (Sdf.ValueTypeNames.DoubleArray, [1.0]),
            (Sdf.ValueTypeNames.Float, 1.0),
            (Sdf.ValueTypeNames.Float2, Gf.Vec2f()),
            (Sdf.ValueTypeNames.Float2Array, [Gf.Vec2f()]),
            (Sdf.ValueTypeNames.Float3, Gf.Vec3f()),
            (Sdf.ValueTypeNames.Float3Array, [Gf.Vec3f()]),
            (Sdf.ValueTypeNames.Float4, Gf.Vec4f()),
            (Sdf.ValueTypeNames.Float4Array, [Gf.Vec4f()]),
            (Sdf.ValueTypeNames.FloatArray, [1.0]),
            (Sdf.ValueTypeNames.Frame4d, Gf.Matrix4d()),
            (Sdf.ValueTypeNames.Frame4dArray, [Gf.Matrix4d()]),
            (Sdf.ValueTypeNames.Half, 1.0),
            (Sdf.ValueTypeNames.Half2, Gf.Vec2h()),
            (Sdf.ValueTypeNames.Half2Array, [Gf.Vec2h()]),
            (Sdf.ValueTypeNames.Half3, Gf.Vec3h()),
            (Sdf.ValueTypeNames.Half3Array, [Gf.Vec3h()]),
            (Sdf.ValueTypeNames.Half4, Gf.Vec4h()),
            (Sdf.ValueTypeNames.Half4Array, [Gf.Vec4h()]),
            (Sdf.ValueTypeNames.HalfArray, [1.0]),
            (Sdf.ValueTypeNames.Int, 1),
            (Sdf.ValueTypeNames.Int2, Gf.Vec2i()),
            (Sdf.ValueTypeNames.Int2Array, [Gf.Vec2i()]),
            (Sdf.ValueTypeNames.Int3, Gf.Vec3i()),
            (Sdf.ValueTypeNames.Int3Array, [Gf.Vec3i()]),
            (Sdf.ValueTypeNames.Int4, Gf.Vec4i()),
            (Sdf.ValueTypeNames.Int4Array, [Gf.Vec4i()]),
            (Sdf.ValueTypeNames.Int64, 1),
            (Sdf.ValueTypeNames.Int64Array, [1]),
            (Sdf.ValueTypeNames.IntArray, [1]),
            (Sdf.ValueTypeNames.Matrix2d, Gf.Matrix2d()),
            (Sdf.ValueTypeNames.Matrix2dArray, [Gf.Matrix2d()]),
            (Sdf.ValueTypeNames.Matrix3d, Gf.Matrix3d()),
            (Sdf.ValueTypeNames.Matrix3dArray, [Gf.Matrix3d()]),
            (Sdf.ValueTypeNames.Matrix4d, Gf.Matrix4d()),
            (Sdf.ValueTypeNames.Matrix4dArray, [Gf.Matrix4d()]),
            (Sdf.ValueTypeNames.Normal3d, Gf.Vec3d()),
            (Sdf.ValueTypeNames.Normal3dArray, [Gf.Vec3d()]),
            (Sdf.ValueTypeNames.Normal3f, Gf.Vec3f()),
            (Sdf.ValueTypeNames.Normal3fArray, [Gf.Vec3f()]),
            (Sdf.ValueTypeNames.Normal3h, Gf.Vec3h()),
            (Sdf.ValueTypeNames.Normal3hArray, [Gf.Vec3h()]),
            (Sdf.ValueTypeNames.Point3d, Gf.Vec3d()),
            (Sdf.ValueTypeNames.Point3dArray, [Gf.Vec3d()]),
            (Sdf.ValueTypeNames.Point3f, Gf.Vec3f()),
            (Sdf.ValueTypeNames.Point3fArray, [Gf.Vec3f()]),
            (Sdf.ValueTypeNames.Point3h, Gf.Vec3h()),
            (Sdf.ValueTypeNames.Point3hArray, [Gf.Vec3h()]),
            (Sdf.ValueTypeNames.Quatd, Gf.Quatd()),
            (Sdf.ValueTypeNames.QuatdArray, [Gf.Quatd()]),
            (Sdf.ValueTypeNames.Quatf, Gf.Quatf()),
            (Sdf.ValueTypeNames.QuatfArray, [Gf.Quatf()]),
            (Sdf.ValueTypeNames.Quath, Gf.Quath()),
            (Sdf.ValueTypeNames.QuathArray, [Gf.Quath()]),
            (Sdf.ValueTypeNames.String, "abcd"),
            (Sdf.ValueTypeNames.StringArray, ["abcd", "efgh"]),
            (Sdf.ValueTypeNames.TexCoord2d, Gf.Vec2d()),
            (Sdf.ValueTypeNames.TexCoord2dArray, [Gf.Vec2d()]),
            (Sdf.ValueTypeNames.TexCoord2f, Gf.Vec2f()),
            (Sdf.ValueTypeNames.TexCoord2fArray, [Gf.Vec2f()]),
            (Sdf.ValueTypeNames.TexCoord2h, Gf.Vec2h()),
            (Sdf.ValueTypeNames.TexCoord2hArray, [Gf.Vec2h()]),
            (Sdf.ValueTypeNames.TexCoord3d, Gf.Vec3d()),
            (Sdf.ValueTypeNames.TexCoord3dArray, [Gf.Vec3d()]),
            (Sdf.ValueTypeNames.TexCoord3f, Gf.Vec3f()),
            (Sdf.ValueTypeNames.TexCoord3fArray, [Gf.Vec3f()]),
            (Sdf.ValueTypeNames.TexCoord3h, Gf.Vec3h()),
            (Sdf.ValueTypeNames.TexCoord3hArray, [Gf.Vec3h()]),
            (Sdf.ValueTypeNames.TimeCode, Sdf.TimeCode()),
            (Sdf.ValueTypeNames.TimeCodeArray, [Sdf.TimeCode()]),
            (Sdf.ValueTypeNames.Token, "abcde"),
            (Sdf.ValueTypeNames.TokenArray, ["abcde", "fghij"]),
            (Sdf.ValueTypeNames.UChar, 1),
            (Sdf.ValueTypeNames.UCharArray, [1]),
            (Sdf.ValueTypeNames.UInt, 1),
            (Sdf.ValueTypeNames.UInt64, 1),
            (Sdf.ValueTypeNames.UInt64Array, [1]),
            (Sdf.ValueTypeNames.UIntArray, [1]),
            (Sdf.ValueTypeNames.Vector3d, Gf.Vec3d()),
            (Sdf.ValueTypeNames.Vector3dArray, [Gf.Vec3d()]),
            (Sdf.ValueTypeNames.Vector3f, Gf.Vec3f()),
            (Sdf.ValueTypeNames.Vector3fArray, [Gf.Vec3f()]),
            (Sdf.ValueTypeNames.Vector3h, Gf.Vec3h()),
            (Sdf.ValueTypeNames.Vector3hArray, [Gf.Vec3h()]),
        ]

        attributes = []
        for i, spec in enumerate(specs):
            name = f"attr_{i}"
            prim.CreateAttribute(name, spec[0])
            prim.GetAttribute(name).Set(spec[1])
            attributes.append((name, spec[0], spec[1]))
        return attributes

    async def test_prim_attribute_value(self) -> None:
        """Test prim attribute value."""
        prim_path = "/prim"
        prim = define_prim(prim_path=prim_path, prim_type="Xform")
        attributes = self._create_attributes(prim)
        await omni.kit.app.get_app().next_update_async()
        for fabric in [False, True]:
            for attribute_name, attribute_type, _ in attributes:
                try:
                    value = get_prim_attribute_value(prim_path, attribute_name=attribute_name, fabric=fabric)
                except Exception as e:
                    carb.log_error(f"[Get '{attribute_name}' ({attribute_type})] {e}")
                try:
                    set_prim_attribute_value(prim_path, attribute_name=attribute_name, value=value, fabric=fabric)
                except Exception as e:
                    carb.log_error(f"[Set '{attribute_name}' ({attribute_type}) to '{value}'] {e}")

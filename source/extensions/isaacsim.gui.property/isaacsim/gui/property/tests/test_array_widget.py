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

"""Regression tests for ``ArrayPropertiesWidget`` covering ``string[]`` and ``token[]`` support."""

from __future__ import annotations

import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.gui.property.array_widget import ArrayPropertiesWidget, _UsdArrayAttributeModel
from pxr import Sdf, Usd


class TestArrayPropertiesWidgetStringSupport(omni.kit.test.AsyncTestCase):
    """Validates the array widget filter and model round-trips for string and token arrays."""

    async def setUp(self) -> None:
        """Create a fresh empty USD stage for each test case."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        self._prim = self._stage.DefinePrim("/World/Test", "Xform")

    def _author(self, name: str, type_name: Sdf.ValueTypeName, value: object) -> Usd.Attribute:
        """Author an attribute on the test prim and set its value.

        Args:
            name: Attribute name to create.
            type_name: USD value type to author.
            value: Initial authored value.

        Returns:
            Created attribute.
        """
        attr = self._prim.CreateAttribute(name, type_name)
        attr.Set(value)
        return attr

    async def test_filter_keeps_string_and_token_arrays(self) -> None:
        """``_filter_props_to_build`` must keep ``string[]``/``token[]`` and drop unsupported scalar arrays."""
        attrs = [
            self._author("ints", Sdf.ValueTypeNames.IntArray, [1, 2, 3]),
            self._author("floats", Sdf.ValueTypeNames.FloatArray, [1.0, 2.0]),
            self._author("strings", Sdf.ValueTypeNames.StringArray, ["a", "b"]),
            self._author("tokens", Sdf.ValueTypeNames.TokenArray, ["x", "y"]),
            self._author("bools", Sdf.ValueTypeNames.BoolArray, [True, False]),
            self._author("doubles", Sdf.ValueTypeNames.DoubleArray, [1.0]),
        ]
        widget = ArrayPropertiesWidget(title="Test", collapsed=True)

        kept = widget._filter_props_to_build(attrs)
        kept_names = {a.GetName() for a in kept}

        self.assertIn("ints", kept_names)
        self.assertIn("floats", kept_names)
        self.assertIn("strings", kept_names)
        self.assertIn("tokens", kept_names)
        self.assertNotIn("bools", kept_names, msg="bool[] is not yet supported by the array widget")
        self.assertNotIn("doubles", kept_names, msg="double[] is not yet supported by the array widget")

    async def _exercise_model_round_trip(self, attr: Usd.Attribute, initial: list[str]) -> None:
        """Drive ``_UsdArrayAttributeModel`` through add/set/remove and verify USD reflects each step.

        Args:
            attr: Attribute to edit through the model.
            initial: Initial authored values.
        """
        model = _UsdArrayAttributeModel(self._stage, [attr.GetPath()], False, {})
        try:
            self.assertEqual(model.get_length(), len(initial))
            self.assertEqual(model.get_item(0), initial[0])

            model.add_item("delta")
            self.assertEqual(list(attr.Get()), [*initial, "delta"])

            model.set_item(0, "ALPHA")
            self.assertEqual(list(attr.Get()), ["ALPHA", *initial[1:], "delta"])

            model.remove_item(1)
            expected_after_remove = ["ALPHA", *initial[2:], "delta"]
            self.assertEqual(list(attr.Get()), expected_after_remove)
        finally:
            model.clean()

    async def test_string_array_model_round_trip(self) -> None:
        """``_UsdArrayAttributeModel`` round-trips edits for a ``string[]`` attribute."""
        attr = self._author("strings", Sdf.ValueTypeNames.StringArray, ["a", "b", "c"])
        await self._exercise_model_round_trip(attr, ["a", "b", "c"])

    async def test_token_array_model_round_trip(self) -> None:
        """``_UsdArrayAttributeModel`` round-trips edits for a ``token[]`` attribute."""
        attr = self._author("tokens", Sdf.ValueTypeNames.TokenArray, ["a", "b", "c"])
        await self._exercise_model_round_trip(attr, ["a", "b", "c"])

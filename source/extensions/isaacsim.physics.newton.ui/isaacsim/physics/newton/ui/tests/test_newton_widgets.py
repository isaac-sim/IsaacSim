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

"""Tests for Newton API schema apply widgets and remove affordances."""

from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import omni.kit.property.usd as usd_property
from omni.kit.test.async_unittest import AsyncTestCase
from pxr import Usd, UsdPhysics

from ..newton_schemas import _NEWTON_SCENE_SCHEMA, ExtendedNewtonSceneWidget
from ..newton_widgets import _base_api_schemas, ignored_schemas, newton_schema_menu_items
from ..utils import NewtonWidgetBase, build_remove_schema_frame_header


class NewtonWidgetTests(AsyncTestCase):
    """Validate Newton custom widgets cover remove and ignore the auto parent path."""

    def test_custom_widgets_own_joint_and_solver_scene_schemas(self) -> None:
        """Own Joint and solver scene schemas with NewtonWidgetBase so they get a remove button."""
        owned = {item.schema for item in newton_schema_menu_items}
        for schema in (
            "NewtonJointAPI",
            "NewtonXpbdSceneAPI",
            "NewtonKaminoSceneAPI",
            "NewtonMPMSceneAPI",
            "NewtonPointsDeformableSimAPI",
            "NewtonMPMMaterialAPI",
            "NewtonCurvesDeformableMaterialAPI",
        ):
            self.assertIn(schema, owned)
            self.assertIn(schema, ignored_schemas)

    def test_base_apis_are_ignored_by_parent_schema_registration(self) -> None:
        """Suppress empty automatic frames for transitively applied base APIs."""
        self.assertTrue(_base_api_schemas.issubset(ignored_schemas))

    def test_ignored_schemas_remain_privately_registered(self) -> None:
        """Keep custom-widget schemas out of the generic Edit API Schema dialog."""
        for schema in (
            "NewtonJointAPI",
            "NewtonXpbdSceneAPI",
            "NewtonKaminoSceneAPI",
            "MjcSceneAPI",
            "MjcJointAPI",
        ):
            _, codes = usd_property.is_registered_schema([], schema)
            self.assertTrue(codes & usd_property.RegisteredSchemaCodes.PRIVATE)

    def test_custom_widget_applies_physics_builders_and_order(self) -> None:
        """Apply registered builders and property order inside a custom schema frame."""
        widget = NewtonWidgetBase("Physics/Newton", "Newton Joint", "Newton Joint", "NewtonJointAPI")
        first = SimpleNamespace(prop_name="newton:first", base_name="newton:first", build_fn=None, metadata={})
        second = SimpleNamespace(prop_name="newton:second", base_name="newton:second", build_fn=None, metadata={})
        builder_calls = []

        def _builder(
            stage: Any,
            prop: Any,
            prim_paths: Any,
            label_kwargs: Any,
            widget_kwargs: Any,
            marker: Any,
        ) -> str:
            builder_calls.append((stage, prop, prim_paths, label_kwargs, widget_kwargs, marker))
            return "model"

        try:
            with (
                patch(
                    "isaacsim.physics.newton.ui.utils.database.get_available_builders",
                    return_value=[[_builder, "registered"]],
                ),
                patch(
                    "isaacsim.physics.newton.ui.utils.database.get_property_order",
                    return_value=["newton:second", "newton:first"],
                ),
            ):
                ordered = widget._customize_props_layout([first, second])

            self.assertEqual(ordered, [second, first])
            self.assertEqual(
                second.build_fn("stage", "newton:second", {}, None, ["prim"], {"label": True}, {"widget": True}),
                "model",
            )
            self.assertEqual(
                builder_calls,
                [("stage", second, ["prim"], {"label": True}, {"widget": True}, "registered")],
            )
        finally:
            widget.clean()

    def test_all_schema_frames_share_one_header_builder(self) -> None:
        """Draw the Newton Scene and Apply-widget remove buttons through the same builder."""
        widget = NewtonWidgetBase("Physics/Newton", "Newton Joint", "Newton Joint", "NewtonJointAPI")
        try:
            with patch("isaacsim.physics.newton.ui.utils.build_remove_schema_frame_header") as header_mock:
                widget._build_frame_header(False, "Newton Joint")
            self.assertEqual(header_mock.call_args.args[2], "NewtonJointAPI")
            self.assertEqual(header_mock.call_args.args[3], widget.on_remove_schema)
        finally:
            widget.clean()

        scene_widget = MagicMock()
        scene_widget._payload = []
        with patch("isaacsim.physics.newton.ui.newton_schemas.build_remove_schema_frame_header") as header_mock:
            ExtendedNewtonSceneWidget._build_frame_header(scene_widget, False, "Newton Scene")
        self.assertEqual(header_mock.call_args.args[2], _NEWTON_SCENE_SCHEMA)

    def test_header_builder_is_shared_across_modules(self) -> None:
        """Import the same header builder in both widget modules."""
        from .. import newton_schemas

        self.assertIs(newton_schemas.build_remove_schema_frame_header, build_remove_schema_frame_header)

    def test_newton_scene_remove_button_requires_applied_api(self) -> None:
        """Hide remove until ``NewtonSceneAPI`` is applied on the PhysicsScene."""
        stage = Usd.Stage.CreateInMemory()
        prim = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").GetPrim()
        widget = MagicMock()
        widget._payload = [prim.GetPath()]
        widget._get_prim = MagicMock(return_value=prim)

        self.assertEqual(ExtendedNewtonSceneWidget._prims_with_schema(widget), [])
        prim.ApplyAPI(_NEWTON_SCENE_SCHEMA)
        self.assertEqual(ExtendedNewtonSceneWidget._prims_with_schema(widget), [prim])

    def test_newton_scene_remove_unapplies_schema(self) -> None:
        """Unapply ``NewtonSceneAPI`` through the undoable codeless command."""
        stage = Usd.Stage.CreateInMemory()
        prim = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").GetPrim()
        prim.ApplyAPI(_NEWTON_SCENE_SCHEMA)
        widget = MagicMock()
        widget._payload = [prim.GetPath()]
        widget._get_prim = MagicMock(return_value=prim)
        widget._prims_with_schema = MagicMock(return_value=[prim])

        with (
            patch(
                "isaacsim.physics.newton.ui.newton_schemas.execute",
                side_effect=lambda _command, api, prim: prim.RemoveAPI(api),
            ) as execute_mock,
            patch("isaacsim.physics.newton.ui.newton_schemas.request_property_window_refresh"),
        ):
            ExtendedNewtonSceneWidget.on_remove_schema(widget)

        self.assertFalse(prim.HasAPI(_NEWTON_SCENE_SCHEMA))
        self.assertEqual(execute_mock.call_args.kwargs["api"], _NEWTON_SCENE_SCHEMA)

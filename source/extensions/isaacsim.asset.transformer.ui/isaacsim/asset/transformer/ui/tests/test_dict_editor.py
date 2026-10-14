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

"""Dictionary row identity survives repeated edits without rebuilding fields."""

import json
from collections.abc import Callable
from typing import Any
from unittest.mock import patch

import omni.kit.test
import omni.ui as ui
from isaacsim.asset.transformer import RuleSpec
from isaacsim.asset.transformer.ui.action_models import RuleActionItem


class TestDictionaryEditor(omni.kit.test.AsyncTestCase):
    """Exercise dictionaryeditor regressions."""

    async def test_rename_value_remove_and_serialize(self) -> None:
        """Verify rename value remove and serialize."""
        spec = RuleSpec(
            name="Flatten Base",
            type="fixture",
            params={"selected_variants": {"Physics": "PhysX", "Robot": "Robot"}},
        )
        action = RuleActionItem(spec)
        models = []
        buttons = []
        string_field = ui.StringField

        def capture_field(*args: Any, **kwargs: Any) -> ui.StringField:
            models.append(kwargs["model"])
            return string_field(*args, **kwargs)

        def capture_button(icon: str, tooltip: str, callback: Callable) -> None:
            buttons.append((tooltip, callback))

        window = ui.Window("Dictionary regression", width=500, height=300)
        try:
            with (
                patch("omni.ui.StringField", side_effect=capture_field),
                patch.object(action, "_icon_btn", side_effect=capture_button),
            ):
                with window.frame:
                    action._build_dict_editor(spec.params["selected_variants"])
                key, value = models[:2]
                key.set_value("Scope")
                key.set_value("Scope2")
                value.set_value("PhysXAlt")
                expected = {"Scope2": "PhysXAlt", "Robot": "Robot"}
                self.assertEqual(spec.params["selected_variants"], expected)
                self.assertEqual(
                    RuleSpec.from_dict(json.loads(json.dumps(action.to_dict()))).params["selected_variants"],
                    expected,
                )
                key.set_value("Robot")
                self.assertEqual(key.get_value_as_string(), "Scope2")
                key.set_value("")
                self.assertEqual(key.get_value_as_string(), "Scope2")
                buttons[0][1]()  # Remove the renamed row using its original button.
                self.assertEqual(spec.params["selected_variants"], {"Robot": "Robot"})
                value.set_value("late event")
                self.assertEqual(spec.params["selected_variants"], {"Robot": "Robot"})
        finally:
            window.destroy()

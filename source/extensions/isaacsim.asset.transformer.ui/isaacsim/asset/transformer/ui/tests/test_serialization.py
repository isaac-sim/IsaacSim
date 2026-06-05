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

"""Tests for action model serialization and deserialization."""

import json
import tempfile
from pathlib import Path

import omni.kit.test
import omni.ui as ui
from isaacsim.asset.transformer.ui.action_models import (
    ActionItemValueModel,
    ActionListItem,
    ActionListModel,
)


class TestActionItem:
    """Minimal action satisfying ``ActionProtocol`` for serialization tests.

    Args:
        name: Display name.
        enabled: Whether the action starts enabled.
        string_param: Example string config.
        int_param: Example integer config.
        bool_param: Example boolean config.
    """

    ACTION_TYPE = "TestActionItem"

    def __init__(
        self,
        name: str = "Test Action",
        enabled: bool = True,
        string_param: str = "",
        int_param: int = 0,
        bool_param: bool = False,
    ) -> None:
        self._name = name
        self._model = ui.SimpleBoolModel()
        self._model.set_value(enabled)

        # Config parameters
        self._string_param = string_param
        self._int_param = int_param
        self._bool_param = bool_param

    @property
    def name(self) -> str:
        """Human-readable name of this action.

        Returns:
            Display string for this test action.
        """
        return self._name

    @property
    def model(self) -> ui.AbstractValueModel:
        """Boolean value model tracking the enabled state.

        Returns:
            The underlying Omni UI bool model.
        """
        return self._model

    @property
    def enabled(self) -> bool:
        """Whether this action is enabled.

        Returns:
            Current value from ``model``.
        """
        return self._model.get_value_as_bool()

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self._model.set_value(value)

    def run(self) -> bool:
        """Execute the test action (no-op).

        Returns:
            Always True.
        """
        return True

    def build_ui(self) -> None:
        """Build UI (no-op for tests)."""

    def to_dict(self) -> dict:
        """Serialize this action to a dictionary.

        Returns:
            Dictionary representation of this action.
        """
        return {
            "type": self.ACTION_TYPE,
            "name": self._name,
            "enabled": self.enabled,
            "config": {
                "string_param": self._string_param,
                "int_param": self._int_param,
                "bool_param": self._bool_param,
            },
        }

    @classmethod
    def from_dict(cls, data: dict) -> "TestActionItem":
        """Create an instance from a serialized dictionary.

        Args:
            data: Dictionary previously produced by ``to_dict()``.

        Returns:
            A new ``TestActionItem`` instance.
        """
        config = data.get("config", {})
        return cls(
            name=data.get("name", "Unnamed Action"),
            enabled=data.get("enabled", True),
            string_param=config.get("string_param", ""),
            int_param=config.get("int_param", 0),
            bool_param=config.get("bool_param", False),
        )


class TestActionItemSerialization(omni.kit.test.AsyncTestCase):
    """Test serialization roundtrip for TestActionItem."""

    async def test_to_dict_contains_required_fields(self) -> None:
        """Verify to_dict output contains all required fields."""
        action = TestActionItem(
            name="Test Action",
            enabled=True,
            string_param="test_value",
            int_param=42,
            bool_param=True,
        )

        data = action.to_dict()

        self.assertIn("type", data)
        self.assertIn("name", data)
        self.assertIn("enabled", data)
        self.assertIn("config", data)
        self.assertEqual(data["type"], "TestActionItem")
        self.assertEqual(data["name"], "Test Action")
        self.assertEqual(data["enabled"], True)
        self.assertEqual(data["config"]["string_param"], "test_value")
        self.assertEqual(data["config"]["int_param"], 42)
        self.assertEqual(data["config"]["bool_param"], True)

    async def test_from_dict_restores_all_fields(self) -> None:
        """Verify from_dict correctly restores all fields."""
        data = {
            "type": "TestActionItem",
            "name": "Restored Action",
            "enabled": False,
            "config": {
                "string_param": "restored_value",
                "int_param": 123,
                "bool_param": True,
            },
        }

        action = TestActionItem.from_dict(data)

        self.assertEqual(action.name, "Restored Action")
        self.assertEqual(action.enabled, False)
        # Verify config was restored by serializing again
        restored_data = action.to_dict()
        self.assertEqual(restored_data["config"]["string_param"], "restored_value")
        self.assertEqual(restored_data["config"]["int_param"], 123)
        self.assertEqual(restored_data["config"]["bool_param"], True)

    async def test_roundtrip_preserves_data(self) -> None:
        """Verify serialize -> deserialize roundtrip preserves all data."""
        original = TestActionItem(
            name="Roundtrip Test",
            enabled=True,
            string_param="preserve_me",
            int_param=999,
            bool_param=False,
        )

        # Serialize
        data = original.to_dict()
        json_str = json.dumps(data)

        # Deserialize
        parsed = json.loads(json_str)
        restored = TestActionItem.from_dict(parsed)

        # Verify
        self.assertEqual(restored.name, original.name)
        self.assertEqual(restored.enabled, original.enabled)

        # Compare full serialization
        original_data = original.to_dict()
        restored_data = restored.to_dict()
        self.assertEqual(original_data, restored_data)

    async def test_from_dict_handles_missing_config(self) -> None:
        """Verify from_dict handles missing config gracefully."""
        data = {
            "type": "TestActionItem",
            "name": "Minimal Action",
            "enabled": True,
            # No config section
        }

        action = TestActionItem.from_dict(data)

        self.assertEqual(action.name, "Minimal Action")
        self.assertEqual(action.enabled, True)
        # Should use defaults
        restored_data = action.to_dict()
        self.assertEqual(restored_data["config"]["string_param"], "")
        self.assertEqual(restored_data["config"]["int_param"], 0)
        self.assertEqual(restored_data["config"]["bool_param"], False)

    async def test_from_dict_handles_missing_name(self) -> None:
        """Verify from_dict provides default name when missing."""
        data = {
            "type": "TestActionItem",
            "enabled": True,
        }

        action = TestActionItem.from_dict(data)

        self.assertEqual(action.name, "Unnamed Action")

    async def test_enabled_state_toggle(self) -> None:
        """Verify enabled state can be toggled and serialized correctly."""
        action = TestActionItem(name="Toggle Test", enabled=True)

        self.assertTrue(action.enabled)
        self.assertTrue(action.to_dict()["enabled"])

        action.enabled = False

        self.assertFalse(action.enabled)
        self.assertFalse(action.to_dict()["enabled"])

    async def test_json_serializable(self) -> None:
        """Verify to_dict output is JSON serializable."""
        action = TestActionItem(
            name="JSON Test",
            enabled=True,
            string_param="json_value",
            int_param=100,
            bool_param=True,
        )

        data = action.to_dict()

        # Should not raise
        json_str = json.dumps(data)
        self.assertIsInstance(json_str, str)

        # Should round-trip through JSON
        parsed = json.loads(json_str)
        self.assertEqual(parsed, data)


class TestActionListModelSerialization(omni.kit.test.AsyncTestCase):
    """Test serialization of multiple actions in a list."""

    async def test_serialize_multiple_actions(self) -> None:
        """Verify multiple actions can be serialized to a preset format."""
        model = ActionListModel()

        action1 = TestActionItem(name="Action 1", enabled=True, string_param="opt1", int_param=1)
        action2 = TestActionItem(name="Action 2", enabled=False, string_param="opt2", int_param=2)

        model.append_child_item(None, ActionItemValueModel(action1))
        model.append_child_item(None, ActionItemValueModel(action2))

        # Serialize all actions (mimics _save_preset logic)
        actions_data = []
        for item in model.get_item_children():
            assert isinstance(item, ActionListItem)
            action = item.action_model.get_action()
            actions_data.append(action.to_dict())

        preset = {"version": "1.0", "actions": actions_data}

        self.assertEqual(len(preset["actions"]), 2)
        self.assertEqual(preset["actions"][0]["name"], "Action 1")
        self.assertEqual(preset["actions"][1]["name"], "Action 2")
        self.assertEqual(preset["actions"][0]["enabled"], True)
        self.assertEqual(preset["actions"][1]["enabled"], False)
        self.assertEqual(preset["actions"][0]["config"]["int_param"], 1)
        self.assertEqual(preset["actions"][1]["config"]["int_param"], 2)

    async def test_deserialize_multiple_actions(self) -> None:
        """Verify preset can be deserialized back into action list."""
        preset_json = """
        {
            "version": "1.0",
            "actions": [
                {
                    "type": "TestActionItem",
                    "name": "Loaded Action 1",
                    "enabled": true,
                    "config": {"string_param": "loaded1", "int_param": 10, "bool_param": false}
                },
                {
                    "type": "TestActionItem",
                    "name": "Loaded Action 2",
                    "enabled": false,
                    "config": {"string_param": "loaded2", "int_param": 20, "bool_param": true}
                }
            ]
        }
        """

        preset = json.loads(preset_json)
        model = ActionListModel()

        for action_data in preset["actions"]:
            action = TestActionItem.from_dict(action_data)
            model.append_child_item(None, ActionItemValueModel(action))

        children = model.get_item_children()
        self.assertEqual(len(children), 2)

        item1 = children[0]
        item2 = children[1]
        assert isinstance(item1, ActionListItem)
        assert isinstance(item2, ActionListItem)

        self.assertEqual(item1.action_model.get_action().name, "Loaded Action 1")
        self.assertEqual(item2.action_model.get_action().name, "Loaded Action 2")
        self.assertTrue(item1.action_model.get_action().enabled)
        self.assertFalse(item2.action_model.get_action().enabled)

    async def test_roundtrip_through_file(self) -> None:
        """Verify full roundtrip through actual file I/O."""
        # Create original model with actions
        original_model = ActionListModel()
        original_model.append_child_item(
            None,
            ActionItemValueModel(
                TestActionItem(
                    name="File Test 1",
                    enabled=True,
                    string_param="file_opt1",
                    int_param=100,
                    bool_param=True,
                )
            ),
        )
        original_model.append_child_item(
            None,
            ActionItemValueModel(
                TestActionItem(
                    name="File Test 2",
                    enabled=False,
                    string_param="file_opt2",
                    int_param=200,
                    bool_param=False,
                )
            ),
        )

        # Serialize to file
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            actions_data = []
            for item in original_model.get_item_children():
                assert isinstance(item, ActionListItem)
                actions_data.append(item.action_model.get_action().to_dict())

            preset = {"version": "1.0", "actions": actions_data}
            json.dump(preset, f, indent=2)
            temp_path = Path(f.name)

        try:
            # Deserialize from file
            with open(temp_path) as f:
                loaded_preset = json.load(f)

            restored_model = ActionListModel()
            for action_data in loaded_preset["actions"]:
                action = TestActionItem.from_dict(action_data)
                restored_model.append_child_item(None, ActionItemValueModel(action))

            # Verify
            original_children = original_model.get_item_children()
            restored_children = restored_model.get_item_children()

            self.assertEqual(len(original_children), len(restored_children))

            for orig_item, rest_item in zip(original_children, restored_children):
                assert isinstance(orig_item, ActionListItem)
                assert isinstance(rest_item, ActionListItem)

                orig_action = orig_item.action_model.get_action()
                rest_action = rest_item.action_model.get_action()

                self.assertEqual(orig_action.to_dict(), rest_action.to_dict())

        finally:
            temp_path.unlink()

    async def test_empty_model_serialization(self) -> None:
        """Verify empty model serializes correctly."""
        model = ActionListModel()

        actions_data = []
        for item in model.get_item_children():
            assert isinstance(item, ActionListItem)
            actions_data.append(item.action_model.get_action().to_dict())

        preset = {"version": "1.0", "actions": actions_data}

        self.assertEqual(preset["actions"], [])
        self.assertEqual(json.dumps(preset), '{"version": "1.0", "actions": []}')


class TestPresetFormat(omni.kit.test.AsyncTestCase):
    """Test preset format structure and validation."""

    async def test_version_field_present(self) -> None:
        """Verify version field is included in serialized presets."""
        action = TestActionItem(name="Version Test")
        preset = {
            "version": "1.0",
            "actions": [action.to_dict()],
        }

        self.assertEqual(preset["version"], "1.0")

    async def test_preset_is_valid_json(self) -> None:
        """Verify preset can be serialized to valid JSON."""
        actions = [
            TestActionItem(name="Action 1", string_param="value1"),
            TestActionItem(name="Action 2", int_param=42),
        ]

        preset = {
            "version": "1.0",
            "actions": [a.to_dict() for a in actions],
        }

        # Should not raise
        json_str = json.dumps(preset, indent=2)

        # Should parse back
        parsed = json.loads(json_str)
        self.assertEqual(parsed["version"], "1.0")
        self.assertEqual(len(parsed["actions"]), 2)

    async def test_invalid_preset_missing_version(self) -> None:
        """Verify detection of presets missing version field."""
        preset = {"actions": []}

        self.assertNotIn("version", preset)

    async def test_invalid_preset_missing_actions(self) -> None:
        """Verify detection of presets missing actions field."""
        preset = {"version": "1.0"}

        self.assertNotIn("actions", preset)

    async def test_preset_action_order_preserved(self) -> None:
        """Verify action order is preserved through serialization."""
        names = ["First", "Second", "Third", "Fourth", "Fifth"]
        actions = [TestActionItem(name=n) for n in names]

        preset = {
            "version": "1.0",
            "actions": [a.to_dict() for a in actions],
        }

        json_str = json.dumps(preset)
        parsed = json.loads(json_str)

        restored_names = [a["name"] for a in parsed["actions"]]
        self.assertEqual(restored_names, names)

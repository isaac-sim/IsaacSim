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

"""Tests for sensor icon viewport functionality."""

from pathlib import Path

import carb.settings
import omni.kit.app
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.usd
from omni.kit.viewport.utility import get_active_viewport, get_active_viewport_window
from omni.ui.tests.test_base import OmniUiTest
from pxr import Gf, Sdf

from ..impl.manipulator import SHOW_TITLE_PATH
from ..impl.scene import VISIBLE_SETTING, SensorIcon

CURRENT_PATH = Path(__file__).parent
TEST_DATA_PATH = CURRENT_PATH.parent.parent.parent.parent.parent.joinpath("data").joinpath("tests")
TEST_DATA_PATH_ICON = CURRENT_PATH.parent.parent.parent.parent.parent.joinpath("icons")
TEST_OBJECT_PRIM_PATH = "/test_obj"


# -- TRANSITION FOR SUPPORTING BOTH VP1 AND VP2
def is_viewport_legacy() -> bool:
    """Check if the active viewport is a legacy viewport.

    Returns:
        True if the active viewport exposes the legacy window API.
    """
    viewport_api = get_active_viewport()
    return hasattr(viewport_api, "legacy_window")


# -- TRANSITION END


class TestSensorIcon(OmniUiTest):
    """Test suite for sensor icon functionality."""

    async def setUp(self) -> None:
        """Set up test fixtures before each test."""
        await super().setUp()
        usd_context = omni.usd.get_context()
        await usd_context.new_stage_async()
        self._settings = carb.settings.get_settings()
        self._golden_img_dir = TEST_DATA_PATH.absolute().joinpath("golden_img").absolute()
        self._icon_scene = SensorIcon.get_instance()
        await ui_test.human_delay(8)
        await ui_test.wait_n_updates(300)
        if is_viewport_legacy():
            self._golden_img_dir = self._golden_img_dir.joinpath("legacy")

        # Set icon visibility to true, default is false
        self._settings.set(VISIBLE_SETTING, True)

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures after each test."""
        usd_context = omni.usd.get_context()
        await usd_context.close_stage_async()
        self._icon_scene.clear()
        self._icon_scene.destroy()
        await super().tearDown()

    async def _dock_viewport(self, width: int = 1200, height: int = 900, block_device: bool = False) -> None:
        """Utility function to dock viewport window and focus on turntable panel.

        Args:
            width: Width of the viewport window.
            height: Height of the viewport window.
            block_device: Whether to block input devices.
        """
        viewport = get_active_viewport_window()
        await self.docked_test_window(window=viewport, width=width, height=height, block_devices=block_device)
        await ui_test.wait_n_updates()

    async def test_sensoricon_default(self) -> None:
        """Test default sensor icon creation and manipulation."""
        create_test_object()
        self._settings.set(SHOW_TITLE_PATH, True)
        await ui_test.human_delay()
        await ui_test.wait_n_updates(30)
        path = TEST_DATA_PATH_ICON.absolute().joinpath("icoSensors.svg")
        self._icon_scene.add_sensor_icon(TEST_OBJECT_PRIM_PATH, str(path))
        await ui_test.wait_n_updates(30)
        model = self._icon_scene.get_model()
        self.assertAlmostEqual(model.get_world_unit(), 1.0)
        self.assertTrue(TEST_OBJECT_PRIM_PATH in model.get_prim_paths())
        save_path = model.get_icon_url(TEST_OBJECT_PRIM_PATH)
        self.assertEqual(save_path, str(path))
        pos = model.get_position(TEST_OBJECT_PRIM_PATH)
        self.assertEqual(Gf.Vec3d(0, 0, 0), Gf.Vec3d(pos))

        pos = self._icon_scene.show_sensor_icon(TEST_OBJECT_PRIM_PATH)
        await ui_test.wait_n_updates(30)
        sdf_path = Sdf.Path(TEST_OBJECT_PRIM_PATH)
        sensor_item = model._icons.get(sdf_path)
        self.assertTrue(sensor_item.visible)
        pos = self._icon_scene.hide_sensor_icon(TEST_OBJECT_PRIM_PATH)
        await ui_test.wait_n_updates(30)
        sensor_item = model._icons.get(sdf_path)
        self.assertFalse(sensor_item.visible)

        def click_fn(*_: object) -> None:
            pass

        self._icon_scene.set_icon_click_fn(TEST_OBJECT_PRIM_PATH, click_fn)
        self.assertEqual(model.get_on_click(TEST_OBJECT_PRIM_PATH), click_fn)
        self._icon_scene.remove_sensor_icon(TEST_OBJECT_PRIM_PATH)
        await ui_test.wait_n_updates(30)
        self.assertEqual(model.get_icon_url(TEST_OBJECT_PRIM_PATH), "")
        self._icon_scene.destroy()
        await ui_test.wait_n_updates(30)

    async def test_sensoricon_saved_stage(self) -> None:
        """Test sensor icon persistence across stage saves."""
        create_test_object()
        self._settings.set(SHOW_TITLE_PATH, True)
        await ui_test.human_delay()
        await ui_test.wait_n_updates(30)
        path = TEST_DATA_PATH_ICON.absolute().joinpath("icoSensors.svg")
        self._icon_scene.add_sensor_icon(TEST_OBJECT_PRIM_PATH, str(path))
        await ui_test.wait_n_updates(30)
        model = self._icon_scene.get_model()
        self.assertAlmostEqual(model.get_world_unit(), 1.0)
        saved = omni.usd.get_context().save_stage()
        self.assertTrue(saved)
        self._icon_scene.destroy()
        await ui_test.wait_n_updates(30)
        self._icon_scene = SensorIcon.get_instance()
        await ui_test.wait_n_updates(30)
        new_model = self._icon_scene.get_model()
        self.assertTrue(TEST_OBJECT_PRIM_PATH in new_model.get_prim_paths())
        retrieved_path = new_model.get_icon_url(TEST_OBJECT_PRIM_PATH)
        self.assertEqual(retrieved_path, str(path))

    async def test_sensoricon_path_types(self) -> None:
        """Tests handling of str and Sdf.Path for prim_path arguments."""
        create_test_object()
        self._settings.set(SHOW_TITLE_PATH, True)
        await ui_test.human_delay()
        await ui_test.wait_n_updates(30)

        icon_path_str = str(TEST_DATA_PATH_ICON.absolute().joinpath("icoSensors.svg"))
        prim_path_str = TEST_OBJECT_PRIM_PATH
        prim_path_sdf = Sdf.Path(TEST_OBJECT_PRIM_PATH)

        model = self._icon_scene.get_model()

        # Test add_sensor_icon
        self._icon_scene.add_sensor_icon(prim_path_str, icon_path_str)
        await ui_test.wait_n_updates(30)
        self.assertEqual(model.get_icon_url(prim_path_str), icon_path_str)
        self.assertEqual(model.get_icon_url(prim_path_sdf), icon_path_str)
        self._icon_scene.remove_sensor_icon(prim_path_str)
        await ui_test.wait_n_updates(30)
        self.assertEqual(model.get_icon_url(prim_path_str), "")

        self._icon_scene.add_sensor_icon(prim_path_sdf, icon_path_str)
        await ui_test.wait_n_updates(30)
        self.assertEqual(model.get_icon_url(prim_path_sdf), icon_path_str)
        self.assertEqual(model.get_icon_url(prim_path_str), icon_path_str)

        # Test show_sensor_icon and hide_sensor_icon
        sensor_item = model.get_item(prim_path_sdf)  # test with Sdf.Path
        self.assertTrue(sensor_item.visible)  # Should be visible by default after add

        self._icon_scene.hide_sensor_icon(prim_path_str)
        await ui_test.wait_n_updates(30)
        self.assertFalse(model.get_item(prim_path_sdf).visible)

        self._icon_scene.show_sensor_icon(prim_path_sdf)
        await ui_test.wait_n_updates(30)
        self.assertTrue(model.get_item(prim_path_str).visible)

        self._icon_scene.hide_sensor_icon(prim_path_sdf)
        await ui_test.wait_n_updates(30)
        self.assertFalse(model.get_item(prim_path_str).visible)

        self._icon_scene.show_sensor_icon(prim_path_str)
        await ui_test.wait_n_updates(30)
        self.assertTrue(model.get_item(prim_path_sdf).visible)

        # Test set_icon_click_fn - unused
        def click_fn_1(*_: object) -> None:
            pass

        self._icon_scene.set_icon_click_fn(prim_path_str, click_fn_1)
        self.assertEqual(model.get_on_click(prim_path_sdf), click_fn_1)

        # Test IconModel with str and Sdf.Path params
        self.assertEqual(model.get_icon_url(prim_path_str), icon_path_str)
        self.assertEqual(model.get_icon_url(prim_path_sdf), icon_path_str)

        pos_str = model.get_position(prim_path_str)
        pos_sdf = model.get_position(prim_path_sdf)
        self.assertIsNotNone(pos_str)
        self.assertIsNotNone(pos_sdf)
        self.assertEqual(Gf.Vec3d(pos_str), Gf.Vec3d(pos_sdf))  # check position

        # Test remove_sensor_icon
        self._icon_scene.remove_sensor_icon(prim_path_sdf)
        await ui_test.wait_n_updates(30)
        self.assertEqual(model.get_icon_url(prim_path_str), "")
        self.assertIsNone(model.get_item(prim_path_sdf))

        # Re-add to test removal with string path
        self._icon_scene.add_sensor_icon(prim_path_sdf, icon_path_str)
        await ui_test.wait_n_updates(30)
        self.assertEqual(model.get_icon_url(prim_path_str), icon_path_str)

        self._icon_scene.remove_sensor_icon(prim_path_str)
        await ui_test.wait_n_updates(30)
        self.assertEqual(model.get_icon_url(prim_path_sdf), "")
        self.assertIsNone(model.get_item(prim_path_str))

    async def test_sensoricon_usd_listening(self) -> None:
        """Tests USD listening and icon visibility."""
        create_test_object()
        self._settings.set(SHOW_TITLE_PATH, True)
        await ui_test.human_delay()
        await ui_test.wait_n_updates(30)
        icon_path_str = str(TEST_DATA_PATH_ICON.absolute().joinpath("icoSensors.svg"))
        prim_path_str = TEST_OBJECT_PRIM_PATH
        prim_path_sdf = Sdf.Path(TEST_OBJECT_PRIM_PATH)

        model = self._icon_scene.get_model()

        # add sensor icon
        self._icon_scene.add_sensor_icon(prim_path_str, icon_path_str)
        await ui_test.wait_n_updates(30)
        self.assertEqual(model.get_icon_url(prim_path_str), icon_path_str)
        self.assertTrue(model.get_item(TEST_OBJECT_PRIM_PATH).visible)
        settings = carb.settings.get_settings()
        # Globally turn off icon visibility
        settings.set(VISIBLE_SETTING, False)
        await ui_test.wait_n_updates(30)
        self.assertIsNone(
            model.get_item(TEST_OBJECT_PRIM_PATH), "Icon item should be None after hide_all as _icons is cleared"
        )
        # Ensure USD Listening is off
        self.assertFalse(model._usd_listening_active)
        # Turn on USD listening
        settings.set(VISIBLE_SETTING, True)
        await ui_test.wait_n_updates(30)
        self.assertTrue(model.get_item(TEST_OBJECT_PRIM_PATH).visible)


def create_test_object(
    prim_path: str | Sdf.Path = TEST_OBJECT_PRIM_PATH,
    prim_type: str = "IsaacContactSensor",
    attrs: dict[str, object] | None = None,
) -> None:
    """Create a test object prim with an icon position attribute.

    Args:
        prim_path: Path where the test prim should be created.
        prim_type: USD prim type to create.
        attrs: Optional default attributes to pass to the create command.
    """
    kwargs = {"prim_type": prim_type, "prim_path": prim_path}
    if attrs:
        kwargs["attributes"] = attrs
    omni.kit.commands.execute("CreatePrimWithDefaultXform", **kwargs)
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    attribute = prim.CreateAttribute("icon_position", Sdf.ValueTypeNames.Float3)
    attribute.Set(Gf.Vec3d(0, 0, 0))

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

"""UI tests for importing URDF from a ROS 2 node."""

import asyncio
import os
import shutil
import tempfile
import threading
import time

import omni.kit.ui_test as ui_test
import omni.usd
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from isaacsim.test.utils import find_widget_with_retry, get_widget_screen_center, menu_click_with_retry
from pxr import Sdf


async def _wait_for_widget_layout(widget_ref: object, timeout: float = 10.0) -> None:
    """Wait until a UI-test widget has usable on-screen click geometry."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        widget = widget_ref.widget
        center_x, center_y = get_widget_screen_center(widget)
        if (
            widget.visible
            and widget.computed_width > 0
            and widget.computed_height > 0
            and center_x > 0
            and center_y > 0
        ):
            return
        await omni.kit.app.get_app().next_update_async()

    raise TimeoutError(
        f"Widget did not receive usable click geometry within {timeout:.1f} seconds: "
        f"position=({widget.screen_position_x}, {widget.screen_position_y}), "
        f"size=({widget.computed_width}, {widget.computed_height}), visible={widget.visible}"
    )


class TestRos2UrdfNodeImporter(ROS2TestCase):
    """Test importing a URDF from a ROS 2 node via the UI."""

    async def setUp(self) -> None:
        """Prepare the UI test fixture."""
        await super().setUp()
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._urdf_path = os.path.normpath(
            os.path.join(
                self._extension_path,
                "data",
                "urdf",
                "tests",
                "test_basic.urdf",
            )
        )
        self._tmpdir = tempfile.mkdtemp(prefix="ros2_urdf_test_")
        self._prev_tempdir = tempfile.tempdir
        tempfile.tempdir = self._tmpdir
        self._success = False
        stage_utils.create_new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up after the test."""
        import rclpy

        if hasattr(self, "_server_stop_event"):
            self._server_stop_event.set()
        if hasattr(self, "_server_thread") and self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        if hasattr(self, "_executor") and hasattr(self, "_server_node"):
            try:
                self._executor.remove_node(self._server_node)
            except Exception:
                pass
        if hasattr(self, "_server_node"):
            try:
                self._server_node.destroy_node()
            except Exception:
                pass
        if hasattr(self, "_server_context"):
            try:
                rclpy.shutdown(context=self._server_context)
            except Exception:
                pass
        tempfile.tempdir = self._prev_tempdir
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)
        await super().tearDown()

    async def test_import_basic_urdf_from_ros2_node(self) -> None:
        """Import a URDF description from ROS 2 and validate the stage."""
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor

        with open(self._urdf_path, encoding="utf-8") as file:
            self._urdf_text = file.read()

        self._node_name = "robot_state_publisher"
        self._server_ready_event = threading.Event()
        self._server_stop_event = threading.Event()

        def _run_service() -> None:
            self._server_context = Context()
            rclpy.init(context=self._server_context)
            self._server_node = rclpy.create_node(self._node_name, context=self._server_context)
            self._server_node.declare_parameter("robot_description", self._urdf_text)
            self._executor = MultiThreadedExecutor(context=self._server_context)
            self._executor.add_node(self._server_node)
            self._server_ready_event.set()

            while not self._server_stop_event.is_set():
                self._executor.spin_once(timeout_sec=0.02)

        self._server_thread = threading.Thread(target=_run_service, daemon=True)
        self._server_thread.start()

        server_started = await asyncio.to_thread(self._server_ready_event.wait, 5.0)
        self.assertTrue(server_started, f"ROS 2 node '{self._node_name}' failed to start within 5 seconds")

        await omni.kit.app.get_app().next_update_async()

        await menu_click_with_retry("File/Import from ROS2 URDF Node", window_name="Import from ROS2 URDF Node")
        await omni.kit.app.get_app().next_update_async()

        string_field = await find_widget_with_retry(
            "Import from ROS2 URDF Node//Frame/**/StringField[*].identifier=='ros2_urdf_node_name'"
        )
        # The initial lookup is triggered by the field's end-edit callback. Wait
        # for usable geometry before input so UI automation cannot click at (0, 0).
        await _wait_for_widget_layout(string_field)
        await string_field.input(self._node_name)
        self.assertEqual(string_field.model.get_value_as_string(), self._node_name)

        import_button = await find_widget_with_retry(
            "Import from ROS2 URDF Node//Frame/**/Button[*].identifier=='ros2_urdf_import'",
            max_frames=300,
        )
        deadline = time.monotonic() + 10.0
        while not import_button.widget.enabled and time.monotonic() < deadline:
            await omni.kit.app.get_app().next_update_async()
        self.assertTrue(
            import_button.widget.enabled,
            "Import button was found but did not become enabled after fetching the ROS 2 robot_description.",
        )

        await _wait_for_widget_layout(import_button)
        await import_button.click()
        await ui_test.human_delay()

        # Wait for the USD stage to finish loading after the import (up to ~3 seconds).
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
            if omni.usd.get_context().get_stage_loading_status()[2] == 0:
                break

        stage = stage_utils.get_current_stage()
        self.assertIsNotNone(stage, "No USD stage is open after the URDF import")

        prim = stage.GetDefaultPrim()
        self.assertTrue(
            prim.IsValid(),
            f"Stage has no valid default prim after importing URDF from '{self._node_name}'. "
            "The URDF importer may have failed or the resulting stage was not opened.",
        )
        self.assertNotEqual(
            prim.GetPath(),
            Sdf.Path.emptyPath,
            f"Default prim path is empty after importing URDF from '{self._node_name}'.",
        )
        self._success = True

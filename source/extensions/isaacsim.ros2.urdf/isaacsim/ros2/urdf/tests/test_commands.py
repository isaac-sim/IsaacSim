# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the deprecated URDFImportFromROS2Node command."""

import shutil
import tempfile
import threading

import omni.kit.commands
import omni.kit.test
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from omni.client import Result


class TestURDFImportFromROS2NodeCommand(ROS2TestCase):
    """Test the deprecated URDFImportFromROS2Node omni.kit.commands command."""

    async def setUp(self) -> None:
        """Prepare the test fixture."""
        await super().setUp()
        self._tmpdir = tempfile.mkdtemp(prefix="ros2_urdf_cmd_test_")
        self._prev_tempdir = tempfile.tempdir
        tempfile.tempdir = self._tmpdir
        self._success = False
        stage_utils.create_new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up ROS 2 resources."""
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

    async def test_command_returns_ok(self) -> None:
        """Test that executing the deprecated command returns Result.OK."""
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor

        self._server_stop_event = threading.Event()
        server_ready = threading.Event()

        def _run_service() -> None:
            self._server_context = Context()
            rclpy.init(context=self._server_context)
            self._server_node = rclpy.create_node("robot_state_publisher", context=self._server_context)
            self._server_node.declare_parameter("robot_description", "<robot name='test'/>")
            self._executor = MultiThreadedExecutor(context=self._server_context)
            self._executor.add_node(self._server_node)
            server_ready.set()
            while not self._server_stop_event.is_set():
                self._executor.spin_once(timeout_sec=0.02)

        self._server_thread = threading.Thread(target=_run_service, daemon=True)
        self._server_thread.start()
        self.assertTrue(server_ready.wait(timeout=5.0), "ROS 2 node failed to start")
        await omni.kit.app.get_app().next_update_async()

        status, result = omni.kit.commands.execute(
            "URDFImportFromROS2Node",
            ros2_node_name="robot_state_publisher",
        )
        self.assertTrue(status)
        self.assertEqual(result, Result.OK)
        self._success = True

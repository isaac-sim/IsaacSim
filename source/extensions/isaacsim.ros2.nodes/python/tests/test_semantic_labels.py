# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies ROS 2 semantic label publishing from Isaac Sim scene semantics."""

from typing import Any

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import omni.kit.viewport.utility
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.utils import semantics as semantics_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase

from .common import get_qos_profile


class TestRos2SemanticLabels(ROS2TestCase):
    """Verify semantic label messages published from labeled scene prims."""

    async def setUp(self) -> None:
        """Create a fresh stage and label dictionary for semantic label tests."""
        await super().setUp()

        await omni.usd.get_context().new_stage_async()

        await omni.kit.app.get_app().next_update_async()
        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clear semantic label test state after each test."""
        await super().tearDown()

    async def test_semantic_labels(self) -> None:
        """Test semantic labels."""
        import json
        from collections import deque

        import rclpy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String

        BACKGROUND_USD_PATH = "/Isaac/Environments/Grid/default_environment.usd"

        # Add Small Warehouse environment to the stage
        result, error = await stage_utils.open_stage_async(self._assets_root_path + BACKGROUND_USD_PATH)
        await omni.kit.app.get_app().next_update_async()
        cube_1 = Cube("/cube_1", sizes=1.0, positions=[0, 0, 0], scales=[1.5, 1, 1])
        semantics_utils.add_labels(cube_1.prims[0], labels=["Cube0"])

        cube_2 = Cube("/cube_2", sizes=1.0, positions=[-4, 4, 0], scales=[1.5, 1, 1])
        semantics_utils.add_labels(cube_2.prims[0], labels=["Cube1"])

        viewport_window = omni.kit.viewport.utility.get_active_viewport_window()
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ClockPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                        ("GetRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CameraHelper.inputs:topicName", "semantic_segmentation"),
                        ("CameraHelper.inputs:type", "semantic_segmentation"),
                        ("CameraHelper.inputs:enableSemanticLabels", True),
                        ("CameraHelper.inputs:semanticLabelsTopicName", "semantic_labels"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "GetRenderProduct.inputs:execIn"),
                        ("GetRenderProduct.outputs:execOut", "CameraHelper.inputs:execIn"),
                        ("GetRenderProduct.outputs:renderProductPath", "CameraHelper.inputs:renderProductPath"),
                        ("OnPlaybackTick.outputs:tick", "ClockPublisher.inputs:execIn"),
                        ("IsaacClock.outputs:simulationTime", "ClockPublisher.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        await omni.kit.app.get_app().next_update_async()

        def clear_data() -> None:
            self._clock_data = deque(maxlen=5)
            self._clock_callback_count = 0
            self._semantic_label_data = None
            self._semantic_label_dict = None
            self._semantic_labels_callback_count = 0
            self._semantic_labels_timestamp = None

        def clock_callback(data: Any) -> None:
            self._clock_callback_count += 1
            self._clock_data.append(round(data.clock.sec + data.clock.nanosec / 1.0e9, 1))

        def semantic_labels_callback(data: Any) -> None:
            self._semantic_labels_callback_count += 1

            # Test that the labels are correct
            self._semantic_label_data = data.data
            self._semantic_label_dict = json.loads(data.data)
            classes = set()
            for label_id, label_info in self._semantic_label_dict.items():
                if label_id == "time_stamp":
                    continue
                self.assertIn("class", label_info)
                classes.add(label_info["class"])
            for expected_class in self._expected_classes:
                self.assertIn(expected_class, classes)
            for unexpected_class in self._unexpected_classes:
                self.assertNotIn(unexpected_class, classes)

            # Update latest received timestamp
            self._semantic_labels_timestamp = (
                self._semantic_label_dict["time_stamp"]["sec"]
                + self._semantic_label_dict["time_stamp"]["nanosec"] / 1.0e9
            )

        node = self.create_node("semantic_label_tester")
        clock_sub = self.create_subscription(node, Clock, "/clock", clock_callback, get_qos_profile())
        label_sub = self.create_subscription(
            node, String, "/semantic_labels", semantic_labels_callback, get_qos_profile()
        )

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        await omni.kit.app.get_app().next_update_async()

        clear_data()
        self._expected_classes = ["cube0"]
        self._unexpected_classes = ["cube1"]

        # Run first test, with camera pointing at cube0
        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "/clock", per_frame_callback=spin)
        await omni.syntheticdata.sensors.next_sensor_data_async(viewport_api)
        await self.simulate_until_condition(
            lambda: self._semantic_label_data is not None,
            max_frames=60,
            per_frame_callback=spin,
        )
        self._timeline.stop()
        self.assertGreater(self._clock_callback_count, 0)
        self.assertGreater(self._semantic_labels_callback_count, 0)
        self.assertIn(round(self._semantic_labels_timestamp, 1), self._clock_data)

        # Spin to clear the message buffers
        await omni.kit.app.get_app().next_update_async()
        spin()
        spin()
        spin()
        await omni.kit.app.get_app().next_update_async()

        # Run second test, with camera pointing at cube1
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=np.array([0, 0, 3]), target=np.array([-4, 4, 0]))
        await omni.kit.app.get_app().next_update_async()

        clear_data()
        self._expected_classes = ["cube1"]
        self._unexpected_classes = ["cube0"]

        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "/clock", per_frame_callback=spin)
        await self.simulate_until_condition(
            lambda: self._semantic_labels_callback_count > 5,
            max_frames=60,
            per_frame_callback=spin,
        )
        self._timeline.stop()
        self.assertGreater(self._clock_callback_count, 0)
        self.assertGreater(self._semantic_labels_callback_count, 0)
        self.assertIn(round(self._semantic_labels_timestamp, 1), self._clock_data)

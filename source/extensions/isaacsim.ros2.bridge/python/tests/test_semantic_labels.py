# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


import carb
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import omni.kit.viewport.utility
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.semantics import add_labels
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path_async

from .common import ROS2TestCase, get_qos_profile


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2SemanticLabels(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()

        await omni.usd.get_context().new_stage_async()

        await omni.kit.app.get_app().next_update_async()
        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))
        await omni.kit.app.get_app().next_update_async()

        pass

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    async def test_semantic_labels(self):
        import json
        from collections import deque

        import rclpy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String

        BACKGROUND_USD_PATH = "/Isaac/Environments/Grid/default_environment.usd"

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # Add Small Warehouse environment to the stage
        (result, error) = await open_stage_async(self._assets_root_path + BACKGROUND_USD_PATH)
        await omni.kit.app.get_app().next_update_async()
        cube_1 = VisualCuboid("/cube_1", position=[0, 0, 0], scale=[1.5, 1, 1])
        add_labels(cube_1.prim, labels=["Cube0"], instance_name="class")

        cube_2 = VisualCuboid("/cube_2", position=[-4, 4, 0], scale=[1.5, 1, 1])
        add_labels(cube_2.prim, labels=["Cube1"], instance_name="class")

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
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CameraHelper.inputs:viewport", viewport_window.title),
                        ("CameraHelper.inputs:topicName", "semantic_segmentation"),
                        ("CameraHelper.inputs:type", "semantic_segmentation"),
                        ("CameraHelper.inputs:enableSemanticLabels", True),
                        ("CameraHelper.inputs:semanticLabelsTopicName", "semantic_labels"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ClockPublisher.inputs:execIn"),
                        ("IsaacClock.outputs:simulationTime", "ClockPublisher.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        await omni.kit.app.get_app().next_update_async()

        self._clock_data = deque(maxlen=5)
        self._label_data = None

        def clear_data():
            self._clock_data.clear()
            self._label_data = None

        def clock_callback(data):
            self._clock_data.append(round(data.clock.sec + data.clock.nanosec / 1.0e9, 1))

        def semantic_labels_callback(data):
            self._label_data = data.data

        node = rclpy.create_node("semantic_label_tester")
        clock_sub = node.create_subscription(Clock, "/clock", clock_callback, get_qos_profile())
        label_sub = node.create_subscription(String, "/semantic_labels", semantic_labels_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        def find_class(label_dict, class_value):
            for label_id, label_info in label_dict.items():
                for key in label_info:
                    if key == "class":
                        if label_info[key] == class_value:
                            return True
            return False

        def find_timestamp(label_dict, clock_data):

            sec = int(label_dict["time_stamp"]["sec"])
            nanosec = int(label_dict["time_stamp"]["nanosec"])

            time_val = round(sec + nanosec / 1.0e9, 1)

            if clock_data.count(time_val) > 0:
                return True

            return False

        viewport_api = omni.kit.viewport.utility.get_active_viewport()

        await omni.kit.app.get_app().next_update_async()

        self.assertIsNone(self._label_data)
        self._timeline.play()

        await omni.syntheticdata.sensors.next_sensor_data_async(viewport_api)

        await simulate_async(1, 60, spin)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()

        self.assertIsNotNone(self._label_data)
        self.assertIsNotNone(self._clock_data)

        labels_dict = json.loads(self._label_data)
        print(labels_dict)
        self.assertTrue(find_class(labels_dict, "cube0"))
        self.assertFalse(find_class(labels_dict, "cube1"))

        self.assertTrue(find_timestamp(labels_dict, self._clock_data))

        # Point Camera towards the other box
        set_camera_view(eye=np.array([0, 0, 3]), target=np.array([-4, 4, 0]), camera_prim_path="/OmniverseKit_Persp")
        await omni.kit.app.get_app().next_update_async()

        clear_data()

        self.assertIsNone(self._label_data)

        self._timeline.play()

        await omni.syntheticdata.sensors.next_sensor_data_async(viewport_api)

        await simulate_async(1, 60, spin)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()

        self.assertIsNotNone(self._label_data)
        self.assertIsNotNone(self._clock_data)

        labels_dict = json.loads(self._label_data)

        self.assertTrue(find_class(labels_dict, "cube1"))
        self.assertFalse(find_class(labels_dict, "cube0"))

        self.assertTrue(find_timestamp(labels_dict, self._clock_data))

        pass

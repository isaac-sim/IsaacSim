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

"""Verifies ROS 2 object ID map publication from scene semantics and rendered sensor data."""

import json
from typing import Any
from uuid import uuid4

import omni
import omni.kit
import omni.replicator.core as rep
import rclpy
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from isaacsim.sensors.experimental.rtx import parse_stable_id_map_data
from std_msgs.msg import String

from .common import create_sarcophagus, get_qos_profile


class TestROS2ObjectIdMap(ROS2TestCase):
    """Verify object ID map messages generated from semantic scene labels."""

    async def setUp(self) -> None:
        """Create a fresh stage for object ID map publication tests."""
        await super().setUp()

        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        self._camera = rep.create.camera()
        self._render_product = rep.create.render_product(self._camera, (128, 128))

        self._annotator = rep.AnnotatorRegistry.get_annotator("StableIdMap")
        self._annotator.attach(self._render_product)
        self._annotator_data = None

        # Add cubes to the scene
        self._cubes = create_sarcophagus(enable_nonvisual_material=False)

        # Configure the ROS2 subscriber to capture the messsge
        self._ros_topic = f"topic_{uuid4().hex}"
        self._ros_msg_data = None
        self._ros_msg_type = String
        self._ros_node = self.create_node(f"subscriber_{self._ros_topic}")
        self._ros_msg_count = 0
        self._ros_msg_timestamp_prev = None
        self._ros_msg_queue_depth = 10

        def ros_callback(data: Any) -> None:
            self._ros_msg_data = data
            self._ros_msg_count += 1

            # Validate the message timestamp
            if self._ros_msg_timestamp_prev is not None:
                current_timestamp = self._ros_msg_data.header.stamp.sec + self._ros_msg_data.header.stamp.nanosec / 1e9
                expected_diff = 1 / 60
                self.assertAlmostEqual(current_timestamp, self._ros_msg_timestamp_prev + expected_diff)
                self._ros_msg_timestamp_prev = current_timestamp

        self._ros_sub = self.create_subscription(
            self._ros_node,
            self._ros_msg_type,
            self._ros_topic,
            ros_callback,
            get_qos_profile(depth=self._ros_msg_queue_depth),
        )

        # Configure the Writer to publish the message
        self._writer = rep.writers.get(f"ROS2PublishObjectIdMap")
        self._writer.initialize(
            nodeNamespace="",
            queueSize=self._ros_msg_queue_depth,
            topicName=self._ros_topic,
        )
        self._writer.attach(self._render_product)

    async def tearDown(self) -> None:
        """Stop the timeline after object ID map publication tests."""
        if self._annotator is not None:
            self._annotator.detach()
        if self._writer is not None:
            self._writer.detach()
        await super().tearDown()

    def spin(self) -> None:
        """Handle spin operation."""
        rclpy.spin_once(self._ros_node, timeout_sec=0.01)

    async def test_object_id_map(self) -> None:
        """Test object id map."""
        # Run the timeline to populate data
        self._timeline.play()
        condition_met = await self.simulate_until_condition(
            lambda: self._ros_msg_data is not None,
            max_frames=120,
            per_frame_callback=self.spin,
        )
        self.assertTrue(condition_met, "Timed out waiting for object-id-map ROS message")

        # Once ROS message is present, fetch annotator output for comparison.
        self._annotator_data = self._annotator.get_data()
        self._timeline.stop()

        self.assertIsNotNone(self._annotator_data)
        self.assertIsNotNone(self._ros_msg_data)

        # Convert the annotator data to a dictionary
        annotator_data_dict = parse_stable_id_map_data(self._annotator_data)

        # Resolve the ROS2 message data to a dictionary
        ros_msg_data_dict = json.loads(self._ros_msg_data.data)["id_to_labels"]

        # Convert annotator dict keys from int to str since JSON keys are always strings
        annotator_data_dict_str_keys = {str(k): v for k, v in annotator_data_dict.items()}

        self.assertEqual(annotator_data_dict_str_keys, ros_msg_data_dict)

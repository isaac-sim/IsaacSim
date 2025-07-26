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


import omni.kit.commands
import omni.kit.test
import omni.kit.usd

from .common import ROS2TestCase


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRclpy(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()
        await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        await super().tearDown()

    async def test_rclpy(self):
        import rclpy
        from std_msgs.msg import String

        msg = String()
        node = rclpy.create_node("minimal_publisher")
        publisher = node.create_publisher(String, "topic", 10)
        publisher.publish(msg)
        node.destroy_node()
        pass

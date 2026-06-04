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

"""Test UCX IMU publishing node functionality."""

import time

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni
import omni.graph.core as og
import ucxx._lib.libucxx as ucx_api
from isaacsim.ucx.nodes.messages.isaac import Imu
from isaacsim.ucx.nodes.tests.common import UCXTestCase, _read_tensor_f32
from ucxx._lib.arr import Array


def unpack_imu_message(buffer: object) -> tuple:
    """Unpack a UCX IMU FlatBuffers message.

    Args:
        buffer: Buffer containing the FlatBuffers-encoded IMU message.

    Returns:
        Tuple of (timestamp, frame_id, orientation, angular_velocity, linear_acceleration).
        orientation is [w, x, y, z] as float32 list, or None if absent.
        angular_velocity and linear_acceleration are (x, y, z) tuples, or None if absent.
    """
    buf = bytearray(buffer.tobytes())
    msg = Imu.Imu.GetRootAs(buf, 0)

    timestamp = msg.Header().Stamp().TimeNs() / 1e9
    frame_id = msg.Header().FrameId().decode("utf-8") if msg.Header().FrameId() else ""

    orientation = None
    if msg.Orientation() is not None:
        orientation = _read_tensor_f32(msg.Orientation())

    angular_velocity = None
    if msg.AngularVelocity() is not None:
        v = msg.AngularVelocity()
        angular_velocity = (v.X(), v.Y(), v.Z())

    linear_acceleration = None
    if msg.LinearAcceleration() is not None:
        v = msg.LinearAcceleration()
        linear_acceleration = (v.X(), v.Y(), v.Z())

    return timestamp, frame_id, orientation, angular_velocity, linear_acceleration


class TestUCXPublishImu(UCXTestCase):
    """Test UCX IMU publishing."""

    async def setUp(self) -> None:
        """Set up a new stage for IMU publishing tests."""
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def receive_imu_message(self, tag: int = 2, timeout_frames: int = 1000) -> tuple:
        """Receive and unpack an IMU message.

        Args:
            tag: UCX tag to receive on.
            timeout_frames: Maximum number of frames to wait.

        Returns:
            Tuple of unpacked IMU message data.
        """
        max_buffer_size = 1024
        buffer = np.empty(max_buffer_size, dtype=np.uint8)

        request = self.client_endpoint.tag_recv(Array(buffer), tag=ucx_api.UCXXTag(tag))

        for _ in range(timeout_frames):
            if request.completed:
                break
            time.sleep(0.001)
            await omni.kit.app.get_app().next_update_async()

        self.assertTrue(request.completed, "Did not receive IMU message")
        request.check_error()

        return unpack_imu_message(buffer)

    async def test_imu_basic(self) -> None:
        """Test basic IMU publishing."""
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishImu", "isaacsim.ucx.nodes.UCXPublishImu"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishImu.inputs:port", self.port),
                        ("PublishImu.inputs:tag", 2),
                        ("PublishImu.inputs:frameId", "test_imu"),
                        ("PublishImu.inputs:orientation", [0.0, 0.0, 0.0, 1.0]),
                        ("PublishImu.inputs:angularVelocity", [0.1, 0.2, 0.3]),
                        ("PublishImu.inputs:linearAcceleration", [0.0, 0.0, 9.81]),
                        ("PublishImu.inputs:timeoutMs", 1000),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishImu.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(f"Error creating graph: {e}")
            raise

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        await app_utils.update_app_async(steps=3)
        self.create_ucx_client(self.port)
        timestamp, frame_id, orientation, angular_vel, linear_accel = await self.receive_imu_message()

        print(f"Received IMU data:")
        print(f"  Timestamp: {timestamp}")
        print(f"  Frame ID: {frame_id}")
        print(f"  Orientation (w,x,y,z): {orientation}")
        print(f"  Angular velocity: {angular_vel}")
        print(f"  Linear acceleration: {linear_accel}")

        self.assertGreater(timestamp, 0.0)
        self.assertEqual(frame_id, "test_imu")

        # Orientation: input was [x=0, y=0, z=0, w=1] (IJKR), stored as [w, x, y, z]
        self.assertIsNotNone(orientation)
        self.assertAlmostEqual(orientation[0], 1.0, places=5)  # w
        self.assertAlmostEqual(orientation[1], 0.0, places=5)  # x
        self.assertAlmostEqual(orientation[2], 0.0, places=5)  # y
        self.assertAlmostEqual(orientation[3], 0.0, places=5)  # z

        self.assertIsNotNone(angular_vel)
        self.assertAlmostEqual(angular_vel[0], 0.1, places=4)
        self.assertAlmostEqual(angular_vel[1], 0.2, places=4)
        self.assertAlmostEqual(angular_vel[2], 0.3, places=4)

        self.assertIsNotNone(linear_accel)
        self.assertAlmostEqual(linear_accel[2], 9.81, places=1)

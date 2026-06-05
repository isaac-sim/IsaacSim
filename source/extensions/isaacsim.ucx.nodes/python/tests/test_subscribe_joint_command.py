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

"""Test UCX joint command subscribing node functionality."""

import flatbuffers
import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni
import omni.graph.core as og
import ucxx._lib.libucxx as ucx_api
from isaacsim.ucx.nodes.messages.isaac import Header, JointCommand, Tensor
from isaacsim.ucx.nodes.messages.isaac import Time as TimeMsg
from isaacsim.ucx.nodes.tests.common import UCXTestCase
from ucxx._lib.arr import Array


class TestUCXSubscribeJointCommand(UCXTestCase):
    """Test UCX joint command subscribing."""

    async def setUp(self) -> None:
        """Set up a new stage for joint command subscription tests."""
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def setup_ucx_client_with_listener(self) -> None:
        """Setup UCX client."""
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        self.create_ucx_client(self.port)
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

    def pack_joint_command_message(self, timestamp: float, positions: list, velocities: list, efforts: list) -> bytes:
        """Pack a joint command FlatBuffers message for UCX.

        Args:
            timestamp: Timestamp value for the message.
            positions: List of joint position values.
            velocities: List of joint velocity values.
            efforts: List of joint effort values.

        Returns:
            Packed binary buffer containing the FlatBuffers-encoded joint command message.
        """
        builder = flatbuffers.Builder(1024)

        def build_f32_tensor(values: list) -> int:
            data_bytes = np.array(values, dtype=np.float32).tobytes()
            Tensor.TensorStartDataVector(builder, len(data_bytes))
            for b in reversed(data_bytes):
                builder.PrependByte(b)
            data_vec = builder.EndVector()

            Tensor.TensorStartShapeVector(builder, 1)
            builder.PrependInt64(len(values))
            shape_vec = builder.EndVector()

            Tensor.TensorStart(builder)
            Tensor.TensorAddData(builder, data_vec)
            Tensor.TensorAddShape(builder, shape_vec)
            return Tensor.TensorEnd(builder)

        pos_tensor = build_f32_tensor(positions)
        vel_tensor = build_f32_tensor(velocities)
        eff_tensor = build_f32_tensor(efforts)

        TimeMsg.TimeStart(builder)
        TimeMsg.TimeAddTimeNs(builder, int(timestamp * 1e9))
        time_obj = TimeMsg.TimeEnd(builder)

        Header.HeaderStart(builder)
        Header.HeaderAddStamp(builder, time_obj)
        header_obj = Header.HeaderEnd(builder)

        JointCommand.JointCommandStart(builder)
        JointCommand.JointCommandAddHeader(builder, header_obj)
        JointCommand.JointCommandAddPosition(builder, pos_tensor)
        JointCommand.JointCommandAddVelocity(builder, vel_tensor)
        JointCommand.JointCommandAddEffort(builder, eff_tensor)
        joint_cmd = JointCommand.JointCommandEnd(builder)

        builder.Finish(joint_cmd)
        return bytes(builder.Output())

    async def send_joint_command(
        self, timestamp: float, positions: list, velocities: list, efforts: list, tag: int = 3
    ) -> None:
        """Send a joint command message via UCX.

        Args:
            timestamp: Timestamp value for the message.
            positions: List of joint position values.
            velocities: List of joint velocity values.
            efforts: List of joint effort values.
            tag: UCX tag to send on.
        """
        message = self.pack_joint_command_message(timestamp, positions, velocities, efforts)
        buffer = np.frombuffer(message, dtype=np.uint8)

        request = self.client_endpoint.tag_send(Array(buffer), tag=ucx_api.UCXXTag(tag))

        # Wait for send to complete
        import time

        for _ in range(1000):
            if request.completed:
                break
            time.sleep(0.001)
            await omni.kit.app.get_app().next_update_async()

        self.assertTrue(request.completed, "Failed to send joint command message")
        request.check_error()

    async def test_joint_command_basic(self) -> None:
        """Test basic joint command subscription."""
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeJointCommand", "isaacsim.ucx.nodes.UCXSubscribeJointCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("SubscribeJointCommand.inputs:port", self.port),
                        ("SubscribeJointCommand.inputs:tag", 3),
                        ("SubscribeJointCommand.inputs:timeoutMs", 10),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(f"Error creating graph: {e}")
            raise

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        await self.setup_ucx_client_with_listener()

        # Send a joint command
        test_timestamp = 1.5
        test_positions = [0.1, 0.2, 0.3]
        test_velocities = [0.5, 0.6, 0.7]
        test_efforts = [1.0, 2.0, 3.0]

        await self.send_joint_command(test_timestamp, test_positions, test_velocities, test_efforts)

        # Wait for message to be processed
        await app_utils.update_app_async(steps=30)

        # Read outputs from the subscriber node
        timestamp_attr = og.Controller.attribute("/ActionGraph/SubscribeJointCommand.outputs:timeStamp")
        position_attr = og.Controller.attribute("/ActionGraph/SubscribeJointCommand.outputs:positionCommand")
        velocity_attr = og.Controller.attribute("/ActionGraph/SubscribeJointCommand.outputs:velocityCommand")
        effort_attr = og.Controller.attribute("/ActionGraph/SubscribeJointCommand.outputs:effortCommand")

        timestamp = timestamp_attr.get()
        positions = position_attr.get()
        velocities = velocity_attr.get()
        efforts = effort_attr.get()

        print(f"Received joint command:")
        print(f"  Timestamp: {timestamp}")
        print(f"  Positions: {positions}")
        print(f"  Velocities: {velocities}")
        print(f"  Efforts: {efforts}")

        # Verify values
        self.assertAlmostEqual(timestamp, test_timestamp, places=5)
        self.assertEqual(len(positions), len(test_positions))

        for i in range(len(test_positions)):
            self.assertAlmostEqual(positions[i], test_positions[i], places=5)
            self.assertAlmostEqual(velocities[i], test_velocities[i], places=5)
            self.assertAlmostEqual(efforts[i], test_efforts[i], places=5)

    async def test_joint_command_multiple(self) -> None:
        """Test receiving multiple joint commands."""
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeJointCommand", "isaacsim.ucx.nodes.UCXSubscribeJointCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("SubscribeJointCommand.inputs:port", self.port),
                        ("SubscribeJointCommand.inputs:tag", 3),
                        ("SubscribeJointCommand.inputs:timeoutMs", 10),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(f"Error creating graph: {e}")
            raise

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        await self.setup_ucx_client_with_listener()

        # Send multiple commands
        position_attr = og.Controller.attribute("/ActionGraph/SubscribeJointCommand.outputs:positionCommand")

        for i in range(3):
            test_positions = [i * 0.1, i * 0.2, i * 0.3]
            test_velocities = [0.0, 0.0, 0.0]
            test_efforts = [0.0, 0.0, 0.0]

            await self.send_joint_command(float(i), test_positions, test_velocities, test_efforts)
            await app_utils.update_app_async(steps=12)

            positions = position_attr.get()
            print(f"Command {i}: received positions = {positions}")

            # Verify the latest command was received
            if len(positions) > 0:
                self.assertAlmostEqual(positions[0], test_positions[0], places=4)

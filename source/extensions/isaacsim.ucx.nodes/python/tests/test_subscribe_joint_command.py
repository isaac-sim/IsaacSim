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

import threading
import time

import flatbuffers
import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni
import omni.graph.core as og
import ucxx._lib.libucxx as ucx_api
from isaacsim.ucx.nodes.messages.holoscan.schema import (
    DLDataType,
    DLDevice,
    Header,
    Tensor,
)
from isaacsim.ucx.nodes.messages.isaac import JointCommand
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

    def pack_joint_command_message(
        self,
        timestamp: float,
        positions: list,
        velocities: list,
        efforts: list,
        include_gains: bool = False,
        empty_gains: bool = False,
    ) -> bytes:
        """Pack a joint command FlatBuffers message for UCX.

        Args:
            timestamp: Timestamp value for the message.
            positions: List of joint position values.
            velocities: List of joint velocity values.
            efforts: List of joint effort values.
            include_gains: Include optional gains.
            empty_gains: Encode optional gains as empty tensors.

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
            Tensor.TensorAddNdim(builder, 1)
            Tensor.TensorAddDtype(builder, DLDataType.CreateDLDataType(builder, 2, 32, 1))
            Tensor.TensorAddDevice(builder, DLDevice.CreateDLDevice(builder, 1, 0))
            return Tensor.TensorEnd(builder)

        pos_tensor = build_f32_tensor(positions)
        vel_tensor = build_f32_tensor(velocities)
        eff_tensor = build_f32_tensor(efforts)

        kp_tensor = build_f32_tensor([] if empty_gains else [30.0] * len(positions)) if include_gains else 0
        kd_tensor = build_f32_tensor([] if empty_gains else [1.0] * len(positions)) if include_gains else 0
        Header.HeaderStart(builder)
        Header.HeaderAddCaptureTimestampNs(builder, int(timestamp * 1e9))
        header_obj = Header.HeaderEnd(builder)

        JointCommand.JointCommandStart(builder)
        JointCommand.JointCommandAddHeader(builder, header_obj)
        JointCommand.JointCommandAddPosition(builder, pos_tensor)
        JointCommand.JointCommandAddVelocity(builder, vel_tensor)
        JointCommand.JointCommandAddEffort(builder, eff_tensor)
        if include_gains:
            JointCommand.JointCommandAddKp(builder, kp_tensor)
            JointCommand.JointCommandAddKd(builder, kd_tensor)
        joint_cmd = JointCommand.JointCommandEnd(builder)

        builder.Finish(joint_cmd, file_identifier=b"IJCM")
        return bytes(builder.Output())

    async def send_joint_command(
        self,
        timestamp: float,
        positions: list,
        velocities: list,
        efforts: list,
        tag: int = 3,
        include_gains: bool = False,
        empty_gains: bool = False,
    ) -> None:
        """Send a joint command message via UCX.

        Args:
            timestamp: Timestamp value for the message.
            positions: List of joint position values.
            velocities: List of joint velocity values.
            efforts: List of joint effort values.
            tag: UCX tag to send on.
            include_gains: Include optional gains.
            empty_gains: Encode optional gains as empty tensors.
        """
        message = self.pack_joint_command_message(timestamp, positions, velocities, efforts, include_gains, empty_gains)
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

        await self.send_joint_command(test_timestamp, test_positions, test_velocities, test_efforts, include_gains=True)

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

        np.testing.assert_allclose(
            og.Controller.attribute("/ActionGraph/SubscribeJointCommand.outputs:stiffness").get(), [30.0] * 3
        )
        np.testing.assert_allclose(
            og.Controller.attribute("/ActionGraph/SubscribeJointCommand.outputs:damping").get(), [1.0] * 3
        )

        # Verify values
        self.assertAlmostEqual(timestamp, test_timestamp, places=5)
        self.assertEqual(len(positions), len(test_positions))

        for i in range(len(test_positions)):
            self.assertAlmostEqual(positions[i], test_positions[i], places=5)
            self.assertAlmostEqual(velocities[i], test_velocities[i], places=5)
            self.assertAlmostEqual(efforts[i], test_efforts[i], places=5)

    def _start_background_sender(self, interval_s: float, tag: int = 3) -> tuple:
        """Publish a joint command on ``tag`` every ``interval_s`` seconds from a worker thread.

        Runs off the Kit update thread, so it keeps producing even while the main thread is
        stuck inside a blocking node ``compute()``.

        Args:
            interval_s: Delay between successive sends, in seconds.
            tag: UCX tag to send on.

        Returns:
            Tuple of (thread, stop_event). The caller must set the event and join
            the thread before tearing the client down.
        """
        message = self.pack_joint_command_message(0.25, [0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9])
        stop_event = threading.Event()

        def _run() -> None:
            while not stop_event.is_set():
                try:
                    buffer = np.frombuffer(message, dtype=np.uint8)
                    request = self.client_endpoint.tag_send(Array(buffer), tag=ucx_api.UCXXTag(tag))
                    while not request.completed and not stop_event.is_set():
                        time.sleep(0.001)
                except Exception:
                    # The endpoint may not be connected yet, or may already be torn
                    # down; either way keep trying until the test says to stop.
                    pass
                stop_event.wait(interval_s)

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()
        return thread, stop_event

    async def test_joint_command_default_timeout_does_not_block_update(self) -> None:
        """Verify the shipped ``timeoutMs`` default never blocks the Kit update thread.

        ``inputs:timeoutMs`` is left unset so the shipped default is exercised. A node that waits
        for a message inside ``compute()`` is paced by ``send_interval_s`` instead of frame time.
        """
        send_interval_s = 0.5
        steps = 10
        # A blocking receive paces the update loop at one frame per send (5.0 s total); a
        # non-blocking one runs at normal frame time. The budget sits between the two.
        budget_s = 2.0

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
                    # inputs:timeoutMs intentionally omitted - exercise the shipped default.
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
                ],
            },
        )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Let the listener come up before connecting. No compute() has seen a peer yet,
        # so a blocking node cannot block here.
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        # The sender must be live before the first tick that observes the connection: a blocking
        # node would otherwise stall in the settle loop below with nothing to release it, hanging
        # the runner instead of failing. No app update runs between these two lines.
        self.create_ucx_client(self.port)
        thread, stop_event = self._start_background_sender(send_interval_s)

        try:
            for _ in range(20):
                await omni.kit.app.get_app().next_update_async()

            start = time.perf_counter()
            await app_utils.update_app_async(steps=steps)
            elapsed = time.perf_counter() - start
        finally:
            stop_event.set()
            thread.join(timeout=5.0)

        # A leaked sender keeps calling into the UCX endpoint while tearDown() drops the client
        # worker. Fail here rather than let it crash an unrelated test later.
        self.assertFalse(thread.is_alive(), "Background sender thread did not stop; it would outlive this test")

        position_attr = og.Controller.attribute("/ActionGraph/SubscribeJointCommand.outputs:positionCommand")
        positions = position_attr.get()

        # Guard against a vacuous pass: if the client never connected the node
        # short-circuits before the receive and would look fast either way.
        self.assertGreater(len(positions), 0, "Subscriber never received a joint command; timing result is meaningless")

        self.assertLess(
            elapsed,
            budget_s,
            f"{steps} app updates took {elapsed:.2f} s (budget {budget_s:.2f} s, "
            f"sender interval {send_interval_s:.2f} s). The subscriber node is blocking "
            f"the Kit update thread inside its UCX receive.",
        )

    async def test_joint_command_tag_change_reposts_receive(self) -> None:
        """Verify a tag change re-posts the receive against the new configuration.

        ``inputs:timeoutMs`` is left at its default (0), so a posted receive stays outstanding until
        a message arrives. Reusing it after ``inputs:tag`` changes would leave the node listening
        on the old tag forever and publish a message that belongs to the old configuration.
        """
        old_tag = 3
        new_tag = 7

        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("SubscribeJointCommand", "isaacsim.ucx.nodes.UCXSubscribeJointCommand"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("SubscribeJointCommand.inputs:port", self.port),
                    ("SubscribeJointCommand.inputs:tag", old_tag),
                    # inputs:timeoutMs intentionally omitted - the default keeps a single
                    # receive outstanding, which is the configuration the defect needs.
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
                ],
            },
        )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Connect and settle, so the node has a receive outstanding on the old tag.
        await self.setup_ucx_client_with_listener()

        position_attr = og.Controller.attribute("/ActionGraph/SubscribeJointCommand.outputs:positionCommand")
        self.assertEqual(len(position_attr.get()), 0, "Nothing was sent yet, so no command should have been published")

        # Reconfigure the node while that receive is still outstanding.
        og.Controller.attribute("/ActionGraph/SubscribeJointCommand.inputs:tag").set(new_tag)
        await app_utils.update_app_async(steps=5)

        # A message on the old tag belongs to the old configuration and must not be published.
        stale_positions = [3.0, 3.1, 3.2]
        await self.send_joint_command(0.5, stale_positions, [0.0] * 3, [0.0] * 3, tag=old_tag)
        await app_utils.update_app_async(steps=20)

        self.assertEqual(
            len(position_attr.get()),
            0,
            f"A joint command sent on the old tag {old_tag} was published after the node moved to tag {new_tag}",
        )

        # A message on the new tag must be received.
        new_positions = [7.0, 7.1, 7.2]
        await self.send_joint_command(1.5, new_positions, [0.0] * 3, [0.0] * 3, tag=new_tag)
        await app_utils.update_app_async(steps=30)

        positions = position_attr.get()
        self.assertEqual(
            len(positions),
            len(new_positions),
            f"No joint command was received on the new tag {new_tag}; the node is still attached to tag {old_tag}",
        )
        for expected, actual in zip(new_positions, positions):
            self.assertAlmostEqual(actual, expected, places=4)

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

            await self.send_joint_command(
                float(i), test_positions, test_velocities, test_efforts, include_gains=(i < 2), empty_gains=(i == 1)
            )
            await app_utils.update_app_async(steps=12)

            positions = position_attr.get()
            print(f"Command {i}: received positions = {positions}")

            # Verify the latest command was received
            self.assertEqual(len(positions), len(test_positions))
            self.assertAlmostEqual(positions[0], test_positions[0], places=4)
            for output, gain in (("stiffness", 30.0), ("damping", 1.0)):
                np.testing.assert_allclose(
                    og.Controller.attribute(f"/ActionGraph/SubscribeJointCommand.outputs:{output}").get(),
                    [gain] * len(test_positions) if i == 0 else [],
                )

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

"""Test UCX joint state publishing node functionality."""

import time

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import ucxx._lib.libucxx as ucx_api
import usdrt.Sdf
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path
from isaacsim.ucx.nodes.messages.isaac import JointState
from ucxx._lib.arr import Array

from .common import UCXTestCase, _read_tensor_f32

# Test configuration constants
CONNECTION_WAIT_FRAMES = 60  # Frames to wait for node listener to initialize
CONNECTION_ESTABLISH_FRAMES = 20  # Additional frames for connection to establish
RECEIVE_TIMEOUT_FRAMES = 1000  # Maximum frames to wait for a message


def unpack_joint_state_message(buffer: object) -> tuple:
    """Unpack a UCX joint state FlatBuffers message.

    Args:
        buffer: Buffer containing the FlatBuffers-encoded joint state message.

    Returns:
        Tuple of (timestamp, num_joints, positions, velocities, efforts).
    """
    buf = bytearray(buffer.tobytes())
    msg = JointState.JointState.GetRootAs(buf, 0)
    timestamp = msg.Header().Stamp().TimeNs() / 1e9
    num_joints = int(msg.Position().Shape(0))
    positions = _read_tensor_f32(msg.Position())
    velocities = _read_tensor_f32(msg.Velocity())
    efforts = _read_tensor_f32(msg.Effort())
    return timestamp, num_joints, positions, velocities, efforts


class TestUCXPublishJointState(UCXTestCase):
    """Test UCX joint state publishing."""

    async def setup_ucx_client_with_listener(self) -> None:
        """Setup UCX client to connect to the OmniGraph node's listener.

        The OmniGraph nodes create their own internal listeners automatically.
        We create a client endpoint to connect and receive messages from them.
        This method ensures proper timing for connection establishment.
        """
        # Give Isaac Sim time to start the node's listener
        for _ in range(CONNECTION_WAIT_FRAMES):
            await omni.kit.app.get_app().next_update_async()

        # Create client connection using the base class helper
        self.create_ucx_client(self.port)

        # Give additional frames for the connection to establish
        for _ in range(CONNECTION_ESTABLISH_FRAMES):
            await omni.kit.app.get_app().next_update_async()

    async def setUp(self) -> None:
        """Set up stage with an articulation and UCX joint state publisher."""
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Load simple articulation asset
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise RuntimeError("Could not find Isaac Sim assets folder")

        self.usd_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        self._stage = await stage_utils.open_stage_async(self.usd_path)
        self.assertIsNotNone(self._stage)
        await omni.kit.app.get_app().next_update_async()

        # Create UCX publish joint state node
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishJointState", "isaacsim.ucx.nodes.UCXPublishJointState"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ReadJointState", "isaacsim.core.nodes.IsaacArticulationState"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ReadJointState.inputs:targetPrim", [usdrt.Sdf.Path("/Articulation")]),
                        ("PublishJointState.inputs:port", self.port),
                        ("PublishJointState.inputs:tag", 1),
                        ("PublishJointState.inputs:timeoutMs", 1000),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                        ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                        ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                        ("ReadJointState.outputs:measuredJointEfforts", "PublishJointState.inputs:jointEfforts"),
                    ],
                },
            )
        except Exception as e:
            print(f"Error creating graph: {e}")
            raise

        # Start timeline FIRST so the node executes and creates its listener
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Setup client with proper connection establishment
        await self.setup_ucx_client_with_listener()

    async def receive_joint_state_message(
        self, tag: int = 1, timeout_frames: int = RECEIVE_TIMEOUT_FRAMES, retry_count: int = 3
    ) -> tuple:
        """Receive and unpack a joint state message from the client endpoint.

        Args:
            tag: UCX tag to receive on (default: 1)
            timeout_frames: Maximum number of frames to wait per attempt (default: RECEIVE_TIMEOUT_FRAMES)
            retry_count: Number of times to retry receiving if it fails (default: 3)

        Returns:
            Tuple of (timestamp, num_joints, positions, velocities, efforts)

        Raises:
            AssertionError: If message is not received after all retry attempts
        """
        last_error = None

        for attempt in range(retry_count):
            try:
                max_buffer_size = 512
                buffer = np.empty(max_buffer_size, dtype=np.uint8)

                # Receive using the endpoint
                request = self.client_endpoint.tag_recv(Array(buffer), tag=ucx_api.UCXXTag(tag))

                # Progress until complete
                for frame in range(timeout_frames):
                    if request.completed:
                        break
                    time.sleep(0.001)
                    await omni.kit.app.get_app().next_update_async()

                # Check if completed
                if request.completed:
                    request.check_error()
                    return unpack_joint_state_message(buffer)
                else:
                    last_error = f"Timeout after {timeout_frames} frames on attempt {attempt + 1}"
                    if attempt < retry_count - 1:
                        print(f"Warning: {last_error}. Retrying...")
                        # Wait a bit before retrying
                        await omni.kit.app.get_app().next_update_async()
            except Exception as e:
                last_error = f"Exception on attempt {attempt + 1}: {e}"
                if attempt < retry_count - 1:
                    print(f"Warning: {last_error}. Retrying...")
                    await omni.kit.app.get_app().next_update_async()

        # All retries failed
        self.fail(
            f"Did not receive joint state message after {retry_count} attempts. "
            f"Last error: {last_error}. "
            "This may indicate a connection issue or the node is not publishing."
        )

    async def test_joint_state_publisher(self) -> None:
        """Test that joint state messages are published via UCX."""
        # Receive joint state message
        timestamp, num_joints, positions, velocities, efforts = await self.receive_joint_state_message()

        print(f"Received joint state: {num_joints} joints")
        print(f"  Timestamp: {timestamp}")
        print(f"  Positions: {positions}")
        print(f"  Velocities: {velocities}")
        print(f"  Efforts: {efforts}")

        # Verify we got data
        self.assertGreater(num_joints, 0, "Should have at least one joint")
        self.assertEqual(len(positions), num_joints)
        self.assertEqual(len(velocities), num_joints)
        self.assertEqual(len(efforts), num_joints)
        self.assertGreater(timestamp, 0.0, "Timestamp should be positive")

    async def test_joint_state_values(self) -> None:
        """Test that joint state values match simulation."""
        # Simulate to initialize physics
        await app_utils.update_app_async(steps=30)

        # Get articulation and initialize it
        articulation = Articulation("/Articulation")

        # Simulate for a few frames to let physics settle
        await app_utils.update_app_async(steps=30)

        # Get joint states from simulation
        sim_positions = articulation.get_dof_positions().numpy()[0]
        sim_velocities = articulation.get_dof_velocities().numpy()[0]

        # Receive joint state message from UCX
        # Note: Adding a small delay to ensure at least one message is sent after physics settles
        await omni.kit.app.get_app().next_update_async()
        timestamp, num_joints, ucx_positions, ucx_velocities, ucx_efforts = await self.receive_joint_state_message()

        # Verify values match (within tolerance)
        self.assertEqual(num_joints, len(sim_positions))

        print(f"Joint state comparison:")
        for i in range(num_joints):
            print(
                f"  Joint {i}: pos_sim={sim_positions[i]:.6f}, pos_ucx={ucx_positions[i]:.6f}, "
                f"vel_sim={sim_velocities[i]:.6f}, vel_ucx={ucx_velocities[i]:.6f}"
            )
            self.assertAlmostEqual(ucx_positions[i], sim_positions[i], places=3, msg=f"Joint {i} position mismatch")
            self.assertAlmostEqual(ucx_velocities[i], sim_velocities[i], places=3, msg=f"Joint {i} velocity mismatch")

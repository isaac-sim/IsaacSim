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

"""Test UCX odometry publishing node functionality."""

import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import ucxx._lib.libucxx as ucx_api
import usdrt.Sdf
from isaacsim.ucx.nodes.messages.isaac import Odometry
from ucxx._lib.arr import Array

from .common import UCXTestCase, _read_tensor_f32

# Test configuration constants
CONNECTION_WAIT_FRAMES = 60  # Frames to wait for node listener to initialize
CONNECTION_ESTABLISH_FRAMES = 20  # Additional frames for connection to establish


async def add_cube(path: str, size: float, offset: list) -> object:
    """Create a cube using experimental API.

    Args:
        path: USD path for the cube.
        size: Size of the cube (edge length).
        offset: Translation offset for the cube.

    Returns:
        The created cube geometry.
    """
    import omni.usd
    from isaacsim.core.experimental.objects import Cube
    from pxr import UsdPhysics

    # Create cube using experimental API with reset_xform_op_properties=True
    cube = Cube(path, sizes=[size], translations=[offset], reset_xform_op_properties=True)

    await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors

    # Apply physics
    stage = omni.usd.get_context().get_stage()
    cube_prim = stage.GetPrimAtPath(path)
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    rigid_api.CreateRigidBodyEnabledAttr(True)
    UsdPhysics.CollisionAPI.Apply(cube_prim)

    return cube


def unpack_odometry_message(buffer: object) -> tuple:
    """Unpack a UCX odometry FlatBuffers message.

    Args:
        buffer: Buffer containing the FlatBuffers-encoded odometry message.

    Returns:
        Tuple of (timestamp, position, orientation, linear_velocity, angular_velocity,
                linear_acceleration, angular_acceleration).
        position is [x, y, z], orientation is [w, x, y, z], velocities are [x, y, z].
        linear_acceleration and angular_acceleration are always (0, 0, 0) as they are
        not encoded in the Odometry schema.
    """
    buf = bytearray(buffer.tobytes())
    msg = Odometry.Odometry.GetRootAs(buf, 0)

    timestamp = msg.Header().Stamp().TimeNs() / 1e9

    pose = msg.Pose().Pose()
    position = _read_tensor_f32(pose.Position())
    orientation = _read_tensor_f32(pose.Orientation())

    twist = msg.Twist().Twist()
    linear_velocity = _read_tensor_f32(twist.Linear())
    angular_velocity = _read_tensor_f32(twist.Angular())

    # Acceleration is not in the Odometry schema
    linear_acceleration = (0.0, 0.0, 0.0)
    angular_acceleration = (0.0, 0.0, 0.0)

    return (
        timestamp,
        position,
        orientation,
        linear_velocity,
        angular_velocity,
        linear_acceleration,
        angular_acceleration,
    )


class TestUCXPublishOdometry(UCXTestCase):
    """Test UCX odometry publishing."""

    async def setUp(self) -> None:
        """Set up a new stage for odometry publishing tests."""
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        self.CUBE_SCALE = 0.5

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

    async def receive_odometry_message(self, tag: int = 7, timeout_frames: int = 1000, retry_count: int = 3) -> tuple:
        """Receive and unpack an odometry message from the client endpoint.

        Args:
            tag: UCX tag to receive on (default: 7)
            timeout_frames: Maximum number of frames to wait per attempt (default: 1000)
            retry_count: Number of times to retry receiving if it fails (default: 3)

        Returns:
            Tuple of (timestamp, position, orientation, linear_velocity, angular_velocity,
                    linear_acceleration, angular_acceleration)

        Raises:
            AssertionError: If message is not received after all retry attempts
        """
        import time

        last_error = None

        for attempt in range(retry_count):
            try:
                max_buffer_size = 1024
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
                    return unpack_odometry_message(buffer)
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
            f"Did not receive odometry message after {retry_count} attempts. "
            f"Last error: {last_error}. "
            "This may indicate a connection issue or the node is not publishing."
        )

    async def test_odometry_input_mode(self) -> None:
        """Test odometry publishing with direct inputs (ROS2-aligned mode)."""
        # Create graph with input-based odometry node
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishOdometry", "isaacsim.ucx.nodes.UCXPublishOdometry"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishOdometry.inputs:port", self.port),
                        ("PublishOdometry.inputs:tag", 7),
                        # Set some test values for inputs
                        ("PublishOdometry.inputs:position", [1.0, 2.0, 3.0]),
                        ("PublishOdometry.inputs:orientation", [0.0, 0.0, 0.0, 1.0]),  # IJKR
                        ("PublishOdometry.inputs:linearVelocity", [0.1, 0.2, 0.3]),
                        ("PublishOdometry.inputs:angularVelocity", [0.01, 0.02, 0.03]),
                        ("PublishOdometry.inputs:timeoutMs", 1000),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(f"Error creating graph: {e}")
            raise

        # Start timeline FIRST so the node executes and creates its listener
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        await self.setup_ucx_client_with_listener()

        # Receive odometry message
        timestamp, position, orientation, lin_vel, ang_vel, lin_accel, ang_accel = await self.receive_odometry_message()

        print(f"Received odometry (input mode):")
        print(f"  Timestamp: {timestamp}")
        print(f"  Position: {position}")
        print(f"  Orientation (w,x,y,z): {orientation}")
        print(f"  Linear velocity: {lin_vel}")
        print(f"  Angular velocity: {ang_vel}")

        # Verify we got data
        self.assertGreater(timestamp, 0.0, "Timestamp should be positive")

        # Since we're providing inputs, we should get relative values
        # (relative to starting pose which is the same as our inputs)
        # So relative position should be near zero
        self.assertAlmostEqual(position[0], 0.0, places=2)
        self.assertAlmostEqual(position[1], 0.0, places=2)
        self.assertAlmostEqual(position[2], 0.0, places=2)

    async def test_odometry_with_cube(self) -> None:
        """Test odometry publishing with a dynamic cube (input mode)."""
        # Create a dynamic cube with physics enabled
        await add_cube("/World/Cube", 1.0, (0, 0, 1.0))

        # Add rigid body physics to the cube so it can fall
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        cube_prim = stage.GetPrimAtPath("/World/Cube")
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)

        await omni.kit.app.get_app().next_update_async()

        # Create graph that reads cube transform and publishes via UCX
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishOdometry", "isaacsim.ucx.nodes.UCXPublishOdometry"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ReadTransform", "isaacsim.core.nodes.IsaacReadWorldPose"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishOdometry.inputs:port", self.port),
                        ("PublishOdometry.inputs:tag", 7),
                        ("PublishOdometry.inputs:timeoutMs", 1000),
                        ("ReadTransform.inputs:prim", [usdrt.Sdf.Path("/World/Cube")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                        ("ReadTransform.outputs:translation", "PublishOdometry.inputs:position"),
                        ("ReadTransform.outputs:orientation", "PublishOdometry.inputs:orientation"),
                    ],
                },
            )
        except Exception as e:
            print(f"Error creating graph: {e}")
            raise

        # Start timeline FIRST so the node executes and creates its listener
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        await self.setup_ucx_client_with_listener()

        # Receive odometry message
        timestamp, position, orientation, lin_vel, ang_vel, lin_accel, ang_accel = await self.receive_odometry_message()

        print(f"Received odometry from cube:")
        print(f"  Timestamp: {timestamp}")
        print(f"  Relative Position: {position}")
        print(f"  Orientation (w,x,y,z): {orientation}")

        # Verify we got valid data
        self.assertGreater(timestamp, 0.0, "Timestamp should be positive")

        # The position is relative to the starting pose (when the node first executed)
        # Since we start the simulation after the node is created, the cube may not have
        # moved much relative to its initial pose at the first frame
        # Just verify we got position data (could be zero or negative depending on timing)
        self.assertIsNotNone(position)
        self.assertEqual(len(position), 3, "Position should be a 3D vector")

    async def test_odometry_multiple_messages(self) -> None:
        """Test receiving multiple odometry messages over time."""
        # Create simple test setup
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishOdometry", "isaacsim.ucx.nodes.UCXPublishOdometry"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishOdometry.inputs:port", self.port),
                        ("PublishOdometry.inputs:tag", 7),
                        ("PublishOdometry.inputs:position", [0.0, 0.0, 0.0]),
                        ("PublishOdometry.inputs:orientation", [0.0, 0.0, 0.0, 1.0]),
                        ("PublishOdometry.inputs:timeoutMs", 1000),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(f"Error creating graph: {e}")
            raise

        # Start timeline FIRST so the node executes and creates its listener
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        await self.setup_ucx_client_with_listener()

        timestamps = []

        # Receive multiple messages
        for i in range(5):
            for _ in range(10):
                await omni.kit.app.get_app().next_update_async()

            timestamp, _, _, _, _, _, _ = await self.receive_odometry_message()
            timestamps.append(timestamp)
            print(f"Message {i+1}: timestamp = {timestamp}")

        # Verify timestamps are increasing
        for i in range(1, len(timestamps)):
            self.assertGreater(
                timestamps[i],
                timestamps[i - 1],
                f"Timestamp should increase (msg {i}: {timestamps[i]} <= msg {i-1}: {timestamps[i-1]})",
            )

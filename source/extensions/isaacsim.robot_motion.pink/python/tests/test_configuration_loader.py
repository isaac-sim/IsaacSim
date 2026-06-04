# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test suite for PINK configuration loader functions."""

import os
import tempfile

import numpy as np
import omni.kit.test
import pinocchio as pin
from isaacsim.robot_motion.pink import PinkRobot, load_pink_robot

_TEST_URDF = """\
<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
  </link>
  <link name="link1">
    <visual><geometry><box size="0.1 0.1 0.5"/></geometry></visual>
  </link>
  <link name="link2">
    <visual><geometry><box size="0.1 0.1 0.5"/></geometry></visual>
  </link>
  <link name="end_effector">
    <visual><geometry><box size="0.05 0.05 0.05"/></geometry></visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="end_effector"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
</robot>
"""


class TestConfigurationLoader(omni.kit.test.AsyncTestCase):
    """Test suite for PinkRobot loading from URDF."""

    async def setUp(self) -> None:
        self._tmpdir = tempfile.mkdtemp()
        self._urdf_path = os.path.join(self._tmpdir, "robot.urdf")
        with open(self._urdf_path, "w") as f:
            f.write(_TEST_URDF)

    async def tearDown(self) -> None:
        if os.path.exists(self._urdf_path):
            os.remove(self._urdf_path)
        if os.path.exists(self._tmpdir):
            os.rmdir(self._tmpdir)

    # ========================================================================
    # load_pink_robot
    # ========================================================================

    async def test_load_pink_robot_returns_pink_robot(self) -> None:
        """load_pink_robot returns a PinkRobot with correct types."""
        robot = load_pink_robot(self._urdf_path)
        self.assertIsInstance(robot, PinkRobot)
        self.assertIsInstance(robot.model, pin.Model)
        self.assertIsInstance(robot.data, pin.Data)
        self.assertIsInstance(robot.controlled_joint_names, list)

    async def test_load_pink_robot_joint_names(self) -> None:
        """Controlled joint names match the URDF joints."""
        robot = load_pink_robot(self._urdf_path)
        self.assertEqual(robot.controlled_joint_names, ["joint1", "joint2", "joint3"])

    async def test_load_pink_robot_model_dimensions(self) -> None:
        """Model nq and nv match expected 3-joint robot."""
        robot = load_pink_robot(self._urdf_path)
        self.assertEqual(robot.model.nq, 3)
        self.assertEqual(robot.model.nv, 3)

    async def test_load_pink_robot_neutral_configuration(self) -> None:
        """Neutral configuration is all zeros for revolute joints."""
        robot = load_pink_robot(self._urdf_path)
        q0 = robot.q0
        self.assertEqual(q0.shape, (3,))
        self.assertTrue(np.allclose(q0, 0.0))

    async def test_load_pink_robot_directory(self) -> None:
        """Directory is set to the URDF's parent."""
        robot = load_pink_robot(self._urdf_path)
        self.assertEqual(str(robot.directory), self._tmpdir)

    async def test_load_pink_robot_no_collision_by_default(self) -> None:
        """Collision model is None when build_collision_model is False."""
        robot = load_pink_robot(self._urdf_path, build_collision_model=False)
        self.assertIsNone(robot.collision_model)
        self.assertIsNone(robot.collision_data)

    async def test_load_pink_robot_with_collision(self) -> None:
        """Collision model is populated when build_collision_model is True."""
        robot = load_pink_robot(self._urdf_path, build_collision_model=True)
        self.assertIsNotNone(robot.collision_model)
        self.assertIsNotNone(robot.collision_data)

    async def test_load_pink_robot_string_path(self) -> None:
        """load_pink_robot accepts a string path."""
        robot = load_pink_robot(urdf_path=self._urdf_path)
        self.assertIsNotNone(robot)

    async def test_load_pink_robot_pathlib_path(self) -> None:
        """load_pink_robot accepts a pathlib.Path."""
        import pathlib

        robot = load_pink_robot(urdf_path=pathlib.Path(self._urdf_path))
        self.assertIsNotNone(robot)

    # ========================================================================
    # Error cases
    # ========================================================================

    async def test_load_nonexistent_urdf(self) -> None:
        """FileNotFoundError for a URDF that does not exist."""
        with self.assertRaises(FileNotFoundError):
            load_pink_robot("/nonexistent/path/robot.urdf")

    async def test_load_supported_robot_nonexistent(self) -> None:
        """FileNotFoundError for an unsupported robot name."""
        from isaacsim.robot_motion.pink import load_pink_supported_robot

        with self.assertRaises(FileNotFoundError):
            load_pink_supported_robot("definitely_not_a_robot")

    async def test_load_invalid_urdf_content(self) -> None:
        """RuntimeError or similar for malformed URDF content."""
        bad_path = os.path.join(self._tmpdir, "bad.urdf")
        with open(bad_path, "w") as f:
            f.write("this is not valid urdf xml")
        try:
            with self.assertRaises(Exception):
                load_pink_robot(bad_path)
        finally:
            os.remove(bad_path)

    # ========================================================================
    # Model consistency
    # ========================================================================

    async def test_two_loads_produce_equal_results(self) -> None:
        """Loading the same URDF twice yields identical joint names."""
        robot_a = load_pink_robot(self._urdf_path)
        robot_b = load_pink_robot(self._urdf_path)
        self.assertEqual(robot_a.controlled_joint_names, robot_b.controlled_joint_names)
        self.assertEqual(robot_a.model.nq, robot_b.model.nq)
        self.assertTrue(np.allclose(robot_a.q0, robot_b.q0))

    async def test_frames_present(self) -> None:
        """The Pinocchio model contains expected frames from the URDF."""
        robot = load_pink_robot(self._urdf_path)
        frame_names = [robot.model.frames[i].name for i in range(robot.model.nframes)]
        for name in ["base_link", "link1", "link2", "end_effector"]:
            self.assertIn(name, frame_names)

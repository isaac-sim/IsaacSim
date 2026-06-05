# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test suite for configuration loader functions."""

import pathlib

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.robot_motion.cumotion as cu_mg
import omni.kit.test


class TestConfigurationLoader(omni.kit.test.AsyncTestCase):
    """Test suite for robot configuration loading."""

    async def setUp(self) -> None:
        """Sets up the test environment before each test method runs."""

    async def tearDown(self) -> None:
        """Cleans up the test environment after each test method completes."""

    async def test_load_franka(self) -> None:
        """Tests loading Franka robot configurations through different methods.

        Verifies that loading a Franka robot from supported packages and from a file path
        produces equivalent configurations with matching controlled joint names.
        """
        # load franka from supported packages:
        franka_from_supported = cu_mg.load_cumotion_supported_robot("franka")
        self.assertIsNotNone(franka_from_supported)

        ext_directory = pathlib.Path(app_utils.get_extension_path("isaacsim.robot_motion.cumotion"))

        # load franka from path:
        franka_from_path = cu_mg.load_cumotion_robot(
            directory=ext_directory / "robot_configurations" / "franka",
        )

        self.assertEqual(franka_from_path.controlled_joint_names, franka_from_supported.controlled_joint_names)

    # TODO: port more robots from cuMotion repository.

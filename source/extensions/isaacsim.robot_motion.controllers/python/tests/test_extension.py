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

"""Verify that Kit imports controllers from the staged wheel."""

import isaacsim.robot_motion.controllers as controllers
import omni.kit.test


class TestExtension(omni.kit.test.AsyncTestCase):
    """Validate the staged controller package origin."""

    async def test_package_is_loaded_from_wheel(self) -> None:
        """Verify the public package resolves from pip_prebundle with its API intact."""
        module_path = getattr(controllers, "__file__", None)
        self.assertIsNotNone(module_path)
        self.assertIn("pip_prebundle", module_path)
        for symbol in ("AckermannController", "DifferentialDriveController", "HolonomicController"):
            self.assertTrue(hasattr(controllers, symbol), f"Missing public controller: {symbol}")

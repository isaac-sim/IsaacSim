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

"""Tests for Asset Validator rule registration."""

import omni.kit.test
from omni.asset_validator.core import ValidationRulesRegistry


class TestRuleRegistration(omni.kit.test.AsyncTestCase):
    """Verify that the extension exposes all validation rule sources."""

    async def test_simready_tier_rules_are_registered(self) -> None:
        """SimReady core and Isaac tier rules appear in the Asset Validator registry."""
        self.assertGreater(len(ValidationRulesRegistry.rules("BaseArticulation")), 0)
        self.assertGreater(len(ValidationRulesRegistry.rules("RobotCore")), 0)

    async def test_isaac_sim_rules_are_registered(self) -> None:
        """Isaac Sim runtime and sensor rules appear in the Asset Validator registry."""
        for category in (
            "IsaacSim.Sensor.Shared",
            "IsaacSim.Sensor.Camera",
            "IsaacSim.Sensor.Camera.Depth",
            "IsaacSim.Sensor.Camera.ISP",
            "IsaacSim.Sensor.Camera.Lens",
            "IsaacSim.Sensor.Lidar",
            "IsaacSim.Sensor.Radar",
            "IsaacSim.Sensor.SPG",
        ):
            self.assertGreater(len(ValidationRulesRegistry.rules(category)), 0, category)
        self.assertGreater(len(ValidationRulesRegistry.rules("BaseArticulation")), 1)

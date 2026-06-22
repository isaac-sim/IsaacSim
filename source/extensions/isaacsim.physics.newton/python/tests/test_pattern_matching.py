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

"""Verifies wildcard and bracket pattern matching used by Newton view path selection. The tests cover simple wildcard, environment-index, and bracket-expanded path patterns."""

import re

import omni.kit.test


class TestPatternMatching(omni.kit.test.AsyncTestCase):
    """Test pattern matching for articulation views."""

    async def test_wildcard_pattern(self) -> None:
        """Test wildcard pattern conversion to regex."""
        pattern = "Franka_*"
        regex = f"^{pattern.replace('*', '.*')}$"

        self.assertEqual(regex, "^Franka_.*$")

        matcher = re.compile(regex)
        self.assertTrue(matcher.match("Franka_1"))
        self.assertTrue(matcher.match("Franka_2"))
        self.assertTrue(matcher.match("Franka_test"))
        self.assertFalse(matcher.match("Robot_1"))
        self.assertFalse(matcher.match("Franka"))

    async def test_bracket_pattern(self) -> None:
        """Test bracket pattern conversion to regex."""
        pattern = "Franka_[1-2]"
        regex = f"^{pattern.replace('*', '.*')}$"

        self.assertEqual(regex, "^Franka_[1-2]$")

        matcher = re.compile(regex)
        self.assertTrue(matcher.match("Franka_1"))
        self.assertTrue(matcher.match("Franka_2"))
        self.assertFalse(matcher.match("Franka_3"))
        self.assertFalse(matcher.match("Franka_test"))

    async def test_simple_wildcard(self) -> None:
        """Test simple wildcard pattern."""
        pattern = "Robot*"
        regex = f"^{pattern.replace('*', '.*')}$"

        self.assertEqual(regex, "^Robot.*$")

        matcher = re.compile(regex)
        self.assertTrue(matcher.match("Robot1"))
        self.assertTrue(matcher.match("RobotA"))
        self.assertTrue(matcher.match("Robot"))
        self.assertFalse(matcher.match("Franka"))

    async def test_env_pattern(self) -> None:
        """Test environment pattern matching."""
        pattern = "env_*"
        regex = f"^{pattern.replace('*', '.*')}$"

        self.assertEqual(regex, "^env_.*$")

        matcher = re.compile(regex)
        self.assertTrue(matcher.match("env_0"))
        self.assertTrue(matcher.match("env_1"))
        self.assertTrue(matcher.match("env_test"))
        self.assertFalse(matcher.match("environment"))
        self.assertFalse(matcher.match("env"))

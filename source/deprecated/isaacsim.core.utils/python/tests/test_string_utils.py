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

"""Tests for string utility functions."""

import omni.kit.test
from isaacsim.core.utils.string import find_root_prim_path_from_regex


class TestStringUtils(omni.kit.test.AsyncTestCase):
    """Test cases for string utilities."""

    async def test_find_root_prim_path_from_regex_pattern(self) -> None:
        """Test regex paths still return the parent of the regex component."""
        root_path, tree_level = find_root_prim_path_from_regex("/World/envs/env_[0-9]+/Robot")

        self.assertEqual(root_path, "/World/envs")
        self.assertEqual(tree_level, 3)

    async def test_find_root_prim_path_from_regex_plain_path(self) -> None:
        """Test plain USD paths return a concrete path and level."""
        root_path, tree_level = find_root_prim_path_from_regex("/World/Cube")

        self.assertEqual(root_path, "/World/Cube")
        self.assertEqual(tree_level, 2)

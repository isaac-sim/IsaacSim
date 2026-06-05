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

"""Test for prim."""

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.core.experimental.prims import Prim


class TestPrim(omni.kit.test.AsyncTestCase):
    """Test prim."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_resolve_paths(self):
        """Test resolve paths."""
        # create new stage
        await stage_utils.create_new_stage_async()
        # define prims
        for i in range(3):
            stage_utils.define_prim(f"/World/A_{i}", "Xform")
            stage_utils.define_prim(f"/World/A_{i}/B", "Cube")
        # test cases (valid results)
        # - single path
        existing_paths, nonexistent_paths = Prim.resolve_paths("/World/A_0")
        assert len(existing_paths) == 1 and len(nonexistent_paths) == 0
        # - single path (non-existing)
        existing_paths, nonexistent_paths = Prim.resolve_paths("/World/C")
        assert len(existing_paths) == 0 and len(nonexistent_paths) == 1
        # - regex
        existing_paths, nonexistent_paths = Prim.resolve_paths("/World/A_.*/B")
        assert len(existing_paths) == 3 and len(nonexistent_paths) == 0
        # test cases
        # - mixed paths
        with self.assertRaises(ValueError):
            Prim.resolve_paths(["/World/A_.*", "/World/C"])
        # no existing or non-existing paths exist
        with self.assertRaises(ValueError):
            Prim.resolve_paths("/World/A_.*/C")
        # incomplete existing paths
        with self.assertRaises(ValueError):
            Prim.resolve_paths(["/World/A_.*/B", "/World/A_.*/C"])
        # incomplete non-existing paths
        with self.assertRaises(ValueError):
            Prim.resolve_paths(["/World/C", "/World/A_.*/C"])

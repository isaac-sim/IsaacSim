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

"""Test stage utils functionality."""

import os
import tempfile

import omni.kit.test
import omni.usd
from isaacsim.asset.importer.utils.impl import stage_utils


class TestStageUtils(omni.kit.test.AsyncTestCase):
    """Test helpers in :mod:`isaacsim.asset.importer.utils.impl.stage_utils`."""

    async def setUp(self) -> None:
        """Create a new stage before each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_stage_utils(self) -> None:
        """Save the current stage to disk and open it again."""
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp_dir:
            usd_path = os.path.join(tmp_dir, "test_stage_utils.usd")
            stage = omni.usd.get_context().get_stage()
            self.assertTrue(stage_utils.save_stage(stage, usd_path))
            stage = stage_utils.open_stage(usd_path)
            stage_id = stage_utils.get_stage_id(stage)
            self.assertIsNotNone(stage)
            self.assertIsInstance(stage_id, int)
            self.assertGreaterEqual(stage_id, 0)

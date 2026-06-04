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

"""Test suite for verifying Newton pip archive imports."""

import omni.kit.test


class TestPipArchive(omni.kit.test.AsyncTestCase):
    """Test that all Newton pip archive packages can be imported."""

    async def test_import_all(self):
        """Test importing all Newton-related packages."""
        import cbor2
        import mujoco
        import mujoco_warp
        import newton

        self.assertIsNotNone(cbor2)
        self.assertIsNotNone(mujoco)
        self.assertIsNotNone(mujoco_warp)
        self.assertIsNotNone(newton)

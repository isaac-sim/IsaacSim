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

"""Smoke tests for isaacsim.examples.ipc native plugin load."""

import omni.kit.test
from isaacsim.examples.ipc.bindings._isaacsim_examples_ipc import acquire_example_ipc_interface


class TestNodeExamplesPlugin(omni.kit.test.AsyncTestCase):
    """Smoke tests for the native IPC plugin interface."""

    async def test_acquire_plugin_interface(self) -> None:
        """Acquire the native plugin interface."""
        # Verify the Carbonite plugin loaded successfully and returns a valid interface.
        # The extension owns the plugin lifecycle; releasing from a test context would
        # decrement the shared per-client refcount to zero and unload the plugin for all
        # subsequent tests.
        iface = acquire_example_ipc_interface()
        self.assertIsNotNone(iface)

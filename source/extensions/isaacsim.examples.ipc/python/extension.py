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
"""Loads the native plugin that registers example OmniGraph nodes."""

import carb
import omni.ext

from .bindings._isaacsim_examples_ipc import acquire_example_ipc_interface, release_example_ipc_interface


class ExamplesIpcExtension(omni.ext.IExt):
    """Extension entry point for isaacsim.examples.ipc."""

    def on_startup(self, ext_id: str) -> None:
        """Acquire the native IPC plugin interface."""
        carb.log_info("isaacsim.examples.ipc starting up")
        self._interface = acquire_example_ipc_interface()
        carb.log_info("isaacsim.examples.ipc started")

    def on_shutdown(self) -> None:
        """Release the native IPC plugin interface."""
        carb.log_info("isaacsim.examples.ipc shutting down")
        if getattr(self, "_interface", None) is not None:
            release_example_ipc_interface(self._interface)
            self._interface = None
        carb.log_info("isaacsim.examples.ipc shut down")

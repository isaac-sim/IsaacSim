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
"""HSB Core Extension — manages lifecycle of the native HSB backend plugin."""

import carb
import omni.ext


class HsbCoreExtension(omni.ext.IExt):
    """HSB Core Extension class."""

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is loaded.

        Args:
            ext_id: Extension identifier provided by Kit.
        """
        carb.log_info("HSB Core Extension starting up")

        from .bindings._hsb_core import acquire_interface

        self._interface = acquire_interface()

        carb.log_info("HSB Core Extension started")

    def on_shutdown(self) -> None:
        """Called when the extension is unloaded."""
        carb.log_info("HSB Core Extension shutting down")

        from .bindings._hsb_core import release_interface

        if hasattr(self, "_interface") and self._interface is not None:
            release_interface(self._interface)
            self._interface = None

        carb.log_info("HSB Core Extension shut down")

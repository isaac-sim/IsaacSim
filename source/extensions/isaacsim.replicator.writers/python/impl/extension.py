# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension class for managing the lifecycle of the isaacsim.replicator.writers.impl extension."""

import omni.ext


class Extension(omni.ext.IExt):
    """Object that tracks the lifetime of the Python part of the extension loading."""

    def on_startup(self) -> None:
        """Set up initial conditions for the Python part of the extension."""
        from isaacsim.replicator.writers.scripts.writers import register_writers

        register_writers()

    def on_shutdown(self) -> None:
        """Shutting down this part of the extension prepares it for hot reload."""

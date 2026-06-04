# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension module that provides behavior-based functionality for Isaac Sim Replicator."""

import omni.ext


class Extension(omni.ext.IExt):
    """Extension for the isaacsim.replicator.behavior module.

    This extension provides behavior-based functionality for Isaac Sim Replicator, enabling
    advanced simulation scenarios with dynamic object behaviors and interactions.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is started.

        Args:
            ext_id: The extension identifier.
        """

    def on_shutdown(self) -> None:
        """Called when the extension is stopped."""

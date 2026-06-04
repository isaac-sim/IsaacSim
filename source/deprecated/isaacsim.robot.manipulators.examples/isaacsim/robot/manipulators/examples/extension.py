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

"""Extension entry point for robot manipulator examples."""

import omni.ext


class Extension(omni.ext.IExt):
    """An extension that provides examples and demonstrations for robot manipulator functionality in Isaac Sim.

    This extension serves as a collection of practical examples showcasing how to work with robot manipulators
    within the Isaac Sim environment. It demonstrates various manipulator operations, configurations, and
    interactions that can be used as reference implementations or starting points for robotics applications.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension starts up.

        Args:
            ext_id: The extension ID.
        """

    def on_shutdown(self) -> None:
        """Called when the extension shuts down."""

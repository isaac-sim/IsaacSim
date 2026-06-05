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

"""Experimental manipulator examples for Isaac Sim."""

from __future__ import annotations

import omni.ext


class Extension(omni.ext.IExt):
    """Experimental manipulator examples for Isaac Sim.

    Provides example implementations using the experimental APIs for Franka and UR10 manipulators,
    including pick-and-place, follow target, and stacking tasks.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension starts up.

        Args:
            ext_id: The extension ID.
        """

    def on_shutdown(self) -> None:
        """Called when the extension shuts down."""

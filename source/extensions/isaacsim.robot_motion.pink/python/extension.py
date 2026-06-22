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


"""Extension for PINK inverse kinematics integration within Isaac Sim."""

from typing import Any

import omni.ext


class Extension(omni.ext.IExt):
    """Extension for PINK inverse kinematics integration.

    This extension enables PINK (Python Inverse Kinematics) capabilities within Isaac Sim,
    providing differential IK solving via Pinocchio and QP solvers. It implements the motion
    generation BaseController interface for reactive end-effector tracking with configurable
    tasks, limits, and safety barriers.
    """

    def on_startup(self, ext_id: Any) -> None:
        """Startup the extension.

        Args:
            ext_id: The extension ID.
        """

    def on_shutdown(self) -> None:
        """Shutdown the extension."""

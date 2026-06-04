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

"""Base utilities for end effector controllers.

Provides shared functionality used by all controller types:
- EndEffectorValidationResult for structured feedback
- Articulation root discovery
"""

from __future__ import annotations

from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import get_current_stage
from pxr import Sdf, UsdPhysics


class EndEffectorValidationResult:
    """Result of end effector validation check."""

    def __init__(self) -> None:
        self.is_valid: bool = True
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def add_error(self, message: str) -> None:
        """Add an error and marks result as invalid."""
        self.errors.append(message)
        self.is_valid = False

    def add_warning(self, message: str) -> None:
        """Add a warning without invalidating the result."""
        self.warnings.append(message)

    def get_summary(self) -> str:
        """Return a formatted summary of validation results."""
        lines = []
        if self.errors:
            lines.append("Errors:")
            for err in self.errors:
                lines.append(f"  - {err}")
        if self.warnings:
            lines.append("Warnings:")
            for warn in self.warnings:
                lines.append(f"  - {warn}")
        if self.is_valid and not self.warnings:
            lines.append("End effector is valid for use.")
        return "\n".join(lines)


def find_owning_articulation_root(prim_path: str) -> str | None:
    """Find the ArticulationRootAPI prim that owns a given prim path."""
    try:
        art_paths = Articulation.fetch_articulation_root_api_prim_paths(prim_path)
        if art_paths:
            return art_paths[0]
    except Exception:
        pass

    stage = get_current_stage()
    if not stage:
        return None

    path = Sdf.Path(prim_path)
    while path != Sdf.Path.absoluteRootPath:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(path)
        path = path.GetParentPath()
    return None

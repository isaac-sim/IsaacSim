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

"""Provides access to the Extension class for transformer rules functionality."""

from pathlib import Path as _Path

from .extension import Extension, discover_rule_classes, register_all_rules  # noqa: F401


def _resolve_default_profile_path() -> str:
    """Locate the profile JSON, preferring the wheel layout over Kit's.

    Returns:
        Absolute path to the default profile JSON.

    """
    here = _Path(__file__).parent
    name = "isaacsim_structure.json"
    # Wheel layout: <package>/data/  (symlink to ../../../../data at wheel-build).
    # Kit layout:   <extension>/data/  (parents[3] from this file = extension root).
    candidates = (here / "data" / name, here.parents[3] / "data" / name)
    return str(next((p for p in candidates if p.is_file()), candidates[-1]))


DEFAULT_PROFILE_PATH = _resolve_default_profile_path()
"""Absolute path to the default Isaac Sim asset-structure profile shipped with this extension."""

__all__ = ["DEFAULT_PROFILE_PATH", "discover_rule_classes", "register_all_rules"]

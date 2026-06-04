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

"""Property widget for the IsaacMotionPlanningAPI schema."""

from __future__ import annotations

from dataclasses import dataclass

from isaacsim.robot_motion.schema import (
    MOTION_PLANNING_API_NAME,
    MOTION_PLANNING_ENABLED_ATTR,
    apply_motion_planning_api,
)
from pxr import Sdf, Usd

from .robot_schema import _RobotSchemaWidgetBase, _singleton


@dataclass(frozen=True)
class _SchemaAttribute:
    """Container describing a schema attribute.

    Args:
        name: Name of the schema attribute.
        display_name: Display name for the schema attribute.
        type: Value type name for the schema attribute.
    """

    name: str
    display_name: str
    type: Sdf.ValueTypeName


@dataclass(frozen=True)
class _SchemaClass:
    """Container describing a schema class identifier.

    Args:
        value: The schema class identifier value.
    """

    value: str


_MOTION_PLANNING_SCHEMA_CLASS = _SchemaClass(MOTION_PLANNING_API_NAME)
_MOTION_PLANNING_ENABLED = _SchemaAttribute(
    MOTION_PLANNING_ENABLED_ATTR,
    "Collision Enabled",
    Sdf.ValueTypeNames.Bool,
)


@_singleton
class MotionPlanningAPIWidget(_RobotSchemaWidgetBase):
    """Widget that exposes IsaacMotionPlanningAPI properties.

    Args:
        title: Widget title shown in the property window.
        collapsed: Whether the widget starts collapsed.
    """

    _MENU_PREFIX = "Isaac/Motion Planning"
    """Menu prefix used for organizing Motion Planning related menu items."""

    def __init__(self, title: str, collapsed: bool = False) -> None:
        super().__init__(
            title,
            collapsed,
            _MOTION_PLANNING_SCHEMA_CLASS,
            [
                _MOTION_PLANNING_ENABLED,
            ],
            "Motion Planning API",
            apply_motion_planning_api,
            exclusive_classes=(_MOTION_PLANNING_SCHEMA_CLASS,),
        )

    def _prim_has_schema(self, prim: object) -> bool:
        """Checks if the prim has the IsaacMotionPlanningAPI schema applied.

        Args:
            prim: The USD prim to check.

        Returns:
            True if the prim has the motion planning schema applied.
        """
        if not prim:
            return False
        return MOTION_PLANNING_API_NAME in prim.GetAppliedSchemas()

    def _get_prim(self, prim_path: object) -> Usd.Prim | None:
        """Retrieves a prim at the given path if it has the IsaacMotionPlanningAPI schema applied.

        Args:
            prim_path: The path to the prim in the USD stage.

        Returns:
            The prim if it exists and has the motion planning schema applied, otherwise None.
        """
        if prim_path:
            stage = self._payload.get_stage()
            if stage:
                prim = stage.GetPrimAtPath(prim_path)
                if prim and self._prim_has_schema(prim):
                    return prim
        return None

    def _has_exclusive_schema(self, prim: object) -> bool:
        """Checks if the prim has the exclusive IsaacMotionPlanningAPI schema.

        Args:
            prim: The USD prim to check.

        Returns:
            True if the prim has the exclusive motion planning schema.
        """
        return self._prim_has_schema(prim)

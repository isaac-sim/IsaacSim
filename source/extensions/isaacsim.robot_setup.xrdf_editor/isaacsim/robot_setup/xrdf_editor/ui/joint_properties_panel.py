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

"""Joint properties panel: per-DOF position, accel/jerk, and active/fixed toggle."""

from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame, DropDown, FloatField
from isaacsim.gui.components.ui_utils import get_style

if TYPE_CHECKING:
    from ..editor_state import EditorState


class JointPropertiesPanel:
    """Per-joint UI for default position, acceleration/jerk limits, and active/fixed status.

    Args:
        state: Shared editor state.
    """

    def __init__(self, state: "EditorState") -> None:
        self._state = state
        self._frame: CollapsableFrame | None = None
        self._joint_frames: list[CollapsableFrame] = []
        self._set_joint_positions_on_step = False

    def build(self) -> None:
        """Build the collapsable joint-properties frame."""
        self._frame = CollapsableFrame("Set Joint Properties", build_fn=self._build_frame_body)
        self._frame.enabled = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def show(self) -> None:
        """Show, expand, enable, and rebuild the panel."""
        if self._frame is None:
            return
        self._frame.collapsed = False
        self.rebuild()

    def hide(self) -> None:
        """Collapse and disable the panel."""
        if self._frame is None:
            return
        self._frame.collapsed = True
        self._frame.enabled = False
        for joint_frame in self._joint_frames:
            joint_frame.rebuild()

    def rebuild(self) -> None:
        """Rebuild the joint UI (e.g. after an XRDF import changes active joints)."""
        if self._frame is None:
            return
        self._frame.enabled = True
        self._frame.rebuild()
        if self._state.articulation is not None:
            self._state.articulation.set_dof_positions(self._state.joint_positions)

    def consume_pending_set_joint_positions(self) -> bool:
        """Return True (once) if the user changed a joint position since last poll.

        The orchestrator's physics-step callback uses this to apply the position
        change to the articulation and zero its velocity.

        Returns:
            True if a pending joint-position update was consumed.
        """
        if self._set_joint_positions_on_step:
            self._set_joint_positions_on_step = False
            return True
        return False

    # ------------------------------------------------------------------
    # Build helpers
    # ------------------------------------------------------------------
    def _build_frame_body(self) -> None:
        self._joint_frames = []
        if self._state.articulation is None:
            return

        with ui.VStack(style=get_style(), spacing=5, height=0):
            for i in range(self._state.articulation.num_dofs):
                frame = CollapsableFrame(
                    self._state.articulation.dof_names[i],
                    build_fn=partial(self._build_joint_frame, i),
                    collapsed=False,
                )
                self._joint_frames.append(frame)

    def _build_joint_frame(self, i: int) -> None:
        if self._state.articulation is None or i >= self._state.articulation.num_dofs:
            return

        dof_name = self._state.articulation.dof_names[i]
        if dof_name in self._state.mimic_joint_names:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                ui.Label(
                    f"Mimic joint (read-only). Position: {float(self._state.joint_positions[i]):.4f}",
                    tooltip=(
                        "Position is derived from a reference joint via the mimic relationship "
                        "and is omitted from XRDF / Lula exports."
                    ),
                )
            return

        lower, upper = self._state.articulation.get_dof_limits()
        lower_joint_limit = lower.numpy()[0, i]
        upper_joint_limit = upper.numpy()[0, i]

        with ui.VStack(style=get_style(), spacing=5, height=0):
            FloatField(
                "Joint Position",
                default_value=float(self._state.joint_positions[i]),
                tooltip=(
                    "If an active joint, this indicates a default position.  "
                    "If a fixed joint, this indicates a fixed position."
                ),
                on_value_changed_fn=partial(self._on_set_joint_position, i),
                lower_limit=lower_joint_limit,
                upper_limit=upper_joint_limit,
            )

            if self._state.active_joints[i]:
                FloatField(
                    "Acceleration Limit",
                    tooltip="Maximum acceleration that can be commanded for this joint.",
                    default_value=float(self._state.acceleration_limits[i]),
                    lower_limit=0.0001,
                    on_value_changed_fn=partial(self._on_max_acceleration_changed, i),
                )
                FloatField(
                    "Jerk Limit",
                    tooltip="Maximum jerk that can be commanded for this joint.",
                    default_value=float(self._state.jerk_limits[i]),
                    lower_limit=0.0001,
                    step=1.0,
                    on_value_changed_fn=partial(self._on_max_jerk_changed, i),
                )

            joint_status = DropDown(
                "Joint Status",
                tooltip=(
                    "Active Joint: Lula will directly control this joint, using a default position equal "
                    "to the value set above.\n"
                    "Fixed Joint: Lula will assume a fixed position of the joint equal to the value set above."
                ),
                populate_fn=lambda: ["Fixed Joint", "Active Joint"],
            )
            joint_status.repopulate()
            joint_status.set_selection_by_index(int(self._state.active_joints[i]))
            joint_status.set_on_selection_fn(partial(self._on_update_active_joints, i))

    # ------------------------------------------------------------------
    # Per-field callbacks
    # ------------------------------------------------------------------
    def _on_set_joint_position(self, i: int, value: float) -> None:
        self._state.joint_positions[i] = value
        self._set_joint_positions_on_step = True

    def _on_max_acceleration_changed(self, i: int, value: float) -> None:
        self._state.acceleration_limits[i] = value

    def _on_max_jerk_changed(self, i: int, value: float) -> None:
        self._state.jerk_limits[i] = value

    def _on_update_active_joints(self, i: int, value: str) -> None:
        is_active = value == "Active Joint"
        self._state.active_joints[i] = is_active
        if 0 <= i < len(self._joint_frames):
            self._joint_frames[i].rebuild()

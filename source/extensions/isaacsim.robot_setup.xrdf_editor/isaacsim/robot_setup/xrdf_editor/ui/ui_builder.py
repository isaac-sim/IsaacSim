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

"""User interface builder module for the Isaac Sim XRDF editor extension."""

from __future__ import annotations

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.timeline

from .. import articulation_discovery
from ..editor_state import EditorState
from .editor_tools_panel import EditorToolsPanel
from .info_panel import InfoPanel
from .joint_properties_panel import JointPropertiesPanel
from .selection_panel import SelectionPanel
from .sphere_editor_panel import SphereEditorPanel


class UIBuilder:
    """User interface builder for the XRDF editor extension.

    Owns the :class:`EditorState` domain object and the five panels that expose
    its functionality, and acts as the orchestrator between them:

    * Stage / timeline / physics events are routed in by the owning
      :class:`Extension` and dispatched to the appropriate panels.
    * Articulation- and link-selection callbacks emitted by panels are routed
      back into :class:`EditorState` and other panels.

    The class follows the standard ``isaacsim`` UI-extension contract: panels
    are constructed in :py:meth:`__init__`, the actual ``omni.ui`` widgets are
    materialised in :py:meth:`build_ui` whenever the window becomes visible,
    and :py:meth:`cleanup` is invoked on extension shutdown.

    Args:
        ext_id: Owning extension identifier.
        source_file: Path to the extension's ``extension.py`` used to locate
            the info-panel logo asset.
    """

    def __init__(self, ext_id: str, source_file: str) -> None:
        # Frame and wrapped-UI-element bookkeeping, kept to mirror the standard
        # isaacsim UIBuilder contract. Wrapped element cleanup is delegated to
        # the panels which own the actual ``element_wrappers`` instances.
        self.frames: list = []
        self.wrapped_ui_elements: list = []

        self._timeline = omni.timeline.get_timeline_interface()
        self._prev_art_prim_path: str | None = None

        # Domain state.
        self.state = EditorState()

        # UI panels (constructed now; ``omni.ui`` widgets created in build_ui()).
        self._info_panel = InfoPanel(ext_id, source_file)
        self._selection_panel = SelectionPanel(
            self.state,
            on_articulation_selected=self._on_articulation_selected_from_ui,
            on_link_selected=self._on_link_selected_from_ui,
            find_articulation_paths=lambda: articulation_discovery.find_all_articulation_base_paths(
                stage_utils.get_current_stage()
            ),
        )
        self._joint_properties_panel = JointPropertiesPanel(self.state)
        self._sphere_editor_panel = SphereEditorPanel(self.state)
        self._editor_tools_panel = EditorToolsPanel(
            self.state,
            get_selected_link_name=self._sphere_editor_panel.get_selected_link_name,
            get_selected_link_path=self._sphere_editor_panel.get_selected_link_path,
            refresh_sphere_comboboxes=self._sphere_editor_panel.refresh_collision_sphere_comboboxes,
            rebuild_joint_properties=self._joint_properties_panel.rebuild,
        )

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self) -> None:
        """Refresh panel state when the extension window is opened from the menu."""
        if not self._timeline.is_stopped():
            self._selection_panel.refresh_articulations()
            self._select_articulation_path(self._selection_panel.get_selected_articulation_path())

    def on_timeline_event(self, event: object) -> None:
        """No-op timeline callback.

        The extension reacts to stage ``SIMULATION_START_PLAY`` /
        ``SIMULATION_STOP_PLAY`` events (see
        :py:meth:`on_simulation_start_play` and
        :py:meth:`on_simulation_stop_play`) instead of the global timeline
        events. Kept for interface parity with other isaacsim UIBuilders.

        Args:
            event: Timeline event payload.
        """

    def on_physics_step(self, step: float) -> None:
        """Apply any pending joint-position changes requested by the joint panel.

        Args:
            step: Physics step size in seconds (unused).
        """
        if self.state.articulation is None:
            return
        if self._joint_properties_panel.consume_pending_set_joint_positions():
            joint_velocities = np.zeros_like(self.state.joint_positions)
            self.state.articulation.set_dof_positions(self.state.joint_positions)
            self.state.articulation.set_dof_velocities(joint_velocities)

    def on_selection_changed(self, event: object) -> None:
        """Refresh the articulation combobox when stage selection changes.

        Args:
            event: Stage selection-changed event payload (unused).
        """
        self._selection_panel.refresh_articulations()
        self.state.collision_sphere_editor.copy_all_sphere_data()
        self._sphere_editor_panel.refresh_collision_sphere_comboboxes(keep_sphere_selection=True)

    def on_stage_opened(self, event: object) -> None:
        """Refresh available articulations when the stage is opened.

        Args:
            event: Stage opened event payload (unused).
        """
        self._selection_panel.refresh_articulations()

    def on_stage_closed(self, event: object) -> None:
        """No-op handler for stage-closed events.

        Args:
            event: Stage closed event payload (unused).
        """

    def on_simulation_start_play(self, event: object) -> None:
        """Begin tracking the selected articulation when simulation starts.

        Args:
            event: Stage simulation-start-play event payload (unused).
        """
        self._selection_panel.refresh_articulations()
        self._select_articulation_path(self._selection_panel.get_selected_articulation_path())

    def on_simulation_stop_play(self, event: object) -> None:
        """Tear down articulation state when simulation stops.

        Args:
            event: Stage simulation-stop-play event payload (unused).
        """
        if self._timeline.is_stopped():
            self._reset_panels()
            self._select_articulation_path("None")

    def cleanup(self) -> None:
        """Release resources held by the panels and the domain state.

        Called on extension shutdown. Restores any hidden visibility, tears
        down the :class:`CollisionSphereEditor`, and clears wrapped UI
        elements.
        """
        self._editor_tools_panel.show_robot_if_hidden()
        self.state.on_shutdown()
        self._prev_art_prim_path = None
        for ui_elem in self.wrapped_ui_elements:
            if hasattr(ui_elem, "cleanup"):
                ui_elem.cleanup()

    def build_ui(self) -> None:
        """Build the panel widgets inside the currently-open container frame.

        Called each time the window becomes visible. If the domain state
        already holds a selected articulation (e.g. the user hid then reopened
        the window during playback) the panels are repopulated to match.
        """
        self._info_panel.build()
        self._selection_panel.build()
        self._joint_properties_panel.build()
        self._sphere_editor_panel.build()
        self._editor_tools_panel.build()

        if self.state.articulation is not None and not self._timeline.is_stopped():
            self._refresh_after_selection()

    ###################################################################################
    #           Private orchestration helpers
    ###################################################################################

    def _on_articulation_selected_from_ui(self, prim_path: str) -> None:
        """Handle a new articulation picked by the selection panel.

        Args:
            prim_path: Selected articulation path.
        """
        self._select_articulation_path(prim_path)

    def _on_link_selected_from_ui(self, link_name: str) -> None:
        """Handle a new link picked by the selection panel.

        Args:
            link_name: Selected link subpath.
        """
        self._sphere_editor_panel.on_link_selected(link_name)
        self._editor_tools_panel.on_link_selected(link_name)

    def _select_articulation_path(self, prim_path: str) -> None:
        if prim_path == self._prev_art_prim_path:
            return
        if not self._timeline.is_stopped():
            self._prev_art_prim_path = prim_path

        if self._selection_panel.articulation_list and prim_path != "None" and not self._timeline.is_stopped():
            self.state.select_articulation(prim_path)
            self._refresh_after_selection()
        else:
            if self.state.articulation is not None:
                self._editor_tools_panel.show_robot_if_hidden()
                self._reset_panels()
                self._selection_panel.refresh_articulations()
            self.state.select_articulation(None)

    def _refresh_after_selection(self) -> None:
        """Update every panel after a new articulation has been chosen."""
        self._selection_panel.refresh_links()
        self._joint_properties_panel.show()
        self._sphere_editor_panel.show()
        self._editor_tools_panel.show()
        self._editor_tools_panel.update_import_button_states()
        self._sphere_editor_panel.refresh_collision_sphere_comboboxes()

    def _reset_panels(self) -> None:
        self._selection_panel.clear_articulations()
        self._joint_properties_panel.hide()
        self._sphere_editor_panel.hide()
        self._editor_tools_panel.hide()
        self._prev_art_prim_path = None

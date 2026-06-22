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

"""Main Teleop window - composes independent panel modules."""

import carb.eventdispatcher
import omni.kit.app
import omni.ui as ui
import omni.usd
from isaacsim.replicator.teleop import (
    STAGE_STATE_LOADING,
    STAGE_STATE_NO_STAGE,
    STAGE_STATE_READY,
    FloatingRigidBodyController,
    MarkersManager,
    TeleopCommand,
    TeleopManager,
    TeleopProfile,
    activate_pre_session_anchor,
    get_last_teleop_profile_path,
    restore_pre_session_anchor,
    save_teleop_profile,
)
from isaacsim.replicator.teleop.controllers import GraspController, LocomotionController, RobotIKController

from .floating_panel import FloatingPanel
from .grasp_panel import GraspPanel
from .ik_panel import IKPanel
from .locomotion_panel import LocomotionPanel
from .session_panel import SessionPanel
from .teleop_profile_panel import TeleopProfilePanel


class TeleopWindow(ui.Window):
    """Main window for the Teleop UI extension.

    Composes six panels: Profiles, Session, Floating, IK, Grasp, Locomotion.
    The Profiles panel manages unified teleop profile save/load/validate; all
    other panels are independent — only the :class:`TeleopManager` is shared.

    Recording and replay live in the standalone
    ``isaacsim.replicator.episode_recorder.ui`` extension
    (*Tools > Replicator > Episode Recorder*). While :class:`TeleopManager` is
    alive, teleop controller / head-pose channels are attached to any session
    opened from that window via the session-injector hook registered by
    :func:`install_teleop_session_injector
    <isaacsim.replicator.teleop.install_teleop_session_injector>`.

    Args:
        title: Window title.
    """

    def __init__(self, title: str) -> None:
        super().__init__(title, dockPreference=ui.DockPreference.MAIN)
        self.deferred_dock_in("Property", ui.DockPolicy.DO_NOTHING)

        self._pre_session_anchor_active = False
        self._teleop_manager = TeleopManager()
        self._markers_manager = MarkersManager()
        self._floating_controller = FloatingRigidBodyController()
        self._ik_controller = RobotIKController()
        self._grasp_controller = GraspController()
        self._locomotion_controller = LocomotionController()

        self._teleop_manager.set_markers_manager(self._markers_manager)
        self._teleop_manager.set_floating_controller(self._floating_controller)
        self._teleop_manager.set_ik_controller(self._ik_controller)
        self._teleop_manager.set_grasp_controller(self._grasp_controller)
        self._teleop_manager.set_locomotion_controller(self._locomotion_controller)

        self._collapsed_states: dict[str, bool] = {}
        self._last_profile_path = get_last_teleop_profile_path()
        self._sub_shutdown = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_POST_QUIT,
            on_event=self._on_editor_quit_event,
            observer_name="isaacsim.replicator.teleop.ui._on_editor_quit_event",
            order=0,
        )
        self._teleop_profile_panel = TeleopProfilePanel(
            collect_profile=self.collect_teleop_profile,
            apply_profile=self.apply_teleop_profile,
            collapsed_states=self._collapsed_states,
        )
        self._session_panel = SessionPanel(self._teleop_manager, self._markers_manager, self._collapsed_states)
        self._floating_panel = FloatingPanel(self._floating_controller, self._teleop_manager, self._collapsed_states)
        self._ik_panel = IKPanel(self._ik_controller, self._teleop_manager, self._collapsed_states)
        self._ik_controller.set_on_status_changed(self._ik_panel.on_reachability_changed)
        self._grasp_panel = GraspPanel(self._grasp_controller, self._teleop_manager, self._collapsed_states)
        self._locomotion_panel = LocomotionPanel(
            self._locomotion_controller, self._teleop_manager, self._collapsed_states
        )

        self._teleop_manager.set_on_stage_closing(self._on_stage_closing)
        self._teleop_manager.set_on_command_executed(self._on_command_executed)
        self._build_window_ui()

        # Override Kit XR's default ``active camera`` anchor with ``scene origin``
        # for the lifetime of the window so the headset tracks real motion before
        # Connect runs. Activated only after all member state is wired so a
        # construction failure can never leave the global override stranded.
        # The original setting is restored in :meth:`destroy`.
        self._pre_session_anchor_active = activate_pre_session_anchor()

    def _on_stage_closing(self) -> None:
        """Called by TeleopManager when the USD stage is about to close.

        Backend controllers are already destroyed by TeleopManager.destroy_all_controllers.
        Preserve the user's configured profile state, but clear stage-bound runtime state.
        """
        self._session_panel.on_stage_closed()
        self._floating_panel.on_stage_closed()
        self._ik_panel.on_stage_closed()
        self._locomotion_panel.on_stage_closed()
        self._grasp_panel.on_stage_closed()

    def _on_editor_quit_event(self, _event: object) -> None:
        """Save the current teleop profile on app quit.

        Args:
            _event: App quit event payload.
        """
        self._save_last_profile()

    def _on_command_executed(self, command: TeleopCommand, success: bool, message: str) -> None:
        """Syncs panel UIs after a command bus execution.

        Keeps the desktop UI consistent when commands arrive from
        external sources (e.g. VR headset overlay).

        Args:
            command: Command that was executed.
            success: Whether the command succeeded.
            message: Result message from the command bus.
        """
        if command == TeleopCommand.DISCONNECT:
            self._reset_all_panels()
        else:
            self._session_panel.sync_from_command(command, success, message)

    def _reset_all_panels(self) -> None:
        """Reset every panel to its idle/disconnected state."""
        self._session_panel.reset_ui()
        self._floating_panel.reset_ui()
        self._ik_panel.reset_ui()
        self._locomotion_panel.reset_ui()
        self._grasp_panel.reset_ui()

    def collect_teleop_profile(self) -> TeleopProfile:
        """Collect the current window state into a unified teleop profile.

        Returns:
            Unified teleop profile for all panels.
        """
        return TeleopProfile(
            session=self._session_panel.collect_profile(),
            floating=self._floating_panel.collect_profile(),
            ik=self._ik_panel.collect_profile(),
            grasp=self._grasp_panel.collect_profile(),
            locomotion=self._locomotion_panel.collect_profile(),
        )

    def apply_teleop_profile(self, profile: TeleopProfile) -> tuple[bool, str]:
        """Apply a unified teleop profile across all panels.

        Args:
            profile: Unified teleop profile to apply.

        Returns:
            Tuple containing success state and a status message.
        """
        stage_state = self._get_stage_state()
        resolve = stage_state == STAGE_STATE_READY

        self._session_panel.apply_profile(profile.session, resolve_stage=resolve)
        self._floating_panel.apply_profile(profile.floating, resolve_stage=resolve)
        self._ik_panel.apply_profile(profile.ik, resolve_stage=resolve)
        self._grasp_panel.apply_profile(profile.grasp, resolve_stage=resolve)
        self._locomotion_panel.apply_profile(profile.locomotion, resolve_stage=resolve)

        if resolve:
            return True, "Loaded and resolved profile"
        return True, f"Loaded profile; stage resolution deferred ({stage_state})"

    @staticmethod
    def _get_stage_state() -> str:
        """Return the current stage state using resolver constants.

        Returns:
            Stage state resolver constant.
        """
        usd_context = omni.usd.get_context()
        if usd_context.get_stage() is None:
            return STAGE_STATE_NO_STAGE
        _, _, remaining = usd_context.get_stage_loading_status()
        if remaining > 0:
            return STAGE_STATE_LOADING
        return STAGE_STATE_READY

    def destroy(self) -> None:
        """Full teardown - session, controllers, markers, and subscriptions.

        The Teleop window owns all these resources; nothing else consumes
        them, so everything is cleaned up when the window closes.
        """
        self._save_last_profile()
        if self._ik_controller:
            self._ik_controller.set_on_status_changed(None)
        self._sub_shutdown = None
        if self._markers_manager:
            self._markers_manager.remove_all_markers()
        if self._teleop_profile_panel:
            self._teleop_profile_panel.destroy()
        if self._floating_panel:
            self._floating_panel.destroy()
        if self._ik_panel:
            self._ik_panel.destroy()
        if self._grasp_panel:
            self._grasp_panel.destroy()
        if self._locomotion_panel:
            self._locomotion_panel.destroy()
        if self._teleop_manager:
            self._teleop_manager.set_on_command_executed(None)
            self._teleop_manager.destroy()
        self._teleop_manager = None
        self._markers_manager = None
        self._floating_controller = None
        self._ik_controller = None
        self._grasp_controller = None
        self._locomotion_controller = None
        if self._pre_session_anchor_active:
            self._pre_session_anchor_active = False
            restore_pre_session_anchor()
        super().destroy()

    def _save_last_profile(self) -> None:
        """Persist the current window state to the extension-managed last profile file."""
        if not self._last_profile_path or self._session_panel is None:
            return
        ok, _message = save_teleop_profile(self._last_profile_path, self.collect_teleop_profile())
        if ok and self._teleop_profile_panel is not None:
            self._teleop_profile_panel.remember_last_profile(self._last_profile_path)

    def _build_window_ui(self) -> None:
        with self.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=0):
                    self._teleop_profile_panel.build()
                    self._session_panel.build()
                    self._floating_panel.build()
                    self._ik_panel.build()
                    self._grasp_panel.build()
                    self._locomotion_panel.build()

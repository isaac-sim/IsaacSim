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

"""Teleop session manager connecting VR hardware to Isaac Sim controllers."""

from __future__ import annotations

import contextlib
import json
from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any

import carb
import carb.eventdispatcher
import carb.events
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import XformPrim
from pxr import Gf, Usd, UsdGeom

from ._xform_utils import WorldPosePrimCache, read_world_pose_gf
from .coordinate_utils import CoordinateSystem, transform_pose
from .teleop_session_injector import install_teleop_session_injector
from .vr_recording_button import VRButton, VRRecordingButton
from .xr_anchor_manager import AnchorRotationMode, XrAnchorManager

if TYPE_CHECKING:
    import isaacteleop.deviceio as deviceio
    import isaacteleop.oxr as oxr

    from .controllers import (
        FloatingRigidBodyController,
        GraspController,
        LocomotionController,
        RobotIKController,
    )
    from .markers_manager import MarkersManager


class TeleopCommand(Enum):
    """Commands accepted by the teleop command bus.

    External systems (e.g. VR headset overlay) dispatch these via
    :func:`dispatch_command` to control the teleop session without the
    desktop UI.
    """

    CONNECT = "connect"
    START = "start"
    STOP = "stop"
    RESET = "reset"
    DISCONNECT = "disconnect"


TELEOP_CMD_EVENT = "isaacsim.replicator.teleop.command"
"""Event name for incoming commands (payload: ``{"command": "<cmd>"}``)."""

TELEOP_STATUS_EVENT = "isaacsim.replicator.teleop.status"
"""Event name for command results (payload: ``{"command", "success", "message"}``).."""


def dispatch_command(command: TeleopCommand | str) -> None:
    """Dispatch a teleop command via the Kit event bus.

    Can be called from any extension or script to control teleop
    externally (e.g. from a VR headset overlay panel).

    Args:
        command: :class:`TeleopCommand` enum value or lowercase string
            (``"connect"``, ``"start"``, ``"stop"``, ``"reset"``,
            ``"disconnect"``).
    """
    cmd_str = command.value if isinstance(command, TeleopCommand) else str(command).lower()
    carb.eventdispatcher.get_eventdispatcher().dispatch_event(
        event_name=TELEOP_CMD_EVENT,
        payload={"command": cmd_str},
    )


def _extract_xr_teleop_command(payload: Any) -> str:
    """Extract the ``command`` string from a ``teleop_command`` event payload.

    Accepts both shapes CloudXR producers use - ``payload["message"]`` as a
    JSON-encoded string (web client) or as a dict (some clients / fixtures)
    - so command dispatch is not coupled to one wire format. Returns ``""``
    when nothing parseable is found.
    """
    if payload is None:
        return ""
    try:
        message = payload.get("message", "")
    except (AttributeError, TypeError):
        return ""
    obj: Any = None
    if isinstance(message, dict):
        obj = message
    elif isinstance(message, str):
        try:
            obj = json.loads(message)
        except (ValueError, TypeError):
            return ""
    if not isinstance(obj, dict):
        return ""
    return str(obj.get("command", "") or "")


@dataclass
class _DebugControllerInputs:
    """Synthetic controller inputs for debug tracking mode.

    Mimics the attribute interface of ``isaacteleop``'s real controller
    inputs so that downstream consumers (grasp, locomotion) can read
    ``.trigger_value``, ``.squeeze_value``, etc. without knowing
    whether the data comes from VR hardware or the debug UI.
    """

    trigger_value: float = 0.0
    squeeze_value: float = 0.0
    thumbstick_x: float = 0.0
    thumbstick_y: float = 0.0
    primary_click: bool = False
    secondary_click: bool = False
    thumbstick_click: bool = False


@dataclass
class _DebugControllerSnapshot:
    """Lightweight stand-in for a real VR controller snapshot.

    Only the ``inputs`` field is used by consumers; pose data is
    sourced separately from the marker world transforms.
    """

    inputs: _DebugControllerInputs = field(default_factory=_DebugControllerInputs)


class TeleopManager:
    """Manages teleop session connection and VR controller/wrist data tracking.

    Provides VR wrist pose data from OpenXR controllers to drive robot
    end effector visualization (markers) and physics controllers.

    Supports multiple teleop control paths:
    - Floating: Uses velocity tracking of a rigid-body handle
    - Articulation: Uses 6DOF joint chain with position drives
    """

    def __init__(self) -> None:
        self._oxr_session: oxr.OpenXRSession | None = None
        self._deviceio_session: deviceio.DeviceIOSession | None = None
        self._session_stack: contextlib.ExitStack | None = None
        self._controller_tracker: deviceio.ControllerTracker | None = None
        self._head_tracker: deviceio.HeadTracker | None = None
        self._update_subscription = None
        self._frame_count = 0
        self._cached_tracking_space: tuple[Gf.Vec3d, Gf.Rotation, Gf.Quatd] | None = None
        self._cached_tracking_space_frame: int = -1
        self._is_connected = False
        self._update_fail_count = 0  # Consecutive tracking update failures
        self._on_status_changed: Callable[[str], None] | None = None

        self._debug_tracking_enabled = False
        self._debug_left_snapshot = _DebugControllerSnapshot()
        self._debug_right_snapshot = _DebugControllerSnapshot()
        self._markers_manager: MarkersManager | None = None
        self._live_tracking_enabled = False
        self._floating_controller: FloatingRigidBodyController | None = None
        self._left_floating_assigned = False
        self._right_floating_assigned = False
        self._floating_tracking_enabled = False
        self._grasp_controller: GraspController | None = None
        self._grasp_tracking_enabled = False
        self._ik_controller: RobotIKController | None = None
        self._locomotion_controller: LocomotionController | None = None
        self._locomotion_tracking_enabled = False
        self._coordinate_system = CoordinateSystem.ISAAC_SIM
        self._tracking_space_enabled = False
        self._tracking_space_retry_failed = False
        self._tracking_space_prim_path: str = ""
        self._active_tracking_space_prim_path: str = ""
        self._tracking_space_xform: XformPrim | None = None
        self._tracking_space_world_pose_cache = WorldPosePrimCache()
        self._xr_anchor: XrAnchorManager | None = None
        self._on_stage_closing: Callable[[], None] | None = None
        usd_ctx = omni.usd.get_context()
        self._stage_closing_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=usd_ctx.stage_event_name(omni.usd.StageEventType.CLOSING),
            on_event=self._handle_stage_closing,
            observer_name="TeleopManager._handle_stage_closing",
        )
        self._on_command_executed: Callable[[TeleopCommand, bool, str], None] | None = None
        self._command_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=TELEOP_CMD_EVENT,
            on_event=self._on_command_event,
            observer_name="TeleopManager._on_command_event",
        )
        self._xr_command_sub = None
        self._timeline_sub = (
            omni.timeline.get_timeline_interface()
            .get_timeline_event_stream()
            .create_subscription_to_pop(self._on_timeline_event, name="TeleopManager_timeline")
        )
        self._controller_inputs_observers: list[Callable[[object | None, object | None], None]] = []
        self._head_observers: list[Callable[[object | None], None]] = []
        self._uninstall_session_injector: Callable[[], None] | None = install_teleop_session_injector(self)
        self._vr_recording_button: VRRecordingButton | None = self._auto_attach_vr_recording_button()

    def _auto_attach_vr_recording_button(self) -> VRRecordingButton | None:
        """Attach the Meta Quest left-Y button to the recorder ``toggle`` command.

        The button dispatches :data:`EPISODE_CMD_EVENT
        <isaacsim.replicator.episode_recorder.EPISODE_CMD_EVENT>` with
        ``session_id=None`` (broadcast). When no recorder has an open session
        the dispatch is a no-op, so keeping the binding alive for the whole
        lifetime of :class:`TeleopManager` is safe whether the user is
        recording or not.
        """
        try:
            button = VRRecordingButton(self, button=VRButton.LEFT_SECONDARY, command="toggle")
            button.attach()
            return button
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"TeleopManager: auto-attach VR recording button failed: {exc}")
            return None

    def set_on_stage_closing(self, callback: Callable[[], None] | None) -> None:
        """Register a callback invoked when the USD stage is about to close.

        The UI layer uses this to sync button/label state after automatic
        disconnect and marker cleanup.
        """
        self._on_stage_closing = callback

    def _handle_stage_closing(self, event: carb.eventdispatcher.Event) -> None:
        """Automatically disconnects session and tears down all controllers on stage close."""
        if self._is_connected:
            print("[Teleop][Session] Stage closing - disconnecting session.")
            self.disconnect()

        self.destroy_all_controllers()

        if self._xr_anchor is not None:
            self._xr_anchor.cleanup()
            self._xr_anchor = None

        self._live_tracking_enabled = False
        if self._markers_manager is not None:
            self._markers_manager.clear_cached_state()
        self._tracking_space_enabled = False
        self._tracking_space_xform = None
        self._tracking_space_world_pose_cache.clear()
        self._tracking_space_prim_path = ""
        self._active_tracking_space_prim_path = ""

        if self._on_stage_closing:
            self._on_stage_closing()

    def destroy_all_controllers(self) -> None:
        """Disable and destroys all controller resources.

        Called on stage close and window destroy to release stale USD references.
        Stored prim paths (including persistent settings) are preserved.
        Only performs work (and prints) if at least one controller has active
        resources.
        """
        any_active = False

        if self._floating_controller is not None:
            for side in ("left", "right"):
                if self._floating_controller.is_configured(side):
                    self._floating_controller.destroy(side)
                    any_active = True
            self._floating_tracking_enabled = False

        if self._ik_controller is not None:
            for side in ("left", "right"):
                if self._ik_controller.is_configured(side):
                    self._ik_controller.destroy(side)
                    any_active = True

        if self._locomotion_controller is not None and self._locomotion_controller.is_running:
            self._locomotion_controller.disable()
            self._locomotion_tracking_enabled = False
            any_active = True

        if self._grasp_controller is not None and self._grasp_controller.is_enabled:
            self._grasp_controller.remove_all()
            self._grasp_tracking_enabled = False
            any_active = True

        self._left_floating_assigned = False
        self._right_floating_assigned = False

        if any_active:
            print("[Teleop][Session] All controllers destroyed.")

    # ------------------------------------------------------------------
    # Command bus
    # ------------------------------------------------------------------

    def set_on_command_executed(self, callback: Callable[[TeleopCommand, bool, str], None] | None) -> None:
        """Register a callback invoked after each command execution.

        The UI layer uses this to sync widget state when commands arrive
        from external sources (e.g. VR headset panel).

        Args:
            callback: ``(command, success, message)`` or *None* to clear.
        """
        self._on_command_executed = callback

    def execute_command(self, command: TeleopCommand) -> tuple[bool, str]:
        """Execute a teleop command and notifies all listeners.

        This is the single entry point for both the desktop UI and
        external command bus.  After execution, it fires:

        * The local ``on_command_executed`` callback (for the UI)
        * A :data:`TELEOP_STATUS_EVENT` event (for any Kit listener)

        Args:
            command: The command to execute.

        Returns:
            ``(success, message)`` tuple.
        """
        handler = {
            TeleopCommand.CONNECT: self._cmd_connect,
            TeleopCommand.START: self._cmd_start,
            TeleopCommand.STOP: self._cmd_stop,
            TeleopCommand.RESET: self._cmd_reset,
            TeleopCommand.DISCONNECT: self._cmd_disconnect,
        }.get(command)

        if handler is None:
            return False, f"Unknown command: {command}"

        success, message = handler()
        print(f"[Teleop][Session] Command {command.value}: {message}")

        if self._on_command_executed:
            self._on_command_executed(command, success, message)

        carb.eventdispatcher.get_eventdispatcher().dispatch_event(
            event_name=TELEOP_STATUS_EVENT,
            payload={"command": command.value, "success": success, "message": message},
        )

        return success, message

    def _on_command_event(self, event: carb.eventdispatcher.Event) -> None:
        """Handle incoming command bus events."""
        cmd_str = event.payload.get("command", "")
        try:
            command = TeleopCommand(cmd_str)
        except ValueError:
            print(f"[Teleop][Session] Unknown command received: '{cmd_str}'")
            return
        self.execute_command(command)

    def _subscribe_xr_command_bus(self) -> None:
        """Subscribe to the XR Core message bus for headset UI commands.

        Deferred to connect time because XR Core may not be initialized
        when the TeleopManager is constructed (causes a crash if
        ``XRCore.get_singleton()`` is called too early in the VR experience).
        """
        if self._xr_command_sub is not None:
            return
        try:
            from omni.kit.xr.core import XRCore

            xr_core = XRCore.get_singleton()
            if xr_core is not None:
                bus = xr_core.get_message_bus()
                self._xr_command_sub = bus.create_subscription_to_pop_by_type(
                    carb.events.type_from_string("teleop_command"),
                    self._on_xr_teleop_command,
                )
                print("[Teleop][Session] Subscribed to XR Core command bus.")
            else:
                print(
                    "[Teleop][Session] WARNING: XRCore singleton not available - headset "
                    "commands (Play/Reset) will not work. Launch with ./isaac-sim.xr.vr.sh."
                )
        except (ImportError, AttributeError):
            print(
                "[Teleop][Session] WARNING: omni.kit.xr.core is not loaded - headset "
                "commands (Play/Reset) will not work. Launch with ./isaac-sim.xr.vr.sh."
            )

    def _on_xr_teleop_command(self, event: Any) -> None:
        """Bridges teleop commands from the XR Core message bus (CloudXR web UI).

        The IsaacTeleop web client sends JSON messages over CloudXR's
        MessageChannel (WebRTC data channel) with the format::

            { "type": "teleop_command",
              "message": { "command": "start teleop" } }

        The CloudXR runtime forwards these onto Kit's XR Core message bus
        with ``payload["message"]`` set to the JSON-encoded inner object.
        See :func:`_extract_xr_teleop_command` for the payload parser.
        """
        command = _extract_xr_teleop_command(getattr(event, "payload", None))
        if command == "start teleop":
            self.execute_command(TeleopCommand.START)
        elif command == "stop teleop":
            self.execute_command(TeleopCommand.STOP)
        elif command == "reset teleop":
            self.execute_command(TeleopCommand.RESET)
        else:
            print(
                f"[Teleop][Session] Unknown XR teleop command (parsed='{command}', raw payload="
                f"{getattr(event, 'payload', None)!r})"
            )

    def _cmd_connect(self) -> tuple[bool, str]:
        """Connect to OpenXR, creates markers, sets up XR anchor, starts live tracking."""
        if self._is_connected:
            return True, "Already connected"

        success = self.connect(on_status_changed=self._on_status_changed)
        if not success:
            return False, "Connection failed"

        marker_warning = ""
        if self._markers_manager:
            for name in ("origin", "left", "right", "head"):
                ok, msg = self._markers_manager.ensure_marker(name)
                if not ok:
                    marker_warning = f" (marker warning: {msg})"
                    break
        self.set_live_tracking(True)

        self._reapply_tracking_space()

        self._setup_xr_anchor()

        return True, f"Connected{marker_warning}"

    def _cmd_start(self) -> tuple[bool, str]:
        """Plays the simulation timeline (headset "Play" button).

        The user configures and starts controllers from the desktop UI.
        This command simply plays the timeline so that physics-based
        controllers (floating, articulation, IK) begin receiving physics
        steps.  Mirrors IsaacLab's Play behavior where the main loop
        switches from ``render()`` to ``env.step(actions)``.
        """
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing():
            return True, "Timeline already playing"

        timeline.play()
        return True, "Timeline playing"

    def _cmd_stop(self) -> tuple[bool, str]:
        """Stop the simulation timeline (headset "Stop" button).

        Pauses physics so controllers stop receiving steps.
        Markers keep tracking so the user can still see hand positions.
        """
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            return True, "Timeline already stopped"

        timeline.stop()
        return True, "Timeline stopped"

    def _cmd_reset(self) -> tuple[bool, str]:
        """Stop the timeline and resets it to frame 0 (headset "Reset" button).

        Stops the simulation, rewinds the timeline to the beginning,
        and re-validates the tracking space.  The XR session stays
        alive and markers keep tracking.  The user can press Play to
        restart.
        """
        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()

        if was_playing:
            timeline.stop()

        timeline.set_current_time(0.0)

        self._reapply_tracking_space()
        if self._xr_anchor is not None:
            self._xr_anchor.reset()

        return True, "Timeline reset to t=0"

    def _cmd_disconnect(self) -> tuple[bool, str]:
        """Full teardown: stops controllers, removes markers, ends session."""
        if not self._is_connected:
            return True, "Already disconnected"

        self._cmd_stop()
        self.set_live_tracking(False)
        if self._xr_anchor is not None:
            self._xr_anchor.cleanup()
            self._xr_anchor = None
        if self._markers_manager:
            self._markers_manager.remove_all_markers()
        self.destroy_all_controllers()
        self.disconnect()

        return True, "Disconnected"

    # ------------------------------------------------------------------
    # Timeline-driven controller lifecycle
    # ------------------------------------------------------------------

    def _on_timeline_event(self, event: Any) -> None:
        """Dispatch timeline play/stop events to controller warm-up/cool-down."""
        if not (self._is_connected or self._debug_tracking_enabled):
            return
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._on_timeline_play()
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._on_timeline_stop()

    def _on_timeline_play(self) -> None:
        """Enable all configured controllers when the timeline starts."""
        enabled: list[str] = []

        if self._floating_controller:
            for side in ("left", "right"):
                if self.is_floating_side_assigned(side) and self._floating_controller.is_configured(side):
                    if self._floating_controller.enable(side):
                        enabled.append(f"floating-{side}")
            if self._floating_controller.is_enabled:
                self._floating_tracking_enabled = True

        if self._ik_controller:
            for side in ("left", "right"):
                if self._ik_controller.is_configured(side) and not self._ik_controller.is_running(side):
                    if self._ik_controller.enable(side):
                        enabled.append(f"IK-{side}")

        if self._grasp_tracking_enabled and self._grasp_controller and self._grasp_controller.is_enabled:
            enabled.append("grasp")

        if self._locomotion_tracking_enabled and self._locomotion_controller:
            if not self._locomotion_controller.is_running:
                if self._markers_manager is not None:
                    self._locomotion_controller.set_edit_layer(self._markers_manager.layer)
                self._locomotion_controller.set_tracking_space_prim_path(self._active_tracking_space_prim_path)
                ok, _ = self._locomotion_controller.enable()
                if ok:
                    enabled.append("locomotion")

        if enabled:
            if len(enabled) == 1:
                entry = enabled[0]
                namespace = {"locomotion": "Locomotion", "grasp": "Grasp"}.get(entry)
                if namespace is None:
                    if entry.startswith("floating-"):
                        namespace = "Floating"
                    elif entry.startswith("IK-"):
                        namespace = "IK"
                if namespace is not None:
                    print(f"[Teleop][{namespace}] Timeline play - enabled: {entry}")
                else:
                    print(f"[Teleop][Session] Timeline play - enabled: {entry}")
            else:
                print(f"[Teleop][Session] Timeline play - enabled: {', '.join(enabled)}")

    def _on_timeline_stop(self) -> None:
        """Disable all controllers and restores grippers when the timeline stops."""
        disabled: list[str] = []

        if self._floating_controller:
            for side in ("left", "right"):
                if self._floating_controller.is_running(side):
                    self._floating_controller.disable(side)
                    disabled.append(f"floating-{side}")
            self._floating_tracking_enabled = False

        if self._ik_controller:
            for side in ("left", "right"):
                if self._ik_controller.is_running(side):
                    self._ik_controller.disable(side)
                    disabled.append(f"IK-{side}")

        if self._grasp_tracking_enabled:
            disabled.append("grasp")

        if self._locomotion_tracking_enabled and self._locomotion_controller:
            self._locomotion_controller.disable()
            disabled.append("locomotion")

        if disabled:
            if len(disabled) == 1:
                entry = disabled[0]
                namespace = {"locomotion": "Locomotion", "grasp": "Grasp"}.get(entry)
                if namespace is None:
                    if entry.startswith("floating-"):
                        namespace = "Floating"
                    elif entry.startswith("IK-"):
                        namespace = "IK"
                if namespace is not None:
                    print(f"[Teleop][{namespace}] Timeline stop - disabled: {entry}")
                else:
                    print(f"[Teleop][Session] Timeline stop - disabled: {entry}")
            else:
                print(f"[Teleop][Session] Timeline stop - disabled: {', '.join(disabled)}")

    @property
    def is_connected(self) -> bool:
        """Return True if the teleop session is connected."""
        return self._is_connected

    @staticmethod
    def _print_cloudxr_start_hint() -> None:
        """Print a stable hint for starting the Isaac Teleop CloudXR runtime."""
        print("[Teleop][Session] Info: Make sure Isaac Teleop CloudXR is running and the headset client is connected.")
        print("[Teleop][Session] Info: Start it in another terminal with `python -m isaacteleop.cloudxr`.")
        print(
            "[Teleop][Session] Info: See `source/extensions/isaacsim.replicator.teleop/docs/Overview.md` for setup steps."
        )

    def _ensure_update_subscription(self) -> None:
        """Ensure a single subscription fires :meth:`_on_update` each app frame."""
        if self._update_subscription is not None:
            return
        self._update_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._on_update,
            observer_name="isaacsim.replicator.teleop.TeleopManager.update",
        )

    def _release_update_subscription(self) -> None:
        """Unregister the app-frame update observer."""
        sub = self._update_subscription
        if sub is None:
            return
        sub.reset()
        self._update_subscription = None

    def connect(self, on_status_changed: Callable[[str], None] | None = None) -> bool:
        """Connect to the teleop session via OpenXR.

        Args:
            on_status_changed: Optional callback for status updates.

        Returns:
            True if connection was successful, False otherwise.
        """
        self._on_status_changed = on_status_changed
        self._update_fail_count = 0

        if self._debug_tracking_enabled:
            self.set_debug_tracking(False)

        if self._is_connected:
            print("[Teleop][Session] Already connected.")
            return True

        try:
            import isaacteleop.deviceio as deviceio
            import isaacteleop.oxr as oxr
        except ImportError as e:
            print(f"[Teleop][Session] Failed to import isaacteleop modules: {e}")
            if on_status_changed:
                on_status_changed("Error: isaacteleop modules not available")
            return False

        self._controller_tracker = deviceio.ControllerTracker()
        self._head_tracker = deviceio.HeadTracker()
        trackers = [self._controller_tracker, self._head_tracker]

        required_extensions = deviceio.DeviceIOSession.get_required_extensions(trackers)
        try:
            self._oxr_session = oxr.OpenXRSession("IsaacSimTeleop", required_extensions)
        except Exception as e:
            print(f"[Teleop][Session] Failed to create OpenXR session: {e}")
            self._print_cloudxr_start_hint()
            if on_status_changed:
                on_status_changed("Error: Failed to create OpenXR session")
            self._cleanup_trackers()
            return False

        stack = contextlib.ExitStack()
        try:
            stack.enter_context(self._oxr_session)
            handles = self._oxr_session.get_handles()
        except Exception as e:
            print(f"[Teleop][Session] Failed to initialize OpenXR session: {e}")
            self._print_cloudxr_start_hint()
            if on_status_changed:
                on_status_changed(f"Error: {e}")
            stack.close()
            self._oxr_session = None
            self._cleanup_trackers()
            return False

        try:
            self._deviceio_session = deviceio.DeviceIOSession.run(trackers, handles)
            stack.enter_context(self._deviceio_session)
        except Exception as e:
            print(f"[Teleop][Session] Failed to create DeviceIO session: {e}")
            if on_status_changed:
                on_status_changed(f"Error: {e}")
            stack.close()
            self._oxr_session = None
            self._deviceio_session = None
            self._cleanup_trackers()
            return False

        self._session_stack = stack

        self._ensure_update_subscription()

        self._is_connected = True
        self._frame_count = 0
        self._subscribe_xr_command_bus()
        if on_status_changed:
            on_status_changed("Connected")
        return True

    def disconnect(self, on_status_changed: Callable[[str], None] | None = None) -> None:
        """Disconnect from the teleop session."""
        if not self._is_connected:
            print("[Teleop][Session] Not connected.")
            return

        if not self._debug_tracking_enabled:
            self._release_update_subscription()
        if self._session_stack is not None:
            try:
                self._session_stack.close()
            except Exception as e:
                print(f"[Teleop][Session] Error closing sessions: {e}")
            self._session_stack = None
        self._deviceio_session = None
        self._oxr_session = None

        self._cleanup_trackers()
        self._xr_command_sub = None
        self._is_connected = False
        self._frame_count = 0
        if on_status_changed:
            on_status_changed("Disconnected")

    @property
    def debug_tracking_enabled(self) -> bool:
        """True when debug tracking mode is active."""
        return self._debug_tracking_enabled

    def _get_debug_snapshot(self, side: str) -> _DebugControllerSnapshot | None:
        """Return the synthetic controller snapshot for the requested side."""
        if side == "left":
            return self._debug_left_snapshot
        if side == "right":
            return self._debug_right_snapshot
        return None

    def set_debug_tracking(self, enabled: bool) -> None:
        """Enable or disables debug tracking mode.

        When enabled, the left/right marker world poses are read each
        frame and fed to all downstream consumers (IK, floating, grasp,
        locomotion) instead of VR controller data.  The user can drag
        the markers in the viewport to drive the robot.

        An update subscription is created automatically so the loop
        runs even without a VR connection.

        Mutually exclusive with a live VR connection — cannot be
        enabled while connected.
        """
        if enabled and self._is_connected:
            print("[Teleop][Debug] Cannot enable debug tracking while VR is connected.")
            return
        if self._debug_tracking_enabled == enabled:
            return
        self._debug_tracking_enabled = enabled
        if enabled:
            self._ensure_update_subscription()
            self._reapply_tracking_space()
            print("[Teleop][Debug] Tracking enabled - reading poses from markers.")
        else:
            if not self._is_connected:
                self._release_update_subscription()
            print("[Teleop][Debug] Tracking disabled.")

    def set_debug_trigger(self, side: str, value: float) -> None:
        """Set the synthetic trigger value for debug tracking mode.

        Args:
            side: ``"left"`` or ``"right"``.
            value: Trigger analog value in [0.0, 1.0].
        """
        snapshot = self._get_debug_snapshot(side)
        if snapshot is None:
            return
        snapshot.inputs.trigger_value = max(0.0, min(1.0, value))

    def set_debug_thumbstick(self, side: str, *, x: float | None = None, y: float | None = None) -> None:
        """Set synthetic thumbstick axes for debug tracking mode."""
        snapshot = self._get_debug_snapshot(side)
        if snapshot is None:
            return
        if x is not None:
            snapshot.inputs.thumbstick_x = max(-1.0, min(1.0, x))
        if y is not None:
            snapshot.inputs.thumbstick_y = max(-1.0, min(1.0, y))

    def set_debug_button(self, side: str, button: str, pressed: bool) -> None:
        """Set a synthetic controller button state for debug tracking mode."""
        snapshot = self._get_debug_snapshot(side)
        if snapshot is None or button not in {"primary_click", "secondary_click", "thumbstick_click"}:
            return
        setattr(snapshot.inputs, button, bool(pressed))

    def set_coordinate_system(self, system: CoordinateSystem) -> None:
        """Set the coordinate system for VR → scene conversion.

        Conversion is performed centrally in ``_on_update`` before data
        reaches markers or controllers.  Managed controllers are set to
        RAW so they do not double-convert.

        Args:
            system: Target coordinate system for VR pose data.
        """
        self._coordinate_system = system
        # Controllers receive pre-converted data; set them to RAW.
        for ctrl in (self._floating_controller, self._ik_controller):
            if ctrl and hasattr(ctrl, "set_coordinate_system"):
                ctrl.set_coordinate_system(CoordinateSystem.RAW)
        print(f"[Teleop][Session] Coordinate system set to: '{system.value}'")

    # ------------------------------------------------------------------
    # Tracking space
    # ------------------------------------------------------------------

    def disable_tracking_space(self) -> None:
        """Disable tracking-space following entirely."""
        self._tracking_space_enabled = False
        self._tracking_space_prim_path = ""
        self._active_tracking_space_prim_path = ""
        self._tracking_space_xform = None
        self._tracking_space_world_pose_cache.set_prim_path("")
        if self._xr_anchor is not None:
            self._xr_anchor.set_tracking_space_prim_path("")
        if self._locomotion_controller is not None:
            self._locomotion_controller.set_tracking_space_prim_path("")

    def set_builtin_tracking_space(self) -> tuple[bool, str]:
        """Use the built-in Teleop origin marker as tracking space."""
        from .markers_manager import MarkersManager

        builtin_path = MarkersManager.MARKER_PATHS["origin"]
        if self._markers_manager is not None:
            ok, msg = self._markers_manager.ensure_marker("origin")
            if not ok:
                return (
                    False,
                    f"Built-in Teleop tracking space is unavailable: {msg}. Current tracking space is unchanged.",
                )
        ok, message = self._apply_tracking_space_path(builtin_path)
        if ok:
            self._tracking_space_enabled = True
            self._tracking_space_prim_path = ""
            print(f"[Teleop][Session] Tracking Space set to built-in Teleop marker '{builtin_path}'.")
        return ok, message

    def set_tracking_space_prim_path(self, path: str) -> tuple[bool, str]:
        """Use a custom scene prim as tracking space."""
        from .markers_manager import MarkersManager

        requested_path = path.strip()
        if not requested_path:
            return self.set_builtin_tracking_space()
        if requested_path.startswith(MarkersManager.MARKERS_SCOPE):
            msg = (
                f"Cannot use teleop marker '{requested_path}' as Tracking Space. "
                "Choose a scene prim or leave the path empty to use the built-in tracking space."
            )
            print(f"[Teleop][Session] {msg}")
            return False, msg

        ok, message = self._apply_tracking_space_path(requested_path)
        if ok:
            self._tracking_space_enabled = True
            self._tracking_space_prim_path = requested_path
            print(f"[Teleop][Session] Tracking Space set to '{requested_path}'.")
        return ok, message

    def _teleop_edit_ctx(self, stage: Usd.Stage, prim_path: str) -> AbstractContextManager[None]:
        """Return an ``Usd.EditContext`` targeting the markers anonymous layer for Teleop prims."""
        layer = self._markers_manager.layer if self._markers_manager is not None else None
        if (
            layer is not None
            and prim_path.startswith("/Teleop/")
            and any(layer.identifier == l.identifier for l in stage.GetLayerStack(includeSessionLayers=True))
        ):
            return Usd.EditContext(stage, layer)
        return nullcontext()

    def _apply_tracking_space_path(self, resolved_path: str) -> tuple[bool, str]:
        """Validate and activate a tracking-space path without changing mode semantics.

        For built-in Teleop prims (under ``/Teleop/``), xformOp writes are
        directed to the markers anonymous layer so no specs leak to the root
        layer.
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return False, "No USD stage available. Current tracking space is unchanged."

        prim = stage.GetPrimAtPath(resolved_path)
        if not prim or not prim.IsValid():
            return False, f"Tracking Space prim not found: {resolved_path}. Current tracking space is unchanged."

        if not prim.IsA(UsdGeom.Xformable):
            return False, f"Tracking Space prim is not Xformable: {resolved_path}. Current tracking space is unchanged."

        props = prim.GetPropertyNames()
        needs_reset = any(op not in props for op in ("xformOp:translate", "xformOp:orient", "xformOp:scale"))

        edit_ctx = self._teleop_edit_ctx(stage, resolved_path)
        with edit_ctx:
            tracking_space_xform = XformPrim(resolved_path, reset_xform_op_properties=needs_reset)

        warning = ""
        if needs_reset:
            warning = " (xformOps reset)"
            print(f"[Teleop][Session] Tracking Space prim xformOps reset at '{resolved_path}'.")

        self._active_tracking_space_prim_path = resolved_path
        self._tracking_space_xform = tracking_space_xform
        self._tracking_space_world_pose_cache.set_prim_path(resolved_path)
        self._cached_tracking_space_frame = -1
        self._tracking_space_retry_failed = False
        if self._xr_anchor is not None:
            self._xr_anchor.set_tracking_space_prim_path(resolved_path)
        if self._locomotion_controller is not None:
            if self._markers_manager is not None:
                self._locomotion_controller.set_edit_layer(self._markers_manager.layer)
            self._locomotion_controller.set_tracking_space_prim_path(resolved_path)
        return True, f"Tracking Space: {resolved_path}{warning}"

    def _reapply_tracking_space(self) -> tuple[bool, str]:
        """Reapply the currently selected tracking space after connect/reset.

        Always activates at least the built-in origin marker so that
        VR pose offsetting and locomotion carry work out of the box.
        """
        if self._tracking_space_prim_path:
            return self.set_tracking_space_prim_path(self._tracking_space_prim_path)
        return self.set_builtin_tracking_space()

    @property
    def tracking_space_prim_path(self) -> str:
        """Return the current tracking-space prim path."""
        return self._active_tracking_space_prim_path

    def _get_tracking_space_transform(self) -> tuple[Gf.Vec3d, Gf.Rotation, Gf.Quatd] | None:
        """Read the tracking-space world transform via the active backend.

        Results are cached per frame so multiple callers within the same
        ``_on_update`` do not re-read the prim.

        Returns:
            ``(position, rotation, quaternion)`` or *None* if no tracking space is set.
        """
        if self._cached_tracking_space_frame == self._frame_count:
            return self._cached_tracking_space

        if self._tracking_space_xform is None:
            return None

        if not self._tracking_space_xform.valid:
            self._tracking_space_xform = None
            self._tracking_space_world_pose_cache.clear()
            return None

        pos, qd = read_world_pose_gf(self._tracking_space_world_pose_cache)
        result = pos, Gf.Rotation(qd), qd
        self._cached_tracking_space = result
        self._cached_tracking_space_frame = self._frame_count
        return result

    @staticmethod
    def _apply_tracking_space_offset(
        pos: tuple[float, float, float] | None,
        orient: tuple[float, float, float, float] | None,
        tracking_space: tuple[Gf.Vec3d, Gf.Rotation, Gf.Quatd] | None,
    ) -> tuple[tuple[float, float, float] | None, tuple[float, float, float, float] | None]:
        """Transform a local VR pose into world space via a pre-fetched tracking space.

        Args:
            pos: (x, y, z) position in scene coordinates (post coord-conversion).
            orient: (x, y, z, w) quaternion in scene coordinates.
            tracking_space: ``(position, rotation, quaternion)`` from
                :meth:`_get_tracking_space_transform`,
                    or *None* to skip the offset.

        Returns:
            (world_pos, world_orient) tuple.
        """
        if pos is None or tracking_space is None:
            return pos, orient

        ts_pos, ts_rot, ts_qd = tracking_space

        world_vec = ts_pos + ts_rot.TransformDir(Gf.Vec3d(pos[0], pos[1], pos[2]))
        new_pos = (world_vec[0], world_vec[1], world_vec[2])

        new_orient = orient
        if orient is not None:
            combined = ts_qd * Gf.Quatd(orient[3], orient[0], orient[1], orient[2])
            im = combined.GetImaginary()
            new_orient = (im[0], im[1], im[2], combined.GetReal())

        return new_pos, new_orient

    # ------------------------------------------------------------------
    # XR Anchor
    # ------------------------------------------------------------------

    def _setup_xr_anchor(self) -> None:
        """Create and configures the XR anchor using the current tracking-space prim path."""
        if self._xr_anchor is not None:
            self._xr_anchor.cleanup()

        self._xr_anchor = XrAnchorManager(
            tracking_space_prim_path=self._active_tracking_space_prim_path,
        )
        self._xr_anchor.setup()

    @property
    def xr_anchor(self) -> XrAnchorManager | None:
        """The active XR anchor manager, or None if not connected."""
        return self._xr_anchor

    def set_xr_anchor_pos(self, pos: tuple[float, float, float]) -> None:
        """Update the XR anchor position offset (live)."""
        if self._xr_anchor is not None:
            self._xr_anchor.set_anchor_pos(pos)

    def set_xr_anchor_rotation_mode(self, mode: AnchorRotationMode) -> None:
        """Update the XR anchor rotation mode (live)."""
        if self._xr_anchor is not None:
            self._xr_anchor.set_rotation_mode(mode)

    def set_xr_anchor_smoothing_time(self, seconds: float) -> None:
        """Update the XR anchor rotation smoothing time (live)."""
        if self._xr_anchor is not None:
            self._xr_anchor.set_smoothing_time(seconds)

    def set_xr_anchor_fixed_height(self, fixed: bool) -> None:
        """Toggle XR anchor fixed-height mode (live)."""
        if self._xr_anchor is not None:
            self._xr_anchor.set_fixed_height(fixed)

    # ------------------------------------------------------------------
    # Markers manager
    # ------------------------------------------------------------------

    def set_markers_manager(self, markers_manager: MarkersManager) -> None:
        """Set the markers manager for live VR wrist tracking updates.

        Args:
            markers_manager: The MarkersManager instance for visualizing VR wrist poses.
        """
        self._markers_manager = markers_manager

    def set_live_tracking(self, enabled: bool) -> None:
        """Enable or disables live tracking of markers to VR wrist positions.

        When enabled, markers (showing end effector geometry) will follow
        VR wrist poses each frame for ground truth visualization.
        When disabled, markers reset to origin.

        Args:
            enabled: True to enable live tracking, False to disable.
        """
        self._live_tracking_enabled = enabled
        if not enabled and self._markers_manager:
            self._markers_manager.reset_marker_transforms()

    @property
    def is_live_tracking(self) -> bool:
        """Return True if live tracking is enabled."""
        return self._live_tracking_enabled

    def set_floating_controller(self, controller: FloatingRigidBodyController | None) -> None:
        """Set the floating rigid-body controller for VR wrist velocity tracking.

        Args:
            controller: The FloatingRigidBodyController instance, or None to clear.
        """
        self._floating_controller = controller
        if self._floating_controller:
            self._floating_controller.set_coordinate_system(CoordinateSystem.RAW)
            self._floating_controller.set_side_enabled("left", self._left_floating_assigned)
            self._floating_controller.set_side_enabled("right", self._right_floating_assigned)

    def set_floating_side_assigned(self, side: str, assigned: bool) -> None:
        """Assigns or clears the floating controller for a specific side."""
        side = side.lower()
        assigned = bool(assigned)
        if side == "left":
            self._left_floating_assigned = assigned
            if self._floating_controller:
                self._floating_controller.set_side_enabled("left", assigned)
        elif side == "right":
            self._right_floating_assigned = assigned
            if self._floating_controller:
                self._floating_controller.set_side_enabled("right", assigned)

    def clear_floating_side(self, side: str) -> None:
        """Clear floating-controller assignment for a specific side."""
        self.set_floating_side_assigned(side, False)

    def is_floating_side_assigned(self, side: str) -> bool:
        """Return whether a side is assigned to the floating controller."""
        return self._right_floating_assigned if side.lower() == "right" else self._left_floating_assigned

    def set_grasp_controller(self, controller: GraspController | None) -> None:
        """Set the grasp controller for VR-driven grasp control.

        Args:
            controller: The GraspController instance, or None to clear.
        """
        self._grasp_controller = controller

    def set_ik_controller(self, controller: RobotIKController | None) -> None:
        """Set the robot arm IK controller for VR-driven articulated arms.

        Args:
            controller: The RobotIKController instance, or None to clear.
        """
        self._ik_controller = controller
        if controller:
            controller.set_coordinate_system(CoordinateSystem.RAW)

    def set_locomotion_controller(self, controller: LocomotionController | None) -> None:
        """Set the locomotion controller for kinematic base movement.

        Args:
            controller: The LocomotionController instance, or None to clear.
        """
        self._locomotion_controller = controller
        if controller is not None:
            controller.set_tracking_space_prim_path(self._active_tracking_space_prim_path)
            if self._markers_manager is not None:
                controller.set_edit_layer(self._markers_manager.layer)

    def set_locomotion_tracking(self, enabled: bool) -> None:
        """Enable or disables locomotion tracking from VR thumbstick input.

        Two workflows are supported depending on the locomotion prim:

        * **Robot base** — thumbstick moves the robot.  The left
          primary button toggles *Carry Tracking Space* to co-move the
          VR origin.
        * **VR origin** — locomotion prim IS the tracking-space origin.
          Every thumbstick movement shifts the VR workspace directly.
          Use this for floating grippers with no physical base.

        Args:
            enabled: True to enable locomotion tracking.
        """
        if self._locomotion_tracking_enabled == enabled:
            return

        if enabled and self._locomotion_controller is not None:
            if self._markers_manager is not None:
                self._locomotion_controller.set_edit_layer(self._markers_manager.layer)
            self._locomotion_controller.set_tracking_space_prim_path(self._active_tracking_space_prim_path)

        self._locomotion_tracking_enabled = enabled
        state = "enabled" if enabled else "disabled"
        print(f"[Teleop][Locomotion] Tracking {state}.")

    @property
    def is_locomotion_tracking(self) -> bool:
        """Return True if locomotion tracking is enabled."""
        return self._locomotion_tracking_enabled

    def set_grasp_tracking(self, enabled: bool) -> None:
        """Enable or disables grasp tracking from VR input.

        Args:
            enabled: True to enable grasp tracking.
        """
        if self._grasp_tracking_enabled == enabled:
            return
        self._grasp_tracking_enabled = enabled
        state = "enabled" if enabled else "disabled"
        print(f"[Teleop][Grasp] Tracking {state}.")

    @property
    def is_grasp_tracking(self) -> bool:
        """Return True if grasp tracking is enabled."""
        return self._grasp_tracking_enabled

    def set_floating_tracking(self, enabled: bool) -> None:
        """Enable or disables floating rigid-body tracking."""
        if self._floating_tracking_enabled == enabled:
            return
        self._floating_tracking_enabled = enabled
        if not enabled and self._floating_controller:
            self._floating_controller.reset_targets()
        status = "enabled" if enabled else "disabled"
        print(f"[Teleop][Floating] Tracking {status}.")

    @property
    def is_floating_tracking(self) -> bool:
        """Return True if floating rigid-body tracking is enabled."""
        return self._floating_tracking_enabled and (self._left_floating_assigned or self._right_floating_assigned)

    def _cleanup_trackers(self) -> None:
        """Clean up tracker objects."""
        self._controller_tracker = None
        self._head_tracker = None

    def add_controller_inputs_observer(self, observer: Callable[[object | None, object | None], None]) -> None:
        """Register an observer invoked once per update with the current controller snapshots.

        The observer is called with ``(left_ctrl, right_ctrl)`` on every update in both live-VR and
        debug tracking modes. Either argument may be ``None`` when the corresponding controller is not
        tracked. Observers are intended to be lightweight (e.g. rising-edge button detection for the
        recording button); long-running work should be deferred to a background task.

        The same observer may be registered multiple times; each registration is independent.
        """
        self._controller_inputs_observers.append(observer)

    def remove_controller_inputs_observer(self, observer: Callable[[object | None, object | None], None]) -> None:
        """Deregister a previously added controller-inputs observer. Silently ignores unknown observers."""
        try:
            self._controller_inputs_observers.remove(observer)
        except ValueError:
            pass

    def _notify_controller_inputs_observers(self, left_ctrl: object | None, right_ctrl: object | None) -> None:
        """Invoke every registered observer with the current controller snapshots.

        Each observer is wrapped in try/except so a faulty subscriber cannot break the tracking loop.
        """
        if not self._controller_inputs_observers:
            return
        for observer in list(self._controller_inputs_observers):
            try:
                observer(left_ctrl, right_ctrl)
            except Exception as exc:  # Keep the update loop alive even if a subscriber raises.
                carb.log_warn(f"[Teleop][Session] controller-inputs observer error: {exc}")

    def add_head_observer(self, observer: Callable[[object | None], None]) -> None:
        """Register an observer invoked once per update with the current headset snapshot.

        The observer is called with a single ``head`` argument on every update — the raw snapshot
        returned by the deviceio ``HeadTracker`` (``is_valid`` / ``pose.position`` / ``pose.orientation``
        attributes, OpenXR ``xyzw`` orientation convention), or ``None`` when the head is not
        tracked (e.g. debug tracking mode, or no VR session). The snapshot is forwarded raw — no
        coordinate-system conversion or tracking-space offset is applied, so consumers that need a
        world-space pose should transform it themselves (or read the ``head`` marker instead).

        Used by :class:`TeleopHeadRecordable` to record head pose channels. Observers are invoked
        from the tracking update loop and should be lightweight.
        """
        self._head_observers.append(observer)

    def remove_head_observer(self, observer: Callable[[object | None], None]) -> None:
        """Deregister a previously added head observer. Silently ignores unknown observers."""
        try:
            self._head_observers.remove(observer)
        except ValueError:
            pass

    def _notify_head_observers(self, head: object | None) -> None:
        """Invoke every registered head observer with the current head snapshot (or ``None``)."""
        if not self._head_observers:
            return
        for observer in list(self._head_observers):
            try:
                observer(head)
            except Exception as exc:  # Keep the update loop alive even if a subscriber raises.
                carb.log_warn(f"[Teleop][Session] head observer error: {exc}")

    def _get_controller_snapshots(self) -> tuple[object | None, object | None]:
        """Return left/right controller snapshots from the current deviceio session."""
        if self._controller_tracker is None or self._deviceio_session is None:
            return None, None

        left_tracked = self._controller_tracker.get_left_controller(self._deviceio_session)
        right_tracked = self._controller_tracker.get_right_controller(self._deviceio_session)
        return (
            left_tracked.data if left_tracked is not None else None,
            right_tracked.data if right_tracked is not None else None,
        )

    def _get_head_snapshot(self) -> Any:
        """Return the current head snapshot from the deviceio session."""
        if self._head_tracker is None or self._deviceio_session is None:
            return None

        head_tracked = self._head_tracker.get_head(self._deviceio_session)
        return head_tracked.data if head_tracked is not None else None

    def _read_marker_world_pose(
        self, name: str
    ) -> tuple[tuple[float, float, float] | None, tuple[float, float, float, float] | None]:
        """Read the world-space pose of a marker for debug tracking.

        Returns:
            ``(position, orientation)`` tuples, or ``(None, None)``
            if the markers manager is unavailable or the marker is
            not active.
        """
        if self._markers_manager is None:
            return None, None
        result = self._markers_manager.get_marker_world_pose(name)
        if result is None:
            return None, None
        return result

    def _on_update_debug(self) -> None:
        """Debug tracking update path — reads composed world poses from markers.

        Since left/right/head are children of the origin marker,
        ``get_world_poses()`` returns the composed world transform —
        moving the origin automatically moves all children.
        """
        self._frame_count += 1

        left_pos, left_orient = self._read_marker_world_pose("left")
        right_pos, right_orient = self._read_marker_world_pose("right")
        left_ctrl = self._debug_left_snapshot
        right_ctrl = self._debug_right_snapshot

        self._notify_controller_inputs_observers(left_ctrl, right_ctrl)
        self._notify_head_observers(None)

        if self._floating_tracking_enabled:
            self._update_floating_targets(left_pos, left_orient, right_pos, right_orient)

        if self._grasp_tracking_enabled:
            self._update_grasp_inputs(left_ctrl, right_ctrl)

        if self._ik_controller is not None:
            self._ik_controller.update_targets(left_pos, left_orient, right_pos, right_orient)

        if self._locomotion_tracking_enabled and self._locomotion_controller is not None:
            self._locomotion_controller.update(left_ctrl, right_ctrl)

    def _on_update(self, event: Any) -> None:
        """Called each frame to update tracking data.

        Data flow (VR mode):
        1. Extract raw VR poses (OpenXR Y-up)
        2. Convert to target coordinate system (origin-local poses)
        3. Apply tracking-space offset → world-space poses
        4. Markers receive origin-local poses; controllers get world-space

        If the tracking-space offset is unavailable, the method falls
        back to reading composed world poses from the markers so that
        controllers still receive world-space targets.

        When debug tracking is active, delegates to
        :meth:`_on_update_debug` which reads marker poses directly.
        """
        if self._debug_tracking_enabled:
            self._on_update_debug()
            return

        if not self._is_connected or self._deviceio_session is None:
            return

        if not self._deviceio_session.update():
            if self._update_fail_count == 0 and self._on_status_changed:
                self._on_status_changed("Connected (no data)")
            self._update_fail_count += 1
            return

        if self._update_fail_count > 0:
            self._update_fail_count = 0
            if self._on_status_changed:
                self._on_status_changed("Connected")

        self._frame_count += 1

        left_ctrl, right_ctrl = self._get_controller_snapshots()

        self._notify_controller_inputs_observers(left_ctrl, right_ctrl)

        # --- Extract raw VR poses (OpenXR aim pose) ---

        left_pos, left_orient = None, None
        if left_ctrl is not None and left_ctrl.aim_pose.is_valid:
            p = left_ctrl.aim_pose.pose.position
            o = left_ctrl.aim_pose.pose.orientation
            left_pos = (p.x, p.y, p.z)
            left_orient = (o.x, o.y, o.z, o.w)

        right_pos, right_orient = None, None
        if right_ctrl is not None and right_ctrl.aim_pose.is_valid:
            p = right_ctrl.aim_pose.pose.position
            o = right_ctrl.aim_pose.pose.orientation
            right_pos = (p.x, p.y, p.z)
            right_orient = (o.x, o.y, o.z, o.w)

        head_pos, head_orient = None, None
        head = self._get_head_snapshot()
        self._notify_head_observers(head)
        if head is not None and head.is_valid and head.pose is not None:
            p = head.pose.position
            o = head.pose.orientation
            head_pos = (p.x, p.y, p.z)
            head_orient = (o.x, o.y, o.z, o.w)

        # --- Convert to target coordinate system (origin-local poses) ---

        cs = self._coordinate_system
        if left_pos is not None:
            left_pos, left_orient = transform_pose(left_pos, left_orient, cs)
        if right_pos is not None:
            right_pos, right_orient = transform_pose(right_pos, right_orient, cs)
        if head_pos is not None:
            head_pos, head_orient = transform_pose(head_pos, head_orient, cs)

        # Origin-local poses for marker children (before tracking-space offset)
        left_local, left_local_orient = left_pos, left_orient
        right_local, right_local_orient = right_pos, right_orient
        head_local, head_local_orient = head_pos, head_orient

        # --- Apply tracking-space offset → world-space poses ---

        tracking_space = self._get_tracking_space_transform()
        if tracking_space is None and self._tracking_space_xform is None and not self._tracking_space_retry_failed:
            try:
                self._reapply_tracking_space()
            except Exception as exc:
                self._tracking_space_retry_failed = True
                print(f"[Teleop][Session] Tracking-space reapply failed, skipping further retries: {exc}")
            tracking_space = self._get_tracking_space_transform()
        left_pos, left_orient = self._apply_tracking_space_offset(left_pos, left_orient, tracking_space)
        right_pos, right_orient = self._apply_tracking_space_offset(right_pos, right_orient, tracking_space)
        head_pos, head_orient = self._apply_tracking_space_offset(head_pos, head_orient, tracking_space)

        # Write origin-local poses to marker children first
        if self._live_tracking_enabled:
            self._update_marker_positions(
                left_local,
                left_local_orient,
                right_local,
                right_local_orient,
                head_local,
                head_local_orient,
            )

        # Controllers need world-space targets.  When the tracking-space
        # offset was applied above the poses are already in world space.
        # If the tracking space was unavailable (offset was a no-op),
        # fall back to reading composed world poses from the markers so
        # controllers still react when the origin moves.
        if tracking_space is None and self._markers_manager is not None:
            lw = self._markers_manager.get_marker_world_pose("left")
            rw = self._markers_manager.get_marker_world_pose("right")
            if lw is not None:
                left_pos, left_orient = lw
            if rw is not None:
                right_pos, right_orient = rw

        if self._floating_tracking_enabled:
            self._update_floating_targets(left_pos, left_orient, right_pos, right_orient)

        if self._grasp_tracking_enabled:
            self._update_grasp_inputs(left_ctrl, right_ctrl)

        if self._ik_controller is not None:
            self._ik_controller.update_targets(left_pos, left_orient, right_pos, right_orient)

        if self._locomotion_tracking_enabled and self._locomotion_controller is not None:
            self._locomotion_controller.update(left_ctrl, right_ctrl)

    def _update_marker_positions(
        self,
        left_pos: tuple | None,
        left_orient: tuple | None,
        right_pos: tuple | None,
        right_orient: tuple | None,
        head_pos: tuple | None = None,
        head_orient: tuple | None = None,
    ) -> None:
        """Update marker transforms from VR pose data (origin-local).

        The origin marker is not written here — only locomotion carry or
        ``move_tracking_space_to`` change the origin.
        """
        if self._markers_manager is None:
            return
        if not self._markers_manager.has_active_markers:
            return

        self._markers_manager.update_marker_transforms(
            left_pos,
            left_orient,
            right_pos,
            right_orient,
            head_pos,
            head_orient,
        )

    def _update_floating_targets(
        self,
        left_pos: tuple | None,
        left_orient: tuple | None,
        right_pos: tuple | None,
        right_orient: tuple | None,
    ) -> None:
        """Update floating controller targets from pre-extracted VR wrist data."""
        left_assigned = self._left_floating_assigned
        right_assigned = self._right_floating_assigned

        if (
            self._floating_tracking_enabled
            and self._floating_controller is not None
            and self._floating_controller.is_enabled
        ):
            floating_left_pos = left_pos if left_assigned else None
            floating_left_orient = left_orient if left_assigned else None
            floating_right_pos = right_pos if right_assigned else None
            floating_right_orient = right_orient if right_assigned else None
            if floating_left_pos is not None or floating_right_pos is not None:
                self._floating_controller.set_targets(
                    left_wrist_position=floating_left_pos,
                    left_wrist_orientation=floating_left_orient,
                    right_wrist_position=floating_right_pos,
                    right_wrist_orientation=floating_right_orient,
                )
                self._floating_controller.apply_tracking()

    def _update_grasp_inputs(self, left_ctrl: Any, right_ctrl: Any) -> None:
        """Update grasp joint targets from pre-extracted VR controller data."""
        if self._grasp_controller is None or not self._grasp_controller.is_enabled:
            return

        if left_ctrl is not None:
            self._grasp_controller.set_input("left", left_ctrl.inputs.trigger_value)

        if right_ctrl is not None:
            self._grasp_controller.set_input("right", right_ctrl.inputs.trigger_value)

    def destroy(self) -> None:
        """Clean up all resources including controllers and subscriptions."""
        self._timeline_sub = None
        self._xr_command_sub = None
        self._command_sub = None
        self._stage_closing_sub = None
        self._on_stage_closing = None
        self._on_command_executed = None
        self._controller_inputs_observers.clear()
        self._head_observers.clear()
        if self._uninstall_session_injector is not None:
            try:
                self._uninstall_session_injector()
            except Exception as exc:  # noqa: BLE001
                carb.log_warn(f"TeleopManager: uninstall session injector raised: {exc}")
            self._uninstall_session_injector = None
        if self._vr_recording_button is not None:
            try:
                self._vr_recording_button.destroy()
            except Exception as exc:  # noqa: BLE001
                carb.log_warn(f"TeleopManager: VRRecordingButton.destroy raised: {exc}")
            self._vr_recording_button = None
        if self._xr_anchor is not None:
            self._xr_anchor.cleanup()
            self._xr_anchor = None
        self.destroy_all_controllers()
        self.disconnect()

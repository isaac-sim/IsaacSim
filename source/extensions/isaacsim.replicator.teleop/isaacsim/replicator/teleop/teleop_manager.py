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
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import TYPE_CHECKING, Any

import carb
import carb.eventdispatcher
import carb.events
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import XformPrim
from pxr import Gf, Usd, UsdGeom

from ._xform_utils import WorldPosePrimCache, read_world_pose_gf
from .capabilities import get_teleop_capabilities
from .cloudxr_env import prepare_live_cloudxr_env
from .coordinate_utils import CoordinateSystem, transform_pose
from .teleop_input import (
    DebugTeleopFrameProvider,
    LiveTeleopFrameProvider,
    McapTeleopFrameProvider,
    TeleopFrame,
    TeleopFrameProvider,
    TeleopInputMode,
    TeleopPollStatus,
    TeleopPose,
)
from .teleop_session_injector import install_teleop_session_injector
from .vr_recording_button import VRButton, VRRecordingButton
from .xr_anchor_manager import AnchorRotationMode, KitXrRuntimeState, XrAnchorManager, get_kit_xr_runtime_state

if TYPE_CHECKING:
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
        command: :class:`TeleopCommand` enum value or lowercase string (``"connect"``, ``"start"``, ``"stop"``, ``"reset"``, ``"disconnect"``).
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

    Args:
        payload: Value for payload.

    Returns:
        The requested value.
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
        self._session_stack: contextlib.ExitStack | None = None
        self._input_provider: TeleopFrameProvider | None = None
        self._last_input_frame: TeleopFrame | None = None
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
        self._left_input_world_pos: tuple[float, float, float] | None = None
        self._right_input_world_pos: tuple[float, float, float] | None = None
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
        self._tracking_space_xform: XformPrim | Usd.Prim | None = None
        self._tracking_space_world_pose_cache = WorldPosePrimCache()
        self._xr_anchor: XrAnchorManager | None = None
        self._xr_anchor_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._xr_anchor_rotation_mode = AnchorRotationMode.FIXED
        self._xr_anchor_smoothing_time = 1.0
        self._xr_anchor_fixed_height = True
        self._on_stage_cleanup_completed: Callable[[], None] | None = None
        usd_ctx = omni.usd.get_context()
        event_dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self._stage_closing_sub = event_dispatcher.observe_event(
            event_name=usd_ctx.stage_event_name(omni.usd.StageEventType.CLOSING),
            on_event=self._handle_stage_closing,
            observer_name="TeleopManager._handle_stage_closing",
        )
        self._on_command_executed: Callable[[TeleopCommand, bool, str], None] | None = None
        self._command_sub = event_dispatcher.observe_event(
            event_name=TELEOP_CMD_EVENT,
            on_event=self._on_command_event,
            observer_name="TeleopManager._on_command_event",
        )
        self._xr_command_sub = None
        self._timeline_play_sub = event_dispatcher.observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_PLAY,
            on_event=self._on_timeline_play_event,
            observer_name="TeleopManager._on_timeline_play_event",
        )
        self._timeline_stop_sub = event_dispatcher.observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._on_timeline_stop_event,
            observer_name="TeleopManager._on_timeline_stop_event",
        )
        self._controller_inputs_observers: list[Callable[[object | None, object | None], None]] = []
        self._head_observers: list[Callable[[object | None], None]] = []
        self._input_frame_observers: list[Callable[[TeleopFrame], None]] = []
        self._uninstall_session_injector: Callable[[], None] | None = install_teleop_session_injector(self)
        self._vr_recording_button: VRRecordingButton | None = self._auto_attach_vr_recording_button()
        self._vr_recording_button_suspended = False

    def _auto_attach_vr_recording_button(self) -> VRRecordingButton | None:
        """Attach the Meta Quest left-Y button to the recorder ``toggle`` command.

        The button dispatches :data:`EPISODE_CMD_EVENT
        <isaacsim.replicator.episode_recorder.EPISODE_CMD_EVENT>` with
        ``session_id=None`` (broadcast). When no recorder has an open session
        the dispatch is a no-op, so keeping the binding alive for the whole
        lifetime of :class:`TeleopManager` is safe whether the user is
        recording or not.

        Returns:
            The requested value.
        """
        try:
            button = VRRecordingButton(self, button=VRButton.LEFT_SECONDARY, command="toggle")
            button.attach()
            return button
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"TeleopManager: auto-attach VR recording button failed: {exc}")
            return None

    def set_on_stage_cleanup_completed(self, callback: Callable[[], None] | None) -> None:
        """Register a callback invoked after stage-bound teleop resources are released.

        The callback runs during the USD stage-closing event, after automatic disconnect,
        controller teardown, and cached-stage-state cleanup.

        Args:
            callback: Value for callback.
        """
        self._on_stage_cleanup_completed = callback

    def _handle_stage_closing(self, event: carb.eventdispatcher.Event) -> None:
        """Automatically disconnects session and tears down all controllers on stage close.

        Args:
            event: Value for event.
        """
        if self._is_connected:
            print("[Teleop][Session] Stage closing - disconnecting session.")
            self.disconnect()
        elif self._debug_tracking_enabled:
            self.set_debug_tracking(False)

        self.destroy_all_controllers()

        if self._xr_anchor is not None:
            self._xr_anchor.cleanup()
            self._xr_anchor = None

        self._live_tracking_enabled = False
        if self._markers_manager is not None:
            self._markers_manager.clear_cached_state()
        self._left_input_world_pos = None
        self._right_input_world_pos = None
        self._last_input_frame = None
        self._debug_left_snapshot = _DebugControllerSnapshot()
        self._debug_right_snapshot = _DebugControllerSnapshot()
        self._frame_count = 0
        self._tracking_space_enabled = False
        self._tracking_space_xform = None
        self._tracking_space_world_pose_cache.clear()
        self._tracking_space_prim_path = ""
        self._active_tracking_space_prim_path = ""
        self._cached_tracking_space = None
        self._cached_tracking_space_frame = -1
        self._tracking_space_retry_failed = False

        if self._on_stage_cleanup_completed:
            self._on_stage_cleanup_completed()

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

    def set_on_status_changed(self, callback: Callable[[str], None] | None) -> None:
        """Register a callback for live input connection and data-status changes.

        The callback is retained across command-driven disconnect/reconnect
        cycles. Passing a callback directly to :meth:`connect` replaces it for
        callers that manage the low-level transport themselves.

        Args:
            callback: Status-message callback, or *None* to clear it.
        """
        self._on_status_changed = callback

    def execute_command(self, command: TeleopCommand) -> tuple[bool, str]:
        """Execute a teleop command and notifies all listeners.

        This is the single entry point for both the desktop UI and
        external command bus.  After execution, it fires:

        * The local ``on_command_executed`` callback (for the UI)
        * A :data:`TELEOP_STATUS_EVENT` event (for any Kit listener)

        Args:
            command: The command to execute.

        Returns:
            The requested value.
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
        """Handle incoming command bus events.

        Args:
            event: Value for event.
        """
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

        Args:
            event: Value for event.
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
        """Connect to OpenXR, creates markers, sets up XR anchor, starts live tracking.

        Returns:
            The requested value.
        """
        if self._is_connected:
            return True, "Already connected"

        status_callback = self._on_status_changed
        connect_status = ""

        def _capture_status(message: str) -> None:
            nonlocal connect_status
            connect_status = message
            if status_callback is not None:
                status_callback(message)

        try:
            success = self.connect(on_status_changed=_capture_status)
        finally:
            # ``connect_provider`` retains the callback it receives for live
            # data-status updates. Keep the explicitly registered callback,
            # not this per-attempt capture wrapper, across reconnects.
            self._on_status_changed = status_callback
        if not success:
            return False, connect_status or "Connection failed"

        try:
            if self._markers_manager:
                for name in ("origin", "left", "right", "head"):
                    ok, msg = self._markers_manager.ensure_marker(name)
                    if not ok:
                        raise RuntimeError(msg)
            self.set_live_tracking(True)

            tracking_ok, tracking_message = self._reapply_tracking_space()
            if not tracking_ok:
                raise RuntimeError(tracking_message)

            if not self._setup_xr_anchor():
                raise RuntimeError("XR anchor setup failed")
        except Exception as exc:  # noqa: BLE001 - roll back every partial session resource.
            if self._xr_anchor is not None:
                self._xr_anchor.cleanup()
                self._xr_anchor = None
            self.set_live_tracking(False)
            if self._markers_manager is not None:
                self._markers_manager.remove_all_markers()
            self.disconnect()
            message = f"Connection rolled back: {exc}"
            carb.log_warn(f"[Teleop][Session] {message}")
            return False, message

        return True, "Connected"

    def _cmd_start(self) -> tuple[bool, str]:
        """Plays the simulation timeline (headset "Play" button).

        The user configures and starts controllers from the desktop UI.
        This command simply plays the timeline so that physics-based
        controllers (floating, articulation, IK) begin receiving physics
        steps.  Mirrors IsaacLab's Play behavior where the main loop
        switches from ``render()`` to ``env.step(actions)``.

        Returns:
            The requested value.
        """
        if app_utils.is_playing():
            return True, "Timeline already playing"

        app_utils.play(commit=False)
        return True, "Timeline playing"

    def _cmd_stop(self) -> tuple[bool, str]:
        """Stop the simulation timeline (headset "Stop" button).

        Pauses physics so controllers stop receiving steps.
        Markers keep tracking so the user can still see hand positions.

        Returns:
            The requested value.
        """
        if not app_utils.is_playing():
            return True, "Timeline already stopped"

        app_utils.stop(commit=False)
        return True, "Timeline stopped"

    def _cmd_reset(self) -> tuple[bool, str]:
        """Stop the timeline and resets it to frame 0 (headset "Reset" button).

        Stops the simulation, rewinds the timeline to the beginning,
        and re-validates the tracking space.  The XR session stays
        alive and markers keep tracking.  The user can press Play to
        restart.

        Returns:
            The requested value.
        """
        timeline = omni.timeline.get_timeline_interface()
        was_playing = app_utils.is_playing()

        if was_playing:
            app_utils.stop(commit=False)

        timeline.set_current_time(0.0)

        self._reapply_tracking_space()
        if self._xr_anchor is not None:
            self._xr_anchor.reset()

        provider = self._input_provider
        if provider is not None and provider.input_mode == TeleopInputMode.MCAP_REPLAY:
            try:
                if not provider.rewind():
                    self.disconnect()
                    return False, "Timeline reset, but the replay provider cannot rewind and was disconnected"
                self._frame_count = 0
                self._last_input_frame = None
                self._left_input_world_pos = None
                self._right_input_world_pos = None
            except Exception as exc:  # noqa: BLE001
                self.disconnect()
                return False, f"Timeline reset, but MCAP rewind failed and replay was disconnected: {exc}"

        return True, "Timeline reset to t=0"

    def _cmd_disconnect(self) -> tuple[bool, str]:
        """Full teardown: stops controllers, removes markers, ends session.

        Returns:
            The requested value.
        """
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

    def _on_timeline_play_event(self, _event: object) -> None:
        if not (self._is_connected or self._debug_tracking_enabled):
            return
        self._on_timeline_play()

    def _on_timeline_stop_event(self, _event: object) -> None:
        if not (self._is_connected or self._debug_tracking_enabled):
            return
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
        """Return True if the teleop session is connected.

        Returns:
            The requested value.
        """
        return self._is_connected

    @staticmethod
    def _print_cloudxr_start_hint() -> None:
        """Print setup steps for the externally managed CloudXR runtime."""
        print("[Teleop][Session] Info: Start CloudXR in a separate terminal:")
        print("[Teleop][Session] Info:   python -m isaacteleop.cloudxr --accept-eula")
        print("[Teleop][Session] Info: Pair the headset at https://nvidia.github.io/IsaacTeleop/client/")
        print("[Teleop][Session] Info: See `isaacsim.replicator.teleop/Overview.md` for setup steps.")

    @staticmethod
    def _short_connect_error(message: str) -> str:
        """Return a compact Session panel error; details belong in the terminal log."""
        lowered = message.lower()
        if "cloudxr not running" in lowered:
            return "CloudXR not running"
        if "cloudxr env missing" in lowered:
            return "CloudXR env missing"
        if "openxr" in lowered or "failed to get openxr" in lowered:
            return "OpenXR unavailable"
        if "already connected" in lowered:
            return "Already connected"
        text = message.strip()
        if len(text) > 36:
            return f"{text[:33]}..."
        return text

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

    def connect(
        self,
        on_status_changed: Callable[[str], None] | None = None,
        *,
        input_mode: TeleopInputMode | str = TeleopInputMode.LIVE,
        mcap_path: str = "",
        mcap_recording_path: str = "",
        mcap_recording_overwrite: bool = False,
    ) -> bool:
        """Open a live OpenXR or headless MCAP input source.

        This is the low-level transport API. It does not create frame markers,
        activate tracking space, or configure Kit's XR anchor. Desktop and
        full live-session callers should use
        ``execute_command(TeleopCommand.CONNECT)`` instead. MCAP replay and
        custom-provider integrations may use this method directly when XR
        rendering is intentionally out of scope.

        Live MCAP recording begins when the source connects and closes on
        :meth:`disconnect`. MCAP replay advances one input frame per Kit app
        update and does not require OpenXR or CloudXR.

        Args:
            on_status_changed: Optional callback for status updates.
            input_mode: ``live`` or ``mcap_replay``. Debug tracking continues
                to use :meth:`set_debug_tracking`.
            mcap_path: Isaac Teleop MCAP file used by replay mode.
            mcap_recording_path: Optional output MCAP for a live session.
            mcap_recording_overwrite: Replace an existing MCAP recording path
                instead of rejecting it.

        Returns:
            Whether the input provider connected successfully.
        """
        try:
            mode = input_mode if isinstance(input_mode, TeleopInputMode) else TeleopInputMode(input_mode)
        except ValueError:
            message = f"Unsupported input mode: {input_mode!r}"
            print(f"[Teleop][Session] {message}")
            if on_status_changed:
                on_status_changed(f"Error: {self._short_connect_error(message)}")
            return False

        if mode == TeleopInputMode.DEBUG:
            message = "Use debug mode (set_debug_tracking)"
            print("[Teleop][Session] Use set_debug_tracking(True) for debug marker input.")
            if on_status_changed:
                on_status_changed(f"Error: {message}")
            return False
        if self._is_connected:
            message = "Already connected"
            print("[Teleop][Session] A teleop input provider is already connected.")
            if on_status_changed:
                on_status_changed(f"Error: {message}")
            return False
        capabilities = get_teleop_capabilities()
        native_input_available = (
            capabilities.mcap_replay if mode == TeleopInputMode.MCAP_REPLAY else capabilities.live_input
        )
        if not native_input_available:
            message = capabilities.native_input_unavailable_reason
            print(f"[Teleop][Session] {message}")
            if on_status_changed:
                on_status_changed(f"Error: {message}")
            return False
        if mode == TeleopInputMode.MCAP_REPLAY:
            if not mcap_path.strip():
                message = "MCAP path required"
                print("[Teleop][Session] MCAP replay requires mcap_path.")
                if on_status_changed:
                    on_status_changed(f"Error: {message}")
                return False
            provider: TeleopFrameProvider = McapTeleopFrameProvider(mcap_path)
        else:
            ready, cloudxr_message = prepare_live_cloudxr_env()
            if not ready:
                print(f"[Teleop][Session] {cloudxr_message}")
                self._print_cloudxr_start_hint()
                if on_status_changed:
                    on_status_changed(f"Error: {cloudxr_message}")
                return False
            provider = LiveTeleopFrameProvider(
                mcap_recording_path=mcap_recording_path or None,
                overwrite=mcap_recording_overwrite,
            )
        return self.connect_provider(provider, on_status_changed=on_status_changed)

    def connect_provider(
        self,
        provider: TeleopFrameProvider,
        on_status_changed: Callable[[str], None] | None = None,
    ) -> bool:
        """Connect a caller-supplied live or replay input provider.

        This is the extension point for additional transport adapters. Debug
        providers are managed by :meth:`set_debug_tracking` so debug mode does
        not masquerade as a hardware connection.

        Args:
            provider: Provider to own after a successful open and until
                :meth:`disconnect`. The caller retains ownership when this
                method returns ``False``.
            on_status_changed: Optional callback for status updates.

        Returns:
            Whether the provider opened successfully.
        """
        if provider.input_mode == TeleopInputMode.DEBUG:
            message = "Use debug mode"
            if on_status_changed:
                on_status_changed(f"Error: {message}")
            return False
        if self._is_connected:
            message = "Already connected"
            print("[Teleop][Session] A teleop input provider is already connected.")
            if on_status_changed:
                on_status_changed(f"Error: {message}")
            return False
        if self._debug_tracking_enabled:
            self.set_debug_tracking(False)

        previous_on_status_changed = self._on_status_changed
        self._on_status_changed = on_status_changed
        self._update_fail_count = 0

        stack = contextlib.ExitStack()
        try:
            stack.enter_context(provider)
        except Exception as exc:  # noqa: BLE001 - native OpenXR/MCAP bindings raise runtime-specific errors.
            print(f"[Teleop][Session] Failed to open {provider.input_mode.value} input: {exc}")
            if provider.input_mode == TeleopInputMode.LIVE:
                self._print_cloudxr_start_hint()
            if on_status_changed:
                on_status_changed(f"Error: {self._short_connect_error(str(exc))}")
            stack.close()
            # A provider that never opened sends no data status, so the callback
            # registered before this attempt stays in place.
            self._on_status_changed = previous_on_status_changed
            return False

        self._session_stack = stack
        self._input_provider = provider
        self._last_input_frame = None
        self._ensure_update_subscription()
        self._is_connected = True
        self._frame_count = 0

        if provider.input_mode == TeleopInputMode.LIVE:
            self._subscribe_xr_command_bus()
        elif provider.input_mode == TeleopInputMode.MCAP_REPLAY:
            self._suspend_auto_recording_button()

        status = self._connected_status()
        if on_status_changed:
            on_status_changed(status)
        return True

    def disconnect(self, on_status_changed: Callable[[str], None] | None = None) -> None:
        """Disconnect from the teleop session.

        Args:
            on_status_changed: Value for on status changed.
        """
        if not self._is_connected:
            print("[Teleop][Session] Not connected.")
            return

        if self._locomotion_controller is not None:
            self._locomotion_controller.stop_motion()
        self._release_update_subscription()
        if self._session_stack is not None:
            try:
                self._session_stack.close()
            except Exception as e:
                print(f"[Teleop][Session] Error closing sessions: {e}")
            self._session_stack = None
        self._input_provider = None
        self._last_input_frame = None
        self._left_input_world_pos = None
        self._right_input_world_pos = None
        self._xr_command_sub = None
        self._is_connected = False
        self._frame_count = 0
        self._resume_auto_recording_button()
        if on_status_changed:
            on_status_changed("Disconnected")

    @property
    def input_mode(self) -> TeleopInputMode | None:
        """Return the active live, debug, or MCAP input mode."""
        return self._input_provider.input_mode if self._input_provider is not None else None

    @property
    def last_input_frame(self) -> TeleopFrame | None:
        """Return the most recently finalized frame, or ``None`` before data arrives."""
        return self._last_input_frame

    def _connected_status(self) -> str:
        """Return the user-facing status for the active provider."""
        if self.input_mode == TeleopInputMode.MCAP_REPLAY:
            return "Connected (MCAP replay)"
        return "Connected"

    def _suspend_auto_recording_button(self) -> None:
        """Prevent replayed button edges from toggling a new HDF5 recording."""
        button = self._vr_recording_button
        if button is None or not button.is_attached:
            return
        button.detach()
        self._vr_recording_button_suspended = True

    def _resume_auto_recording_button(self) -> None:
        """Restore the automatic recording binding after MCAP replay closes."""
        if not self._vr_recording_button_suspended:
            return
        self._vr_recording_button_suspended = False
        if self._vr_recording_button is None:
            return
        try:
            self._vr_recording_button.attach()
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"TeleopManager: failed to restore VR recording button: {exc}")

    @property
    def debug_tracking_enabled(self) -> bool:
        """True when debug tracking mode is active.

        Returns:
            The requested value.
        """
        return self._debug_tracking_enabled

    def _get_debug_snapshot(self, side: str) -> _DebugControllerSnapshot | None:
        """Return the synthetic controller snapshot for the requested side.

        Args:
            side: Value for side.

        Returns:
            The requested value.
        """
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

        Args:
            enabled: Value for enabled.
        """
        if enabled and self._is_connected:
            print("[Teleop][Debug] Cannot enable debug tracking while another input source is connected.")
            return
        if self._debug_tracking_enabled == enabled:
            return
        self._debug_tracking_enabled = enabled
        if enabled:
            debug_provider = DebugTeleopFrameProvider(
                self._read_marker_world_pose,
                self._debug_left_snapshot,
                self._debug_right_snapshot,
            )
            debug_provider.open()
            self._input_provider = debug_provider
            self._last_input_frame = None
            self._ensure_update_subscription()
            self._reapply_tracking_space()
            print("[Teleop][Debug] Tracking enabled - reading poses from markers.")
        else:
            provider = self._input_provider
            if provider is not None and provider.input_mode == TeleopInputMode.DEBUG:
                provider.close()
                self._input_provider = None
            self._last_input_frame = None
            self._left_input_world_pos = None
            self._right_input_world_pos = None
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

    def set_debug_squeeze(self, side: str, value: float) -> None:
        """Set the synthetic grip/squeeze value for debug tracking mode.

        Args:
            side: ``"left"`` or ``"right"``.
            value: Squeeze analog value in [0.0, 1.0].
        """
        snapshot = self._get_debug_snapshot(side)
        if snapshot is None:
            return
        snapshot.inputs.squeeze_value = max(0.0, min(1.0, value))

    def set_debug_thumbstick(self, side: str, *, x: float | None = None, y: float | None = None) -> None:
        """Set synthetic thumbstick axes for debug tracking mode.

        Args:
            side: Value for side.
            x: Value for x.
            y: Value for y.
        """
        snapshot = self._get_debug_snapshot(side)
        if snapshot is None:
            return
        if x is not None:
            snapshot.inputs.thumbstick_x = max(-1.0, min(1.0, x))
        if y is not None:
            snapshot.inputs.thumbstick_y = max(-1.0, min(1.0, y))

    def set_debug_button(self, side: str, button: str, pressed: bool) -> None:
        """Set a synthetic controller button state for debug tracking mode.

        Args:
            side: Value for side.
            button: Value for button.
            pressed: Value for pressed.
        """
        snapshot = self._get_debug_snapshot(side)
        if snapshot is None or button not in {"primary_click", "secondary_click", "thumbstick_click"}:
            return
        setattr(snapshot.inputs, button, bool(pressed))

    @property
    def carry_tracking_space_enabled(self) -> bool:
        """True when locomotion Carry Tracking Space is active.

        Returns:
            Whether carry is enabled on the registered locomotion controller.
        """
        if self._locomotion_controller is None:
            return False
        return self._locomotion_controller.carry_tracking_space_enabled

    @property
    def carry_tracking_space_available(self) -> bool:
        """True when Carry Tracking Space can be toggled for the current locomotion setup."""
        if self._locomotion_controller is None:
            return False
        return self._locomotion_controller.carry_tracking_space_available

    def set_carry_tracking_space(self, enabled: bool) -> bool:
        """Enable or disable locomotion Carry Tracking Space.

        Args:
            enabled: Whether locomotion should co-move the tracking-space prim.

        Returns:
            True if the requested state was applied.
        """
        if self._locomotion_controller is None:
            return False
        return self._locomotion_controller.set_carry_tracking_space(enabled)

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
        """Use the built-in Teleop origin marker as tracking space.

        Returns:
            The requested value.
        """
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
            self._apply_builtin_anchor_config()
            print(f"[Teleop][Session] Tracking Space set to built-in Teleop marker '{builtin_path}'.")
        return ok, message

    def set_tracking_space_prim_path(self, path: str) -> tuple[bool, str]:
        """Use a custom scene prim as tracking space.

        Args:
            path: Value for path.

        Returns:
            The requested value.
        """
        from .markers_manager import MarkersManager

        requested_path = path.strip()
        if not requested_path:
            return self.set_builtin_tracking_space()
        if requested_path == MarkersManager.MARKERS_SCOPE or requested_path.startswith(
            f"{MarkersManager.MARKERS_SCOPE}/"
        ):
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
            self._apply_custom_anchor_config()
            print(f"[Teleop][Session] Tracking Space set to '{requested_path}'.")
        return ok, message

    def _apply_builtin_anchor_config(self) -> None:
        """Apply absolute built-in-origin placement without a self-referential offset."""
        if self._markers_manager is not None and self._markers_manager.has_active_markers:
            existing = self._markers_manager.get_marker_world_pose("origin")
            orientation = existing[1] if existing is not None else (0.0, 0.0, 0.0, 1.0)
            self._markers_manager.set_origin_world_pose(self._xr_anchor_pos, orientation)
        if self._xr_anchor is not None:
            self._xr_anchor.set_anchor_pos((0.0, 0.0, 0.0))
            self._xr_anchor.set_rotation_mode(AnchorRotationMode.FOLLOW_PRIM)
            self._xr_anchor.set_fixed_height(False)
        self._cached_tracking_space_frame = -1

    def _apply_custom_anchor_config(self) -> None:
        """Apply the user-facing anchor controls to an active custom scene anchor."""
        if self._xr_anchor is not None:
            self._xr_anchor.set_anchor_pos(self._xr_anchor_pos)
            self._xr_anchor.set_rotation_mode(self._xr_anchor_rotation_mode)
            self._xr_anchor.set_smoothing_time(self._xr_anchor_smoothing_time)
            self._xr_anchor.set_fixed_height(self._xr_anchor_fixed_height)
        self._cached_tracking_space_frame = -1

    def _teleop_edit_ctx(self, stage: Usd.Stage, prim_path: str) -> AbstractContextManager[None]:
        """Return an ``Usd.EditContext`` targeting the markers anonymous layer for Teleop prims.

        Args:
            stage: Value for stage.
            prim_path: Value for prim path.

        Returns:
            The requested value.
        """
        layer = self._markers_manager.layer if self._markers_manager is not None else None
        if (
            layer is not None
            and prim_path.startswith("/Teleop/")
            and any(
                layer.identifier == stage_layer.identifier
                for stage_layer in stage.GetLayerStack(includeSessionLayers=True)
            )
        ):
            return Usd.EditContext(stage, layer)
        return nullcontext()

    def _apply_tracking_space_path(self, resolved_path: str) -> tuple[bool, str]:
        """Validate and activate a tracking-space path without changing mode semantics.

        For built-in Teleop prims (under ``/Teleop/``), xformOp writes are
        directed to the markers anonymous layer so no specs leak to the root
        layer.

        Args:
            resolved_path: Value for resolved path.

        Returns:
            The requested value.
        """
        if not stage_utils.is_stage_set() and omni.usd.get_context().get_stage() is None:
            return False, "No USD stage available. Current tracking space is unchanged."
        stage = stage_utils.get_current_stage()

        prim = stage.GetPrimAtPath(resolved_path)
        if not prim or not prim.IsValid():
            return False, f"Tracking Space prim not found: {resolved_path}. Current tracking space is unchanged."

        if not prim.IsA(UsdGeom.Xformable):
            return False, f"Tracking Space prim is not Xformable: {resolved_path}. Current tracking space is unchanged."

        is_builtin = resolved_path.startswith("/Teleop/")
        props = prim.GetPropertyNames()
        needs_reset = is_builtin and any(
            op not in props for op in ("xformOp:translate", "xformOp:orient", "xformOp:scale")
        )

        if is_builtin:
            edit_ctx = self._teleop_edit_ctx(stage, resolved_path)
            with edit_ctx:
                tracking_space_xform = XformPrim(resolved_path, reset_xform_op_properties=needs_reset)
        else:
            tracking_space_xform = prim

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
        return True, f"Tracking Space: {resolved_path}"

    def _reapply_tracking_space(self) -> tuple[bool, str]:
        """Reapply the currently selected tracking space after connect/reset.

        Always activates at least the built-in origin marker so that
        VR pose offsetting and locomotion carry work out of the box.

        Returns:
            The requested value.
        """
        if self._tracking_space_prim_path:
            return self.set_tracking_space_prim_path(self._tracking_space_prim_path)
        if self._markers_manager is None:
            self._active_tracking_space_prim_path = ""
            self._tracking_space_xform = None
            self._tracking_space_world_pose_cache.set_prim_path("")
            self._cached_tracking_space_frame = -1
            return True, "Tracking Space: world origin"
        return self.set_builtin_tracking_space()

    @property
    def tracking_space_prim_path(self) -> str:
        """Return the current tracking-space prim path.

        Returns:
            The requested value.
        """
        return self._active_tracking_space_prim_path

    def _get_tracking_space_transform(self) -> tuple[Gf.Vec3d, Gf.Rotation, Gf.Quatd] | None:
        """Read the tracking-space world transform via the active backend.

        Results are cached per frame so multiple callers within the same
        ``_on_update`` do not re-read the prim.

        Returns:
            The requested value.
        """
        if self._cached_tracking_space_frame == self._frame_count:
            return self._cached_tracking_space

        if self._xr_anchor is not None:
            result = self._sync_canonical_anchor()
            if result is None:
                return None
            self._cached_tracking_space = result
            self._cached_tracking_space_frame = self._frame_count
            return result

        if self._tracking_space_xform is None:
            return None

        tracking_space_valid = (
            self._tracking_space_xform.valid
            if isinstance(self._tracking_space_xform, XformPrim)
            else self._tracking_space_xform.IsValid()
        )
        if not tracking_space_valid:
            self._tracking_space_xform = None
            self._tracking_space_world_pose_cache.clear()
            return None

        pos, qd = read_world_pose_gf(self._tracking_space_world_pose_cache)
        result = pos, Gf.Rotation(qd), qd
        self._cached_tracking_space = result
        self._cached_tracking_space_frame = self._frame_count
        return result

    def _sync_canonical_anchor(self) -> tuple[Gf.Vec3d, Gf.Rotation, Gf.Quatd] | None:
        """Synchronize the generated XR anchor and its custom-origin marker proxy."""
        if self._xr_anchor is None or not self._xr_anchor.sync():
            return None
        pos, qd = self._xr_anchor.get_world_pose()
        result = pos, Gf.Rotation(qd), qd

        # With a custom scene anchor, the runtime marker origin is a visual
        # proxy. Keep it on the same final transform even while input is idle.
        if self._tracking_space_prim_path and self._markers_manager is not None:
            image = qd.GetImaginary()
            self._markers_manager.set_origin_world_pose(
                (float(pos[0]), float(pos[1]), float(pos[2])),
                (float(image[0]), float(image[1]), float(image[2]), float(qd.GetReal())),
            )
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
            tracking_space: ``(position, rotation, quaternion)`` from :meth:`_get_tracking_space_transform`, or *None* to skip the offset.

        Returns:
            The requested value.
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

    def _setup_xr_anchor(self) -> bool:
        """Create the canonical XR/teleop anchor for the active tracking space.

        Returns:
            Whether the generated runtime anchor was created successfully.
        """
        if self._xr_anchor is not None:
            self._xr_anchor.cleanup()

        from .markers_manager import MarkersManager

        is_builtin = (
            not self._tracking_space_prim_path
            and self._active_tracking_space_prim_path == MarkersManager.MARKER_PATHS["origin"]
        )
        self._xr_anchor = XrAnchorManager(
            anchor_pos=(0.0, 0.0, 0.0) if is_builtin else self._xr_anchor_pos,
            tracking_space_prim_path=self._active_tracking_space_prim_path,
            rotation_mode=AnchorRotationMode.FOLLOW_PRIM if is_builtin else self._xr_anchor_rotation_mode,
            smoothing_time=self._xr_anchor_smoothing_time,
            fixed_height=False if is_builtin else self._xr_anchor_fixed_height,
        )
        setup_succeeded = self._xr_anchor.setup()
        if setup_succeeded:
            # Re-evaluate after setup because Kit XR can initialize while
            # Teleop is acquiring the optional profile settings.
            xr_runtime_state = get_kit_xr_runtime_state()
            if xr_runtime_state != KitXrRuntimeState.ACTIVE or self._xr_anchor.is_xr_profile_configured:
                self._cached_tracking_space_frame = -1
                return True
            carb.log_error(
                f"[Teleop][Anchor] Kit XR is {xr_runtime_state.value}, "
                "but the session could not acquire its profile anchor settings."
            )
        self._xr_anchor.cleanup()
        self._xr_anchor = None
        return False

    @property
    def xr_anchor(self) -> XrAnchorManager | None:
        """The active XR anchor manager, or None if not connected.

        Returns:
            The requested value.
        """
        return self._xr_anchor

    def set_xr_anchor_pos(self, pos: tuple[float, float, float]) -> None:
        """Store the XR anchor position offset and apply it to an active session.

        Args:
            pos: Value for pos.
        """
        self._xr_anchor_pos = tuple(float(value) for value in pos)
        self._cached_tracking_space_frame = -1
        if self._tracking_space_prim_path:
            if self._xr_anchor is not None:
                self._xr_anchor.set_anchor_pos(self._xr_anchor_pos)
        elif self._active_tracking_space_prim_path:
            self._apply_builtin_anchor_config()
        elif self._xr_anchor is not None:
            self._xr_anchor.set_anchor_pos(self._xr_anchor_pos)

    def set_xr_anchor_rotation_mode(self, mode: AnchorRotationMode) -> None:
        """Store the XR anchor rotation mode and apply it to an active session.

        Args:
            mode: Value for mode.
        """
        self._xr_anchor_rotation_mode = mode
        self._cached_tracking_space_frame = -1
        if self._xr_anchor is not None and self._tracking_space_prim_path:
            self._xr_anchor.set_rotation_mode(mode)

    def set_xr_anchor_smoothing_time(self, seconds: float) -> None:
        """Store the XR anchor smoothing time and apply it to an active session.

        Args:
            seconds: Value for seconds.
        """
        self._xr_anchor_smoothing_time = max(0.01, float(seconds))
        self._cached_tracking_space_frame = -1
        if self._xr_anchor is not None and self._tracking_space_prim_path:
            self._xr_anchor.set_smoothing_time(self._xr_anchor_smoothing_time)

    def set_xr_anchor_fixed_height(self, fixed: bool) -> None:
        """Store the XR anchor fixed-height mode and apply it to an active session.

        Args:
            fixed: Value for fixed.
        """
        self._xr_anchor_fixed_height = bool(fixed)
        self._cached_tracking_space_frame = -1
        if self._xr_anchor is not None and self._tracking_space_prim_path:
            self._xr_anchor.set_fixed_height(self._xr_anchor_fixed_height)

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
        """Return True if live tracking is enabled.

        Returns:
            The requested value.
        """
        return self._live_tracking_enabled

    def get_input_world_position(self, side: str) -> tuple[float, float, float] | None:
        """Return the cached world-space controller input position for one side.

        This is the same aim-pose position fed to Floating and IK each frame
        (after coordinate conversion and tracking-space offset).  In debug
        mode it reflects the Left/Right marker world poses instead of VR
        hardware.  Returns ``None`` when no pose is available.
        """
        side = side.lower()
        if side == "left":
            return self._left_input_world_pos
        if side == "right":
            return self._right_input_world_pos
        return None

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
        """Assigns or clears the floating controller for a specific side.

        Args:
            side: Value for side.
            assigned: Value for assigned.
        """
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
        """Clear floating-controller assignment for a specific side.

        Args:
            side: Value for side.
        """
        self.set_floating_side_assigned(side, False)

    def is_floating_side_assigned(self, side: str) -> bool:
        """Return whether a side is assigned to the floating controller.

        Args:
            side: Value for side.

        Returns:
            The requested value.
        """
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
        elif not enabled and self._locomotion_controller is not None:
            self._locomotion_controller.stop_motion()

        self._locomotion_tracking_enabled = enabled
        state = "enabled" if enabled else "disabled"
        print(f"[Teleop][Locomotion] Tracking {state}.")

    @property
    def is_locomotion_tracking(self) -> bool:
        """Return True if locomotion tracking is enabled.

        Returns:
            The requested value.
        """
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
        """Return True if grasp tracking is enabled.

        Returns:
            The requested value.
        """
        return self._grasp_tracking_enabled

    def set_floating_tracking(self, enabled: bool) -> None:
        """Enable or disables floating rigid-body tracking.

        Args:
            enabled: Value for enabled.
        """
        if self._floating_tracking_enabled == enabled:
            return
        self._floating_tracking_enabled = enabled
        if not enabled and self._floating_controller:
            self._floating_controller.reset_targets()
        status = "enabled" if enabled else "disabled"
        print(f"[Teleop][Floating] Tracking {status}.")

    @property
    def is_floating_tracking(self) -> bool:
        """Return True if floating rigid-body tracking is enabled.

        Returns:
            The requested value.
        """
        return self._floating_tracking_enabled and (self._left_floating_assigned or self._right_floating_assigned)

    def add_input_frame_observer(self, observer: Callable[[TeleopFrame], None]) -> None:
        """Register an observer for finalized, source-independent input frames.

        Args:
            observer: Callback invoked once for every successfully polled frame.
        """
        self._input_frame_observers.append(observer)

    def remove_input_frame_observer(self, observer: Callable[[TeleopFrame], None]) -> None:
        """Remove a previously registered frame observer.

        Args:
            observer: Callback to remove. Unknown callbacks are ignored.
        """
        try:
            self._input_frame_observers.remove(observer)
        except ValueError:
            pass

    def _notify_input_frame_observers(self, frame: TeleopFrame) -> None:
        """Notify frame observers without allowing one callback to stop teleoperation."""
        for observer in list(self._input_frame_observers):
            try:
                observer(frame)
            except Exception as exc:  # noqa: BLE001
                carb.log_warn(f"[Teleop][Session] input-frame observer error: {exc}")

    def add_controller_inputs_observer(self, observer: Callable[[object | None, object | None], None]) -> None:
        """Register an observer invoked once per update with the current controller snapshots.

        The observer is called with ``(left_ctrl, right_ctrl)`` on every update in both live-VR and
        debug tracking modes. Either argument may be ``None`` when the corresponding controller is not
        tracked. Observers are intended to be lightweight (e.g. rising-edge button detection for the
        recording button); long-running work should be deferred to a background task.

        The same observer may be registered multiple times; each registration is independent.

        Args:
            observer: Value for observer.
        """
        self._controller_inputs_observers.append(observer)

    def remove_controller_inputs_observer(self, observer: Callable[[object | None, object | None], None]) -> None:
        """Deregister a previously added controller-inputs observer. Silently ignores unknown observers.

        Args:
            observer: Value for observer.
        """
        try:
            self._controller_inputs_observers.remove(observer)
        except ValueError:
            pass

    def _notify_controller_inputs_observers(self, left_ctrl: object | None, right_ctrl: object | None) -> None:
        """Invoke every registered observer with the current controller snapshots.

        Each observer is wrapped in try/except so a faulty subscriber cannot break the tracking loop.

        Args:
            left_ctrl: Value for left ctrl.
            right_ctrl: Value for right ctrl.
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

        Args:
            observer: Value for observer.
        """
        self._head_observers.append(observer)

    def remove_head_observer(self, observer: Callable[[object | None], None]) -> None:
        """Deregister a previously added head observer. Silently ignores unknown observers.

        Args:
            observer: Value for observer.
        """
        try:
            self._head_observers.remove(observer)
        except ValueError:
            pass

    def _notify_head_observers(self, head: object | None) -> None:
        """Invoke every registered head observer with the current head snapshot (or ``None``).

        Args:
            head: Value for head.
        """
        if not self._head_observers:
            return
        for observer in list(self._head_observers):
            try:
                observer(head)
            except Exception as exc:  # Keep the update loop alive even if a subscriber raises.
                carb.log_warn(f"[Teleop][Session] head observer error: {exc}")

    def _read_marker_world_pose(
        self, name: str
    ) -> tuple[tuple[float, float, float] | None, tuple[float, float, float, float] | None]:
        """Read the world-space pose of a marker for debug tracking.

        Args:
            name: Value for name.

        Returns:
            The requested value.
        """
        if self._markers_manager is None:
            return None, None
        result = self._markers_manager.get_marker_world_pose(name)
        if result is None:
            return None, None
        return result

    @staticmethod
    def _convert_source_pose(pose: TeleopPose, coordinate_system: CoordinateSystem) -> TeleopPose:
        """Convert a valid source pose into the configured scene coordinate system."""
        if not pose.is_valid or pose.position is None or pose.orientation_xyzw is None:
            return TeleopPose()
        position, orientation = transform_pose(pose.position, pose.orientation_xyzw, coordinate_system)
        return TeleopPose.from_values(position, orientation)

    @staticmethod
    def _pose_values(
        pose: TeleopPose,
    ) -> tuple[tuple[float, float, float] | None, tuple[float, float, float, float] | None]:
        """Return optional position/orientation tuples from a typed pose."""
        if not pose.is_valid:
            return None, None
        return pose.position, pose.orientation_xyzw

    def _apply_tracking_space_to_pose(
        self,
        pose: TeleopPose,
        tracking_space: tuple[Gf.Vec3d, Gf.Rotation, Gf.Quatd] | None,
    ) -> TeleopPose:
        """Apply the current tracking-space transform to a local pose."""
        position, orientation = self._pose_values(pose)
        position, orientation = self._apply_tracking_space_offset(position, orientation, tracking_space)
        return TeleopPose.from_values(position, orientation)

    def _finalize_source_frame(self, frame: TeleopFrame) -> TeleopFrame:
        """Convert source poses, update markers, and compose world-space poses."""
        coordinate_system = self._coordinate_system
        left_local = self._convert_source_pose(frame.left.source_aim_pose, coordinate_system)
        right_local = self._convert_source_pose(frame.right.source_aim_pose, coordinate_system)
        head_local = self._convert_source_pose(frame.head.source_pose, coordinate_system)

        tracking_space = self._get_tracking_space_transform()
        if tracking_space is None and self._tracking_space_xform is None and not self._tracking_space_retry_failed:
            try:
                self._reapply_tracking_space()
            except Exception as exc:
                self._tracking_space_retry_failed = True
                print(f"[Teleop][Session] Tracking-space reapply failed, skipping further retries: {exc}")
            tracking_space = self._get_tracking_space_transform()

        left_world = self._apply_tracking_space_to_pose(left_local, tracking_space)
        right_world = self._apply_tracking_space_to_pose(right_local, tracking_space)
        head_world = self._apply_tracking_space_to_pose(head_local, tracking_space)

        if self._live_tracking_enabled:
            left_world_values = self._pose_values(left_world)
            right_world_values = self._pose_values(right_world)
            head_world_values = self._pose_values(head_world)
            self._update_marker_positions(
                left_world_values[0],
                left_world_values[1],
                right_world_values[0],
                right_world_values[1],
                head_world_values[0],
                head_world_values[1],
            )

        # Preserve the existing marker fallback when no tracking-space Xform is
        # available. It lets a moved marker origin continue to produce world
        # targets without changing the raw snapshots observed by recorders.
        if tracking_space is None and self._markers_manager is not None:
            left_marker = self._markers_manager.get_marker_world_pose("left")
            right_marker = self._markers_manager.get_marker_world_pose("right")
            if left_marker is not None:
                left_world = TeleopPose.from_values(*left_marker)
            if right_marker is not None:
                right_world = TeleopPose.from_values(*right_marker)

        return replace(
            frame,
            left=replace(frame.left, local_aim_pose=left_local, world_aim_pose=left_world),
            right=replace(frame.right, local_aim_pose=right_local, world_aim_pose=right_world),
            head=replace(frame.head, local_pose=head_local, world_pose=head_world),
        )

    def _consume_input_frame(self, frame: TeleopFrame) -> None:
        """Dispatch one finalized frame to observers, markers, and controllers."""
        self._last_input_frame = frame
        left_ctrl = frame.left.snapshot
        right_ctrl = frame.right.snapshot
        self._notify_input_frame_observers(frame)

        left_pos, left_orient = self._pose_values(frame.left.world_aim_pose)
        right_pos, right_orient = self._pose_values(frame.right.world_aim_pose)
        self._left_input_world_pos = left_pos
        self._right_input_world_pos = right_pos

        if self._floating_tracking_enabled:
            self._update_floating_targets(left_pos, left_orient, right_pos, right_orient)

        if self._grasp_tracking_enabled:
            self._update_grasp_inputs(left_ctrl, right_ctrl)

        if self._ik_controller is not None:
            self._ik_controller.update_targets(left_pos, left_orient, right_pos, right_orient)

    def _on_update(self, event: Any) -> None:
        """Called each frame to update tracking data.

        Data flow (VR mode):
        1. Extract raw VR poses (OpenXR Y-up)
        2. Convert to target coordinate system (origin-local poses)
        3. Apply the canonical XR/teleop anchor transform
        4. Markers and controllers receive the same finalized world poses

        If the tracking-space offset is unavailable, the method falls
        back to reading composed world poses from the markers so that
        controllers still receive world-space targets.

        Live and MCAP providers supply source-space poses. The debug provider
        supplies marker poses that are already in world space, so they bypass
        coordinate conversion and tracking-space composition.

        Args:
            event: Value for event.
        """
        provider = self._input_provider
        if provider is None:
            return
        if provider.input_mode == TeleopInputMode.MCAP_REPLAY and not app_utils.is_playing():
            return
        if self._xr_anchor is not None:
            self._sync_canonical_anchor()

        try:
            result = provider.poll()
        except Exception as exc:  # noqa: BLE001 - keep Kit's update loop alive on native input errors.
            if self._update_fail_count == 0 and self._on_status_changed:
                self._on_status_changed("Error: Input update failed")
            if self._update_fail_count == 0:
                carb.log_warn(f"[Teleop][Session] Input update failed: {exc}")
            self._update_fail_count += 1
            return

        if result.status == TeleopPollStatus.NO_DATA:
            if self._update_fail_count == 0 and self._on_status_changed:
                self._on_status_changed(f"{self._connected_status()} (no data)")
            self._update_fail_count += 1
            return
        if result.frame is None:
            return

        if self._update_fail_count > 0:
            self._update_fail_count = 0
            if self._on_status_changed:
                self._on_status_changed(self._connected_status())

        self._frame_count += 1
        frame = result.frame

        # Preserve the legacy raw-snapshot contract: recorder and button
        # observers run immediately after polling, before coordinate, USD, or
        # controller work that may fail independently.
        self._notify_controller_inputs_observers(frame.left.snapshot, frame.right.snapshot)
        self._notify_head_observers(frame.head.snapshot)

        # Locomotion may move the active tracking-space prim. Apply it before
        # resolving this frame's canonical anchor so XR, markers, and robot
        # targets all observe the same post-locomotion transform.
        if self._locomotion_tracking_enabled and self._locomotion_controller is not None:
            self._locomotion_controller.update(frame.left.snapshot, frame.right.snapshot)

        if frame.input_mode != TeleopInputMode.DEBUG:
            frame = self._finalize_source_frame(frame)
        self._consume_input_frame(frame)

    def _update_marker_positions(
        self,
        left_pos: tuple | None,
        left_orient: tuple | None,
        right_pos: tuple | None,
        right_orient: tuple | None,
        head_pos: tuple | None = None,
        head_orient: tuple | None = None,
    ) -> None:
        """Update marker transforms from finalized world-space VR poses.

        The origin marker is synchronized separately from the canonical
        anchor when a custom scene prim is active.

        Args:
            left_pos: Value for left pos.
            left_orient: Value for left orient.
            right_pos: Value for right pos.
            right_orient: Value for right orient.
            head_pos: Value for head pos.
            head_orient: Value for head orient.
        """
        if self._markers_manager is None:
            return
        if not self._markers_manager.has_active_markers:
            return

        self._markers_manager.update_marker_world_transforms(
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
        """Update floating controller targets from pre-extracted VR wrist data.

        Args:
            left_pos: Value for left pos.
            left_orient: Value for left orient.
            right_pos: Value for right pos.
            right_orient: Value for right orient.
        """
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
        """Update grasp joint targets from pre-extracted VR controller data.

        Args:
            left_ctrl: Value for left ctrl.
            right_ctrl: Value for right ctrl.
        """
        if self._grasp_controller is None or not self._grasp_controller.is_enabled:
            return

        from .retargeting_grasp import (
            GraspDriveMode,
            compute_retargeted_joint_targets,
            parse_grasp_retargeter_kind,
        )

        for side, ctrl in (("left", left_ctrl), ("right", right_ctrl)):
            if ctrl is None:
                continue
            if not self._grasp_controller.is_side_tracking_enabled(side):
                continue
            drive_mode, retargeter_kind, joint_aliases = self._grasp_controller.get_side_drive_settings(side)
            if drive_mode == GraspDriveMode.RETARGETED.value:
                kind = parse_grasp_retargeter_kind(retargeter_kind)
                config = self._grasp_controller.get_side_config(side)
                if kind is None or config is None:
                    continue
                targets = compute_retargeted_joint_targets(
                    retargeter_kind=kind,
                    controller_snapshot=ctrl,
                    grasp_config=config,
                    hand_side=side,
                    joint_aliases=joint_aliases,
                )
                self._grasp_controller.set_joint_targets(side, targets)
            else:
                self._grasp_controller.set_input(side, ctrl.inputs.trigger_value)

    def destroy(self) -> None:
        """Clean up all resources including controllers and subscriptions."""
        if self._debug_tracking_enabled:
            self.set_debug_tracking(False)
        if self._is_connected:
            self.disconnect()
        else:
            self._release_update_subscription()
        self._timeline_play_sub = None
        self._timeline_stop_sub = None
        self._xr_command_sub = None
        self._command_sub = None
        self._stage_closing_sub = None
        self._on_stage_cleanup_completed = None
        self._on_command_executed = None
        self._on_status_changed = None
        self._controller_inputs_observers.clear()
        self._head_observers.clear()
        self._input_frame_observers.clear()
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

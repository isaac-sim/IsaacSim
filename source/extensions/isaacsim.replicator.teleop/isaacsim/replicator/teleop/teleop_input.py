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

"""Typed input frames and source providers for teleoperation."""

from __future__ import annotations

import contextlib
import importlib
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from .capabilities import get_teleop_capabilities
from .xr_anchor_manager import KitXrRuntimeState, get_kit_xr_runtime_state


class TeleopInputUnavailableError(RuntimeError):
    """Raised when an optional native teleop input backend is unavailable."""


class TeleopInputMode(str, Enum):
    """Available sources for teleoperation input frames."""

    LIVE = "live"
    DEBUG = "debug"
    MCAP_REPLAY = "mcap_replay"


class TeleopPollStatus(str, Enum):
    """Outcome of polling an input provider once."""

    FRAME = "frame"
    NO_DATA = "no_data"


@dataclass(frozen=True)
class TeleopPose:
    """One pose in a documented coordinate space.

    Orientations always use OpenXR's ``(x, y, z, w)`` quaternion order.
    Invalid poses retain ``None`` values rather than inventing a zero pose.
    """

    position: tuple[float, float, float] | None = None
    orientation_xyzw: tuple[float, float, float, float] | None = None
    is_valid: bool = False

    @classmethod
    def from_values(
        cls,
        position: tuple[float, float, float] | None,
        orientation_xyzw: tuple[float, float, float, float] | None,
    ) -> TeleopPose:
        """Build a valid pose when both position and orientation are available."""
        if position is None or orientation_xyzw is None:
            return cls()
        return cls(position=position, orientation_xyzw=orientation_xyzw, is_valid=True)


@dataclass(frozen=True)
class TeleopControllerState:
    """Controller snapshot plus its aim pose at each processing stage.

    ``snapshot`` is retained losslessly for existing grasp, locomotion, recorder,
    and button observers. Providers fill ``source_aim_pose``. The manager fills
    ``local_aim_pose`` after coordinate conversion and ``world_aim_pose`` after
    applying the tracking-space transform. Debug input supplies only the world
    pose because marker transforms are already in scene coordinates.
    """

    snapshot: object | None = None
    source_aim_pose: TeleopPose = field(default_factory=TeleopPose)
    local_aim_pose: TeleopPose = field(default_factory=TeleopPose)
    world_aim_pose: TeleopPose = field(default_factory=TeleopPose)


@dataclass(frozen=True)
class TeleopHeadState:
    """Head snapshot plus source, local, and world pose representations."""

    snapshot: object | None = None
    source_pose: TeleopPose = field(default_factory=TeleopPose)
    local_pose: TeleopPose = field(default_factory=TeleopPose)
    world_pose: TeleopPose = field(default_factory=TeleopPose)


@dataclass(frozen=True)
class TeleopFrame:
    """One coherent controller/head sample from a teleop input source.

    ``source_time_ns`` is populated when the tracker exposes a comparable
    device sample clock; otherwise providers leave it ``None``.
    """

    sequence_id: int
    monotonic_time_ns: int
    input_mode: TeleopInputMode
    left: TeleopControllerState = field(default_factory=TeleopControllerState)
    right: TeleopControllerState = field(default_factory=TeleopControllerState)
    head: TeleopHeadState = field(default_factory=TeleopHeadState)
    source_time_ns: int | None = None


@dataclass(frozen=True)
class TeleopPollResult:
    """Typed result returned by :meth:`TeleopFrameProvider.poll`."""

    status: TeleopPollStatus
    frame: TeleopFrame | None = None
    message: str = ""


class TeleopFrameProvider(ABC):
    """Lifecycle and polling contract implemented by teleop input sources."""

    @property
    @abstractmethod
    def input_mode(self) -> TeleopInputMode:
        """Return the provider's input mode."""

    @abstractmethod
    def open(self) -> None:
        """Acquire source resources."""

    @abstractmethod
    def poll(self) -> TeleopPollResult:
        """Poll the source once."""

    @abstractmethod
    def close(self) -> None:
        """Release source resources. Implementations must be idempotent."""

    def rewind(self) -> bool:
        """Restart a replay provider from its beginning when supported.

        Returns:
            ``True`` when the provider rewound, otherwise ``False``.
        """
        return False

    def __enter__(self) -> TeleopFrameProvider:
        """Open and return the provider."""
        self.open()
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        """Close the provider."""
        self.close()


def _load_deviceio() -> Any:
    """Load the optional Isaac Teleop session/tracker modules lazily."""
    capabilities = get_teleop_capabilities()
    if not capabilities.live_input:
        raise TeleopInputUnavailableError(capabilities.native_input_unavailable_reason)
    try:
        from isaacteleop.deviceio_session import (
            DeviceIOSession,
            McapRecordingConfig,
            McapReplayConfig,
            ReplaySession,
        )
        from isaacteleop.deviceio_trackers import ControllerTracker, HeadTracker
    except (ImportError, OSError) as exc:
        raise TeleopInputUnavailableError(f"Isaac Teleop native input failed to load: {exc}") from exc

    return SimpleNamespace(
        ControllerTracker=ControllerTracker,
        HeadTracker=HeadTracker,
        DeviceIOSession=DeviceIOSession,
        McapRecordingConfig=McapRecordingConfig,
        McapReplayConfig=McapReplayConfig,
        ReplaySession=ReplaySession,
    )


def _load_oxr() -> Any:
    """Load the optional Isaac Teleop OpenXR facade lazily."""
    capabilities = get_teleop_capabilities()
    if not capabilities.live_input:
        raise TeleopInputUnavailableError(capabilities.native_input_unavailable_reason)
    try:
        import isaacteleop.oxr as oxr
    except (ImportError, OSError) as exc:
        raise TeleopInputUnavailableError(f"Isaac Teleop OpenXR support failed to load: {exc}") from exc

    return oxr


def _get_kit_xr_session_handles(oxr: object) -> object | None:
    """Build Isaac Teleop handles for Kit's active OpenXR session.

    The 2D application returns None so the live provider owns a headless
    Isaac Teleop OpenXR session. An active Kit XR display must share Kit's
    session because this Kit and CloudXR integration supports one active
    OpenXR session per process.

    Args:
        oxr: Loaded Isaac Teleop OpenXR module.

    Returns:
        Shared Kit OpenXR handles, or None when Kit XR is unavailable or inactive.

    Raises:
        TeleopInputUnavailableError: If Kit XR is active but its bridge or handles are unavailable.
    """
    if get_kit_xr_runtime_state() != KitXrRuntimeState.ACTIVE:
        return None

    try:
        bridge = importlib.import_module("isaacsim.kit.xr.teleop.bridge")
        raw_handles = (
            int(bridge.get_instance_handle()),
            int(bridge.get_session_handle()),
            int(bridge.get_stage_space_handle()),
            int(bridge.get_instance_proc_addr()),
        )
    except (AttributeError, ImportError, ModuleNotFoundError, OSError, RuntimeError, TypeError, ValueError) as exc:
        raise TeleopInputUnavailableError(
            "Kit XR is active, but its Isaac Teleop handle bridge is unavailable. "
            "Launch the isaacsim.exp.base.xr.vr.kit experience with isaacsim.kit.xr.teleop.bridge enabled."
        ) from exc

    if not all(raw_handles):
        raise TeleopInputUnavailableError(
            "Kit XR is active, but its OpenXR instance, session, stage-space, or procedure-address handle is not ready. "
            "Wait for the headset session to start, then reconnect Teleop."
        )

    handles_type = getattr(oxr, "OpenXRSessionHandles", None)
    if handles_type is None:
        try:
            handles_type = importlib.import_module("teleopcore.oxr").OpenXRSessionHandles
        except (AttributeError, ImportError, ModuleNotFoundError, OSError) as exc:
            raise TeleopInputUnavailableError(
                "Isaac Teleop does not expose OpenXRSessionHandles required to share Kit's XR session."
            ) from exc
    try:
        return handles_type(*raw_handles)
    except (RuntimeError, TypeError, ValueError) as exc:
        raise TeleopInputUnavailableError(
            "Isaac Teleop rejected the OpenXR handles supplied by Kit XR. "
            "Check that the Isaac Teleop and XR bridge versions are compatible."
        ) from exc


def _tracked_data(tracked: object | None) -> object | None:
    """Return a tracker's payload without manufacturing data for an inactive source."""
    return getattr(tracked, "data", None) if tracked is not None else None


def _tracked_source_time_ns(*tracked_values: object | None) -> int | None:
    """Return the newest comparable device sample time exposed by tracked wrappers."""
    values: list[int] = []
    for tracked in tracked_values:
        timestamp = getattr(tracked, "timestamp", None)
        value = getattr(timestamp, "sample_time_local_common_clock", None)
        if isinstance(value, int) and value > 0:
            values.append(value)
    return max(values) if values else None


def _pose_from_tracked_value(value: object | None) -> TeleopPose:
    """Extract a pose from a controller-pose or head-pose value."""
    if value is None or not bool(getattr(value, "is_valid", False)):
        return TeleopPose()
    pose = getattr(value, "pose", None)
    if pose is None:
        return TeleopPose()
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is None or orientation is None:
        return TeleopPose()
    try:
        return TeleopPose.from_values(
            (float(position.x), float(position.y), float(position.z)),
            (float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)),
        )
    except (AttributeError, TypeError, ValueError):
        return TeleopPose()


def _device_frame(
    *,
    sequence_id: int,
    input_mode: TeleopInputMode,
    left_tracked: object | None,
    right_tracked: object | None,
    head_tracked: object | None,
) -> TeleopFrame:
    """Build one lossless source-space frame from DeviceIO tracker outputs."""
    left_snapshot = _tracked_data(left_tracked)
    right_snapshot = _tracked_data(right_tracked)
    head_snapshot = _tracked_data(head_tracked)
    return TeleopFrame(
        sequence_id=sequence_id,
        monotonic_time_ns=time.monotonic_ns(),
        source_time_ns=_tracked_source_time_ns(left_tracked, right_tracked, head_tracked),
        input_mode=input_mode,
        left=TeleopControllerState(
            snapshot=left_snapshot,
            source_aim_pose=_pose_from_tracked_value(getattr(left_snapshot, "aim_pose", None)),
        ),
        right=TeleopControllerState(
            snapshot=right_snapshot,
            source_aim_pose=_pose_from_tracked_value(getattr(right_snapshot, "aim_pose", None)),
        ),
        head=TeleopHeadState(
            snapshot=head_snapshot,
            source_pose=_pose_from_tracked_value(head_snapshot),
        ),
    )


class LiveTeleopFrameProvider(TeleopFrameProvider):
    """Read controller/head frames from a live OpenXR DeviceIO session."""

    def __init__(
        self,
        *,
        app_name: str = "IsaacSimTeleop",
        mcap_recording_path: str | Path | None = None,
        controller_channel: str = "controllers",
        head_channel: str = "head",
        overwrite: bool = False,
    ) -> None:
        self._app_name = app_name
        self._mcap_recording_path = Path(mcap_recording_path).expanduser() if mcap_recording_path else None
        self._controller_channel = controller_channel
        self._head_channel = head_channel
        self._overwrite = overwrite
        self._stack: contextlib.ExitStack | None = None
        self._session: object | None = None
        self._controller_tracker: object | None = None
        self._head_tracker: object | None = None
        self._sequence_id = 0

    @property
    def input_mode(self) -> TeleopInputMode:
        """Return the live input mode."""
        return TeleopInputMode.LIVE

    @property
    def session(self) -> object | None:
        """Return the active DeviceIO session for output adapters such as haptics."""
        return self._session

    @property
    def controller_tracker(self) -> object | None:
        """Return the active shared controller tracker."""
        return self._controller_tracker

    def open(self) -> None:
        """Create and enter OpenXR and DeviceIO sessions."""
        if self._stack is not None:
            return
        if self._mcap_recording_path is not None and not self._mcap_recording_path.parent.is_dir():
            raise FileNotFoundError(f"MCAP recording directory does not exist: {self._mcap_recording_path.parent}")
        if self._mcap_recording_path is not None and self._mcap_recording_path.exists() and not self._overwrite:
            raise FileExistsError(
                f"MCAP recording file already exists: {self._mcap_recording_path}. Pass overwrite=True to replace it."
            )

        deviceio = _load_deviceio()
        oxr = _load_oxr()
        controller_tracker = deviceio.ControllerTracker()
        head_tracker = deviceio.HeadTracker()
        trackers = [controller_tracker, head_tracker]
        required_extensions = deviceio.DeviceIOSession.get_required_extensions(trackers)

        stack = contextlib.ExitStack()
        try:
            handles = _get_kit_xr_session_handles(oxr)
            if handles is None:
                oxr_session = oxr.OpenXRSession(self._app_name, required_extensions)
                stack.enter_context(oxr_session)
                handles = oxr_session.get_handles()
            recording_config = None
            if self._mcap_recording_path is not None:
                recording_config = deviceio.McapRecordingConfig(
                    str(self._mcap_recording_path),
                    [(controller_tracker, self._controller_channel), (head_tracker, self._head_channel)],
                )
            session = stack.enter_context(deviceio.DeviceIOSession.run(trackers, handles, recording_config))
        except Exception:
            stack.close()
            raise

        self._stack = stack
        self._session = session
        self._controller_tracker = controller_tracker
        self._head_tracker = head_tracker
        self._sequence_id = 0

    def poll(self) -> TeleopPollResult:
        """Update the live session and return one coherent tracker frame."""
        if self._session is None or self._controller_tracker is None or self._head_tracker is None:
            raise RuntimeError("Live teleop input provider is not open")
        # ``DeviceIOSession.update()`` returns ``None``.
        # Tracker payloads, not the return value, carry the current state.
        self._session.update()

        left_tracked = self._controller_tracker.get_left_controller(self._session)
        right_tracked = self._controller_tracker.get_right_controller(self._session)
        head_tracked = self._head_tracker.get_head(self._session)
        self._sequence_id += 1
        return TeleopPollResult(
            TeleopPollStatus.FRAME,
            _device_frame(
                sequence_id=self._sequence_id,
                input_mode=self.input_mode,
                left_tracked=left_tracked,
                right_tracked=right_tracked,
                head_tracked=head_tracked,
            ),
        )

    def close(self) -> None:
        """Close DeviceIO/OpenXR and clear all native handles."""
        stack = self._stack
        self._stack = None
        self._session = None
        self._controller_tracker = None
        self._head_tracker = None
        self._sequence_id = 0
        if stack is not None:
            stack.close()


class McapTeleopFrameProvider(TeleopFrameProvider):
    """Read controller/head frames from an Isaac Teleop MCAP without OpenXR."""

    def __init__(
        self,
        path: str | Path,
        *,
        controller_channel: str = "controllers",
        head_channel: str = "head",
    ) -> None:
        self._path = Path(path).expanduser()
        self._controller_channel = controller_channel
        self._head_channel = head_channel
        self._stack: contextlib.ExitStack | None = None
        self._session: object | None = None
        self._controller_tracker: object | None = None
        self._head_tracker: object | None = None
        self._sequence_id = 0

    @property
    def input_mode(self) -> TeleopInputMode:
        """Return the MCAP replay input mode."""
        return TeleopInputMode.MCAP_REPLAY

    @property
    def path(self) -> Path:
        """Return the configured MCAP path."""
        return self._path

    @property
    def session(self) -> object | None:
        """Return the active replay session."""
        return self._session

    @property
    def controller_tracker(self) -> object | None:
        """Return the active shared controller tracker."""
        return self._controller_tracker

    def open(self) -> None:
        """Open the MCAP replay session headlessly."""
        if self._stack is not None:
            return
        if not self._path.is_file():
            raise FileNotFoundError(f"MCAP replay file does not exist: {self._path}")

        deviceio = _load_deviceio()
        controller_tracker = deviceio.ControllerTracker()
        head_tracker = deviceio.HeadTracker()
        config = deviceio.McapReplayConfig(
            str(self._path),
            [(controller_tracker, self._controller_channel), (head_tracker, self._head_channel)],
        )
        stack = contextlib.ExitStack()
        try:
            session = stack.enter_context(deviceio.ReplaySession.run(config))
        except Exception:
            stack.close()
            raise

        self._stack = stack
        self._session = session
        self._controller_tracker = controller_tracker
        self._head_tracker = head_tracker
        self._sequence_id = 0

    def poll(self) -> TeleopPollResult:
        """Advance replay by one source frame and poll controller/head trackers.

        The native replay API does not expose EOF or a success return. Its
        ``update()`` result is deliberately ignored, and an all-missing tracker
        frame remains a valid frame rather than being interpreted as EOF.
        """
        if self._session is None or self._controller_tracker is None or self._head_tracker is None:
            raise RuntimeError("MCAP teleop input provider is not open")
        self._session.update()
        left_tracked = self._controller_tracker.get_left_controller(self._session)
        right_tracked = self._controller_tracker.get_right_controller(self._session)
        head_tracked = self._head_tracker.get_head(self._session)
        self._sequence_id += 1
        return TeleopPollResult(
            TeleopPollStatus.FRAME,
            _device_frame(
                sequence_id=self._sequence_id,
                input_mode=self.input_mode,
                left_tracked=left_tracked,
                right_tracked=right_tracked,
                head_tracked=head_tracked,
            ),
        )

    def close(self) -> None:
        """Close the replay session."""
        stack = self._stack
        self._stack = None
        self._session = None
        self._controller_tracker = None
        self._head_tracker = None
        self._sequence_id = 0
        if stack is not None:
            stack.close()

    def rewind(self) -> bool:
        """Close and reopen the MCAP so the next poll starts from the beginning."""
        self.close()
        self.open()
        return True


class DebugTeleopFrameProvider(TeleopFrameProvider):
    """Build world-space input frames from draggable debug markers."""

    def __init__(
        self,
        pose_reader: Callable[
            [str],
            tuple[
                tuple[float, float, float] | None,
                tuple[float, float, float, float] | None,
            ],
        ],
        left_snapshot: object,
        right_snapshot: object,
    ) -> None:
        self._pose_reader = pose_reader
        self._left_snapshot = left_snapshot
        self._right_snapshot = right_snapshot
        self._sequence_id = 0
        self._is_open = False

    @property
    def input_mode(self) -> TeleopInputMode:
        """Return the synthetic debug input mode."""
        return TeleopInputMode.DEBUG

    def open(self) -> None:
        """Enable polling."""
        self._is_open = True
        self._sequence_id = 0

    def poll(self) -> TeleopPollResult:
        """Read left/right marker world poses into a typed frame."""
        if not self._is_open:
            raise RuntimeError("Debug teleop input provider is not open")
        left_position, left_orientation = self._pose_reader("left")
        right_position, right_orientation = self._pose_reader("right")
        self._sequence_id += 1
        return TeleopPollResult(
            TeleopPollStatus.FRAME,
            TeleopFrame(
                sequence_id=self._sequence_id,
                monotonic_time_ns=time.monotonic_ns(),
                input_mode=self.input_mode,
                left=TeleopControllerState(
                    snapshot=self._left_snapshot,
                    world_aim_pose=TeleopPose.from_values(left_position, left_orientation),
                ),
                right=TeleopControllerState(
                    snapshot=self._right_snapshot,
                    world_aim_pose=TeleopPose.from_values(right_position, right_orientation),
                ),
            ),
        )

    def close(self) -> None:
        """Disable polling."""
        self._is_open = False
        self._sequence_id = 0


__all__ = [
    "DebugTeleopFrameProvider",
    "LiveTeleopFrameProvider",
    "McapTeleopFrameProvider",
    "TeleopInputUnavailableError",
    "TeleopControllerState",
    "TeleopFrame",
    "TeleopFrameProvider",
    "TeleopHeadState",
    "TeleopInputMode",
    "TeleopPollResult",
    "TeleopPollStatus",
    "TeleopPose",
]

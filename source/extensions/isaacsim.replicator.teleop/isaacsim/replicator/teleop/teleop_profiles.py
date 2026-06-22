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

"""Teleop profile data classes and file I/O utilities."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, get_origin, get_type_hints

from .coordinate_utils import CoordinateSystem
from .markers_manager import MarkersManager
from .xr_anchor_manager import AnchorRotationMode


@dataclass
class TeleopSettingsProfile:
    """Top-level settings shared across teleop controllers."""

    coordinate_system: str = CoordinateSystem.ISAAC_SIM.value
    tracking_space_enabled: bool = False
    tracking_space_path: str = ""
    marker_scale: float = MarkersManager.DEFAULT_FRAME_SCALE
    anchor_x: float = 0.0
    anchor_y: float = 0.0
    anchor_z: float = 0.0
    anchor_rotation_mode: str = AnchorRotationMode.FIXED.value
    anchor_smoothing: float = 1.0
    anchor_fixed_height: bool = True


@dataclass
class ControllerSideProfile:
    """Exact settings plus desired enabled state for one controller side."""

    enabled: bool = False
    settings: dict[str, Any] = field(default_factory=dict)


@dataclass
class BimanualControllerProfile:
    """Left/right controller profile container."""

    left: ControllerSideProfile = field(default_factory=ControllerSideProfile)
    right: ControllerSideProfile = field(default_factory=ControllerSideProfile)


@dataclass
class GraspSideProfile:
    """Exact grasp settings for one side."""

    enabled: bool = False
    prim_path: str = ""
    config_path: str = ""


@dataclass
class GraspControllerProfile:
    """Left/right grasp controller profile container."""

    left: GraspSideProfile = field(default_factory=GraspSideProfile)
    right: GraspSideProfile = field(default_factory=GraspSideProfile)


@dataclass
class LocomotionProfile:
    """Locomotion settings plus desired enabled state."""

    enabled: bool = False
    settings: dict[str, Any] = field(default_factory=dict)


@dataclass
class TeleopProfile:
    """Unified teleop profile.

    The on-disk YAML is an exact mirror of this dataclass hierarchy.
    Adding, removing, or renaming fields here automatically updates
    the file format — no manual parsers or version checks needed.
    """

    session: TeleopSettingsProfile = field(default_factory=TeleopSettingsProfile)
    floating: BimanualControllerProfile = field(default_factory=BimanualControllerProfile)
    ik: BimanualControllerProfile = field(default_factory=BimanualControllerProfile)
    grasp: GraspControllerProfile = field(default_factory=GraspControllerProfile)
    locomotion: LocomotionProfile = field(default_factory=LocomotionProfile)

    def to_dict(self) -> dict[str, Any]:
        """Return a YAML-serializable representation.

        Returns:
            The requested value.
        """
        return asdict(self)


def _from_dict(cls: type, data: Any) -> Any:
    """Reconstruct a dataclass from a plain dict, recursing into nested dataclasses.

    Missing keys use the dataclass defaults.  Extra keys are silently ignored.
    This keeps the YAML format in lockstep with the dataclass definitions.

    Args:
        cls: Dataclass to reconstruct.
        data: Value for data.

    Returns:
        The requested value.
    """
    if not isinstance(data, dict):
        return cls()

    hints = get_type_hints(cls)
    kwargs: dict[str, Any] = {}
    for f in fields(cls):
        if f.name not in data:
            continue
        value = data[f.name]
        resolved_type = hints.get(f.name)
        if resolved_type is not None and is_dataclass(resolved_type):
            kwargs[f.name] = _from_dict(resolved_type, value)
        elif get_origin(resolved_type) is dict and isinstance(value, Mapping):
            kwargs[f.name] = dict(value)
        elif get_origin(resolved_type) is dict:
            continue
        else:
            kwargs[f.name] = value
    return cls(**kwargs)


# -- File I/O --


def get_builtin_teleop_profiles_dir() -> str:
    """Return the absolute path to the built-in teleop profile directory.

    Returns:
        The requested value.
    """
    try:
        import omni.kit.app

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_path = ext_manager.get_extension_path_by_module("isaacsim.replicator.teleop")
        if ext_path:
            return str(Path(ext_path) / "data" / "teleop_profiles")
    except Exception:
        pass
    return ""


def get_last_teleop_profile_path() -> str:
    """Return the extension-managed path for the auto-saved last profile.

    This is ``<builtin data>/teleop_profiles/last_profile.yaml``. The repo ships a
    default YAML at that path; the Teleop window overwrites it when the session state
    is persisted, so treat it as the autosave slot rather than a static preset.

    Returns:
        The requested value.
    """
    profiles_dir = get_builtin_teleop_profiles_dir()
    if not profiles_dir:
        return ""
    return str(Path(profiles_dir) / "last_profile.yaml")


def scan_teleop_profiles(directory: str) -> list[tuple[str, str]]:
    """Return available YAML teleop profiles from a directory.

    Args:
        directory: Value for directory.

    Returns:
        The requested value.
    """
    profiles_dir = Path(directory)
    if not profiles_dir.is_dir():
        return []

    results: list[tuple[str, str]] = []
    for suffix in ("*.yaml", "*.yml"):
        for path in sorted(profiles_dir.glob(suffix)):
            results.append((path.stem, str(path)))
    return results


def save_teleop_profile(path: str, profile: TeleopProfile) -> tuple[bool, str]:
    """Write a unified teleop profile to YAML.

    Args:
        path: Value for path.
        profile: Value for profile.

    Returns:
        The requested value.
    """
    import yaml

    try:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", encoding="utf-8") as file_obj:
            yaml.safe_dump(profile.to_dict(), file_obj, default_flow_style=False, sort_keys=False)
        return True, f"Saved to {target.name}"
    except Exception as exc:
        return False, f"Save failed: {exc}"


def load_teleop_profile(path: str) -> tuple[TeleopProfile | None, list[str]]:
    """Read a unified teleop profile from YAML.

    The file structure must be a YAML mapping whose keys correspond to
    ``TeleopProfile`` fields.  Unknown keys are ignored and missing
    keys fall back to the dataclass defaults, so this function stays
    compatible with both older and newer profile files automatically.

    Args:
        path: Value for path.

    Returns:
        The requested value.
    """
    import yaml

    try:
        with Path(path).open("r", encoding="utf-8") as file_obj:
            data = yaml.safe_load(file_obj)
    except Exception as exc:
        return None, [f"Failed to read {path}: {exc}"]

    if not isinstance(data, dict):
        return None, [f"Expected a YAML mapping in {path}"]

    return _from_dict(TeleopProfile, data), []

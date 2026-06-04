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

"""Grasp controller for VR teleop.

Maps VR controller analog input (trigger) to drive joint targets using
configurable per-joint mappings.  Supports YAML-based grasp configurations
for multi-finger hands with sequenced joint activation.

When the gripper is part of a larger articulation (e.g. assembled onto a
robot arm via Robot Assembler), the controller automatically discovers the
articulation root and uses the tensor-backed Articulation API to set drive
targets.  This avoids conflicts with other controllers (IK, etc.) that use
the same Articulation backend for the same robot.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

from .base import find_owning_articulation_root

# ── Data types ───────────────────────────────────────────────────────

BUILTIN_GRASP_CONFIG_SCHEME = "builtin://"


@dataclass
class JointMapping:
    """Maps a portion of trigger input [0,1] to a joint target range.

    Attributes:
        name: Joint prim name to match in the gripper/hand hierarchy.
        input_range: Sub-range of [0,1] trigger input that activates this joint.
            Joints with higher start values begin moving later in the squeeze.
        target_range: Joint position [open, closed] mapped from input_range.
    """

    name: str
    input_range: tuple[float, float] = (0.0, 1.0)
    target_range: tuple[float, float] = (0.0, 1.0)

    def compute_target(self, input_value: float) -> float:
        """Compute drive target for a given trigger input value.

        Below input_range[0] returns target_range[0].
        Above input_range[1] returns target_range[1].
        Within range, linearly interpolates.
        """
        input_value = max(0.0, min(1.0, input_value))
        lo, hi = self.input_range
        if hi <= lo:
            return self.target_range[1]
        if input_value <= lo:
            return self.target_range[0]
        if input_value >= hi:
            return self.target_range[1]
        t = (input_value - lo) / (hi - lo)
        return self.target_range[0] + t * (self.target_range[1] - self.target_range[0])


@dataclass
class GraspConfig:
    """Grasp configuration - loaded from YAML or constructed programmatically.

    Attributes:
        name: Human-readable name for display in UI.
        description: Optional description of the grasp behaviour.
        joints: Per-joint mappings from trigger input to drive targets.
    """

    name: str = ""
    description: str = ""
    joints: list[JointMapping] = field(default_factory=list)


@dataclass
class GraspValidationResult:
    """Result of grasp prim validation."""

    is_valid: bool = False
    total_joints: int = 0
    drive_joints: int = 0
    mimic_joints: int = 0
    controllable_joints: int = 0
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    drive_joint_paths: list[str] = field(default_factory=list)


# ── Per-side runtime state ───────────────────────────────────────────


@dataclass
class _GraspState:
    prim_path: str | None = None
    config: GraspConfig | None = None
    # joint USD path -> (mapping, cached DriveAPI) -- USD fallback path
    active_joints: dict[str, tuple[JointMapping, UsdPhysics.DriveAPI]] = field(default_factory=dict)
    input_value: float = 0.0
    # Articulation-backed path (used when gripper is part of a larger articulation)
    articulation: Articulation | None = None
    # joint USD path -> (mapping, DOF index in articulation)
    art_joint_map: dict[str, tuple[JointMapping, int]] = field(default_factory=dict)


# ── YAML loading ─────────────────────────────────────────────────────


def _get_builtin_grasp_configs_dir() -> Path | None:
    try:
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_path = ext_manager.get_extension_path_by_module("isaacsim.replicator.teleop")
        if not ext_path:
            return None
        configs_dir = Path(ext_path) / "data" / "grasp_configs"
        if not configs_dir.is_dir():
            return None
        return configs_dir
    except Exception:
        return None


def get_builtin_grasp_config_uri(name: str) -> str:
    """Return the symbolic URI for a built-in grasp config name."""
    return f"{BUILTIN_GRASP_CONFIG_SCHEME}{name.strip()}"


def normalize_grasp_config_path(path: str) -> str:
    """Normalize grasp config references to a portable built-in URI when possible."""
    raw_path = path.strip()
    if not raw_path:
        return ""

    if raw_path.startswith(BUILTIN_GRASP_CONFIG_SCHEME):
        builtin_name = raw_path[len(BUILTIN_GRASP_CONFIG_SCHEME) :].strip()
        return get_builtin_grasp_config_uri(builtin_name) if builtin_name else ""

    candidate = Path(raw_path)
    if candidate.suffix.lower() in (".yaml", ".yml"):
        config_name = candidate.stem
        parent_name = candidate.parent.name.lower()
        if parent_name == "grasp_configs":
            builtin_dir = _get_builtin_grasp_configs_dir()
            if builtin_dir is not None:
                for suffix in (".yaml", ".yml"):
                    if (builtin_dir / f"{config_name}{suffix}").is_file():
                        return get_builtin_grasp_config_uri(config_name)

    return raw_path


def resolve_grasp_config_path(path: str) -> str:
    """Resolve a grasp config reference to a filesystem path when possible."""
    normalized = normalize_grasp_config_path(path)
    if not normalized:
        return ""

    if not normalized.startswith(BUILTIN_GRASP_CONFIG_SCHEME):
        return normalized

    builtin_name = normalized[len(BUILTIN_GRASP_CONFIG_SCHEME) :].strip()
    if not builtin_name:
        return ""

    builtin_dir = _get_builtin_grasp_configs_dir()
    if builtin_dir is None:
        return ""

    for suffix in (".yaml", ".yml"):
        candidate = builtin_dir / f"{builtin_name}{suffix}"
        if candidate.is_file():
            return str(candidate)
    return ""


def load_grasp_config(path: str) -> tuple[GraspConfig | None, list[str]]:
    """Load a grasp configuration from a YAML file.

    Args:
        path: Filesystem path to the YAML file.

    Returns:
        (config, errors) - config is None if loading failed.
    """
    import yaml  # noqa: delayed import - only needed when user loads a config

    errors: list[str] = []
    normalized_path = normalize_grasp_config_path(path)
    resolved_path = resolve_grasp_config_path(normalized_path)
    if normalized_path.startswith(BUILTIN_GRASP_CONFIG_SCHEME):
        if not resolved_path:
            return None, [f"Built-in grasp config not found: '{normalized_path}'"]
        p = Path(resolved_path)
    else:
        p = Path(resolved_path or normalized_path)

    if not p.exists():
        return None, [f"File not found: '{normalized_path or path}'"]
    if p.suffix.lower() not in (".yaml", ".yml"):
        return None, [f"Expected .yaml or .yml file: '{normalized_path or path}'"]

    try:
        with open(p) as f:
            data = yaml.safe_load(f)
    except Exception as e:
        return None, [f"Failed to parse YAML: {e}"]

    if not isinstance(data, dict):
        return None, ["YAML root must be a mapping"]

    joints_data = data.get("joints", [])
    if not isinstance(joints_data, list) or not joints_data:
        return None, ["'joints' must be a non-empty list"]

    config = GraspConfig(name=data.get("name", p.stem), description=data.get("description", ""))

    for i, jd in enumerate(joints_data):
        if not isinstance(jd, dict):
            errors.append(f"Entry {i}: must be a mapping")
            continue
        name = jd.get("name", "")
        if not name:
            errors.append(f"Entry {i}: missing 'name'")
            continue
        ir = jd.get("input_range", [0.0, 1.0])
        tr = jd.get("target_range", [0.0, 1.0])
        if not (isinstance(ir, (list, tuple)) and len(ir) == 2):
            errors.append(f"Joint '{name}': input_range must be [start, end]")
            continue
        if not (isinstance(tr, (list, tuple)) and len(tr) == 2):
            errors.append(f"Joint '{name}': target_range must be [start, end]")
            continue
        config.joints.append(
            JointMapping(
                name=name,
                input_range=(float(ir[0]), float(ir[1])),
                target_range=(float(tr[0]), float(tr[1])),
            )
        )

    if errors:
        return None, errors

    return config, []


def get_builtin_grasp_configs() -> list[tuple[str, str]]:
    """Return (display_name, portable_config_path) pairs for built-in grasp configs.

    Scans the extension's ``data/grasp_configs/`` directory.
    """
    configs_dir = _get_builtin_grasp_configs_dir()
    if configs_dir is None:
        return []

    result: list[tuple[str, str]] = []
    for suffix in ("*.yaml", "*.yml"):
        for p in sorted(configs_dir.glob(suffix)):
            display = p.stem
            result.append((display, get_builtin_grasp_config_uri(display)))
    return result


# ── Controller ───────────────────────────────────────────────────────


class GraspController:
    """Controls grasping via VR trigger input.

    Maps VR controller analog input (trigger) to drive joint targets
    using YAML-based per-joint mappings with custom input sub-ranges
    for sequenced multi-finger grasps.  Each side (left/right) has its
    own prim path and config, supporting different end effectors.
    """

    def __init__(self) -> None:
        self._sides: dict[str, _GraspState] = {
            "left": _GraspState(),
            "right": _GraspState(),
        }
        self._enabled = False
        self._tracking_enabled: dict[str, bool] = {"left": False, "right": False}

    def _side(self, side: str) -> _GraspState:
        return self._sides[side.lower()]

    # ── Validation ───────────────────────────────────────────────────

    def validate_prim(self, prim_path: str) -> GraspValidationResult:
        """Validate whether a prim has controllable drive joints.

        A valid prim must have at least one joint with DriveAPI that is not
        a mimic joint.

        Args:
            prim_path: USD path to the gripper/hand root prim.

        Returns:
            GraspValidationResult with details.
        """
        result = GraspValidationResult()

        stage = omni.usd.get_context().get_stage()
        if not stage:
            result.errors.append("Stage not available")
            return result

        if not prim_path or not Sdf.Path.IsValidPathString(prim_path):
            result.errors.append(f"Invalid path: '{prim_path}'")
            return result

        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            result.errors.append(f"Prim not found: '{prim_path}'")
            return result

        for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if p.IsA(UsdPhysics.Joint):
                result.total_joints += 1
                path = str(p.GetPath())
                is_mimic = p.HasAPI(PhysxSchema.PhysxMimicJointAPI)
                has_drive = p.HasAPI(UsdPhysics.DriveAPI, "angular") or p.HasAPI(UsdPhysics.DriveAPI, "linear")
                if is_mimic:
                    result.mimic_joints += 1
                if has_drive:
                    result.drive_joints += 1
                if has_drive and not is_mimic:
                    result.controllable_joints += 1
                    result.drive_joint_paths.append(path)

        if result.controllable_joints == 0:
            result.errors.append("No controllable joints (need DriveAPI without MimicJointAPI)")
        else:
            result.is_valid = True

        if result.total_joints == 0:
            result.warnings.append("No joints found in hierarchy")

        return result

    # ── Configure ─────────────────────────────────────────────────────

    def configure(self, prim_path: str, side: str, config: GraspConfig) -> bool:
        """Configure grasp control for a side.

        Matches YAML joint names in the config to USD drive joints under
        the given prim path.

        Args:
            prim_path: USD path to the gripper/hand root prim.
            side: "left" or "right".
            config: Grasp configuration (loaded from YAML).

        Returns:
            True if configuration succeeded.
        """
        if not prim_path:
            print(f"[Teleop][Grasp] Cannot configure {side}: empty path")
            return False

        result = self.validate_prim(prim_path)
        if not result.is_valid:
            print(f"[Teleop][Grasp] Validation failed for '{prim_path}': {result.errors}")
            return False

        state = self._side(side)
        state.prim_path = prim_path
        state.config = config
        state.articulation = None
        state.art_joint_map = {}

        state.active_joints = self._match_config_joints(config, result.drive_joint_paths)

        # Try to bind to the owning Articulation so drive targets go through
        # the tensor backend (required when an IK controller or similar is
        # managing the same articulation). Any failure here is only reported
        # below if the DriveAPI fallback is also unusable.
        art_bind_error: str | None = None
        art_root = find_owning_articulation_root(prim_path)
        if art_root:
            art_bind_error = self._bind_articulation(state, art_root)

        if not state.active_joints and not state.art_joint_map:
            if art_bind_error:
                print(f"[Teleop][Grasp] {art_bind_error}")
            print(f"[Teleop][Grasp] No joints matched for {side}")
            return False

        self._tracking_enabled[side.lower()] = False
        self._enabled = True
        matched = len(state.art_joint_map or state.active_joints)
        mode = "articulation" if state.articulation else "DriveAPI"
        print(f"[Teleop][Grasp] Configured {side}: '{prim_path}' ({matched} joint(s), {mode})")
        return True

    def _bind_articulation(self, state: _GraspState, art_root_path: str) -> str | None:
        """Bind matched joints to an Articulation's DOF indices.

        Uses ``dof_names`` (from the physics tensor) for matching rather
        than ``dof_paths`` (from USD), because assembled robots can have
        DOF type mismatches that make USD-derived paths unreliable.

        Returns an error message describing why binding did not happen, or
        ``None`` on success or when the articulation simply has no matching
        DOFs. Callers decide whether to surface the message based on whether
        the DriveAPI fallback is viable.
        """
        try:
            robot = Articulation(art_root_path)
            dof_names_attr = robot.dof_names
        except Exception as exc:
            return f"Could not create Articulation at '{art_root_path}': {exc}"

        if not dof_names_attr:
            return f"Articulation at '{art_root_path}' exposes no DOFs yet"

        dof_names = list(dof_names_attr)
        dof_name_to_idx: dict[str, int] = {name: idx for idx, name in enumerate(dof_names)}

        art_map: dict[str, tuple[JointMapping, int]] = {}
        for jp, (mapping, _drive_api) in state.active_joints.items():
            joint_name = Sdf.Path(jp).name
            dof_idx = dof_name_to_idx.get(joint_name)
            if dof_idx is not None:
                art_map[jp] = (mapping, dof_idx)
                print(f"[Teleop][Grasp] Mapped '{joint_name}' -> DOF index {dof_idx}")
            else:
                print(f"[Teleop][Grasp] Joint '{joint_name}' not found in articulation DOFs ({dof_names})")

        if art_map:
            state.articulation = robot
            state.art_joint_map = art_map
        return None

    def _match_config_joints(
        self,
        config: GraspConfig,
        drive_joint_paths: list[str],
    ) -> dict[str, tuple[JointMapping, UsdPhysics.DriveAPI]]:
        """Matches config joint names to USD joints and caches DriveAPIs."""
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return {}

        name_to_path: dict[str, str] = {}
        for jp in drive_joint_paths:
            name_to_path[Sdf.Path(jp).name] = jp

        active: dict[str, tuple[JointMapping, UsdPhysics.DriveAPI]] = {}
        for mapping in config.joints:
            jp = name_to_path.get(mapping.name)
            if jp is None:
                print(f"[Teleop][Grasp] Warning: config joint '{mapping.name}' not found in prim hierarchy")
                continue

            prim = stage.GetPrimAtPath(jp)
            if not prim or not prim.IsValid():
                continue

            if prim.IsA(UsdPhysics.RevoluteJoint):
                drive_type = "angular"
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                drive_type = "linear"
            else:
                continue

            drive_api = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if drive_api:
                active[jp] = (mapping, drive_api)

        return active

    # ── Runtime ──────────────────────────────────────────────────────

    def set_input(self, side: str, input_value: float) -> None:
        """Set trigger input for a side and applies to joints.

        Args:
            side: "left" or "right".
            input_value: Trigger value (0=open, 1=closed).
        """
        if not self.is_side_tracking_enabled(side):
            return
        state = self._side(side)
        state.input_value = max(0.0, min(1.0, input_value))
        self._apply_input(state)

    def _apply_input(self, state: _GraspState) -> None:
        """Apply current input to all matched joints.

        Uses the Articulation tensor API when available (required for
        assembled robots where another controller owns the articulation).
        Falls back to direct DriveAPI writes for standalone grippers.
        """
        if state.articulation and state.art_joint_map:
            self._apply_via_articulation(state)
        else:
            self._apply_via_drive_api(state)

    def _apply_via_articulation(self, state: _GraspState) -> None:
        """Set drive targets through the Articulation tensor API."""
        robot = state.articulation
        if robot is None:
            return
        try:
            indices: list[int] = []
            targets: list[float] = []
            for _jp, (mapping, dof_idx) in state.art_joint_map.items():
                indices.append(dof_idx)
                targets.append(mapping.compute_target(state.input_value))
            robot.set_dof_position_targets(
                np.array([targets], dtype=np.float32),
                dof_indices=indices,
            )
        except (AssertionError, RuntimeError):
            # Tensor not ready (simulation not playing) -- fall back to USD
            self._apply_via_drive_api(state)

    def _apply_via_drive_api(self, state: _GraspState) -> None:
        """Set drive targets directly on USD DriveAPI attributes."""
        for _jp, (mapping, drive_api) in state.active_joints.items():
            target = mapping.compute_target(state.input_value)
            target_attr = drive_api.GetTargetPositionAttr()
            if target_attr:
                target_attr.Set(target)

    def remove(self, side: str) -> None:
        """Clear grasp configuration for one side."""
        side = side.lower()
        state = self._sides.get(side)
        if state is None:
            return
        state.prim_path = None
        state.config = None
        state.active_joints.clear()
        state.art_joint_map.clear()
        state.articulation = None
        state.input_value = 0.0
        self._tracking_enabled[side] = False
        if not any(s.active_joints for s in self._sides.values()):
            self._enabled = False

    def remove_all(self) -> None:
        """Clear all grasp configurations for both sides."""
        for side in list(self._sides):
            self.remove(side)
        print("[Teleop][Grasp] Controllers removed.")

    def set_side_tracking_enabled(self, side: str, enabled: bool) -> None:
        """Enable/disable trigger tracking for one side."""
        side = side.lower()
        if side not in self._tracking_enabled:
            return
        self._tracking_enabled[side] = bool(enabled)

    def is_side_tracking_enabled(self, side: str) -> bool:
        """Return True if trigger tracking is enabled for one side."""
        side = side.lower()
        state = self._sides.get(side)
        if state is None:
            return False
        return bool(self._tracking_enabled.get(side, False) and state.active_joints)

    @property
    def has_any_side_tracking_enabled(self) -> bool:
        """True when at least one side accepts trigger tracking."""
        return self.is_side_tracking_enabled("left") or self.is_side_tracking_enabled("right")

    # ── Properties ───────────────────────────────────────────────────

    @property
    def is_enabled(self) -> bool:
        """True if any side has active joints."""
        return self._enabled and any(s.active_joints for s in self._sides.values())

    @property
    def left_prim_path(self) -> str | None:
        """Return the left side prim path."""
        return self._side("left").prim_path

    @property
    def right_prim_path(self) -> str | None:
        """Return the right side prim path."""
        return self._side("right").prim_path

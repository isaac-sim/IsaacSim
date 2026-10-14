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

"""Config-driven floating-gripper helpers for scripted teleop validation."""

from __future__ import annotations

import copy
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

from ._asset_paths import (
    _resolve_definition_grasp_config_path,
    _resolve_definition_relative_asset_path,
    resolve_floating_gripper_asset_path_async,
)
from .controllers import FloatingRigidBodyController, GraspController, load_grasp_config
from .coordinate_utils import CoordinateSystem
from .markers_manager import MarkersManager
from .scripted_motion import (
    MotionResult,
    PoseMap,
    make_pose,
    move_debug_markers_to_world_targets_async,
    set_debug_grasp_async,
    set_debug_markers_world_poses,
    wait_for_controllers_running_async,
)
from .teleop_manager import TeleopManager
from .visual_cues_manager import VisualCuesManager

_REQUIRED_GRIPPER_FIELDS = (
    "name",
    "asset_path",
    "root_xform_name",
    "rigid_root_name",
    "gripper_prim_name",
    "base_link_name",
    "tcp",
    "floating_controller",
    "grasp_controller",
)


def _get_floating_grippers_dir() -> Path:
    try:
        ext_path = app_utils.get_extension_path("isaacsim.replicator.teleop")
        if ext_path:
            candidate = Path(ext_path) / "data" / "floating_grippers"
            if candidate.is_dir():
                return candidate
    except Exception:
        pass

    candidate = Path(__file__).resolve().parents[3] / "data" / "floating_grippers"
    if candidate.is_dir():
        return candidate
    raise RuntimeError("Built-in floating-gripper definitions directory was not found")


def _load_floating_gripper_definitions() -> dict[str, dict[str, Any]]:
    import yaml

    definitions: dict[str, dict[str, Any]] = {}
    definitions_dir = _get_floating_grippers_dir()
    for path in sorted((*definitions_dir.glob("*.yaml"), *definitions_dir.glob("*.yml"))):
        try:
            with path.open(encoding="utf-8") as file_obj:
                data = yaml.safe_load(file_obj)
        except Exception as exc:
            raise RuntimeError(f"Could not read floating-gripper definition '{path}': {exc}") from exc
        if not isinstance(data, dict):
            raise RuntimeError(f"Floating-gripper definition '{path}' must contain a YAML mapping")
        missing = [field for field in _REQUIRED_GRIPPER_FIELDS if field not in data]
        if missing:
            raise RuntimeError(f"Floating-gripper definition '{path}' is missing: {', '.join(missing)}")

        name = str(data["name"]).strip().lower()
        if not name:
            raise RuntimeError(f"Floating-gripper definition '{path}' has an empty name")
        if name in definitions:
            raise RuntimeError(f"Duplicate floating-gripper definition '{name}'")

        floating = data["floating_controller"]
        grasp = data["grasp_controller"]
        if not isinstance(floating, dict) or not isinstance(grasp, dict):
            raise RuntimeError(f"Controller settings in '{path}' must be YAML mappings")
        rotation_offset = floating.get("rotation_offset_degrees")
        if not isinstance(rotation_offset, list) or len(rotation_offset) != 3:
            raise RuntimeError(f"'{path}' must define three floating-controller rotation offsets")
        tcp = data["tcp"]
        if not isinstance(tcp, dict):
            raise RuntimeError(f"'{path}' tcp must be a YAML mapping")
        tcp_parent_link = str(tcp.get("parent_link", "")).strip()
        tcp_prim_name = str(tcp.get("prim_name", "")).strip()
        tcp_translation = tcp.get("translation")
        if not tcp_parent_link or not tcp_prim_name:
            raise RuntimeError(f"'{path}' tcp must define parent_link and prim_name")
        if not isinstance(tcp_translation, list) or len(tcp_translation) != 3:
            raise RuntimeError(f"'{path}' tcp must define a three-value translation")
        if not str(grasp.get("config_path", "")).strip():
            raise RuntimeError(f"'{path}' must define grasp_controller.config_path")

        definitions[name] = {
            "name": name,
            "asset_path": _resolve_definition_relative_asset_path(str(data["asset_path"]), path.parent)[0],
            "root_xform_name": str(data["root_xform_name"]),
            "rigid_root_name": str(data["rigid_root_name"]),
            "gripper_prim_name": str(data["gripper_prim_name"]),
            "base_link_name": str(data["base_link_name"]),
            "root_joint_name": str(data.get("root_joint_name", "root_joint")),
            "tcp": {
                "parent_link": tcp_parent_link,
                "prim_name": tcp_prim_name,
                "translation": tuple(float(value) for value in tcp_translation),
            },
            "floating_controller": {
                "position_kp": float(floating.get("position_kp", 20.0)),
                "position_kd": float(floating.get("position_kd", 0.5)),
                "orientation_kp": float(floating.get("orientation_kp", 20.0)),
                "orientation_kd": float(floating.get("orientation_kd", 0.2)),
                "rotation_offset_degrees": tuple(float(value) for value in rotation_offset),
            },
            "grasp_controller": {
                "config_path": _resolve_definition_grasp_config_path(str(grasp["config_path"]), path.parent),
            },
        }
    if not definitions:
        raise RuntimeError("No built-in floating-gripper definitions were found")
    return definitions


def get_supported_floating_grippers() -> tuple[str, ...]:
    """Return names of all config-defined floating grippers."""
    return tuple(sorted(_load_floating_gripper_definitions()))


def get_floating_gripper_spec(name: str) -> dict[str, Any]:
    """Return one config-defined floating-gripper specification.

    Args:
        name: Stable gripper name.

    Returns:
        Independent copy of the matching YAML definition.
    """
    definitions = _load_floating_gripper_definitions()
    normalized = name.strip().lower()
    if normalized not in definitions:
        supported = ", ".join(sorted(definitions))
        raise ValueError(f"Unsupported floating gripper '{name}'. Supported grippers: {supported}")
    return copy.deepcopy(definitions[normalized])


def get_floating_gripper_paths(teleop_root: str, side: str, gripper: str | Mapping[str, Any]) -> dict[str, str]:
    """Resolve instance prim paths from a config-defined gripper specification."""
    normalized_side = side.strip().lower()
    if normalized_side not in ("left", "right"):
        raise ValueError(f"Unsupported controller side '{side}'")
    spec = get_floating_gripper_spec(gripper) if isinstance(gripper, str) else dict(gripper)
    root_xform = f"{teleop_root}/gripper_origin_xform/{normalized_side}_{spec['root_xform_name']}"
    gripper_path = f"{root_xform}/{spec['gripper_prim_name']}"
    tcp = spec["tcp"]
    return {
        "root_xform": root_xform,
        "rigid_root": f"{root_xform}/{spec['rigid_root_name']}",
        "gripper": gripper_path,
        "root_joint": f"{gripper_path}/{spec['root_joint_name']}",
        "base_link": f"{gripper_path}/{spec['base_link_name']}",
        "tcp": f"{gripper_path}/{tcp['parent_link']}/{tcp['prim_name']}",
    }


def _create_tcp_xform(path: str, translation: tuple[float, float, float]) -> None:
    """Create the configured asset-local TCP frame beneath its parent link."""
    from isaacsim.core.experimental.prims import XformPrim

    stage_utils.define_prim(path, "Xform")
    XformPrim(path, reset_xform_op_properties=True).set_local_poses(translations=np.asarray([translation]))


def _create_floating_rigid_root(path: str) -> None:
    from isaacsim.core.experimental.objects import Cube
    from isaacsim.core.experimental.prims import GeomPrim

    Cube(path, sizes=1.0, scales=(0.01, 0.01, 0.01))
    geometry = GeomPrim(path, apply_collision_apis=True)
    geometry.set_enabled_collisions([False])
    RigidPrim(path)
    stage = stage_utils.get_current_stage()
    prim = stage.GetPrimAtPath(path)
    prim_utils.ensure_api(prim, UsdPhysics.ArticulationRootAPI)
    prim_utils.ensure_api(prim, PhysxSchema.PhysxArticulationAPI)
    Articulation(path).set_enabled_self_collisions(False)


def _attach_to_floating_root(stage: Usd.Stage, paths: Mapping[str, str]) -> None:
    """Retarget an asset's authored root joint to the floating rigid body."""
    root_joint_prim = stage.GetPrimAtPath(paths["root_joint"])
    if not root_joint_prim or not root_joint_prim.IsValid() or not root_joint_prim.IsA(UsdPhysics.FixedJoint):
        raise RuntimeError(f"Expected an authored fixed root joint at {paths['root_joint']}")

    joint = UsdPhysics.FixedJoint(root_joint_prim)
    if not joint.GetBody1Rel().GetTargets():
        joint.CreateBody1Rel().SetTargets([Sdf.Path(paths["base_link"])])
    joint.GetBody0Rel().SetTargets([Sdf.Path(paths["rigid_root"])])


def _disable_gravity(stage: Usd.Stage, root_path: str) -> None:
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return
    rigid_bodies = prim_utils.get_all_matching_child_prims(
        root,
        predicate=lambda prim, _: prim_utils.has_api(prim, UsdPhysics.RigidBodyAPI),
        include_self=True,
    )
    if rigid_bodies:
        RigidPrim([str(prim.GetPath()) for prim in rigid_bodies]).set_enabled_gravities(False)


async def load_floating_grippers_async(
    assignments: Mapping[str, str],
    spawn_poses: PoseMap,
    *,
    teleop_root: str = "/World/Teleop",
    disable_gravity: bool = True,
) -> dict[str, dict[str, str]]:
    """Load config-defined grippers and attach them to floating rigid roots."""
    import isaacsim.core.experimental.utils.app as app_utils
    from isaacsim.core.experimental.prims import XformPrim

    if set(assignments) != set(spawn_poses):
        raise ValueError("Assignment and spawn-pose sides must match")
    if not assignments:
        raise ValueError("At least one gripper assignment is required")

    # Resolve all assets before modifying the stage; local-only loads need no assets server.
    specs = {side: get_floating_gripper_spec(name) for side, name in assignments.items()}
    asset_paths = {
        side: await resolve_floating_gripper_asset_path_async(spec["asset_path"]) for side, spec in specs.items()
    }

    stage_utils.define_prim(teleop_root, "Xform")
    origins_path = f"{teleop_root}/gripper_origin_xform"
    stage_utils.define_prim(origins_path, "Xform")
    XformPrim(origins_path, reset_xform_op_properties=True).set_world_poses(
        positions=np.zeros((1, 3), dtype=np.float64)
    )

    resolved: dict[str, dict[str, str]] = {}
    for side, spec in specs.items():
        paths = get_floating_gripper_paths(teleop_root, side, spec)
        position, _orientation = make_pose(*spawn_poses[side])
        stage_utils.define_prim(paths["root_xform"], "Xform")
        # The floating controller caches the controlled body's authored world
        # orientation as an asset offset. Keep scripted wrapper roots at identity
        # so the marker orientation and YAML tool-frame offset are each applied once.
        XformPrim(paths["root_xform"], reset_xform_op_properties=True).set_world_poses(
            positions=np.asarray([position], dtype=np.float64),
            orientations=np.asarray([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
        )
        _create_floating_rigid_root(paths["rigid_root"])
        stage_utils.add_reference_to_stage(asset_paths[side], paths["gripper"])
        resolved[side] = paths

    for _ in range(300):
        await app_utils.update_app_async(steps=1)
        if not stage_utils.is_stage_loading():
            break
    if stage_utils.is_stage_loading():
        raise RuntimeError("Timed out while loading floating-gripper assets")

    stage = stage_utils.get_current_stage()
    for side, paths in resolved.items():
        spec = specs[side]
        _attach_to_floating_root(stage, paths)
        _create_tcp_xform(paths["tcp"], spec["tcp"]["translation"])
        if disable_gravity:
            _disable_gravity(stage, paths["root_xform"])
    return resolved


async def setup_floating_gripper_controllers_async(
    assignments: Mapping[str, str],
    paths: Mapping[str, Mapping[str, str]],
    initial_poses: PoseMap,
    *,
    marker_scale: float = 0.05,
    visual_cues: bool = True,
    visual_cue_reference_z: float = 0.0,
) -> dict[str, Any]:
    """Configure controllers and return an opaque context for helper functions."""
    import isaacsim.core.experimental.utils.app as app_utils

    if set(assignments) != set(paths) or set(initial_poses) != set(assignments):
        raise ValueError("Assignment, path, and initial-pose sides must match")

    active_sides = tuple(assignments)
    context: dict[str, Any] = {
        "assignments": dict(assignments),
        "paths": {side: dict(value) for side, value in paths.items()},
        "active_sides": active_sides,
        "markers": MarkersManager(),
        "floating": FloatingRigidBodyController(),
        "grasp": GraspController(),
        "manager": TeleopManager(),
        "visual_cues": VisualCuesManager() if visual_cues else None,
        "handles": {},
        "grasp_sides": (),
    }

    context["manager"].set_markers_manager(context["markers"])
    context["manager"].set_floating_controller(context["floating"])
    context["manager"].set_grasp_controller(context["grasp"])
    context["manager"].set_coordinate_system(CoordinateSystem.ISAAC_SIM)
    context["manager"].disable_tracking_space()
    context["markers"].set_frame_scale(marker_scale)
    for marker_name in ("origin", *active_sides):
        ok, message = context["markers"].ensure_marker(marker_name)
        if not ok:
            raise RuntimeError(f"Could not create marker '{marker_name}': {message}")
    context["manager"].set_debug_tracking(True)
    for side in ("left", "right"):
        context["manager"].set_floating_side_assigned(side, side in active_sides)

    for side in active_sides:
        settings = get_floating_gripper_spec(assignments[side])["floating_controller"]
        context["floating"].set_prim_path(side, paths[side]["rigid_root"])
        context["floating"].set_gains(
            settings["position_kp"],
            settings["position_kd"],
            settings["orientation_kp"],
            settings["orientation_kd"],
            side=side,
        )
        context["floating"].set_target_rotation_offsets(side, *settings["rotation_offset_degrees"])
        valid, message = context["floating"].validate(side)
        if not valid or not context["floating"].configure(side):
            raise RuntimeError(f"Could not configure {side} floating controller: {message}")
        context["handles"][side] = RigidPrim(paths[side]["rigid_root"])

    set_debug_markers_world_poses(context["markers"], initial_poses)

    app_utils.play()
    context["manager"].set_floating_tracking(True)
    await app_utils.update_app_async(steps=1)
    controllers = dict.fromkeys(active_sides, context["floating"])
    if not await wait_for_controllers_running_async(
        controllers,
        app_utils.update_app_async,
        max_steps=29,
    ):
        raise RuntimeError("Floating controllers did not start")

    grasp_sides = []
    for side in active_sides:
        settings = get_floating_gripper_spec(assignments[side])["grasp_controller"]
        config, errors = load_grasp_config(settings["config_path"])
        if config is None or errors:
            raise RuntimeError(f"Could not load {side} grasp config: {errors}")
        if not context["grasp"].configure(paths[side]["gripper"], side, config):
            raise RuntimeError(f"Could not configure {side} grasp controller")
        context["grasp"].set_side_tracking_enabled(side, True)
        context["manager"].set_debug_trigger(side, 0.0)
        # The debug snapshot is initialized to zero, so assigning zero again
        # does not create an input transition. Apply the open target explicitly
        # before the scripted approach begins.
        context["grasp"].set_input(side, 0.0)
        grasp_sides.append(side)
    context["grasp_sides"] = tuple(grasp_sides)
    context["manager"].set_grasp_tracking(bool(grasp_sides))

    if context["visual_cues"] is not None:
        context["visual_cues"].set_reference_z(visual_cue_reference_z)
        for side in active_sides:
            context["visual_cues"].set_override_prim_path(side, paths[side]["tcp"])
            ok, message = context["visual_cues"].show_side(side)
            if not ok:
                raise RuntimeError(f"Could not create {side} visual cue: {message}")
    return context


async def move_floating_grippers_to_targets_async(
    context: dict[str, Any],
    targets: PoseMap,
    *,
    sample_count: int = 100,
    position_tolerance: float = 0.01,
    orientation_tolerance: float = np.pi,
    max_steps_per_target: int = 30,
) -> MotionResult:
    """Execute one tolerance-gated scripted move for configured grippers."""
    import isaacsim.core.experimental.utils.app as app_utils

    async def update_async() -> None:
        await app_utils.update_app_async(steps=1)

    if set(targets) != set(context["active_sides"]):
        raise ValueError("Target sides must match active sides")
    return await move_debug_markers_to_world_targets_async(
        context["markers"],
        context["handles"],
        targets,
        update_async,
        sample_count=sample_count,
        position_tolerance=position_tolerance,
        orientation_tolerance=orientation_tolerance,
        max_steps_per_target=max_steps_per_target,
    )


async def set_floating_gripper_grasp_async(context: Mapping[str, Any], close: bool, *, settle_steps: int = 60) -> None:
    """Open or close configured floating grippers and settle the simulation."""
    await set_debug_grasp_async(
        context["manager"],
        context["grasp_sides"],
        close,
        app_utils.update_app_async,
        settle_steps=settle_steps,
    )


def stop_floating_gripper_controllers(context: Mapping[str, Any]) -> None:
    """Stop scripted controllers and release their teleop-session resources."""
    context["manager"].set_floating_tracking(False)
    context["manager"].set_grasp_tracking(False)
    if context["visual_cues"] is not None:
        context["visual_cues"].hide_all()
    context["markers"].remove_all_markers()
    context["manager"].destroy()

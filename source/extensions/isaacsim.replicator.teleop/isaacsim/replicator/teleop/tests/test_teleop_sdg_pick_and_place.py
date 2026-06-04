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

"""End-to-end pick-and-place tests for the SDG teleop demo, one per scenario.

Each test drives a stage / profile pair through the full teleop pipeline in
debug mode (no CloudXR / device connections): place markers, reach, locomote,
grasp, lift, drop, and optionally record + replay the episode with SDG capture.
"""

from __future__ import annotations

import os

import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.test.utils.file_validation import validate_folder_contents
from isaacsim.test.utils.image_comparison import compare_images_in_directories


async def run_teleop_pick_and_place_async(scenario_config: dict) -> None:
    """Drive a stage / profile pair through the full teleop pick-and-place flow.

    Opens the scenario stage, configures the floating / IK / grasp / locomotion
    controllers from the loaded teleop profile, then plays the timeline and
    moves the markers through the place, reach, locomote, grasp, lift, and
    drop stages. Optionally records the live episode and replays it, and
    optionally captures SDG images at each action boundary in both phases.
    """
    from dataclasses import dataclass
    from pathlib import Path

    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.core.experimental.utils import app as app_utils
    from isaacsim.core.experimental.utils import stage as stage_utils
    from isaacsim.replicator.episode_recorder import EpisodeReplayer, ReplayPolicy
    from isaacsim.replicator.teleop import (
        OXR_TO_ISS_QUAT,
        CoordinateSystem,
        FloatingRigidBodyController,
        GraspController,
        IKMethod,
        IKSolverType,
        LocomotionController,
        MarkersManager,
        RobotIKController,
        TeleopManager,
        build_teleop_recorder,
        get_builtin_teleop_profiles_dir,
        load_grasp_config,
        load_teleop_profile,
    )
    from isaacsim.storage.native import get_assets_root_path_async
    from pxr import Gf, Usd, UsdGeom

    TELEOP_DEFAULT_ORIENTATION_XYZW = OXR_TO_ISS_QUAT

    # Forward thumbstick value applied while the demo is locomoting toward
    # the asset; full forward (1.0) gives a deterministic, reproducible glide.
    LOCOMOTION_FORWARD_INPUT = 1.0

    @dataclass
    class SdgReplayCapture:
        """Live SDG capture point that should be captured again during replay."""

        action_name: str
        recorded_frame_index: int
        live_capture_index: int

    # -----------------------------------------------------------------------
    # Generic helpers
    # -----------------------------------------------------------------------

    def world_to_tracking_origin_local(
        markers_manager: MarkersManager, world_position: np.ndarray
    ) -> np.ndarray | None:
        """Convert a world target to the current TrackingOrigin-local marker position."""
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        origin_path = markers_manager.MARKER_PATHS.get("origin")
        if not origin_path:
            return None
        origin_prim = stage.GetPrimAtPath(origin_path)
        if not origin_prim or not origin_prim.IsValid():
            return None
        origin_to_world = UsdGeom.Xformable(origin_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local_position = origin_to_world.GetInverse().Transform(Gf.Vec3d(*world_position.tolist()))
        return np.array([local_position[0], local_position[1], local_position[2]], dtype=np.float64)

    def asset_relative_world_target(state: dict, relative_location: tuple[float, float, float]) -> np.ndarray:
        """Return ``asset_origin + relative_location + tcp_offset`` for one side."""
        return state["asset_origin"] + np.asarray(relative_location, dtype=np.float64) + state["tcp_offset"]

    # -----------------------------------------------------------------------
    # Per-controller configuration helpers (one side at a time)
    # -----------------------------------------------------------------------

    def detect_active_motion(profile, scenario_sides: list[str]) -> dict[str, str]:
        """Return ``{side: 'floating'|'ik'}`` for sides enabled in both profile and scenario."""
        active: dict[str, str] = {}
        for side in scenario_sides:
            if getattr(profile.floating, side).enabled:
                active[side] = "floating"
            elif getattr(profile.ik, side).enabled:
                active[side] = "ik"
        return active

    def configure_floating_side(
        profile,
        teleop_manager: TeleopManager,
        controller: FloatingRigidBodyController,
        side: str,
    ) -> str | None:
        """Configure one floating side and return its target prim path."""
        settings = getattr(profile.floating, side).settings
        prim_path = str(settings.get("prim_path", "")).strip()
        if not prim_path:
            print(f"[TeleopDemo][Setup]   {side.capitalize()} floating profile is missing 'prim_path'")
            return None

        controller.set_prim_path(side, prim_path)
        controller.set_gains(
            float(settings.get("pos_kp", 15.0)),
            float(settings.get("pos_kd", 0.5)),
            float(settings.get("orient_kp", 15.0)),
            float(settings.get("orient_kd", 0.2)),
            side=side,
        )
        controller.set_target_rotation_offsets(
            side,
            float(settings.get("target_rot_x_deg", 0.0)),
            float(settings.get("target_rot_y_deg", 0.0)),
            float(settings.get("target_rot_z_deg", 0.0)),
        )
        valid, message = controller.validate(side)
        if not valid:
            print(f"[TeleopDemo][Setup]   {side.capitalize()} floating controller is invalid: {message}")
            return None
        if not controller.configure(side):
            print(f"[TeleopDemo][Setup]   Could not configure {side} floating controller at '{prim_path}'")
            return None
        teleop_manager.set_floating_side_assigned(side, True)
        print(f"[TeleopDemo][Setup]   Configured {side} floating controller: {prim_path}")
        return prim_path

    def configure_ik_side(profile, controller: RobotIKController, side: str) -> bool:
        """Configure one IK side from the profile."""
        settings = getattr(profile.ik, side).settings
        robot_path = str(settings.get("robot_path", "")).strip()
        ee_link = str(settings.get("ee_link", "")).strip()
        if not robot_path or not ee_link:
            print(f"[TeleopDemo][Setup]   {side.capitalize()} IK profile is missing 'robot_path' or 'ee_link'")
            return False

        controller.set_articulation_path(side, robot_path)
        controller.set_ee_link_name(side, ee_link)
        try:
            ok, message = controller.set_solver_type(side, IKSolverType(str(settings.get("solver", "position-based"))))
        except ValueError:
            print(f"[TeleopDemo][Setup]   Unknown IK solver for {side}: {settings.get('solver')}")
            return False
        if not ok:
            print(f"[TeleopDemo][Setup]   IK solver unavailable for {side}: {message}")
            return False

        method = settings.get("method")
        if method:
            try:
                controller.set_ik_method(side, IKMethod(str(method)))
            except ValueError:
                print(f"[TeleopDemo][Setup]   Unknown IK method for {side}: {method}")
                return False

        controller.set_gain(side, float(settings.get("gain", 5.0)))
        controller.set_vr_target_filter(side, float(settings.get("vr_target_filter", 0.0)))
        controller.set_max_joint_step(side, float(settings.get("max_joint_step", 0.0)))
        controller.set_ee_rotation_offsets(
            side,
            float(settings.get("ee_rot_x_deg", 0.0)),
            float(settings.get("ee_rot_y_deg", 0.0)),
            float(settings.get("ee_rot_z_deg", 0.0)),
        )
        if "pink_qp_solver" in settings:
            ok, message = controller.set_pink_qp_solver(side, str(settings["pink_qp_solver"]))
            if not ok:
                print(f"[TeleopDemo][Setup]   PINK QP solver unavailable for {side}: {message}")
                return False
        controller.set_pink_task_gain(side, float(settings.get("pink_task_gain", controller.get_pink_task_gain(side))))
        controller.set_pink_posture_cost(
            side, float(settings.get("pink_posture_cost", controller.get_pink_posture_cost(side)))
        )
        controller.set_pink_lm_damping(
            side, float(settings.get("pink_lm_damping", controller.get_pink_lm_damping(side)))
        )

        validation = controller.validate(side)
        if not validation.valid:
            print(f"[TeleopDemo][Setup]   {side.capitalize()} IK controller is invalid: {validation.message}")
            return False
        chain_dofs = controller.compute_arm_dofs(side)
        if chain_dofs is not None and chain_dofs > 0:
            controller.set_num_arm_dofs(side, chain_dofs)
        if not controller.configure(side):
            validation = controller.validate(side)
            print(f"[TeleopDemo][Setup]   Could not configure {side} IK controller: {validation.message}")
            return False
        print(f"[TeleopDemo][Setup]   Configured {side} IK controller: {robot_path} -> {ee_link}")
        return True

    def configure_grasp_side(
        profile,
        teleop_manager: TeleopManager,
        controller: GraspController,
        side: str,
    ) -> str | None:
        """Configure one grasp side. Returns the gripper prim path or None when disabled / invalid."""
        side_profile = getattr(profile.grasp, side)
        if not side_profile.enabled:
            return None
        config, errors = load_grasp_config(side_profile.config_path)
        if config is None or errors:
            print(
                f"[TeleopDemo][Setup]   Could not load {side} grasp config "
                f"'{side_profile.config_path}': {'; '.join(errors)}"
            )
            return None
        if not controller.configure(side_profile.prim_path, side, config):
            print(
                f"[TeleopDemo][Setup]   Could not configure {side} grasp controller " f"at '{side_profile.prim_path}'"
            )
            return None
        controller.set_side_tracking_enabled(side, True)
        teleop_manager.set_debug_trigger(side, 0.0)
        print(f"[TeleopDemo][Setup]   Configured {side} grasp controller: {side_profile.prim_path}")
        return side_profile.prim_path

    def configure_locomotion(profile, teleop_manager: TeleopManager, controller: LocomotionController) -> bool:
        """Configure the (single, scenario-wide) locomotion controller.

        Locomotion is optional: profiles that disable it (e.g. solo floating
        scenarios) are accepted; the locomotion phase is then skipped at runtime.
        """
        if not profile.locomotion.enabled:
            print("[TeleopDemo][Setup]   Locomotion disabled in profile, skipping configuration")
            return True
        prim_path = str(profile.locomotion.settings.get("prim_path", "")).strip()
        if not prim_path:
            print("[TeleopDemo][Setup]   Locomotion profile is missing 'prim_path', exiting")
            return False
        controller.set_prim_path(prim_path)
        controller.set_linear_step(
            float(profile.locomotion.settings.get("linear_step", LocomotionController.DEFAULT_LINEAR_STEP))
        )
        controller.set_angular_step(
            float(profile.locomotion.settings.get("angular_step", LocomotionController.DEFAULT_ANGULAR_STEP))
        )
        valid, message = controller.validate()
        if not valid:
            print(f"[TeleopDemo][Setup]   Locomotion target is invalid: {message}, exiting")
            return False
        teleop_manager.set_locomotion_tracking(True)
        print(f"[TeleopDemo][Setup]   Configured locomotion controller: {prim_path}")
        return True

    def configure_session(
        profile,
        teleop_manager: TeleopManager,
        markers_manager: MarkersManager,
        sides: list[str],
    ) -> bool:
        """Apply session settings and create only the markers needed for the active sides."""
        try:
            coordinate_system = CoordinateSystem(profile.session.coordinate_system)
        except ValueError:
            coordinate_system = CoordinateSystem.ISAAC_SIM
        teleop_manager.set_coordinate_system(coordinate_system)
        markers_manager.set_frame_scale(float(profile.session.marker_scale))

        if profile.session.tracking_space_enabled:
            ok, message = teleop_manager.set_tracking_space_prim_path(profile.session.tracking_space_path)
            if not ok:
                print(f"[TeleopDemo][Setup]   Tracking-space setup skipped: {message}")
        else:
            teleop_manager.disable_tracking_space()

        for marker_name in ("origin", *sides):
            ok, message = markers_manager.ensure_marker(marker_name)
            if not ok:
                print(f"[TeleopDemo][Setup]   Could not create debug marker " f"'{marker_name}': {message}, exiting")
                return False
        teleop_manager.set_debug_tracking(True)
        return True

    # -----------------------------------------------------------------------
    # Per-side state assembly
    # -----------------------------------------------------------------------

    def build_sides_state(
        profile,
        teleop_manager: TeleopManager,
        floating_controller: FloatingRigidBodyController,
        ik_controller: RobotIKController,
        grasp_controller: GraspController,
    ) -> dict[str, dict] | None:
        """Configure each active side and return per-side state, or None on failure.

        The returned per-side state bundles everything the rest of the demo needs:
        motion type and controller, the controlled gripper prim, asset path /
        TCP offset, and grasp availability.
        """
        scenario_sides = list(scenario_config["sides"].keys())
        active_motion = detect_active_motion(profile, scenario_sides)
        if not active_motion:
            print("[TeleopDemo][Setup]   No active motion controllers in the profile, exiting")
            return None

        sides_state: dict[str, dict] = {}
        for side, motion_type in active_motion.items():
            if motion_type == "floating":
                ok = configure_floating_side(profile, teleop_manager, floating_controller, side) is not None
                motion_controller = floating_controller
            else:
                ok = configure_ik_side(profile, ik_controller, side)
                motion_controller = ik_controller
            if not ok:
                return None
            if motion_type == "floating":
                controlled_path = str(getattr(profile.floating, side).settings.get("prim_path", "")).strip()
            else:
                controlled_path = str(getattr(profile.grasp, side).prim_path).strip()
            side_config = scenario_config["sides"][side]
            sides_state[side] = {
                "motion_type": motion_type,
                "motion_controller": motion_controller,
                "controlled_path": controlled_path,
                "controlled_gripper": XformPrim(controlled_path),
                "asset_path": side_config["asset_path"],
                "drop_target_path": side_config.get("drop_target_path"),
                "tcp_offset": np.asarray(side_config["tcp_offset_world"], dtype=np.float64),
                "start_offset": tuple(side_config["start_offset"]),
                "reach_offset": tuple(side_config["reach_offset"]),
                "pre_grasp_offset": tuple(side_config["pre_grasp_offset"]),
                "lift_offset": tuple(side_config["lift_offset"]),
                "drop_offset": tuple(side_config["drop_offset"]),
                "grasp_enabled": False,
            }

        for side in sides_state:
            if configure_grasp_side(profile, teleop_manager, grasp_controller, side) is not None:
                sides_state[side]["grasp_enabled"] = True
        teleop_manager.set_grasp_tracking(grasp_controller.has_any_side_tracking_enabled)
        return sides_state

    def cache_target_origins(stage, sides_state: dict[str, dict]) -> bool:
        """Cache each side's asset world position once, used as the relative-motion origin."""
        for side, state in sides_state.items():
            prim = stage.GetPrimAtPath(state["asset_path"])
            if prim is None or not prim.IsValid():
                print(f"[TeleopDemo][Setup]   {side.capitalize()} asset not found: {state['asset_path']}, exiting")
                return False
            positions, _ = XformPrim(state["asset_path"]).get_world_poses()
            state["asset_origin"] = np.asarray(positions.numpy(), dtype=np.float64).reshape(-1, 3)[0]
            drop_target_path = state.get("drop_target_path")
            if drop_target_path:
                drop_target_prim = stage.GetPrimAtPath(drop_target_path)
                if drop_target_prim is None or not drop_target_prim.IsValid():
                    print(
                        f"[TeleopDemo][Setup]   {side.capitalize()} drop target not found: "
                        f"{drop_target_path}, exiting"
                    )
                    return False
                positions, _ = XformPrim(drop_target_path).get_world_poses()
                state["drop_target_origin"] = np.asarray(positions.numpy(), dtype=np.float64).reshape(-1, 3)[0]
        return True

    # -----------------------------------------------------------------------
    # Motion primitives
    # -----------------------------------------------------------------------

    async def wait_for_motion_controllers(sides_state: dict[str, dict], max_frames: int = 5) -> bool:
        """Block briefly until every active side's motion controller is running."""
        for _ in range(max_frames):
            if all(state["motion_controller"].is_running(side) for side, state in sides_state.items()):
                return True
            await app_utils.update_app_async()
        return False

    async def place_markers_at_start(
        markers_manager: MarkersManager,
        sides_state: dict[str, dict],
    ) -> bool:
        """Snap each side's marker to its per-side asset-relative start pose.

        Pure snap, no settle: this is called before the timeline plays so that
        the recording begins with markers already at each side's ``start_offset``.
        The matching wait for the controllers to converge happens after
        ``timeline.play()`` via ``settle_at_start_pose``.
        """
        offsets_summary = ", ".join(
            f"{side.capitalize()}={list(state['start_offset'])}" for side, state in sides_state.items()
        )
        print(f"[TeleopDemo][Setup] Place markers at start: {offsets_summary}")
        for side, state in sides_state.items():
            local_target = world_to_tracking_origin_local(
                markers_manager, asset_relative_world_target(state, state["start_offset"])
            )
            if local_target is None:
                print("[TeleopDemo][Setup]   Tracking origin pose unavailable, exiting")
                return False
            markers_manager.update_marker_transform(side, tuple(local_target.tolist()), TELEOP_DEFAULT_ORIENTATION_XYZW)
        return True

    async def settle_at_start_pose(
        sides_state: dict[str, dict],
        settle_frames: int,
        target_min_distance_error: float,
    ) -> None:
        """Hold each marker at its per-side start pose while the timeline runs so the controllers can converge.

        Must be called after ``timeline.play()``: physics only advances while the
        timeline is playing, so a settle scheduled before ``play()`` is a no-op.
        Prints a per-side gripper-vs-start-pose tracking summary at the end.
        """
        if settle_frames > 0:
            await app_utils.update_app_async(steps=settle_frames)

        summary_parts = []
        for side, state in sides_state.items():
            target_world = asset_relative_world_target(state, state["start_offset"])
            gripper_positions, _ = state["controlled_gripper"].get_world_poses()
            gripper_position = np.asarray(gripper_positions.numpy(), dtype=np.float64).reshape(-1, 3)[0]
            error = float(np.linalg.norm(gripper_position - target_world))
            flag = "OK" if error <= target_min_distance_error else "OUT"
            summary_parts.append(f"{side.capitalize()} {flag} {error:.4f}/{target_min_distance_error:.4f} m")
        print(f"[TeleopDemo][Settle]   Start: {settle_frames} frames | " + ", ".join(summary_parts))

    async def move_markers_to_asset_relative_target(
        markers_manager: MarkersManager,
        sides_state: dict[str, dict],
        offset_key: str,
        motion_frames: int,
        settle_frames: int,
        target_min_distance_error: float,
        phase_name: str,
    ) -> bool:
        """Drive each side's marker along its per-side asset-relative trajectory, then settle.

        ``offset_key`` selects which per-side offset to read from ``sides_state``
        (e.g. ``"reach_offset"`` or ``"lift_offset"``). Markers update once per
        frame (one trajectory waypoint per frame) so the motion controllers can
        track continuously, then the marker is held at the final pose for
        ``settle_frames`` frames so the controller can converge. Prints a per-side
        gripper-vs-target tracking summary at the end and flags any side whose
        error exceeds ``target_min_distance_error``.
        """
        if motion_frames < 1:
            print(f"[TeleopDemo][Motion]     {phase_name}: motion_frames must be >= 1")
            return False

        trajectories: dict[str, dict] = {}
        alphas = np.linspace(0.0, 1.0, motion_frames)
        for side, base in sides_state.items():
            marker_pose = markers_manager.get_marker_world_pose(side)
            if marker_pose is None:
                print(f"[TeleopDemo][Motion]     {phase_name} {side}: Debug marker pose unavailable")
                return False
            start = np.asarray(marker_pose[0], dtype=np.float64)
            target = asset_relative_world_target(base, base[offset_key])
            trajectories[side] = {
                "controlled_gripper": base["controlled_gripper"],
                "waypoints": [start + (target - start) * a for a in alphas],
                "target_world": target,
            }

        for frame_index in range(motion_frames):
            for side, traj in trajectories.items():
                local_target = world_to_tracking_origin_local(markers_manager, traj["waypoints"][frame_index])
                if local_target is None:
                    print(f"[TeleopDemo][Motion]     {phase_name} {side}: Tracking origin pose unavailable")
                    return False
                markers_manager.update_marker_transform(
                    side,
                    tuple(local_target.tolist()),
                    TELEOP_DEFAULT_ORIENTATION_XYZW,
                )
            await app_utils.update_app_async()

        if settle_frames > 0:
            await app_utils.update_app_async(steps=settle_frames)

        summary_parts = []
        for side, traj in trajectories.items():
            gripper_positions, _ = traj["controlled_gripper"].get_world_poses()
            gripper_position = np.asarray(gripper_positions.numpy(), dtype=np.float64).reshape(-1, 3)[0]
            error = float(np.linalg.norm(gripper_position - traj["target_world"]))
            flag = "OK" if error <= target_min_distance_error else "OUT"
            summary_parts.append(f"{side.capitalize()} {flag} {error:.4f}/{target_min_distance_error:.4f} m")
        print(
            f"[TeleopDemo][Motion]   {phase_name} motion {motion_frames} + settle {settle_frames} frames | "
            + ", ".join(summary_parts)
        )
        return True

    async def move_markers_to_drop_target(
        markers_manager: MarkersManager,
        sides_state: dict[str, dict],
        motion_frames: int,
        settle_frames: int,
        target_min_distance_error: float,
    ) -> bool:
        """Drive each side's marker to ``drop_target_origin + tcp_offset + drop_offset``, then settle.

        Reads each side's ``drop_offset`` from ``sides_state``. Same per-frame
        trajectory + settle + tracking summary contract as
        ``move_markers_to_asset_relative_target``. Sides without a configured
        drop target are skipped silently.
        """
        if motion_frames < 1:
            print("[TeleopDemo][Motion]     Drop: motion_frames must be >= 1")
            return False

        active_states = {
            side: state for side, state in sides_state.items() if state.get("drop_target_origin") is not None
        }
        if not active_states:
            print("[TeleopDemo][Motion]   No drop target prims configured, skipping")
            return True

        trajectories: dict[str, dict] = {}
        alphas = np.linspace(0.0, 1.0, motion_frames)
        for side, base in active_states.items():
            marker_pose = markers_manager.get_marker_world_pose(side)
            if marker_pose is None:
                print(f"[TeleopDemo][Motion]     Drop {side}: Debug marker pose unavailable")
                return False
            start = np.asarray(marker_pose[0], dtype=np.float64)
            drop_offset_vec = np.asarray(base["drop_offset"], dtype=np.float64)
            target = np.asarray(base["drop_target_origin"], dtype=np.float64) + base["tcp_offset"] + drop_offset_vec
            trajectories[side] = {
                "controlled_gripper": base["controlled_gripper"],
                "waypoints": [start + (target - start) * a for a in alphas],
                "target_world": target,
            }

        for frame_index in range(motion_frames):
            for side, traj in trajectories.items():
                local_target = world_to_tracking_origin_local(markers_manager, traj["waypoints"][frame_index])
                if local_target is None:
                    print(f"[TeleopDemo][Motion]     Drop {side}: Tracking origin pose unavailable")
                    return False
                markers_manager.update_marker_transform(
                    side,
                    tuple(local_target.tolist()),
                    TELEOP_DEFAULT_ORIENTATION_XYZW,
                )
            await app_utils.update_app_async()

        if settle_frames > 0:
            await app_utils.update_app_async(steps=settle_frames)

        summary_parts = []
        for side, traj in trajectories.items():
            gripper_positions, _ = traj["controlled_gripper"].get_world_poses()
            gripper_position = np.asarray(gripper_positions.numpy(), dtype=np.float64).reshape(-1, 3)[0]
            error = float(np.linalg.norm(gripper_position - traj["target_world"]))
            flag = "OK" if error <= target_min_distance_error else "OUT"
            summary_parts.append(f"{side.capitalize()} {flag} {error:.4f}/{target_min_distance_error:.4f} m")
        print(
            f"[TeleopDemo][Motion]   Drop motion {motion_frames} + settle {settle_frames} frames | "
            + ", ".join(summary_parts)
        )
        return True

    async def locomote_to_asset_relative_target(
        markers_manager: MarkersManager,
        teleop_manager: TeleopManager,
        sides_state: dict[str, dict],
        offset_key: str,
        target_min_distance_error: float,
        max_locomotion_steps: int,
        settle_frames: int,
    ) -> bool:
        """Drive the tracking origin forward until every active side is within tolerance, then settle.

        ``offset_key`` selects which per-side offset to read from ``sides_state``
        (typically ``"pre_grasp_offset"``).
        """
        targets = {side: asset_relative_world_target(state, state[offset_key]) for side, state in sides_state.items()}
        final_errors = {side: float("inf") for side in sides_state}

        targets_summary = ", ".join(
            f"{side.capitalize()}={list(sides_state[side][offset_key])}" for side in sides_state
        )
        print(f"[TeleopDemo][Locomotion]   Target: {targets_summary}")
        teleop_manager.set_debug_thumbstick("left", y=LOCOMOTION_FORWARD_INPUT)
        try:
            for step_index in range(max_locomotion_steps):
                await app_utils.update_app_async()
                for side, target in targets.items():
                    marker_pose = markers_manager.get_marker_world_pose(side)
                    if marker_pose is None:
                        print(f"[TeleopDemo][Locomotion]     {side}: " f"Debug marker pose unavailable, exiting")
                        return False
                    final_errors[side] = float(np.linalg.norm(np.asarray(marker_pose[0], dtype=np.float64) - target))
                max_error = max(final_errors.values())
                if (step_index + 1) % 25 == 0:
                    print(
                        f"[TeleopDemo][Locomotion]   Step {step_index + 1}/{max_locomotion_steps}: "
                        f"Error {max_error:.4f}/{target_min_distance_error:.4f} m"
                    )
                if all(error <= target_min_distance_error for error in final_errors.values()):
                    print(
                        f"[TeleopDemo][Locomotion]   Reached after "
                        f"{step_index + 1}/{max_locomotion_steps} steps, "
                        f"Error {max_error:.4f}/{target_min_distance_error:.4f} m"
                    )
                    break
        finally:
            teleop_manager.set_debug_thumbstick("left", y=0.0)

        if any(error > target_min_distance_error for error in final_errors.values()):
            max_error = max(final_errors.values())
            print(
                f"[TeleopDemo][Locomotion]   Missed after {max_locomotion_steps} steps, "
                f"Error {max_error:.4f}/{target_min_distance_error:.4f} m, exiting"
            )
            return False
        if settle_frames > 0:
            await app_utils.update_app_async(steps=settle_frames)
        print(f"[TeleopDemo][Locomotion]   Settle {settle_frames} frames")
        return True

    async def close_grasps(teleop_manager: TeleopManager, sides_state: dict[str, dict], settle_frames: int) -> None:
        """Trigger every active grasp side and let the drives settle."""
        grasp_sides = [side for side, state in sides_state.items() if state["grasp_enabled"]]
        if not grasp_sides:
            print("[TeleopDemo][Grasp]   No grasp sides enabled, skipping")
            return
        for side in grasp_sides:
            teleop_manager.set_debug_trigger(side, 1.0)
        await app_utils.update_app_async(steps=settle_frames)
        print(f"[TeleopDemo][Grasp]   Closed ({', '.join(grasp_sides)}) after settle {settle_frames} frames")

    async def open_grasps(teleop_manager: TeleopManager, sides_state: dict[str, dict], settle_frames: int) -> None:
        """Open every active grasp side and let dropped objects settle."""
        grasp_sides = [side for side, state in sides_state.items() if state["grasp_enabled"]]
        if not grasp_sides:
            print("[TeleopDemo][Grasp]   No grasp sides enabled, skipping drop release")
            return
        for side in grasp_sides:
            teleop_manager.set_debug_trigger(side, 0.0)
        await app_utils.update_app_async(steps=settle_frames)
        print(f"[TeleopDemo][Grasp]   Opened ({', '.join(grasp_sides)}) after settle {settle_frames} frames")

    # -----------------------------------------------------------------------
    # Episode recorder targets
    # -----------------------------------------------------------------------

    def build_recorder_targets(
        profile,
        sides_state: dict[str, dict],
        markers_manager: MarkersManager,
    ) -> tuple[dict[str, str], dict[str, str]]:
        """Build the ``xforms`` and ``articulations`` dicts for the EpisodeRecorder."""
        xforms: dict[str, str] = {"tracking_origin": markers_manager.MARKER_PATHS["origin"]}
        articulations: dict[str, str] = {}
        if profile.locomotion.enabled:
            locomotion_prim_path = str(profile.locomotion.settings.get("prim_path", "")).strip()
            if locomotion_prim_path:
                xforms["locomotion_base"] = locomotion_prim_path
        for side, state in sides_state.items():
            xforms[f"{side}_asset"] = state["asset_path"]
            xforms[f"{side}_marker"] = markers_manager.MARKER_PATHS[side]
            if state["motion_type"] == "floating":
                xforms[f"{side}_controlled_gripper"] = state["controlled_path"]
            else:
                articulations[f"{side}_robot"] = str(getattr(profile.ik, side).settings["robot_path"])
            if state["grasp_enabled"]:
                articulations[f"{side}_gripper"] = getattr(profile.grasp, side).prim_path
        return xforms, articulations

    # -----------------------------------------------------------------------
    # Capture (BasicWriter on disabled-by-default render products)
    # -----------------------------------------------------------------------

    def setup_sdg_render_products(
        stage,
        camera_paths: list[str],
        resolution: tuple[int, int],
    ):
        """Build per-camera render products for SDG captures."""
        valid_camera_paths: list[str] = []
        for camera_path in camera_paths:
            camera_prim = stage.GetPrimAtPath(camera_path)
            if camera_prim is None or not camera_prim.IsValid():
                print(f"[TeleopDemo][SDG]   Capture camera missing, skipping: {camera_path}")
                continue
            if not camera_prim.IsA(UsdGeom.Camera):
                print(f"[TeleopDemo][SDG]   Capture prim is not a camera, skipping: {camera_path}")
                continue
            valid_camera_paths.append(camera_path)

        if not valid_camera_paths:
            print("[TeleopDemo][SDG]   No valid capture cameras, SDG capture disabled")
            return []

        render_products = []
        for camera_path in valid_camera_paths:
            camera_name = camera_path.rsplit("/", 1)[-1]
            render_product = rep.create.render_product(
                camera_path,
                resolution,
                name=f"rp_{camera_name}",
                force_new=True,
            )
            render_product.hydra_texture.set_updates_enabled(False)
            render_products.append(render_product)
        print(f"[TeleopDemo][SDG]   Render products: {len(render_products)} at {resolution}")
        return render_products

    async def capture_live_sdg_action_image(
        action_name: str,
        recorder,
        writer,
        render_products: list,
        capture_index: int,
        rt_subframes: int,
        replay_capture_points: list[SdgReplayCapture],
    ) -> int:
        """Capture the live end-of-action SDG image set and remember its episode frame.

        When ``recorder`` is ``None`` (record + replay disabled in the scenario)
        the image is still written but no replay capture point is tracked, since
        there is no episode to replay against.
        """
        if writer is None or not render_products:
            return capture_index
        recorded_frame_index: int | None = None
        if recorder is not None:
            recorded_frame_count = int(recorder.current_episode_frames)
            if recorded_frame_count <= 0:
                print(f"[TeleopDemo][SDG]   Skipped live capture for '{action_name}': no recorded frame yet")
                return capture_index
            recorded_frame_index = recorded_frame_count - 1

        next_capture_index = capture_index + 1
        frame_label = f"frame {recorded_frame_index}" if recorded_frame_index is not None else "no recorder"
        print(
            f"[TeleopDemo][SDG]   Capturing live {frame_label} for '{action_name}' "
            f"(output image {next_capture_index})"
        )
        if recorder is not None:
            recorder.pause()
        for render_product in render_products:
            render_product.hydra_texture.set_updates_enabled(True)
        await rep.orchestrator.step_async(delta_time=0.0, pause_timeline=False, rt_subframes=rt_subframes)
        for render_product in render_products:
            render_product.hydra_texture.set_updates_enabled(False)
        if recorder is not None:
            recorder.resume()
        if recorded_frame_index is not None:
            replay_capture_points.append(
                SdgReplayCapture(
                    action_name=action_name,
                    recorded_frame_index=recorded_frame_index,
                    live_capture_index=next_capture_index,
                )
            )
        print(f"[TeleopDemo][SDG]     Saved live RGB image {next_capture_index}")
        return next_capture_index

    async def capture_replayed_sdg_action_image(
        capture_point: SdgReplayCapture,
        writer,
        render_products: list,
        capture_index: int,
        rt_subframes: int,
    ) -> int:
        """Capture the replayed frame matching a live SDG action capture."""
        if writer is None or not render_products:
            return capture_index
        next_capture_index = capture_index + 1
        print(
            f"[TeleopDemo][SDG]   Capturing replay frame {capture_point.recorded_frame_index} "
            f"for '{capture_point.action_name}' "
            f"(output image {next_capture_index}, live image {capture_point.live_capture_index})"
        )
        for render_product in render_products:
            render_product.hydra_texture.set_updates_enabled(True)
        await rep.orchestrator.step_async(delta_time=0.0, pause_timeline=False, rt_subframes=rt_subframes)
        for render_product in render_products:
            render_product.hydra_texture.set_updates_enabled(False)
        print(f"[TeleopDemo][SDG]     Saved replay RGB image {next_capture_index}")
        return next_capture_index

    # -----------------------------------------------------------------------
    # Cleanup
    # -----------------------------------------------------------------------

    async def cleanup_teleop_pick_and_place_async(
        *,
        scenario_name: str,
        markers_manager: MarkersManager | None,
        teleop_manager: TeleopManager | None,
        recorder=None,
        episode_active: bool = False,
        timeline=None,
        timeline_started: bool = False,
        sdg_capture_writer=None,
        sdg_capture_render_products: list | None = None,
        replayer=None,
    ) -> None:
        """Best-effort, never-raising teardown for the live + replay phases.

        Each step is wrapped so a single failure does not skip the rest. Useful
        when the user closes the stage, stops the app, or any setup step fails
        mid-flight: this still tears down whatever was already brought up.
        """

        def _safe(label: str, fn, *args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except Exception as exc:
                print(f"[TeleopDemo][Cleanup]   '{label}' skipped: {exc}")

        async def _safe_async(label: str, coro):
            try:
                await coro
            except Exception as exc:
                print(f"[TeleopDemo][Cleanup]   '{label}' skipped: {exc}")

        if replayer is not None:
            if getattr(replayer, "is_replaying", False):
                _safe("replayer.stop_replay", replayer.stop_replay)
                await _safe_async("app update after stop_replay", app_utils.update_app_async())
            _safe("replayer.close", replayer.close)

        if recorder is not None:
            if episode_active:
                _safe(
                    "recorder.end_episode",
                    recorder.end_episode,
                    success=False,
                    metadata={"scenario": scenario_name, "reason": "interrupted"},
                )
            _safe("recorder.close_session", recorder.close_session)

        if sdg_capture_writer is not None:
            await _safe_async("sdg capture flush", rep.orchestrator.wait_until_complete_async())
            _safe("sdg writer.detach", sdg_capture_writer.detach)
        for render_product in sdg_capture_render_products or []:
            _safe("sdg render product destroy", render_product.destroy)

        if timeline is not None and timeline_started:
            _safe("timeline.stop", timeline.stop)

        if teleop_manager is not None:
            _safe("teleop_manager.set_debug_tracking", teleop_manager.set_debug_tracking, False)
            _safe("teleop_manager.destroy", teleop_manager.destroy)

        if markers_manager is not None:
            _safe("markers_manager.remove_all_markers", markers_manager.remove_all_markers)

    # -----------------------------------------------------------------------
    # Top-level scenario flow
    # -----------------------------------------------------------------------

    scenario_name = str(scenario_config["name"])
    stage_url = str(scenario_config["stage_url"])
    profile_name = str(scenario_config["profile_name"])
    profile_stem = Path(profile_name).stem
    scenario_output_dir = Path.cwd() / f"_out_teleop_{profile_stem}"
    episode_output_dir = scenario_output_dir / "episode_data"
    reach_motion_frames = int(scenario_config["reach_motion_frames"])
    lift_motion_frames = int(scenario_config["lift_motion_frames"])
    drop_motion_frames = int(scenario_config["drop_motion_frames"])
    start_settle_frames = int(scenario_config["start_settle_frames"])
    reach_settle_frames = int(scenario_config["reach_settle_frames"])
    locomotion_settle_frames = int(scenario_config["locomotion_settle_frames"])
    grasp_settle_frames = int(scenario_config["grasp_settle_frames"])
    lift_settle_frames = int(scenario_config["lift_settle_frames"])
    drop_settle_frames = int(scenario_config["drop_settle_frames"])
    release_settle_frames = int(scenario_config["release_settle_frames"])
    target_min_distance_error = float(scenario_config["target_min_distance_error"])
    max_locomotion_steps = int(scenario_config["max_locomotion_steps"])
    capture_sdg = bool(scenario_config["capture_sdg"])
    record_and_replay_episode = bool(scenario_config["record_and_replay_episode"])
    camera_paths = list(scenario_config["camera_paths"])
    capture_resolution = tuple(scenario_config["capture_resolution"])
    capture_rt_subframes = int(scenario_config["capture_rt_subframes"])

    # Resources tracked for safe cleanup. Each is set as it is acquired and
    # cleared back to None after explicit teardown so the finally block does
    # not double-tear-down on the happy path.
    timeline = omni.timeline.get_timeline_interface()
    timeline_started = False
    markers_manager: MarkersManager | None = None
    teleop_manager: TeleopManager | None = None
    recorder = None
    episode_active = False
    # Live and replay phases use separate writers attached to the same render
    # products. sdg_capture_index resets between phases so live and replay
    # images share the same 1..N numbering scheme.
    writer_live = None
    writer_replay = None
    sdg_capture_render_products: list = []
    sdg_capture_index = 0
    replayer = None
    # Live SDG captures happen at action boundaries. Each recorded frame index
    # below is replayed later so the replay captures match the live moments.
    replay_sdg_capture_points: list[SdgReplayCapture] = []

    print(f"\n[TeleopDemo] Scenario '{scenario_name}'")
    print(f"[TeleopDemo]   Stage: {stage_url}")
    print(f"[TeleopDemo]   Profile: {profile_name}")
    print(f"[TeleopDemo]   Output folder: {scenario_output_dir}")

    try:
        print("[TeleopDemo][Setup] Open stage")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            print("[TeleopDemo][Setup]   Could not resolve the Isaac Sim assets root path, exiting")
            return
        stage_path = assets_root_path + stage_url
        print(f"[TeleopDemo][Setup]   Path: {stage_path}")
        opened, stage = await stage_utils.open_stage_async(stage_path)
        if not opened or stage is None:
            print(f"[TeleopDemo][Setup]   Failed to open stage: {stage_path}, exiting")
            return
        if capture_sdg:
            rep.orchestrator.set_capture_on_play(False)
            print("[TeleopDemo][SDG]   Replicator capture-on-play disabled")

        print("[TeleopDemo][Setup] Create teleop controllers")
        markers_manager = MarkersManager()
        floating_controller = FloatingRigidBodyController()
        ik_controller = RobotIKController()
        grasp_controller = GraspController()
        locomotion_controller = LocomotionController()
        teleop_manager = TeleopManager()
        teleop_manager.set_markers_manager(markers_manager)
        teleop_manager.set_floating_controller(floating_controller)
        teleop_manager.set_ik_controller(ik_controller)
        teleop_manager.set_grasp_controller(grasp_controller)
        teleop_manager.set_locomotion_controller(locomotion_controller)

        print("[TeleopDemo][Setup] Load profile")
        profile_path = Path(get_builtin_teleop_profiles_dir()) / profile_name
        profile, errors = load_teleop_profile(str(profile_path))
        if profile is None or errors:
            print(f"[TeleopDemo][Setup]   Could not load profile '{profile_path}': {'; '.join(errors)}, exiting")
            return
        print(f"[TeleopDemo][Setup]   Loaded: {profile_path}")

        print("[TeleopDemo][Setup] Configure motion / grasp")
        sides_state = build_sides_state(profile, teleop_manager, floating_controller, ik_controller, grasp_controller)
        if sides_state is None:
            return
        active_sides = list(sides_state.keys())
        print(
            "[TeleopDemo][Setup]   Active sides: "
            + ", ".join(f"{side}={state['motion_type']}" for side, state in sides_state.items())
        )

        print("[TeleopDemo][Setup] Configure session")
        if not configure_session(profile, teleop_manager, markers_manager, active_sides):
            return
        if not configure_locomotion(profile, teleop_manager, locomotion_controller):
            return

        print("[TeleopDemo][Setup] Cache target origins")
        if not cache_target_origins(stage, sides_state):
            return

        if not await place_markers_at_start(markers_manager, sides_state):
            return

        if capture_sdg:
            sdg_capture_render_products = setup_sdg_render_products(
                stage,
                camera_paths,
                capture_resolution,
            )
            if sdg_capture_render_products:
                live_backend = rep.backends.get("DiskBackend")
                live_backend.initialize(output_dir=str(scenario_output_dir / "sdg_live"))
                writer_live = rep.writers.get("BasicWriter")
                writer_live.initialize(backend=live_backend, rgb=True)
                writer_live.attach(sdg_capture_render_products)
                print(f"[TeleopDemo][SDG]   Output: {scenario_output_dir / 'sdg_live'}")

        episode_session_path = None
        episode_index = None
        if record_and_replay_episode:
            recorder_xforms, recorder_articulations = build_recorder_targets(profile, sides_state, markers_manager)
            print("[TeleopDemo][Recorder] Open recorder")
            print(f"[TeleopDemo][Recorder]   Output: {episode_output_dir}")
            recorder = build_teleop_recorder(
                str(episode_output_dir),
                teleop_manager=teleop_manager,
                articulations=recorder_articulations,
                xforms=recorder_xforms,
                session_metadata={
                    "demo": "sdg_teleop_demo",
                    "scenario": scenario_name,
                    "stage_url": stage_url,
                    "profile_name": profile_name,
                },
                file_prefix=scenario_name,
            )
            episode_session_path = recorder.open_session()
            episode_index = recorder.start_episode(metadata={"scenario": scenario_name, "profile_name": profile_name})
            episode_active = True
            print(f"[TeleopDemo][Recorder]   Recording episode_{episode_index:05d}: {episode_session_path}")
        else:
            print("[TeleopDemo][Recorder] Disabled in scenario, skipping record + replay")

        print("[TeleopDemo] Start timeline")
        timeline.play()
        timeline_started = True
        if not await wait_for_motion_controllers(sides_state):
            print("[TeleopDemo][Setup]   Motion controllers did not activate after 5 frames, exiting")
            return

        locomotion_enabled = bool(profile.locomotion.enabled)
        if not locomotion_enabled:
            print("[TeleopDemo][Setup]   Locomotion disabled in profile, skipping carry setup")
        elif locomotion_controller.carries_tracking_space_implicitly:
            print("[TeleopDemo][Setup]   Locomotion carries tracking origin implicitly")
        else:
            print("[TeleopDemo][Setup]   Enable locomotion carry")
            teleop_manager.set_debug_button("left", "primary_click", True)
            await app_utils.update_app_async()
            teleop_manager.set_debug_button("left", "primary_click", False)
            await app_utils.update_app_async()
            print("[TeleopDemo][Setup]     Locomotion carry flag toggled")

        print(f"[TeleopDemo][Settle] Start: {start_settle_frames} frames (timeline running)")
        await settle_at_start_pose(sides_state, start_settle_frames, target_min_distance_error)

        reach_summary = ", ".join(
            f"{side.capitalize()}={list(state['reach_offset'])}" for side, state in sides_state.items()
        )
        print(f"[TeleopDemo][Motion] Reach: {reach_summary}")
        await move_markers_to_asset_relative_target(
            markers_manager,
            sides_state,
            "reach_offset",
            reach_motion_frames,
            reach_settle_frames,
            target_min_distance_error,
            "Reach",
        )
        sdg_capture_index = await capture_live_sdg_action_image(
            "reach",
            recorder,
            writer_live,
            sdg_capture_render_products,
            sdg_capture_index,
            capture_rt_subframes,
            replay_sdg_capture_points,
        )

        if locomotion_enabled:
            pre_grasp_summary = ", ".join(
                f"{side.capitalize()}={list(state['pre_grasp_offset'])}" for side, state in sides_state.items()
            )
            print(f"[TeleopDemo][Locomotion] Pre-grasp: {pre_grasp_summary}")
            if not await locomote_to_asset_relative_target(
                markers_manager,
                teleop_manager,
                sides_state,
                "pre_grasp_offset",
                target_min_distance_error,
                max_locomotion_steps,
                locomotion_settle_frames,
            ):
                return
            sdg_capture_index = await capture_live_sdg_action_image(
                "locomotion",
                recorder,
                writer_live,
                sdg_capture_render_products,
                sdg_capture_index,
                capture_rt_subframes,
                replay_sdg_capture_points,
            )
        else:
            print("[TeleopDemo][Locomotion] Skipped (disabled in profile)")

        print("[TeleopDemo][Grasp] Close grippers")
        await close_grasps(teleop_manager, sides_state, grasp_settle_frames)
        sdg_capture_index = await capture_live_sdg_action_image(
            "grasp",
            recorder,
            writer_live,
            sdg_capture_render_products,
            sdg_capture_index,
            capture_rt_subframes,
            replay_sdg_capture_points,
        )

        lift_summary = ", ".join(
            f"{side.capitalize()}={list(state['lift_offset'])}" for side, state in sides_state.items()
        )
        print(f"[TeleopDemo][Motion] Lift: {lift_summary}")
        await move_markers_to_asset_relative_target(
            markers_manager,
            sides_state,
            "lift_offset",
            lift_motion_frames,
            lift_settle_frames,
            target_min_distance_error,
            "Lift",
        )
        sdg_capture_index = await capture_live_sdg_action_image(
            "lift",
            recorder,
            writer_live,
            sdg_capture_render_products,
            sdg_capture_index,
            capture_rt_subframes,
            replay_sdg_capture_points,
        )

        drop_summary = ", ".join(
            f"{side.capitalize()}={list(state['drop_offset'])}" for side, state in sides_state.items()
        )
        print(f"[TeleopDemo][Motion] Drop: {drop_summary}")
        if not await move_markers_to_drop_target(
            markers_manager,
            sides_state,
            drop_motion_frames,
            drop_settle_frames,
            target_min_distance_error,
        ):
            return
        print("[TeleopDemo][Grasp] Open grippers")
        await open_grasps(teleop_manager, sides_state, release_settle_frames)
        sdg_capture_index = await capture_live_sdg_action_image(
            "drop",
            recorder,
            writer_live,
            sdg_capture_render_products,
            sdg_capture_index,
            capture_rt_subframes,
            replay_sdg_capture_points,
        )
        if writer_live is not None:
            await rep.orchestrator.wait_until_complete_async()
            writer_live.detach()
            print("[TeleopDemo][SDG]   Live writer detached")
            writer_live = None

        # End the live phase explicitly so the recorder / timeline / teleop
        # session are released before the replay phase.
        if recorder is not None:
            recorder.end_episode(success=True, metadata={"scenario": scenario_name})
            episode_active = False
            recorder.close_session()
            recorder = None
        timeline.stop()
        timeline_started = False
        print("[TeleopDemo][Recorder] End live teleop phase")
        teleop_manager.set_debug_tracking(False)
        teleop_manager.destroy()
        teleop_manager = None
        if episode_session_path is not None:
            print(f"[TeleopDemo][Recorder] Recorded session: {episode_session_path}")

        if not record_and_replay_episode:
            print("[TeleopDemo][Replay] Skipped (disabled in scenario)")
        else:
            if replay_sdg_capture_points:
                print(
                    "[TeleopDemo][SDG] Live captures mapped to replay frames: "
                    + ", ".join(
                        f"{capture.action_name}=frame{capture.recorded_frame_index}"
                        for capture in replay_sdg_capture_points
                    )
                )
            else:
                print("[TeleopDemo][SDG] Live captures mapped to replay frames: None")

            print("[TeleopDemo][Replay] Replay")
            # Replay revisits live SDG action endpoints; it does not produce
            # one image per replay frame.
            replay_sdg_captures_by_frame: dict[int, list[SdgReplayCapture]] = {}
            if capture_sdg and replay_sdg_capture_points and sdg_capture_render_products:
                replay_backend = rep.backends.get("DiskBackend")
                replay_backend.initialize(output_dir=str(scenario_output_dir / "sdg_replay"))
                writer_replay = rep.writers.get("BasicWriter")
                writer_replay.initialize(backend=replay_backend, rgb=True)
                writer_replay.attach(sdg_capture_render_products)
                print(f"[TeleopDemo][SDG]   Output: {scenario_output_dir / 'sdg_replay'}")
                sdg_capture_index = 0
                for capture in replay_sdg_capture_points:
                    replay_sdg_captures_by_frame.setdefault(capture.recorded_frame_index, []).append(capture)

            try:
                replayer = EpisodeReplayer(
                    episode_session_path,
                    policy=ReplayPolicy(strictness="best_effort"),
                    pose_backend="usd",
                )
                episodes = replayer.list_episodes()
                print(f"[TeleopDemo][Replay]   Episodes: {episodes}")
                replay_frames = replayer.num_frames(episode_index)
                print(f"[TeleopDemo][Replay]   Start episode_{episode_index:05d}: {replay_frames} frames")
                await replayer.prepare_episode_async(episode=episode_index)
                replay_log_interval = max(1, replay_frames // 10)
                for replay_frame_index in range(replay_frames):
                    replayer.apply_frame(replay_frame_index)
                    if replay_frame_index % replay_log_interval == 0 or replay_frame_index == replay_frames - 1:
                        print(f"[TeleopDemo][Replay]   Applied frame {replay_frame_index}/{replay_frames - 1}")
                    for capture_point in replay_sdg_captures_by_frame.get(replay_frame_index, []):
                        sdg_capture_index = await capture_replayed_sdg_action_image(
                            capture_point,
                            writer_replay,
                            sdg_capture_render_products,
                            sdg_capture_index,
                            capture_rt_subframes,
                        )
                    if replay_frame_index not in replay_sdg_captures_by_frame:
                        await app_utils.update_app_async()
                print(f"[TeleopDemo][Replay]   Applied through frame {replay_frames - 1}/{replay_frames - 1}")
                replayer.apply_frame(0)
                await app_utils.update_app_async()
                replayer.stop_replay()
                await app_utils.update_app_async()
                print("[TeleopDemo][Replay]   Reverted to first recorded frame")
            except Exception as exc:
                print(f"[TeleopDemo][Replay]   Skipped: {exc}")
        if writer_replay is not None:
            await rep.orchestrator.wait_until_complete_async()
            writer_replay.detach()
            print("[TeleopDemo][SDG]   Replay writer detached")
            writer_replay = None
        for render_product in sdg_capture_render_products:
            render_product.destroy()
        sdg_capture_render_products = []
    except Exception as exc:
        print(f"[TeleopDemo][Cleanup]   Aborting scenario '{scenario_name}': {exc}")
    finally:
        print("[TeleopDemo][Cleanup] Cleanup")
        await cleanup_teleop_pick_and_place_async(
            scenario_name=scenario_name,
            markers_manager=markers_manager,
            teleop_manager=teleop_manager,
            recorder=recorder,
            episode_active=episode_active,
            timeline=timeline,
            timeline_started=timeline_started,
            sdg_capture_writer=writer_replay or writer_live,
            sdg_capture_render_products=sdg_capture_render_products,
            replayer=replayer,
        )


class TestTeleopSDGPickAndPlace(omni.kit.test.AsyncTestCase):
    """Run the teleop pick-and-place demo once per scenario in debug mode."""

    # Per-pixel mean tolerance when comparing captured frames against golden
    # images. Scenes are deterministic but RTX denoising / lighting jitter can
    # introduce small per-pixel variation, so a moderate budget is used.
    MEAN_DIFF_TOLERANCE = 10

    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_floating_xarm_dex3(self):
        """Bimanual floating teleop: xArm gripper (left) + Dex3 gripper (right)."""
        scenario_config = {
            "name": "floating_xarm_dex3",
            "stage_url": "/Isaac/Samples/Replicator/Teleop/teleop_scenario_floating_xarm_dex3.usd",
            "profile_name": "floating_xarm_dex3.yaml",
            "sides": {
                "left": {
                    "asset_path": "/World/teleop_env/teleop_assets/Assets/_06_mustard_bottle",
                    "drop_target_path": "/World/teleop_env/teleop_tables/TableMain/SM_Crate_A08_Blue_01_physics",
                    "tcp_offset_world": (-0.10, 0.0, 0.0),
                    "start_offset": (0.0, 0.0, 0.5),
                    "reach_offset": (-0.5, 0.0, 0.0),
                    "pre_grasp_offset": (0.0, 0.0, 0.0),
                    "lift_offset": (0.0, 0.0, 0.5),
                    "drop_offset": (0.0, 0.0, 0.5),
                },
                "right": {
                    "asset_path": "/World/teleop_env/teleop_assets/Assets/_05_tomato_soup_can",
                    "drop_target_path": "/World/teleop_env/teleop_tables/TableMain/SM_Crate_A07_Yellow_01_physics",
                    "tcp_offset_world": (-0.05, -0.05, 0.0),
                    "start_offset": (0.0, 0.0, 0.5),
                    "reach_offset": (-0.5, 0.0, 0.0),
                    "pre_grasp_offset": (0.0, 0.0, 0.0),
                    "lift_offset": (0.0, 0.0, 0.5),
                    "drop_offset": (0.0, 0.0, 0.5),
                },
            },
            "reach_motion_frames": 60,
            "lift_motion_frames": 60,
            "drop_motion_frames": 60,
            "start_settle_frames": 30,
            "reach_settle_frames": 10,
            "locomotion_settle_frames": 10,
            "grasp_settle_frames": 30,
            "lift_settle_frames": 10,
            "drop_settle_frames": 10,
            "release_settle_frames": 60,
            "target_min_distance_error": 0.05,
            "max_locomotion_steps": 150,
            "capture_sdg": True,
            "record_and_replay_episode": True,
            "capture_resolution": (400, 400),
            "capture_rt_subframes": 16,
            "camera_paths": [
                "/World/teleop_xarm_dex3/gripper_origin_xform/xarm_gripper_root_xform/xarm_gripper/xarm_gripper_base_link/xarm_view_cam",
                "/World/teleop_xarm_dex3/gripper_origin_xform/dex3_1_r_root_xform/dex3_1_r/right_hand_palm_link/dex3_view_cam",
            ],
        }
        await run_teleop_pick_and_place_async(scenario_config)

        # 2 cameras x 5 captures (reach + locomotion + grasp + lift + drop) per phase.
        golden_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "_out_teleop_floating_xarm_dex3")
        for phase in ("sdg_live", "sdg_replay"):
            out_dir = os.path.join(os.getcwd(), "_out_teleop_floating_xarm_dex3", phase)
            self.assertTrue(
                validate_folder_contents(path=out_dir, expected_counts={"png": 10}, recursive=True),
                f"Folder contents validation failed ({phase}). Output dir: {out_dir}",
            )
            for camera_dir in ("rp_dex3_view_cam", "rp_xarm_view_cam"):
                golden_camera_dir = os.path.join(golden_dir, camera_dir, "rgb")
                test_camera_dir = os.path.join(out_dir, camera_dir, "rgb")
                result = compare_images_in_directories(
                    golden_dir=golden_camera_dir,
                    test_dir=test_camera_dir,
                    path_pattern=r"\.png$",
                    allclose_rtol=None,
                    allclose_atol=None,
                    mean_tolerance=self.MEAN_DIFF_TOLERANCE,
                    print_all_stats=False,
                )
                self.assertTrue(
                    result["all_passed"],
                    f"Image comparison failed ({phase}/{camera_dir}). Output dir: {test_camera_dir}",
                )

    async def test_floating_xarm(self):
        """Solo floating teleop: a single xArm gripper on the right side."""
        scenario_config = {
            "name": "floating_xarm",
            "stage_url": "/Isaac/Samples/Replicator/Teleop/teleop_scenario_floating_xarm.usd",
            "profile_name": "floating_xarm.yaml",
            "sides": {
                "right": {
                    "asset_path": "/World/teleop_env/teleop_assets/Assets/_05_tomato_soup_can",
                    "drop_target_path": "/World/teleop_env/teleop_tables/TableMain/SM_Crate_A07_Yellow_01_physics",
                    "tcp_offset_world": (-0.05, 0.0, 0.0),
                    "start_offset": (0.0, 0.0, 0.5),
                    "reach_offset": (-0.5, 0.0, 0.0),
                    "pre_grasp_offset": (0.0, 0.0, 0.0),
                    "lift_offset": (0.0, 0.0, 0.5),
                    "drop_offset": (0.0, 0.0, 0.5),
                },
            },
            "reach_motion_frames": 60,
            "lift_motion_frames": 60,
            "drop_motion_frames": 60,
            "start_settle_frames": 30,
            "reach_settle_frames": 10,
            "locomotion_settle_frames": 10,
            "grasp_settle_frames": 30,
            "lift_settle_frames": 10,
            "drop_settle_frames": 10,
            "release_settle_frames": 60,
            "target_min_distance_error": 0.1,
            "max_locomotion_steps": 150,
            "capture_sdg": True,
            "record_and_replay_episode": True,
            "capture_resolution": (400, 400),
            "capture_rt_subframes": 16,
            "camera_paths": [
                "/World/teleop_xarm/gripper_origin_xform/xarm_gripper_root_xform/xarm_gripper/xarm_gripper_base_link/xarm_view_cam",
            ],
        }
        await run_teleop_pick_and_place_async(scenario_config)

        # 1 camera x 5 captures (reach + locomotion + grasp + lift + drop) per phase.
        golden_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "_out_teleop_floating_xarm")
        for phase in ("sdg_live", "sdg_replay"):
            out_dir = os.path.join(os.getcwd(), "_out_teleop_floating_xarm", phase)
            self.assertTrue(
                validate_folder_contents(path=out_dir, expected_counts={"png": 5}, recursive=True),
                f"Folder contents validation failed ({phase}). Output dir: {out_dir}",
            )
            result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=out_dir,
                path_pattern=r"\.png$",
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=self.MEAN_DIFF_TOLERANCE,
                print_all_stats=False,
            )
            self.assertTrue(result["all_passed"], f"Image comparison failed ({phase}). Output dir: {out_dir}")

    async def test_ik_dual_ur3_xarm_dex3(self):
        """Bimanual IK teleop: dual UR3 arms with xArm gripper (left) + Dex3 (right)."""
        scenario_config = {
            "name": "ik_dual_ur3_xarm_dex3",
            "stage_url": "/Isaac/Samples/Replicator/Teleop/teleop_scenario_dual_ur3_xarm_dex3.usd",
            "profile_name": "ik_dual_ur3_xarm_dex3.yaml",
            "sides": {
                "left": {
                    "asset_path": "/World/teleop_env/teleop_assets/Assets/_06_mustard_bottle",
                    "drop_target_path": "/World/teleop_env/teleop_tables/TableMain/SM_Crate_A08_Blue_01_physics",
                    "tcp_offset_world": (-0.05, 0.0, 0.0),
                    "start_offset": (-0.35, 0.0, 0.3),
                    "reach_offset": (-0.45, 0.0, 0.0),
                    "pre_grasp_offset": (0.0, 0.0, 0.0),
                    "lift_offset": (0.0, 0.0, 0.35),
                    "drop_offset": (-0.1, -0.1, 0.35),
                },
                "right": {
                    "asset_path": "/World/teleop_env/teleop_assets/Assets/_05_tomato_soup_can",
                    "drop_target_path": "/World/teleop_env/teleop_tables/TableMain/SM_Crate_A07_Yellow_01_physics",
                    "tcp_offset_world": (-0.02, -0.04, 0.0),
                    "start_offset": (-0.35, 0.0, 0.3),
                    "reach_offset": (-0.45, 0.0, 0.0),
                    "pre_grasp_offset": (0.0, 0.0, 0.0),
                    "lift_offset": (0.0, 0.0, 0.35),
                    "drop_offset": (-0.2, 0.1, 0.35),
                },
            },
            "reach_motion_frames": 90,
            "lift_motion_frames": 90,
            "drop_motion_frames": 90,
            "start_settle_frames": 90,
            "reach_settle_frames": 10,
            "locomotion_settle_frames": 10,
            "grasp_settle_frames": 30,
            "lift_settle_frames": 10,
            "drop_settle_frames": 10,
            "release_settle_frames": 60,
            "target_min_distance_error": 0.08,
            "max_locomotion_steps": 200,
            "capture_sdg": True,
            "record_and_replay_episode": True,
            "capture_resolution": (400, 400),
            "capture_rt_subframes": 16,
            "camera_paths": [
                "/World/teleop_dual_ur3_xarm_dex3/dual_arm/left_arm_ur3e_xarm/xarm_gripper/xarm_gripper_base_link/xarm_view_cam",
                "/World/teleop_dual_ur3_xarm_dex3/dual_arm/right_arm_ur3e_dex3/dex3_1_r/right_hand_palm_link/dex3_view_cam",
            ],
        }
        await run_teleop_pick_and_place_async(scenario_config)

        # 2 cameras x 5 captures (reach + locomotion + grasp + lift + drop) per phase.
        golden_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "data", "_out_teleop_ik_dual_ur3_xarm_dex3"
        )
        for phase in ("sdg_live", "sdg_replay"):
            out_dir = os.path.join(os.getcwd(), "_out_teleop_ik_dual_ur3_xarm_dex3", phase)
            self.assertTrue(
                validate_folder_contents(path=out_dir, expected_counts={"png": 10}, recursive=True),
                f"Folder contents validation failed ({phase}). Output dir: {out_dir}",
            )
            for camera_dir in ("rp_dex3_view_cam", "rp_xarm_view_cam"):
                golden_camera_dir = os.path.join(golden_dir, camera_dir, "rgb")
                test_camera_dir = os.path.join(out_dir, camera_dir, "rgb")
                result = compare_images_in_directories(
                    golden_dir=golden_camera_dir,
                    test_dir=test_camera_dir,
                    path_pattern=r"\.png$",
                    allclose_rtol=None,
                    allclose_atol=None,
                    mean_tolerance=self.MEAN_DIFF_TOLERANCE,
                    print_all_stats=False,
                )
                self.assertTrue(
                    result["all_passed"],
                    f"Image comparison failed ({phase}/{camera_dir}). Output dir: {test_camera_dir}",
                )

    async def test_ik_solo_ur3_xarm(self):
        """Solo IK teleop: single UR3 arm with an xArm gripper on the right."""
        scenario_config = {
            "name": "ik_solo_ur3_xarm",
            "stage_url": "/Isaac/Samples/Replicator/Teleop/teleop_scenario_solo_ur3_xarm.usd",
            "profile_name": "ik_solo_ur3_xarm.yaml",
            "sides": {
                "right": {
                    "asset_path": "/World/teleop_env/teleop_assets/Assets/_05_tomato_soup_can",
                    "drop_target_path": "/World/teleop_env/teleop_tables/TableMain/SM_Crate_A07_Yellow_01_physics",
                    "tcp_offset_world": (-0.05, 0.0, 0.0),
                    "start_offset": (-0.5, 0.0, 0.3),
                    "reach_offset": (-0.45, 0.0, 0.0),
                    "pre_grasp_offset": (0.0, 0.0, 0.0),
                    "lift_offset": (-0.2, 0.0, 0.3),
                    "drop_offset": (-0.25, 0.05, 0.3),
                },
            },
            "reach_motion_frames": 90,
            "lift_motion_frames": 90,
            "drop_motion_frames": 90,
            "start_settle_frames": 60,
            "reach_settle_frames": 10,
            "locomotion_settle_frames": 10,
            "grasp_settle_frames": 30,
            "lift_settle_frames": 10,
            "drop_settle_frames": 10,
            "release_settle_frames": 60,
            "target_min_distance_error": 0.08,
            "max_locomotion_steps": 150,
            "capture_sdg": True,
            "record_and_replay_episode": True,
            "capture_resolution": (400, 400),
            "capture_rt_subframes": 16,
            "camera_paths": [
                "/World/teleop_solo_ur3_xarm/solo_arm/xarm_gripper/xarm_gripper_base_link/xarm_view_cam",
            ],
        }
        await run_teleop_pick_and_place_async(scenario_config)

        # 1 camera x 5 captures (reach + locomotion + grasp + lift + drop) per phase.
        golden_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "_out_teleop_ik_solo_ur3_xarm")
        for phase in ("sdg_live", "sdg_replay"):
            out_dir = os.path.join(os.getcwd(), "_out_teleop_ik_solo_ur3_xarm", phase)
            self.assertTrue(
                validate_folder_contents(path=out_dir, expected_counts={"png": 5}, recursive=True),
                f"Folder contents validation failed ({phase}). Output dir: {out_dir}",
            )
            result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=out_dir,
                path_pattern=r"\.png$",
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=self.MEAN_DIFF_TOLERANCE,
                print_all_stats=False,
            )
            self.assertTrue(result["all_passed"], f"Image comparison failed ({phase}). Output dir: {out_dir}")

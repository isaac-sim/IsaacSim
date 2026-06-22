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

"""Floating rigid-body velocity controller for VR teleop.

Uses an explicitly authored free ``RigidPrim`` as the control handle. The
controller does not modify articulations or create physics bodies at runtime.
"""

import math
from collections.abc import Generator

import numpy as np
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.experimental.utils.stage import get_current_stage
from pxr import PhysxSchema, Usd, UsdPhysics

from .._xform_utils import read_world_pose_arrays
from ..coordinate_utils import CoordinateSystem
from ._utils import (
    DEFAULT_ROTATION_OFFSET_DEG,
    quat_mul_xyzw,
    rotation_offset_quat_xyzw,
)


class FloatingRigidBodyController:
    """Velocity-based floating rigid-body controller for VR teleop.

    Each side drives an explicit rigid-body handle selected by the user.
    If that handle is fixed-jointed to some payload, the payload follows,
    but this controller only commands the rigid body itself.

    Args:
        target_coordinate_system: Value for target coordinate system.
    """

    def __init__(self, target_coordinate_system: CoordinateSystem = CoordinateSystem.ISAAC_SIM) -> None:
        self._target_coordinate_system = target_coordinate_system
        self._left_end_effector_path: str | None = None
        self._right_end_effector_path: str | None = None
        self._enabled = False

        self._left_position_kp: float = 15.0
        self._left_position_kd: float = 0.5
        self._left_orientation_kp: float = 15.0
        self._left_orientation_kd: float = 0.2
        self._right_position_kp: float = 15.0
        self._right_position_kd: float = 0.5
        self._right_orientation_kp: float = 15.0
        self._right_orientation_kd: float = 0.2
        self._left_target_rot_x_deg: float = DEFAULT_ROTATION_OFFSET_DEG
        self._left_target_rot_y_deg: float = DEFAULT_ROTATION_OFFSET_DEG
        self._left_target_rot_z_deg: float = DEFAULT_ROTATION_OFFSET_DEG
        self._right_target_rot_x_deg: float = DEFAULT_ROTATION_OFFSET_DEG
        self._right_target_rot_y_deg: float = DEFAULT_ROTATION_OFFSET_DEG
        self._right_target_rot_z_deg: float = DEFAULT_ROTATION_OFFSET_DEG

        self._left_target_pos: tuple[float, float, float] | None = None
        self._left_target_orient: tuple[float, float, float, float] | None = None
        self._right_target_pos: tuple[float, float, float] | None = None
        self._right_target_orient: tuple[float, float, float, float] | None = None

        self._left_initial_pose: tuple[tuple[float, float, float], tuple[float, float, float, float]] | None = None
        self._right_initial_pose: tuple[tuple[float, float, float], tuple[float, float, float, float]] | None = None
        self._left_quat_offset: tuple[float, float, float, float] | None = None
        self._right_quat_offset: tuple[float, float, float, float] | None = None

        self._left_rigid_prim: RigidPrim | None = None
        self._right_rigid_prim: RigidPrim | None = None
        self._left_physics_path: str | None = None
        self._right_physics_path: str | None = None
        self._left_reset_xform_ops: bool = False
        self._right_reset_xform_ops: bool = False
        self._left_active: bool = False
        self._right_active: bool = False

        self._pos_buf = np.zeros((1, 3), dtype=np.float32)
        self._orient_buf = np.zeros((1, 4), dtype=np.float32)
        self._vel_buf = np.zeros((1, 3), dtype=np.float32)
        self._ang_vel_buf = np.zeros((1, 3), dtype=np.float32)
        self._zero_vel_buf = np.zeros((1, 3), dtype=np.float32)

    # =========================================================================
    # Configuration
    # =========================================================================

    def set_coordinate_system(self, target_coordinate_system: CoordinateSystem) -> None:
        """Set the source coordinate system for input VR wrist pose data.

        Args:
            target_coordinate_system: Value for target coordinate system.
        """
        self._target_coordinate_system = target_coordinate_system

    def set_target_rotation_offsets(
        self,
        side: str,
        x_deg: float = DEFAULT_ROTATION_OFFSET_DEG,
        y_deg: float = DEFAULT_ROTATION_OFFSET_DEG,
        z_deg: float = DEFAULT_ROTATION_OFFSET_DEG,
    ) -> None:
        """Set the local-frame XYZ target rotation offsets for one side.

        Args:
            side: Value for side.
            x_deg: Value for x deg.
            y_deg: Value for y deg.
            z_deg: Value for z deg.
        """
        side = side.lower()
        if side == "right":
            self._right_target_rot_x_deg = float(x_deg)
            self._right_target_rot_y_deg = float(y_deg)
            self._right_target_rot_z_deg = float(z_deg)
        else:
            self._left_target_rot_x_deg = float(x_deg)
            self._left_target_rot_y_deg = float(y_deg)
            self._left_target_rot_z_deg = float(z_deg)

    def _get_target_rotation_offsets(self, side: str) -> tuple[float, float, float]:
        if side == "right":
            return self._right_target_rot_x_deg, self._right_target_rot_y_deg, self._right_target_rot_z_deg
        return self._left_target_rot_x_deg, self._left_target_rot_y_deg, self._left_target_rot_z_deg

    @staticmethod
    def _capture_world_pose(
        prim: Usd.Prim,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        """Extract position and orientation (xyzw) from a prim's world transform.

        Args:
            prim: Value for prim.

        Returns:
            The requested value.
        """
        pos_arr, quat_arr = read_world_pose_arrays(prim)
        pos = pos_arr.reshape(-1, 3)[0]
        quat = quat_arr.reshape(-1, 4)[0]
        return (
            (float(pos[0]), float(pos[1]), float(pos[2])),
            (float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])),
        )

    def _apply_rotation_offset(
        self, orient: tuple[float, float, float, float] | None, side: str
    ) -> tuple[float, float, float, float] | None:
        """Composes target orientation with the cached rotation offset.

        Args:
            orient: Value for orient.
            side: Value for side.

        Returns:
            The requested value.
        """
        if orient is None:
            return None
        quat_offset = self._left_quat_offset if side == "left" else self._right_quat_offset
        if quat_offset is None:
            return orient

        q = quat_mul_xyzw(orient, quat_offset)
        n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if n > 0.0:
            return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)
        return q

    def _apply_target_rotation_offsets(
        self, orient: tuple[float, float, float, float] | None, side: str
    ) -> tuple[float, float, float, float] | None:
        """Apply the configured local-frame XYZ rotation offsets to the target orientation.

        Args:
            orient: Value for orient.
            side: Value for side.

        Returns:
            The requested value.
        """
        if orient is None:
            return None
        rot_x_deg, rot_y_deg, rot_z_deg = self._get_target_rotation_offsets(side)
        rotation_offset = rotation_offset_quat_xyzw(rot_x_deg, rot_y_deg, rot_z_deg)
        return quat_mul_xyzw(orient, rotation_offset)

    def _compose_target_orientation(
        self, orient: tuple[float, float, float, float] | None, side: str
    ) -> tuple[float, float, float, float] | None:
        rotated_orient = self._apply_target_rotation_offsets(orient, side)
        return self._apply_rotation_offset(rotated_orient, side)

    def set_gains(
        self,
        position_kp: float = 15.0,
        position_kd: float = 0.5,
        orientation_kp: float = 15.0,
        orientation_kd: float = 0.2,
        side: str | None = None,
    ) -> None:
        """Set the PD control gains.

        Args:
            position_kp: Value for position kp.
            position_kd: Value for position kd.
            orientation_kp: Value for orientation kp.
            orientation_kd: Value for orientation kd.
            side: Value for side.
        """
        if side is None:
            self._set_side_gains("left", position_kp, position_kd, orientation_kp, orientation_kd)
            self._set_side_gains("right", position_kp, position_kd, orientation_kp, orientation_kd)
        else:
            self._set_side_gains(side, position_kp, position_kd, orientation_kp, orientation_kd)

    def _set_side_gains(
        self,
        side: str,
        position_kp: float,
        position_kd: float,
        orientation_kp: float,
        orientation_kd: float,
    ) -> None:
        side = side.lower()
        if side == "left":
            self._left_position_kp = position_kp
            self._left_position_kd = position_kd
            self._left_orientation_kp = orientation_kp
            self._left_orientation_kd = orientation_kd
        elif side == "right":
            self._right_position_kp = position_kp
            self._right_position_kd = position_kd
            self._right_orientation_kp = orientation_kp
            self._right_orientation_kd = orientation_kd

    def _get_side_gains(self, side: str) -> tuple[float, float, float, float]:
        side = side.lower()
        if side == "right":
            return (
                self._right_position_kp,
                self._right_position_kd,
                self._right_orientation_kp,
                self._right_orientation_kd,
            )
        return (
            self._left_position_kp,
            self._left_position_kd,
            self._left_orientation_kp,
            self._left_orientation_kd,
        )

    @staticmethod
    def _is_finite_vec(vec: tuple[float, ...] | np.ndarray | None) -> bool:
        if vec is None:
            return False
        return bool(np.isfinite(np.asarray(vec, dtype=np.float64)).all())

    def _has_finite_state(
        self,
        current_pos: tuple[float, float, float],
        current_orient: tuple[float, float, float, float] | None,
        cur_lin_vel: tuple[float, float, float],
        cur_ang_vel: tuple[float, float, float],
    ) -> bool:
        return (
            self._is_finite_vec(current_pos)
            and self._is_finite_vec(current_orient)
            and self._is_finite_vec(cur_lin_vel)
            and self._is_finite_vec(cur_ang_vel)
        )

    @staticmethod
    def _has_required_xform_ops(prim: Usd.Prim) -> bool:
        props = set(prim.GetPropertyNames())
        return all(op in props for op in ("xformOp:translate", "xformOp:orient", "xformOp:scale"))

    def _clear_runtime_handle(self, side: str) -> None:
        if side == "left":
            self._left_rigid_prim = None
        else:
            self._right_rigid_prim = None

    def _get_runtime_handle(self, side: str) -> RigidPrim | None:
        return self._left_rigid_prim if side == "left" else self._right_rigid_prim

    def _ensure_runtime_handle(self, side: str) -> RigidPrim | None:
        """Create the runtime ``RigidPrim`` lazily after physics is live.

        Args:
            side: Value for side.

        Returns:
            The requested value.
        """
        existing = self._get_runtime_handle(side)
        if existing is not None:
            return existing

        physics_path = self._left_physics_path if side == "left" else self._right_physics_path
        reset_xform_ops = self._left_reset_xform_ops if side == "left" else self._right_reset_xform_ops
        if not physics_path:
            return None

        try:
            rigid_prim = RigidPrim(physics_path, reset_xform_op_properties=reset_xform_ops)
        except Exception:
            return None

        if side == "left":
            self._left_rigid_prim = rigid_prim
        else:
            self._right_rigid_prim = rigid_prim
        return rigid_prim

    # =========================================================================
    # Controller lifecycle
    # =========================================================================

    def _prepare_side(self, stage: Usd.Stage, prim_path: str, side: str) -> bool:
        """Validate the rigid body selection and caches its initial world pose.

        Args:
            stage: Value for stage.
            prim_path: Value for prim path.
            side: Value for side.

        Returns:
            The requested value.
        """
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            print(f"[Teleop][FloatingRigidBody] Control rigid body not found: '{prim_path}'")
            return False

        pos, orient_xyzw = self._capture_world_pose(prim)
        is_identity = abs(abs(orient_xyzw[3]) - 1.0) < 1e-6
        reset_xform_ops = not self._has_required_xform_ops(prim)

        if side == "left":
            self._left_initial_pose = (pos, orient_xyzw)
            self._left_quat_offset = None if is_identity else orient_xyzw
            self._left_physics_path = prim_path
            self._left_reset_xform_ops = reset_xform_ops
        else:
            self._right_initial_pose = (pos, orient_xyzw)
            self._right_quat_offset = None if is_identity else orient_xyzw
            self._right_physics_path = prim_path
            self._right_reset_xform_ops = reset_xform_ops

        euler = self._quat_xyzw_to_euler_deg(orient_xyzw)
        offset_str = f"rotation offset ({euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}) deg"
        reset_str = ", xformOps reset on enable" if reset_xform_ops else ""
        print(
            f"[Teleop][FloatingRigidBody] {side.capitalize()} prepared control rigid body: '{prim_path}' "
            f"({offset_str}{reset_str})"
        )
        return True

    # =========================================================================
    # Per-side lifecycle: set_prim_path / validate / configure / start / stop / destroy
    # =========================================================================

    def set_prim_path(self, side: str, path: str | None) -> None:
        """Set the prim path for a side. Destroys existing controller if path changed.

        Args:
            side: Value for side.
            path: Value for path.
        """
        side = side.lower()
        current = self._left_end_effector_path if side == "left" else self._right_end_effector_path
        if current == path:
            return
        self.destroy(side)
        if side == "left":
            self._left_end_effector_path = path
        else:
            self._right_end_effector_path = path

    def get_prim_path(self, side: str) -> str | None:
        """Return the stored prim path for a side.

        Args:
            side: Value for side.

        Returns:
            The requested value.
        """
        return self._left_end_effector_path if side.lower() == "left" else self._right_end_effector_path

    def validate(self, side: str) -> tuple[bool, str]:
        """Validate the stored prim path for the rigid-body controller.

        The selected rigid body may carry an attached articulation payload; the
        controller only validates and drives the selected rigid body itself.

        Args:
            side: Value for side.

        Returns:
            The requested value.
        """
        side = side.lower()
        path = self._left_end_effector_path if side == "left" else self._right_end_effector_path
        if not path or not path.strip():
            return False, "Set prim path first."

        stage = get_current_stage()
        if not stage:
            return False, "No stage available."

        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return False, f"Prim not found: {path}"

        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return False, f"'{path}' must already have RigidBodyAPI."

        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        kinematic_attr = rigid_body_api.GetKinematicEnabledAttr()
        if kinematic_attr and bool(kinematic_attr.Get()):
            return False, f"'{path}' is kinematic. Use a dynamic rigid body handle."

        warnings: list[str] = []
        if not self._has_required_xform_ops(prim):
            warnings.append("xformOps will be normalized on enable")
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            warnings.append("selected rigid body is also an articulation root; driving the rigid body directly")

        if warnings:
            return True, f"Valid ({'; '.join(warnings)}): {path}"
        return True, f"Valid: {path}"

    def configure(self, side: str) -> bool:
        """Prepare a side by validating and caching the rigid-body handle pose.

        Args:
            side: Value for side.

        Returns:
            The requested value.
        """
        side = side.lower()
        self.destroy(side)
        valid, _msg = self.validate(side)
        if not valid:
            return False

        stage = get_current_stage()
        if not stage:
            return False

        path = self._left_end_effector_path if side == "left" else self._right_end_effector_path
        return self._prepare_side(stage, path, side)

    def enable(self, side: str) -> bool:
        """Activates a side so runtime updates can lazily create the ``RigidPrim``.

        Args:
            side: Value for side.

        Returns:
            The requested value.
        """
        side = side.lower()
        if not self.is_configured(side):
            if not self.configure(side):
                return False

        physics_path = self._left_physics_path if side == "left" else self._right_physics_path
        stage = get_current_stage()
        if not stage or not physics_path:
            return False

        physics_prim = stage.GetPrimAtPath(physics_path)
        if not physics_prim or not physics_prim.IsValid():
            return False

        if not physics_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(physics_prim)

        self._clear_runtime_handle(side)
        if side == "left":
            self._left_active = True
        else:
            self._right_active = True
        self._enabled = self._left_active or self._right_active

        print(f"[Teleop][FloatingRigidBody] {side.capitalize()} activated: velocity control='{physics_path}'")
        return True

    def disable(self, side: str) -> None:
        """Deactivates a side on Stop and restores the initial handle pose.

        Args:
            side: Value for side.
        """
        side = side.lower()
        self._restore_initial_pose(side, self._get_runtime_handle(side))
        if side == "left":
            self._left_active = False
        else:
            self._right_active = False
        self._clear_runtime_handle(side)
        self._enabled = self._left_active or self._right_active

    def destroy(self, side: str) -> None:
        """Full teardown for a side: disables + clears all cached state.

        Path is preserved so the user can re-enable without re-entering it.
        Called from the UI Disable/Clear button or when the prim path changes.

        Args:
            side: Value for side.
        """
        side = side.lower()
        self.disable(side)
        if side == "left":
            self._left_physics_path = None
            self._left_target_pos = None
            self._left_target_orient = None
            self._left_initial_pose = None
            self._left_quat_offset = None
            self._left_reset_xform_ops = False
        else:
            self._right_physics_path = None
            self._right_target_pos = None
            self._right_target_orient = None
            self._right_initial_pose = None
            self._right_quat_offset = None
            self._right_reset_xform_ops = False

    def _restore_initial_pose(self, side: str, control_prim: RigidPrim | None) -> None:
        """Snaps the active control handle back to its cached initial world pose.

        Args:
            side: Value for side.
            control_prim: Value for control prim.
        """
        initial = self._left_initial_pose if side == "left" else self._right_initial_pose
        if initial is None or control_prim is None:
            return

        pos, orient_xyzw = initial
        self._pos_buf[0] = pos
        self._orient_buf[0] = (orient_xyzw[3], orient_xyzw[0], orient_xyzw[1], orient_xyzw[2])
        try:
            control_prim.set_world_poses(positions=self._pos_buf, orientations=self._orient_buf)
            control_prim.set_velocities(linear_velocities=self._zero_vel_buf, angular_velocities=self._zero_vel_buf)
        except Exception:
            pass

    def is_configured(self, side: str) -> bool:
        """True if the side has been prepared (initial pose cached, rigid body validated).

        Args:
            side: Value for side.

        Returns:
            The requested value.
        """
        side = side.lower()
        if side == "left":
            return self._left_initial_pose is not None and self._left_physics_path is not None
        return self._right_initial_pose is not None and self._right_physics_path is not None

    def is_running(self, side: str) -> bool:
        """True if floating velocity tracking is active for this side.

        Args:
            side: Value for side.

        Returns:
            The requested value.
        """
        side = side.lower()
        return self._left_active if side == "left" else self._right_active

    # =========================================================================
    # Target setting
    # =========================================================================

    def set_targets(
        self,
        left_wrist_position: tuple[float, float, float] | None = None,
        left_wrist_orientation: tuple[float, float, float, float] | None = None,
        right_wrist_position: tuple[float, float, float] | None = None,
        right_wrist_orientation: tuple[float, float, float, float] | None = None,
    ) -> None:
        """Set target poses for velocity tracking.

        Positions and orientations must already be in Isaac Sim coordinates
        (Z-up). Coordinate conversion is handled upstream.

        Args:
            left_wrist_position: Value for left wrist position.
            left_wrist_orientation: Value for left wrist orientation.
            right_wrist_position: Value for right wrist position.
            right_wrist_orientation: Value for right wrist orientation.
        """
        if not self._enabled:
            return

        if left_wrist_position is not None:
            self._left_target_pos = left_wrist_position
            self._left_target_orient = self._compose_target_orientation(left_wrist_orientation, "left")

        if right_wrist_position is not None:
            self._right_target_pos = right_wrist_position
            self._right_target_orient = self._compose_target_orientation(right_wrist_orientation, "right")

    # =========================================================================
    # Control application
    # =========================================================================

    def apply_tracking(self) -> None:
        """Apply velocity tracking to the active rigid-body handles."""
        if not self._enabled:
            return

        for side, control_prim, target_pos, target_orient in self._iter_active_sides():
            self._apply_velocity_side(side, control_prim, target_pos, target_orient)

    def _apply_velocity_side(
        self,
        side: str,
        control_prim: RigidPrim,
        target_pos: tuple[float, float, float],
        target_orient: tuple[float, float, float, float] | None,
    ) -> None:
        """Velocity tracking: PD control sets linear and angular velocity on the active control handle.

        Args:
            side: Value for side.
            control_prim: Value for control prim.
            target_pos: Value for target pos.
            target_orient: Value for target orient.
        """
        pos_kp, pos_kd, orient_kp, orient_kd = self._get_side_gains(side)

        try:
            current_pos, current_orient = self._get_pose(control_prim)
            cur_lin_vel, cur_ang_vel = self._get_velocities(control_prim)
            if not self._has_finite_state(current_pos, current_orient, cur_lin_vel, cur_ang_vel):
                print(
                    f"[Teleop][FloatingRigidBody] {side.capitalize()} velocity control skipped: "
                    "rigid body state is non-finite."
                )
                return

            pe = self._position_error(current_pos, target_pos)
            self._vel_buf[0] = (
                pos_kp * pe[0] - pos_kd * cur_lin_vel[0],
                pos_kp * pe[1] - pos_kd * cur_lin_vel[1],
                pos_kp * pe[2] - pos_kd * cur_lin_vel[2],
            )

            oe = self._orientation_error(current_orient, target_orient)
            self._ang_vel_buf[0] = (
                orient_kp * oe[0] - orient_kd * cur_ang_vel[0],
                orient_kp * oe[1] - orient_kd * cur_ang_vel[1],
                orient_kp * oe[2] - orient_kd * cur_ang_vel[2],
            )

            control_prim.set_velocities(
                linear_velocities=self._vel_buf,
                angular_velocities=self._ang_vel_buf,
            )
        except Exception as exc:
            print(f"[Teleop][FloatingRigidBody] {side.capitalize()} velocity update failed: {exc}")
            if not control_prim.valid:
                self._clear_runtime_handle(side)

    # =========================================================================
    # Helpers
    # =========================================================================

    def _iter_active_sides(
        self,
    ) -> Generator[
        tuple[str, RigidPrim, tuple[float, float, float], tuple[float, float, float, float] | None], None, None
    ]:
        """Yield active rigid-body handles whose prims still exist.

        Yields:
            Active side state used by the controller update loop.
        """
        for side, active, target_pos, target_orient in (
            ("left", self._left_active, self._left_target_pos, self._left_target_orient),
            ("right", self._right_active, self._right_target_pos, self._right_target_orient),
        ):
            control_prim = self._get_runtime_handle(side) or self._ensure_runtime_handle(side)
            if not active:
                continue
            if control_prim is None:
                continue
            if target_pos is None:
                continue
            if not control_prim.valid:
                print(
                    f"[Teleop][FloatingRigidBody] {side.capitalize()} control rigid body removed from stage - stopping."
                )
                self.destroy(side)
                continue
            yield side, control_prim, target_pos, target_orient

    @staticmethod
    def _get_pose(control_prim: RigidPrim) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        """Get current world pose from the active control handle.

        Args:
            control_prim: Value for control prim.

        Returns:
            The requested value.
        """
        positions, orientations = control_prim.get_world_poses()
        pos = positions.numpy()[0]
        orient_wxyz = orientations.numpy()[0]
        return (
            (float(pos[0]), float(pos[1]), float(pos[2])),
            (float(orient_wxyz[1]), float(orient_wxyz[2]), float(orient_wxyz[3]), float(orient_wxyz[0])),
        )

    @staticmethod
    def _get_velocities(control_prim: RigidPrim) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Get current velocities from the active control handle.

        Args:
            control_prim: Value for control prim.

        Returns:
            The requested value.
        """
        lin_vels, ang_vels = control_prim.get_velocities()
        lin = lin_vels.numpy()[0]
        ang = ang_vels.numpy()[0]
        return (
            (float(lin[0]), float(lin[1]), float(lin[2])),
            (float(ang[0]), float(ang[1]), float(ang[2])),
        )

    @staticmethod
    def _position_error(
        current: tuple[float, float, float], target: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """Compute position error (target - current).

        Args:
            current: Value for current.
            target: Value for target.

        Returns:
            The requested value.
        """
        return (target[0] - current[0], target[1] - current[1], target[2] - current[2])

    @staticmethod
    def _orientation_error(
        current_orient: tuple[float, float, float, float] | None,
        target_orient: tuple[float, float, float, float] | None,
    ) -> tuple[float, float, float]:
        """Compute orientation error as axis-angle using the shortest quaternion path.

        Args:
            current_orient: Value for current orient.
            target_orient: Value for target orient.

        Returns:
            The requested value.
        """
        if current_orient is None or target_orient is None:
            return (0.0, 0.0, 0.0)

        cx, cy, cz, cw = current_orient
        current_conj = (-cx, -cy, -cz, cw)
        ex, ey, ez, ew = quat_mul_xyzw(target_orient, current_conj)

        if ew < 0.0:
            ex, ey, ez, ew = -ex, -ey, -ez, -ew
        ew = max(-1.0, min(1.0, ew))
        angle = 2.0 * math.acos(ew)
        if abs(angle) < 1e-6:
            return (0.0, 0.0, 0.0)

        sin_half = math.sqrt(ex * ex + ey * ey + ez * ez)
        if sin_half < 1e-6:
            return (0.0, 0.0, 0.0)

        return (ex / sin_half * angle, ey / sin_half * angle, ez / sin_half * angle)

    @staticmethod
    def _quat_xyzw_to_euler_deg(
        quat_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        """Intrinsic ``X -> Y -> Z`` (roll, pitch, yaw) decomposition in degrees.

        Args:
            quat_xyzw: Value for quat xyzw.

        Returns:
            The requested value.
        """
        x, y, z, w = quat_xyzw
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n > 0.0:
            x, y, z, w = x / n, y / n, z / n, w / n
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sinp)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def reset_targets(self) -> None:
        """Reset all controller targets to origin/identity."""
        self._left_target_pos = (0.0, 0.0, 0.0)
        self._left_target_orient = (0.0, 0.0, 0.0, 1.0)
        self._right_target_pos = (0.0, 0.0, 0.0)
        self._right_target_orient = (0.0, 0.0, 0.0, 1.0)
        if self._enabled:
            self.apply_tracking()

    def set_side_enabled(self, side: str, enabled: bool) -> None:
        """Marks whether a side is assigned to this floating controller.

        This does not start runtime tracking. Tracking starts in :meth:`enable`
        when the timeline begins. Disabling a side clears its targets and stops
        the runtime handle if needed.

        Args:
            side: Value for side.
            enabled: Value for enabled.
        """
        side = side.lower()
        if side == "left":
            if not enabled:
                self._left_active = False
                self._left_target_pos = None
                self._left_target_orient = None
                self._clear_runtime_handle("left")
        elif side == "right":
            if not enabled:
                self._right_active = False
                self._right_target_pos = None
                self._right_target_orient = None
                self._clear_runtime_handle("right")
        self._enabled = self._left_active or self._right_active

    @property
    def is_enabled(self) -> bool:
        """Return True if floating rigid-body tracking is active.

        Returns:
            The requested value.
        """
        return self._enabled

    @property
    def left_end_effector_path(self) -> str | None:
        """Return the path to the left control rigid body.

        Returns:
            The requested value.
        """
        return self._left_end_effector_path

    @property
    def right_end_effector_path(self) -> str | None:
        """Return the path to the right control rigid body.

        Returns:
            The requested value.
        """
        return self._right_end_effector_path

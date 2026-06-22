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

"""Kinematic locomotion controller for VR-driven base movement.

Supports two workflows:

1. **Robot base** — target prim is a robot base.  Carry Tracking Space
   optionally co-moves the VR origin so the user follows the robot.
2. **VR origin** — target prim is the tracking-space origin marker.
   Carry is implicit (moving the base IS moving the VR workspace).
   Use this for floating grippers that have no physical base.

VR controller mapping:
- Left thumbstick Y:       Forward / backward slide (local frame)
- Left thumbstick X:       Left / right slide (local frame)
- Right thumbstick X:      Yaw rotation (turn left/right)
- Right primary button:    Move down (world Z-axis, ``A`` on Meta)
- Right secondary button:  Move up (world Z-axis, ``B`` on Meta)
- Left primary button:     Toggle Carry Tracking Space (``X`` on Meta)

All horizontal movement uses the prim's local +X projected onto the world
ground plane, so "forward" is the direction the prim faces on the XY plane
regardless of local frame tilt.  Vertical movement is always along world Z.
"""

from __future__ import annotations

import math
from contextlib import AbstractContextManager, nullcontext
from typing import Any

import numpy as np
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.utils.stage import get_current_stage
from pxr import Sdf, Usd, UsdGeom

from .._backend import teleop_backend_ctx
from .._xform_utils import WorldPosePrimCache, read_world_pose_arrays, to_numpy_array


class LocomotionController:
    """Kinematic locomotion controller — moves an explicit prim via VR input.

    Reads thumbstick and grab-trigger values each frame and applies
    incremental position and yaw changes through ``XformPrim`` world-pose API.
    No physics simulation is involved.

    Two workflows are supported:

    **Robot base locomotion** — the target prim is a robot's base link.
    Moving it kinematically repositions the entire robot.  Toggling
    *Carry Tracking Space* (left primary button) also moves the VR
    origin so the user's workspace follows the robot from one work
    area to another.

    **VR origin locomotion** — the target prim is the built-in
    tracking-space origin marker.  Because the base prim *is* the
    tracking space, carry is implicit: every movement simultaneously
    shifts the VR workspace.  This is the primary workflow for
    floating grippers that have no physical base.
    """

    DEADZONE = 0.1
    DEFAULT_LINEAR_STEP = 0.2 / 60.0
    DEFAULT_ANGULAR_STEP = 0.2 / 60.0

    def __init__(self) -> None:
        self._prim_path: str = ""
        self._tracking_space_prim_path: str = ""
        self._base_xform: XformPrim | None = None
        self._tracking_space_xform: XformPrim | None = None
        self._base_world_pose_cache = WorldPosePrimCache()
        self._tracking_space_world_pose_cache = WorldPosePrimCache()

        self._initial_base_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._initial_base_scale: np.ndarray | None = None
        self._initial_tracking_space_pose: tuple[np.ndarray, np.ndarray] | None = None

        self._edit_layer: Sdf.Layer | None = None

        self._linear_step: float = self.DEFAULT_LINEAR_STEP
        self._angular_step: float = self.DEFAULT_ANGULAR_STEP
        self._running = False
        self._carry_tracking_space: bool = False
        self._prev_left_primary_click: bool = False

        # Pre-allocated output buffers for set_world_poses (avoid per-frame allocs)
        self._pos_buf = np.zeros((1, 3), dtype=np.float32)
        self._orient_buf = np.zeros((1, 4), dtype=np.float32)

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    @property
    def prim_path(self) -> str:
        """USD path of the locomotion target prim.

        Returns:
            The requested value.
        """
        return self._prim_path

    @property
    def tracking_space_prim_path(self) -> str:
        """USD path of the tracking-space prim used for carry.

        Returns:
            The requested value.
        """
        return self._tracking_space_prim_path

    @property
    def linear_step(self) -> float:
        """Linear movement distance in metres per app update.

        Returns:
            The requested value.
        """
        return self._linear_step

    @property
    def angular_step(self) -> float:
        """Yaw rotation angle in radians per app update.

        Returns:
            The requested value.
        """
        return self._angular_step

    @property
    def is_running(self) -> bool:
        """True if the locomotion controller is active.

        Returns:
            The requested value.
        """
        return self._running

    def set_prim_path(self, path: str) -> None:
        """Set the USD path of the locomotion target prim.

        Args:
            path: Value for path.
        """
        self._prim_path = path
        self._base_world_pose_cache.set_prim_path(path)

    def set_tracking_space_prim_path(self, path: str) -> None:
        """Set the tracking-space prim carried with the base when Carry Tracking Space is enabled.

        Args:
            path: Value for path.
        """
        self._tracking_space_prim_path = path
        self._refresh_tracking_space_xform()

    def set_linear_step(self, step: float) -> None:
        """Set the linear movement distance per app update.

        Args:
            step: Value for step.
        """
        self._linear_step = max(0.0, step)

    def set_angular_step(self, step: float) -> None:
        """Set the yaw rotation angle per app update.

        Args:
            step: Value for step.
        """
        self._angular_step = max(0.0, step)

    def set_edit_layer(self, layer: Sdf.Layer | None) -> None:
        """Set the USD layer for prim writes.

        Marker prims have their xformOps in an anonymous session sublayer.
        Without directing writes to that layer, ``set_world_poses`` writes
        to the root layer, which is shadowed by the session sublayer.

        Args:
            layer: Value for layer.
        """
        self._edit_layer = layer

    # ------------------------------------------------------------------
    # Validate / Enable / Disable
    # ------------------------------------------------------------------

    def validate(self) -> tuple[bool, str]:
        """Validate the target prim and caches XformPrim wrappers.

        Only resets the xform stack when the prim lacks the standard
        ``translate/orient/scale`` ops required by ``set_world_poses``.
        When a reset is necessary the original local scale is preserved.

        Returns:
            The requested value.
        """
        stage = get_current_stage()
        if not stage:
            return False, "No USD stage available"

        if not self._prim_path or not Sdf.Path.IsValidPathString(self._prim_path):
            return False, "Invalid prim path"

        prim = stage.GetPrimAtPath(self._prim_path)
        if not prim or not prim.IsValid():
            return False, f"Prim not found at '{self._prim_path}'"

        props = set(prim.GetPropertyNames())
        needs_reset = not {"xformOp:translate", "xformOp:orient", "xformOp:scale"}.issubset(props)

        with self._teleop_edit_ctx(stage, self._prim_path):
            saved_scale = self._read_local_scale(prim) if needs_reset else None
            try:
                self._base_xform = XformPrim(self._prim_path, reset_xform_op_properties=needs_reset)
            except Exception as exc:
                return False, f"XformPrim error: {exc}"
            if saved_scale is not None:
                self._base_xform.set_local_scales(saved_scale)

        self._refresh_tracking_space_xform()

        return True, "Valid"

    def enable(self) -> tuple[bool, str]:
        """Validates, caches initial poses, and enables the controller.

        Returns:
            The requested value.
        """
        ok, msg = self.validate()
        if not ok:
            return False, msg

        self._cache_initial_poses()
        self._running = True
        if self.carries_tracking_space_implicitly:
            print(f"[Teleop][Locomotion] Enabled on VR origin '{self._prim_path}' (carry is implicit).")
        else:
            print(f"[Teleop][Locomotion] Enabled on '{self._prim_path}'.")
        return True, "Running"

    def disable(self) -> None:
        """Disable the controller and restores prims to their initial poses."""
        self._restore_initial_poses()
        self._running = False
        self._carry_tracking_space = False
        self._prev_left_primary_click = False

    # ------------------------------------------------------------------
    # Per-frame update
    # ------------------------------------------------------------------

    def update(self, left_ctrl: Any, right_ctrl: Any) -> None:
        """Apply one frame of locomotion from VR controller data.

        Args:
            left_ctrl: Left VR controller snapshot (or None).
            right_ctrl: Right VR controller snapshot (or None).
        """
        if not self._running or self._base_xform is None:
            return

        left_stick_x = self._get_input(left_ctrl, "thumbstick_x")
        left_stick_y = self._get_input(left_ctrl, "thumbstick_y")
        right_stick_x = self._get_input(right_ctrl, "thumbstick_x")
        right_primary = self._get_bool_input(right_ctrl, "primary_click")
        right_secondary = self._get_bool_input(right_ctrl, "secondary_click")
        left_primary = self._get_bool_input(left_ctrl, "primary_click")

        if left_primary and not self._prev_left_primary_click:
            if not self._tracking_space_prim_path:
                print("[Teleop][Locomotion] Carry Tracking Space toggle ignored: no tracking-space prim selected.")
            elif self._tracking_space_xform is None:
                print("[Teleop][Locomotion] Carry Tracking Space toggle ignored: tracking-space prim is unavailable.")
            elif self.carries_tracking_space_implicitly:
                print("[Teleop][Locomotion] Carry is implicit — locomotion prim IS the tracking space.")
            else:
                self._carry_tracking_space = not self._carry_tracking_space
                state = "enabled" if self._carry_tracking_space else "disabled"
                print(f"[Teleop][Locomotion] Carry Tracking Space: {state}")
        self._prev_left_primary_click = left_primary

        left_stick_x = self._apply_deadzone(left_stick_x)
        left_stick_y = self._apply_deadzone(left_stick_y)
        right_stick_x = self._apply_deadzone(right_stick_x)

        if (
            left_stick_x == 0.0
            and left_stick_y == 0.0
            and right_stick_x == 0.0
            and not right_primary
            and not right_secondary
        ):
            return

        delta_yaw = -right_stick_x * self._angular_step
        delta_forward = left_stick_y * self._linear_step
        delta_lateral = left_stick_x * self._linear_step
        delta_up = (float(right_secondary) - float(right_primary)) * self._linear_step

        self._apply_movement(delta_forward, delta_lateral, delta_up, delta_yaw, self._carry_tracking_space)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _get_input(ctrl: Any, attr: str) -> float:
        """Safely reads a float attribute from a VR controller snapshot.

        Args:
            ctrl: Value for ctrl.
            attr: Value for attr.

        Returns:
            The requested value.
        """
        if ctrl is None:
            return 0.0
        return getattr(ctrl.inputs, attr, 0.0)

    @staticmethod
    def _get_bool_input(ctrl: Any, attr: str) -> bool:
        """Safely reads a bool attribute from a VR controller snapshot.

        Args:
            ctrl: Value for ctrl.
            attr: Value for attr.

        Returns:
            The requested value.
        """
        if ctrl is None:
            return False
        return bool(getattr(ctrl.inputs, attr, False))

    def _apply_deadzone(self, value: float) -> float:
        """Zeros out values within the deadzone, remaps the rest to [0, 1].

        Args:
            value: Value for value.

        Returns:
            The requested value.
        """
        if abs(value) < self.DEADZONE:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.DEADZONE) / (1.0 - self.DEADZONE)

    @staticmethod
    def _read_local_scale(prim: Usd.Prim) -> np.ndarray | None:
        """Extract local scale from the prim's composed local transform matrix.

        Pre-reset fallback: runs before ``XformPrim`` normalizes xformOps, so it
        cannot use ``XformPrim.get_local_scales()`` (which requires an authored
        ``xformOp:scale`` property).

        Args:
            prim: Value for prim.

        Returns:
            The requested value.
        """
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            return None
        try:
            mtx = xformable.GetLocalTransformation(Usd.TimeCode.Default())
        except Exception:
            return None
        rows = np.array(
            [[mtx[i][0], mtx[i][1], mtx[i][2]] for i in range(3)],
            dtype=np.float32,
        )
        return np.linalg.norm(rows, axis=1).reshape(1, 3)

    def _teleop_edit_ctx(self, stage: Usd.Stage, prim_path: str) -> AbstractContextManager[None]:
        """Return an edit context for Teleop prim writes, or ``nullcontext``.

        Validates that ``_edit_layer`` is still in the stage's layer stack
        before creating the ``Usd.EditContext`` to avoid crashes when the
        anonymous marker layer has been dropped between sessions.

        Args:
            stage: Value for stage.
            prim_path: Value for prim path.

        Returns:
            The requested value.
        """
        if (
            self._edit_layer is not None
            and prim_path.startswith("/Teleop/")
            and any(self._edit_layer.identifier == l.identifier for l in stage.GetLayerStack(includeSessionLayers=True))
        ):
            return Usd.EditContext(stage, self._edit_layer)
        return nullcontext()

    def _refresh_tracking_space_xform(self) -> None:
        """Refresh the cached tracking-space wrapper if one is configured."""
        self._tracking_space_xform = None
        self._tracking_space_world_pose_cache.clear()
        if not self._tracking_space_prim_path:
            self._tracking_space_world_pose_cache.set_prim_path("")
            return

        stage = get_current_stage()
        if not stage:
            return

        tracking_space_prim = stage.GetPrimAtPath(self._tracking_space_prim_path)
        if not tracking_space_prim or not tracking_space_prim.IsValid():
            return

        self._tracking_space_world_pose_cache.set_prim_path(self._tracking_space_prim_path)
        with self._teleop_edit_ctx(stage, self._tracking_space_prim_path):
            try:
                self._tracking_space_xform = XformPrim(self._tracking_space_prim_path, reset_xform_op_properties=False)
            except Exception:
                try:
                    self._tracking_space_xform = XformPrim(
                        self._tracking_space_prim_path, reset_xform_op_properties=True
                    )
                except Exception:
                    pass

    def _cache_initial_poses(self) -> None:
        """Snapshots the current world poses and local scale of base and tracking-space prims."""
        with teleop_backend_ctx():
            if self._base_xform is not None:
                self._initial_base_pose = read_world_pose_arrays(self._base_world_pose_cache, copy=True)
                scales = self._base_xform.get_local_scales()
                self._initial_base_scale = to_numpy_array(scales, copy=True)
            if self._tracking_space_xform is not None:
                self._initial_tracking_space_pose = read_world_pose_arrays(
                    self._tracking_space_world_pose_cache, copy=True
                )

    def _restore_initial_poses(self) -> None:
        """Restores base and tracking-space prims to the poses and scale cached on enable."""
        stage = get_current_stage()
        with teleop_backend_ctx():
            if self._initial_base_pose is not None and self._base_xform is not None and self._base_xform.valid:
                with self._teleop_edit_ctx(stage, self._prim_path):
                    self._base_xform.set_world_poses(
                        positions=self._initial_base_pose[0],
                        orientations=self._initial_base_pose[1],
                    )
                    if self._initial_base_scale is not None:
                        self._base_xform.set_local_scales(self._initial_base_scale)
            if (
                self._initial_tracking_space_pose is not None
                and self._tracking_space_xform is not None
                and self._tracking_space_xform.valid
            ):
                with self._teleop_edit_ctx(stage, self._tracking_space_prim_path):
                    self._tracking_space_xform.set_world_poses(
                        positions=self._initial_tracking_space_pose[0],
                        orientations=self._initial_tracking_space_pose[1],
                    )
        self._initial_base_pose = None
        self._initial_base_scale = None
        self._initial_tracking_space_pose = None

    @property
    def carries_tracking_space_implicitly(self) -> bool:
        """True when the base prim IS the tracking-space prim.

        This happens when the user points locomotion at the VR origin
        marker to reposition floating grippers.  Moving the base
        already moves the tracking space, so no explicit carry toggle
        is needed.

        Returns:
            The requested value.
        """
        return bool(self._tracking_space_prim_path and self._tracking_space_prim_path == self._prim_path)

    def _apply_movement(
        self,
        delta_forward: float,
        delta_lateral: float,
        delta_up: float,
        delta_yaw: float,
        carry_tracking_space: bool = False,
    ) -> None:
        """Apply incremental translation and yaw rotation to the target prim.

        Horizontal movement uses the prim's local +X projected onto the world
        ground plane (XY).  Vertical movement and yaw are in world frame (Z-up).

        Two locomotion workflows are supported:

        **Robot base** — the target prim is a robot base.  The carry
        toggle (left primary button) enables co-moving the tracking-space
        prim so the VR workspace follows the robot.

        **VR origin** — the target prim IS the tracking-space origin
        marker.  Moving the base already moves the VR workspace, so
        carry is implicit and the toggle has no additional effect.
        This is useful for floating grippers that have no physical base.

        Args:
            delta_forward: Value for delta forward.
            delta_lateral: Value for delta lateral.
            delta_up: Value for delta up.
            delta_yaw: Value for delta yaw.
            carry_tracking_space: Value for carry tracking space.
        """
        if self._base_xform is None or not self._base_xform.valid:
            return

        stage = get_current_stage()

        with teleop_backend_ctx():
            pos_arr, quat_arr = read_world_pose_arrays(self._base_world_pose_cache)
            pos = pos_arr.reshape(-1, 3)[0].astype(np.float64)
            quat = quat_arr.reshape(-1, 4)[0].astype(np.float64)
            w, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])

            # Rotate prim's local +X by the base orientation; only the XY
            # components are needed since movement is constrained to the
            # ground plane.
            fx = 1.0 - 2.0 * (qy * qy + qz * qz)
            fy = 2.0 * (qx * qy + w * qz)
            fwd_len = math.hypot(fx, fy)
            if fwd_len > 1e-6:
                forward = np.array([fx / fwd_len, fy / fwd_len, 0.0])
            else:
                forward = np.array([1.0, 0.0, 0.0])
            right = np.array([forward[1], -forward[0], 0.0])

            new_pos = pos + forward * delta_forward + right * delta_lateral + np.array([0.0, 0.0, delta_up])

            half = delta_yaw * 0.5
            c, s = math.cos(half), math.sin(half)
            new_orient = np.array([c * w - s * qz, c * qx - s * qy, c * qy + s * qx, c * qz + s * w])

            self._fill_pose_buf(self._pos_buf, self._orient_buf, new_pos, new_orient)
            with self._teleop_edit_ctx(stage, self._prim_path):
                self._base_xform.set_world_poses(positions=self._pos_buf, orientations=self._orient_buf)
                if self._initial_base_scale is not None:
                    self._base_xform.set_local_scales(self._initial_base_scale)

            if (
                carry_tracking_space
                and self._tracking_space_xform is not None
                and self._tracking_space_prim_path != self._prim_path
            ):
                o_pos_arr, o_quat_arr = read_world_pose_arrays(self._tracking_space_world_pose_cache)
                o_pos = o_pos_arr.reshape(-1, 3)[0].astype(np.float64)
                o_quat = o_quat_arr.reshape(-1, 4)[0].astype(np.float64)
                ow, oqx, oqy, oqz = float(o_quat[0]), float(o_quat[1]), float(o_quat[2]), float(o_quat[3])

                # yaw_q rotates around world Z by angle delta_yaw; rotate the
                # base->tracking-space offset and apply to the new base pose.
                offset = o_pos - pos
                cos_yaw, sin_yaw = math.cos(delta_yaw), math.sin(delta_yaw)
                carried_offset = np.array(
                    [
                        offset[0] * cos_yaw - offset[1] * sin_yaw,
                        offset[0] * sin_yaw + offset[1] * cos_yaw,
                        offset[2],
                    ]
                )
                new_o_pos = new_pos + carried_offset
                new_o_orient = np.array([c * ow - s * oqz, c * oqx - s * oqy, c * oqy + s * oqx, c * oqz + s * ow])

                self._fill_pose_buf(self._pos_buf, self._orient_buf, new_o_pos, new_o_orient)
                with self._teleop_edit_ctx(stage, self._tracking_space_prim_path):
                    self._tracking_space_xform.set_world_poses(positions=self._pos_buf, orientations=self._orient_buf)

    @staticmethod
    def _fill_pose_buf(
        pos_buf: np.ndarray,
        orient_buf: np.ndarray,
        pos: np.ndarray,
        orient_wxyz: np.ndarray,
    ) -> None:
        """Write a single pose into the pre-allocated ``(1, 3)`` / ``(1, 4)`` buffers.

        Args:
            pos_buf: Value for pos buf.
            orient_buf: Value for orient buf.
            pos: Value for pos.
            orient_wxyz: Value for orient wxyz.
        """
        pos_buf[0, 0] = pos[0]
        pos_buf[0, 1] = pos[1]
        pos_buf[0, 2] = pos[2]
        orient_buf[0, 0] = orient_wxyz[0]
        orient_buf[0, 1] = orient_wxyz[1]
        orient_buf[0, 2] = orient_wxyz[2]
        orient_buf[0, 3] = orient_wxyz[3]

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

"""Locomotion controller for VR-driven base movement.

Supports two workflows:

1. **Robot base** — target prim is a robot base.  Carry Tracking Space
   optionally co-moves the VR origin so the user follows the robot.
2. **VR origin** — target prim is the tracking-space origin marker.
   Carry is implicit (moving the base IS moving the VR workspace).
   Use this for floating grippers that have no physical base.

Two drive modes are supported (see :class:`LocomotionDriveMode`):

* **Teleport** — kinematic.  Each frame a new world pose is written through
  ``XformPrim.set_world_poses``.  Because this is a USD-hierarchy operation,
  every descendant prim (arms, grippers, whether they are rigid bodies or
  articulations) rides along even when it is only *parented* to the base and
  not physically jointed.  No physics is involved.
* **Velocity** — dynamic.  Each frame a linear/angular velocity is commanded
  on the base rigid body through ``RigidPrim.set_velocities`` and PhysX
  integrates the motion, producing real contacts and collisions.  Only a
  dynamic (non-kinematic) rigid body can be driven this way, and only the
  base itself moves — attached payloads follow *solely* when they are
  physically jointed to the base.  A pure ``Xform`` (such as the VR origin
  marker) or a kinematic body cannot be velocity-driven.

The default mode is :attr:`LocomotionDriveMode.AUTO`, which selects Velocity
for a dynamic rigid-body target and Teleport otherwise.

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
from enum import Enum
from typing import Any

import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

from .._backend import teleop_backend_ctx
from .._xform_utils import WorldPosePrimCache, read_world_pose_arrays, to_numpy_array


class LocomotionDriveMode(str, Enum):
    """How the locomotion controller moves its target prim.

    Attributes:
        AUTO: Pick Velocity for a dynamic rigid-body target, Teleport otherwise.
        TELEPORT: Kinematic world-pose writes; carries parented children.
        VELOCITY: Physics velocity commands on a dynamic rigid body.
    """

    AUTO = "auto"
    TELEPORT = "teleport"
    VELOCITY = "velocity"


class LocomotionController:
    """Locomotion controller — moves an explicit prim via VR input.

    Reads thumbstick and face-button values each frame and either writes an
    incremental world pose (Teleport mode) or commands a base velocity
    (Velocity mode).  The active mode is resolved from
    :meth:`drive_mode` and the target prim (see :class:`LocomotionDriveMode`).

    Two workflows are supported:

    **Robot base locomotion** — the target prim is a robot's base link.
    Moving it repositions the robot.  A dynamic rigid-body base is driven
    with physics velocities (Velocity mode); a plain-``Xform`` or kinematic
    base is teleported (Teleport mode), carrying every parented child.
    Toggling *Carry Tracking Space* (left primary button) also moves the VR
    origin so the user's workspace follows the robot from one work area to
    another.

    **VR origin locomotion** — the target prim is the built-in
    tracking-space origin marker.  Because the base prim *is* the
    tracking space, carry is implicit: every movement simultaneously
    shifts the VR workspace.  This is the primary workflow for
    floating grippers that have no physical base.  The VR origin is a
    plain ``Xform``, so it is always teleported.
    """

    DEADZONE = 0.1
    DEFAULT_LINEAR_STEP = 0.2 / 60.0
    DEFAULT_ANGULAR_STEP = 0.2 / 60.0
    DEFAULT_LINEAR_SPEED = 1.0
    DEFAULT_ANGULAR_SPEED = 1.0

    def __init__(self) -> None:
        self._prim_path: str = ""
        self._tracking_space_prim_path: str = ""
        self._base_xform: XformPrim | None = None
        self._tracking_space_xform: XformPrim | None = None
        self._tracking_space_writable: bool = False
        self._base_world_pose_cache = WorldPosePrimCache()
        self._tracking_space_world_pose_cache = WorldPosePrimCache()

        self._initial_base_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._initial_base_scale: np.ndarray | None = None
        self._initial_tracking_space_pose: tuple[np.ndarray, np.ndarray] | None = None

        self._edit_layer: Sdf.Layer | None = None

        self._linear_step: float = self.DEFAULT_LINEAR_STEP
        self._angular_step: float = self.DEFAULT_ANGULAR_STEP
        self._linear_speed: float = self.DEFAULT_LINEAR_SPEED
        self._angular_speed: float = self.DEFAULT_ANGULAR_SPEED
        self._running = False
        self._carry_tracking_space: bool = False
        self._prev_left_primary_click: bool = False

        self._drive_mode: LocomotionDriveMode = LocomotionDriveMode.AUTO
        self._effective_mode: LocomotionDriveMode = LocomotionDriveMode.TELEPORT
        self._rigid_prim: RigidPrim | None = None
        self._prev_base_pos: np.ndarray | None = None
        self._prev_base_yaw: float | None = None

        # Pre-allocated output buffers for set_world_poses (avoid per-frame allocs)
        self._pos_buf = np.zeros((1, 3), dtype=np.float32)
        self._orient_buf = np.zeros((1, 4), dtype=np.float32)
        self._vel_buf = np.zeros((1, 3), dtype=np.float32)
        self._ang_vel_buf = np.zeros((1, 3), dtype=np.float32)
        self._zero_vel_buf = np.zeros((1, 3), dtype=np.float32)

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
    def linear_speed(self) -> float:
        """Maximum base linear speed in metres per second (Velocity mode).

        Returns:
            The requested value.
        """
        return self._linear_speed

    @property
    def angular_speed(self) -> float:
        """Maximum base yaw rate in radians per second (Velocity mode).

        Returns:
            The requested value.
        """
        return self._angular_speed

    @property
    def drive_mode(self) -> LocomotionDriveMode:
        """Requested drive mode (``AUTO`` resolves per target on validate).

        Returns:
            The requested value.
        """
        return self._drive_mode

    @property
    def effective_drive_mode(self) -> LocomotionDriveMode:
        """Concrete drive mode (``TELEPORT`` or ``VELOCITY``) resolved on validate.

        Returns:
            The requested value.
        """
        return self._effective_mode

    @property
    def is_running(self) -> bool:
        """True if the locomotion controller is active.

        Returns:
            The requested value.
        """
        return self._running

    @property
    def carry_tracking_space_enabled(self) -> bool:
        """True when locomotion co-moves the tracking-space prim with the base.

        Returns:
            Whether Carry Tracking Space is active.
        """
        return self._carry_tracking_space

    @property
    def carry_tracking_space_available(self) -> bool:
        """True when Carry Tracking Space can be toggled for the current setup.

        Returns:
            False when carry is implicit or the tracking-space prim is unavailable.
        """
        return (
            bool(self._tracking_space_prim_path)
            and self._tracking_space_xform is not None
            and self._tracking_space_writable
            and not self.carries_tracking_space_implicitly
        )

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

    def set_linear_speed(self, speed: float) -> None:
        """Set the maximum base linear speed used in Velocity mode.

        Args:
            speed: Value for speed.
        """
        self._linear_speed = max(0.0, speed)

    def set_angular_speed(self, speed: float) -> None:
        """Set the maximum base yaw rate used in Velocity mode.

        Args:
            speed: Value for speed.
        """
        self._angular_speed = max(0.0, speed)

    def set_drive_mode(self, mode: LocomotionDriveMode | str) -> None:
        """Set the requested drive mode.

        Args:
            mode: A :class:`LocomotionDriveMode` or its string value.
        """
        self._drive_mode = LocomotionDriveMode(mode)

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
        """Validate the target prim, resolve the drive mode, and cache wrappers.

        The effective drive mode (Teleport or Velocity) is resolved from the
        requested :meth:`drive_mode` and the target prim.  In Teleport mode the
        xform stack is only reset when the prim lacks the standard
        ``translate/orient/scale`` ops required by ``set_world_poses``, and the
        original local scale is preserved.  In Velocity mode the xform stack is
        left untouched because physics owns the transform.

        Returns:
            The requested value.
        """
        stage = stage_utils.get_current_stage()
        if not stage:
            return False, "No USD stage available"

        if not self._prim_path or not Sdf.Path.IsValidPathString(self._prim_path):
            return False, "Invalid prim path"

        prim = stage.GetPrimAtPath(self._prim_path)
        if not prim or not prim.IsValid():
            return False, f"Prim not found at '{self._prim_path}'"

        self._effective_mode = self._resolve_effective_mode(prim)
        self._rigid_prim = None

        if self._effective_mode is LocomotionDriveMode.VELOCITY:
            with self._teleop_edit_ctx(stage, self._prim_path):
                try:
                    self._base_xform = XformPrim(self._prim_path, reset_xform_op_properties=False)
                except Exception as exc:
                    return False, f"XformPrim error: {exc}"
            self._initial_base_scale = None
            self._refresh_tracking_space_xform()
            return True, "Valid (velocity: dynamic rigid body)"

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

        return True, "Valid (teleport)"

    def _resolve_effective_mode(self, prim: Usd.Prim) -> LocomotionDriveMode:
        """Resolve the concrete drive mode for a target prim.

        The VR-origin marker (and any plain ``Xform``) always teleports.
        Velocity is used only for a dynamic (non-kinematic) rigid body; an
        explicit Velocity request on an incompatible prim falls back to
        Teleport with a warning.

        Args:
            prim: Value for prim.

        Returns:
            The requested value.
        """
        if self.carries_tracking_space_implicitly:
            return LocomotionDriveMode.TELEPORT

        is_dynamic_rb = self._is_dynamic_rigid_body(prim)

        if self._drive_mode is LocomotionDriveMode.VELOCITY:
            if is_dynamic_rb:
                return LocomotionDriveMode.VELOCITY
            print(
                f"[Teleop][Locomotion] Velocity mode requested but '{self._prim_path}' is not a dynamic "
                "rigid body; falling back to teleport."
            )
            return LocomotionDriveMode.TELEPORT

        if self._drive_mode is LocomotionDriveMode.TELEPORT:
            return LocomotionDriveMode.TELEPORT

        return LocomotionDriveMode.VELOCITY if is_dynamic_rb else LocomotionDriveMode.TELEPORT

    @staticmethod
    def _is_dynamic_rigid_body(prim: Usd.Prim) -> bool:
        """True when the prim is a simulated, non-kinematic rigid body.

        Args:
            prim: Value for prim.

        Returns:
            The requested value.
        """
        if not prim_utils.has_api(prim, UsdPhysics.RigidBodyAPI):
            return False
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        enabled_attr = rigid_body_api.GetRigidBodyEnabledAttr()
        if enabled_attr and not bool(enabled_attr.Get()):
            return False
        kinematic_attr = rigid_body_api.GetKinematicEnabledAttr()
        if kinematic_attr and bool(kinematic_attr.Get()):
            return False
        return True

    def enable(self) -> tuple[bool, str]:
        """Validates, caches initial poses, and enables the controller.

        Returns:
            The requested value.
        """
        ok, msg = self.validate()
        if not ok:
            return False, msg

        if self._effective_mode is LocomotionDriveMode.VELOCITY:
            self._ensure_physx_rigid_body_api()

        self._cache_initial_poses()
        self._running = True
        mode = self._effective_mode.value
        if self.carries_tracking_space_implicitly:
            print(f"[Teleop][Locomotion] Enabled on VR origin '{self._prim_path}' (carry is implicit, {mode}).")
        else:
            print(f"[Teleop][Locomotion] Enabled on '{self._prim_path}' ({mode}).")
        return True, "Running"

    def _ensure_physx_rigid_body_api(self) -> None:
        """Apply ``PhysxRigidBodyAPI`` to the base so velocity ops are available."""
        stage = stage_utils.get_current_stage()
        if not stage:
            return
        prim = stage.GetPrimAtPath(self._prim_path)
        if prim and prim.IsValid():
            prim_utils.ensure_api(prim, PhysxSchema.PhysxRigidBodyAPI)

    def disable(self) -> None:
        """Disable the controller.

        Teleport mode restores the initial poses to preserve its historical
        reset-on-disable behavior. Velocity mode stops the rigid body at its
        current physics-integrated pose.
        """
        self.stop_motion()
        if self._effective_mode is LocomotionDriveMode.TELEPORT:
            self._restore_initial_poses()
        else:
            self._discard_initial_poses()
        self._running = False
        self._carry_tracking_space = False
        self._prev_left_primary_click = False
        self._rigid_prim = None
        self._prev_base_pos = None
        self._prev_base_yaw = None

    def set_carry_tracking_space(self, enabled: bool) -> bool:
        """Enable or disable Carry Tracking Space.

        Used by the debug UI and VR left-primary edge toggle. Ignored when the
        locomotion prim is the tracking-space origin (carry is implicit).

        Args:
            enabled: Whether locomotion should co-move the tracking-space prim.

        Returns:
            True if the requested state was applied.
        """
        if enabled == self._carry_tracking_space:
            return True
        if not self._tracking_space_prim_path:
            print("[Teleop][Locomotion] Carry Tracking Space toggle ignored: no tracking-space prim selected.")
            return False
        if self._tracking_space_xform is None:
            print("[Teleop][Locomotion] Carry Tracking Space toggle ignored: tracking-space prim is unavailable.")
            return False
        if not self._tracking_space_writable:
            print(
                "[Teleop][Locomotion] Carry Tracking Space toggle ignored: the custom anchor's xform stack "
                "is read-only for safe teleop use."
            )
            return False
        if self.carries_tracking_space_implicitly:
            print("[Teleop][Locomotion] Carry is implicit — locomotion prim IS the tracking space.")
            return False
        self._carry_tracking_space = enabled
        state = "enabled" if enabled else "disabled"
        print(f"[Teleop][Locomotion] Carry Tracking Space: {state}")
        return True

    def stop_motion(self) -> None:
        """Stop commanded velocity without disabling or repositioning the controller."""
        if self._effective_mode is not LocomotionDriveMode.VELOCITY:
            return
        rigid_prim = self._rigid_prim
        if rigid_prim is None or not rigid_prim.valid:
            return
        with backend_utils.use_backend("tensor" if rigid_prim.is_physics_tensor_entity_valid() else "usd"):
            try:
                rigid_prim.set_velocities(
                    linear_velocities=self._zero_vel_buf,
                    angular_velocities=self._zero_vel_buf,
                )
            except Exception:
                pass

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
            self.set_carry_tracking_space(not self._carry_tracking_space)
        self._prev_left_primary_click = left_primary

        left_stick_x = self._apply_deadzone(left_stick_x)
        left_stick_y = self._apply_deadzone(left_stick_y)
        right_stick_x = self._apply_deadzone(right_stick_x)
        vertical = float(right_secondary) - float(right_primary)

        if self._effective_mode is LocomotionDriveMode.VELOCITY:
            self._apply_velocity(left_stick_y, left_stick_x, vertical, right_stick_x, self._carry_tracking_space)
            return

        if left_stick_x == 0.0 and left_stick_y == 0.0 and right_stick_x == 0.0 and vertical == 0.0:
            return

        delta_yaw = -right_stick_x * self._angular_step
        delta_forward = left_stick_y * self._linear_step
        delta_lateral = left_stick_x * self._linear_step
        delta_up = vertical * self._linear_step

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
            and any(
                self._edit_layer.identifier == layer.identifier
                for layer in stage.GetLayerStack(includeSessionLayers=True)
            )
        ):
            return Usd.EditContext(stage, self._edit_layer)
        return nullcontext()

    def _refresh_tracking_space_xform(self) -> None:
        """Refresh the cached tracking-space wrapper if one is configured."""
        self._tracking_space_xform = None
        self._tracking_space_writable = False
        self._tracking_space_world_pose_cache.clear()
        if not self._tracking_space_prim_path:
            self._tracking_space_world_pose_cache.set_prim_path("")
            return

        stage = stage_utils.get_current_stage()
        if not stage:
            return

        tracking_space_prim = stage.GetPrimAtPath(self._tracking_space_prim_path)
        if not tracking_space_prim or not tracking_space_prim.IsValid():
            return

        self._tracking_space_world_pose_cache.set_prim_path(self._tracking_space_prim_path)
        is_builtin = self._tracking_space_prim_path == "/Teleop" or self._tracking_space_prim_path.startswith(
            "/Teleop/"
        )
        properties = set(tracking_space_prim.GetPropertyNames())
        has_writable_stack = {"xformOp:translate", "xformOp:orient", "xformOp:scale"}.issubset(properties)
        with self._teleop_edit_ctx(stage, self._tracking_space_prim_path):
            try:
                self._tracking_space_xform = XformPrim(self._tracking_space_prim_path, reset_xform_op_properties=False)
            except Exception:
                if is_builtin:
                    try:
                        self._tracking_space_xform = XformPrim(
                            self._tracking_space_prim_path, reset_xform_op_properties=True
                        )
                        has_writable_stack = True
                    except Exception:
                        pass
        self._tracking_space_writable = self._tracking_space_xform is not None and (is_builtin or has_writable_stack)

    def _cache_initial_poses(self) -> None:
        """Snapshots the current world poses and local scale of base and tracking-space prims."""
        with teleop_backend_ctx():
            if self._base_xform is not None:
                self._initial_base_pose = read_world_pose_arrays(self._base_world_pose_cache, copy=True)
                if self._effective_mode is LocomotionDriveMode.VELOCITY:
                    self._initial_base_scale = None
                else:
                    scales = self._base_xform.get_local_scales()
                    self._initial_base_scale = to_numpy_array(scales, copy=True)
                pos = self._initial_base_pose[0].reshape(-1, 3)[0].astype(np.float64)
                quat = self._initial_base_pose[1].reshape(-1, 4)[0].astype(np.float64)
                self._prev_base_pos = pos.copy()
                self._prev_base_yaw = self._yaw_from_quat(quat[0], quat[1], quat[2], quat[3])
            if self._tracking_space_xform is not None:
                self._initial_tracking_space_pose = read_world_pose_arrays(
                    self._tracking_space_world_pose_cache, copy=True
                )

    def _restore_initial_poses(self) -> None:
        """Restores base and tracking-space prims to the poses and scale cached on enable."""
        stage = stage_utils.get_current_stage()
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
        self._discard_initial_poses()

    def _discard_initial_poses(self) -> None:
        """Discard enable-time pose snapshots without repositioning any prims."""
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

    @staticmethod
    def _ground_basis(w: float, qx: float, qy: float, qz: float) -> tuple[np.ndarray, np.ndarray]:
        """Return the ground-plane forward/right unit vectors for an orientation.

        Rotates the prim's local +X by the orientation and projects it onto the
        world XY plane, so "forward" is the prim's heading regardless of tilt.

        Args:
            w: Quaternion real component.
            qx: Quaternion x component.
            qy: Quaternion y component.
            qz: Quaternion z component.

        Returns:
            The requested value.
        """
        fx = 1.0 - 2.0 * (qy * qy + qz * qz)
        fy = 2.0 * (qx * qy + w * qz)
        fwd_len = math.hypot(fx, fy)
        if fwd_len > 1e-6:
            forward = np.array([fx / fwd_len, fy / fwd_len, 0.0])
        else:
            forward = np.array([1.0, 0.0, 0.0])
        right = np.array([forward[1], -forward[0], 0.0])
        return forward, right

    @staticmethod
    def _yaw_from_quat(w: float, qx: float, qy: float, qz: float) -> float:
        """Return the world-Z yaw angle (radians) of a quaternion.

        Args:
            w: Quaternion real component.
            qx: Quaternion x component.
            qy: Quaternion y component.
            qz: Quaternion z component.

        Returns:
            The requested value.
        """
        return math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _ensure_rigid_prim(self) -> RigidPrim | None:
        """Create the base ``RigidPrim`` lazily once physics is live.

        Returns:
            The requested value.
        """
        if self._rigid_prim is not None and self._rigid_prim.valid:
            return self._rigid_prim
        try:
            self._rigid_prim = RigidPrim(self._prim_path, reset_xform_op_properties=False)
        except Exception:
            self._rigid_prim = None
        return self._rigid_prim

    def _apply_velocity(
        self,
        stick_forward: float,
        stick_lateral: float,
        stick_up: float,
        stick_yaw: float,
        carry_tracking_space: bool,
    ) -> None:
        """Command the base rigid-body velocity from thumbstick input.

        Thumbstick input maps directly to a commanded body velocity — PhysX
        integrates the motion, so attached payloads follow only through their
        physics joints.  All axes are velocity-controlled (idle input commands
        zero velocity), which pins the base against gravity like the floating
        controller.  Carry is applied by teleporting the tracking-space prim to
        follow the measured base motion, since it is a plain ``Xform``.

        Args:
            stick_forward: Deadzoned left-thumbstick Y in [-1, 1].
            stick_lateral: Deadzoned left-thumbstick X in [-1, 1].
            stick_up: Vertical command in [-1, 1] (right face buttons).
            stick_yaw: Deadzoned right-thumbstick X in [-1, 1].
            carry_tracking_space: Whether to co-move the tracking-space prim.
        """
        rigid_prim = self._ensure_rigid_prim()
        if rigid_prim is None or not rigid_prim.valid:
            return

        with teleop_backend_ctx():
            positions, orientations = rigid_prim.get_world_poses()
            pos = to_numpy_array(positions).reshape(-1, 3)[0].astype(np.float64)
            quat = to_numpy_array(orientations).reshape(-1, 4)[0].astype(np.float64)
            w, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])

            forward, right = self._ground_basis(w, qx, qy, qz)
            linear = (
                forward * (stick_forward * self._linear_speed)
                + right * (stick_lateral * self._linear_speed)
                + np.array([0.0, 0.0, stick_up * self._linear_speed])
            )
            angular_z = stick_yaw * self._angular_speed * -1.0

            self._vel_buf[0, 0] = linear[0]
            self._vel_buf[0, 1] = linear[1]
            self._vel_buf[0, 2] = linear[2]
            self._ang_vel_buf[0, 0] = 0.0
            self._ang_vel_buf[0, 1] = 0.0
            self._ang_vel_buf[0, 2] = angular_z
            try:
                # Rigid-body velocity writes support Tensor/USD, not the selected transform backend.
                with backend_utils.use_backend("tensor" if rigid_prim.is_physics_tensor_entity_valid() else "usd"):
                    rigid_prim.set_velocities(
                        linear_velocities=self._vel_buf,
                        angular_velocities=self._ang_vel_buf,
                    )
            except Exception as exc:
                print(f"[Teleop][Locomotion] Velocity update failed: {exc}")
                if not rigid_prim.valid:
                    self._rigid_prim = None
                return

            yaw = self._yaw_from_quat(w, qx, qy, qz)
            if (
                carry_tracking_space
                and self._tracking_space_xform is not None
                and self._tracking_space_prim_path != self._prim_path
            ):
                self._carry_tracking_space_by_delta(pos, yaw)
            self._prev_base_pos = pos.copy()
            self._prev_base_yaw = yaw

    def _carry_tracking_space_by_delta(self, base_pos: np.ndarray, base_yaw: float) -> None:
        """Move the tracking-space prim by the base's per-frame delta.

        Velocity mode does not author the base pose, so carry follows the
        measured base translation and yaw change (rotating the offset about the
        base pivot) rather than a commanded delta.

        Args:
            base_pos: Current base world position.
            base_yaw: Current base world-Z yaw (radians).
        """
        if self._prev_base_pos is None or self._prev_base_yaw is None:
            return

        delta_pos = base_pos - self._prev_base_pos
        delta_yaw = base_yaw - self._prev_base_yaw
        if abs(delta_pos[0]) < 1e-9 and abs(delta_pos[1]) < 1e-9 and abs(delta_pos[2]) < 1e-9 and abs(delta_yaw) < 1e-9:
            return

        stage = stage_utils.get_current_stage()
        o_pos_arr, o_quat_arr = read_world_pose_arrays(self._tracking_space_world_pose_cache)
        o_pos = o_pos_arr.reshape(-1, 3)[0].astype(np.float64)
        o_quat = o_quat_arr.reshape(-1, 4)[0].astype(np.float64)
        ow, oqx, oqy, oqz = float(o_quat[0]), float(o_quat[1]), float(o_quat[2]), float(o_quat[3])

        offset = o_pos - self._prev_base_pos
        cos_yaw, sin_yaw = math.cos(delta_yaw), math.sin(delta_yaw)
        rotated_offset = np.array(
            [
                offset[0] * cos_yaw - offset[1] * sin_yaw,
                offset[0] * sin_yaw + offset[1] * cos_yaw,
                offset[2],
            ]
        )
        new_o_pos = base_pos + rotated_offset

        half = delta_yaw * 0.5
        c, s = math.cos(half), math.sin(half)
        new_o_orient = np.array([c * ow - s * oqz, c * oqx - s * oqy, c * oqy + s * oqx, c * oqz + s * ow])

        self._fill_pose_buf(self._pos_buf, self._orient_buf, new_o_pos, new_o_orient)
        with self._teleop_edit_ctx(stage, self._tracking_space_prim_path):
            self._tracking_space_xform.set_world_poses(positions=self._pos_buf, orientations=self._orient_buf)

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

        stage = stage_utils.get_current_stage()

        with teleop_backend_ctx():
            pos_arr, quat_arr = read_world_pose_arrays(self._base_world_pose_cache)
            pos = pos_arr.reshape(-1, 3)[0].astype(np.float64)
            quat = quat_arr.reshape(-1, 4)[0].astype(np.float64)
            w, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])

            forward, right = self._ground_basis(w, qx, qy, qz)

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

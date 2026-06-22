# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""OmniGraph node implementation for Stanley steering control with PID."""

import math

import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni
import omni.graph.core as og
from isaacsim.core.nodes import BaseResetNode
from isaacsim.robot.experimental.wheeled_robots.controllers import (
    State,
    normalize_angle,
    pid_control,
    stanley_control,
)
from isaacsim.robot.wheeled_robots.nodes.ogn.OgnStanleyControlPIDDatabase import OgnStanleyControlPIDDatabase


class OgnStanleyControlPIDInternalState(BaseResetNode):
    """Per-instance state for the StanleyControlPID OmniGraph node."""

    def __init__(self) -> None:
        self.target_idx = 0  # path array index
        # store target pos to prevent repeated & unnecessary db.inputs.target access
        self.target = [0, 0, 0]  # [x, y, z_rot]
        self.node = None
        self.rv = []  # store path arrays to avoid repeated inputs access
        self.rx = []
        self.ry = []
        self.ryaw = []
        self.sp = []  # speed profile, used for linear speed control and path drawing
        self.argb = []  # stores color info for path drawing
        # Threshold to switch to rotate-only mode. Only index [0] (distance) is used;
        # expected to be the same double[2] array as the CheckGoal2D node's thresholds input.
        self.thresholds = []

        self.wb = 0  # save wheelBase and step to prevent unnecessary input calls
        self.s = 0

        super().__init__(initialize=False)

    def custom_reset(self) -> None:
        """Reset all saved values to prevent carrying over into a different run."""
        self.target_idx = 0
        self.target = [0, 0, 0]
        self.rv = []
        self.rx = []
        self.ry = []
        self.ryaw = []
        self.sp = []
        self.thresholds = []

        self.wb = 0
        self.s = 0


class OgnStanleyControlPID:
    """OmniGraph node for Stanley steering control with PID velocity regulation."""

    @staticmethod
    def init_instance(node: og.Node, graph_instance_id: int) -> None:
        """Initialize the per-instance state for this node.

        Args:
            node: OmniGraph node instance.
            graph_instance_id: Graph instance identifier.
        """
        state = OgnStanleyControlPIDDatabase.get_internal_state(node, graph_instance_id)
        state.node = node

    @staticmethod
    def internal_state() -> OgnStanleyControlPIDInternalState:
        """Return a new internal state instance.

        Returns:
            Per-instance Stanley control PID state.
        """
        return OgnStanleyControlPIDInternalState()

    @staticmethod
    def compute(db: OgnStanleyControlPIDDatabase) -> bool:
        """Compute linear and angular velocity outputs using Stanley control and PID.

        Args:
            db: OmniGraph database for this node.

        Returns:
            True when velocity outputs are updated, False when required wheel base or
            step inputs are invalid.
        """
        state = db.per_instance_state

        # save thresholds if changed
        state.thresholds = db.inputs.thresholds

        # Pull `reachedGoal` input from CheckGoal2D node
        reached_goal = db.inputs.reachedGoal
        # If target pos & rot reached, stop movement (and allow future nodes to run)
        if reached_goal[0] and reached_goal[1]:
            db.outputs.linearVelocity = 0
            db.outputs.angularVelocity = 0
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return True

        # get pos/rot/velocity data
        pos = db.inputs.currentPosition
        x = pos[0]
        y = pos[1]
        _, _, rot = quatd4_to_euler(db.inputs.currentOrientation)
        cs = db.inputs.currentSpeed
        v = np.hypot(cs[0], cs[1])

        if db.inputs.targetChanged:  # if retargeted
            state.target_idx = 0  # reset path array index
            state.target = db.inputs.target  # store new target

            path_arrays = db.inputs.pathArrays
            # rv, rx, ry, and ryaw are concatenated; each is 1/4 of the full input array.
            arr_length = int(len(path_arrays) / 4)

            # Separate into component path arrays
            state.rv = np.array(path_arrays[0:arr_length])
            state.rx = np.array(path_arrays[arr_length : arr_length * 2])
            state.ry = np.array(path_arrays[arr_length * 2 : arr_length * 3])
            state.ryaw = np.array(path_arrays[arr_length * 3 : arr_length * 4])

            # calculate speed profile with suggested/arbitrary target & min speed values
            state.sp = calc_speed_profile(np.array(state.rv), db.inputs.maxVelocity, 0.5, 0.05)

        # Check if rotate_only using distance threshold
        state.rotate_only = np.hypot(x - state.target[0], y - state.target[1]) <= state.thresholds[0] or reached_goal[0]

        # store wheelBase and step to prevent unnecessary input calls
        if state.wb == 0:
            state.wb = db.inputs.wheelBase
            state.s = db.inputs.step

        # if wheelBase or step are 0, divide by 0 errors will occur
        if state.wb == 0:
            db.log_error("Wheel base is 0")
            return False
        elif state.s == 0:
            db.log_error("Step is 0")
            return False

        gains = db.inputs.gains
        K = gains[0]
        Kp = gains[1]
        Ks = gains[2]
        # create new stanley control State object to store current odometry info about robot
        stanley_state = State(state.wb * Kp, x=x, y=y, yaw=rot % (2 * np.pi), v=v, max_steering_angle=Ks)

        if not state.rotate_only:  # if driving & steering is needed
            ai = pid_control(state.sp[state.target_idx], stanley_state.v, Kp) / state.s  # linear acceleration
            di, state.target_idx = stanley_control(
                stanley_state, state.rx, state.ry, state.ryaw, state.target_idx, K
            )  # delta rot and path array index closest to current pos/rot

            stanley_state.update(
                ai, di, state.s
            )  # use acceleration and delta rot values to determine linear and angular velocity outputs
            v = stanley_state.v  # save linear and angular velocity outputs
            w = stanley_state.w

        else:  # if position reached but not rotation
            v = 0  # stop linear velocity
            theta_diff = math.atan2(
                math.sin(state.target[2] - rot), math.cos(state.target[2] - rot)
            )  # find diff between current rotation and target rotation

            # rotation direction determined by pos/neg sign of `theta_diff`, rotation magnitude limited to 1
            if theta_diff > 0:
                w = min(((theta_diff) * Kp / state.s), 1)
            else:
                w = max(((theta_diff) * Kp / state.s), -1)

        kw = 1
        # Allow additional steering to use differential drive
        # (backwards spin on one wheel to tighten the cornering radius)
        if not reached_goal[0] and v > 0:
            kw = 1 + abs((state.wb * w) / v) * (1 * Kp / state.s)

        # output linear/angular velocity values
        db.outputs.linearVelocity = v
        db.outputs.angularVelocity = kw * w

        # begin next node (Differential and Angular controllers, if configured)
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        # If user enables path drawing, draw the path using previously computed color values and path arrays
        if db.inputs.drawPath:
            draw_path(state.rx, state.ry, state.argb)

        return True


def quatd4_to_euler(orientation: np.ndarray) -> tuple[float, float, float]:
    """Convert a (x, y, z, w) quaternion to normalized Euler angles (roll, pitch, yaw).

    Args:
        orientation: Quaternion in (x, y, z, w) order.

    Returns:
        Tuple of normalized (roll, pitch, yaw) in radians.

    """
    orientation = np.asarray(orientation, dtype=float)
    if orientation.size != 4 or not np.all(np.isfinite(orientation)) or np.linalg.norm(orientation) == 0.0:
        return 0.0, 0.0, 0.0

    orientation = orientation.reshape(4)
    x, y, z, w = tuple(orientation)
    roll, pitch, yaw = transform_utils.quaternion_to_euler_angles(np.array([w, x, y, z])).numpy()

    return normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)


def calc_speed_profile(
    cyaw: np.ndarray, max_speed: float, target_speed: float, min_speed: float = 1
) -> np.ndarray | bool:
    """Compute a speed profile that decelerates near the end of the path.

    Args:
        cyaw: Array of yaw velocities along the planned path.
        max_speed: Maximum allowed speed.
        target_speed: Desired cruising speed.
        min_speed: Minimum speed during deceleration.

    Returns:
        Speed profile array, or False if the maximum yaw curvature is zero.

    """
    max_c = max([abs(c) for c in cyaw])
    if max_c == 0:
        return False

    speed_profile = np.array(cyaw) / max_c * max_speed

    # speed down
    res = min(int(len(cyaw) / 3), int(max_speed * 60))

    for i in range(1, res):
        speed_profile[-i] = min(speed_profile[-i], speed_profile[-i] / (float(res - i)) ** 0.5)  # / (res))
        if speed_profile[-i] <= min_speed:
            speed_profile[-i] = min_speed

    return speed_profile


def draw_path_setup(sp: np.ndarray) -> list[int]:
    """Build an ARGB color array from a speed profile for path visualization.

    Args:
        sp: Speed profile array.

    Returns:
        List of ARGB integer values for each path point.

    """
    color = [(0, t / np.max(sp), 0) for t in sp]
    rgb_bytes = [(np.clip(c, 0, 1.0) * 255).astype("uint8").tobytes() for c in color]
    argb_bytes = [b"\xff" + b for b in rgb_bytes]
    argb = [int.from_bytes(b, byteorder="big") for b in argb_bytes]

    return argb


def draw_path(rx: list[float], ry: list[float], argb: list[int]) -> None:
    """Draw the planned path as a spline in the viewport using debug draw.

    Args:
        rx: X-coordinates of the path points.
        ry: Y-coordinates of the path points.
        argb: ARGB color values for the path (currently unused for per-point coloring).

    """
    try:
        from isaacsim.util.debug_draw import _debug_draw
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        stage_unit = UsdGeom.GetStageMetersPerUnit(stage)
        points = []
        for i in range(len(rx) - 1):
            points.append((rx[i] / stage_unit, ry[i] / stage_unit, 0.14 / stage_unit))
        _debug_draw.acquire_debug_draw_interface().clear_lines()
        _debug_draw.acquire_debug_draw_interface().draw_lines_spline(points, (1, 1, 1, 1), 0.05, True)
    except ImportError:
        import carb

        carb.log_error("isaacsim.util.debug_draw must be enabled to draw path")

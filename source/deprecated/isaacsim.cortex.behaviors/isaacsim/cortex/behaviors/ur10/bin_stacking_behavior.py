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

"""UR10 bin stacking behavior using cortex decision framework."""

from __future__ import annotations

import time
from typing import Any

import isaacsim.cortex.framework.math_util as math_util
import numpy as np
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.math import normalized
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import (
    DfDecider,
    DfDecision,
    DfNetwork,
    DfSetLockState,
    DfState,
    DfStateMachineDecider,
    DfStateSequence,
    DfTimedDeciderState,
    DfWaitState,
    DfWriteContextState,
)
from isaacsim.cortex.framework.dfb import DfDiagnosticsMonitor, DfLift, make_go_home
from isaacsim.cortex.framework.motion_commander import ApproachParams, MotionCommand, PosePq
from isaacsim.cortex.framework.obstacle_monitor_context import ObstacleMonitor, ObstacleMonitorContext


class BinState:
    """Track the state of a single bin during the stacking behavior.

    Args:
        bin_obj: The bin's simulation object.
    """

    def __init__(self, bin_obj: Any) -> None:
        self.bin_obj = bin_obj
        self.bin_base = SingleXFormPrim(self.bin_obj.prim_path + "/Collision/Cube_03")
        self.grasp_T = None
        self.is_grasp_reached = None
        self.is_attached = None
        self.needs_flip = None


class FlipStationObstacleMonitor(ObstacleMonitor):
    """Monitor that toggles the flip station obstacle based on end-effector proximity.

    Args:
        context: The bin stacking context.
    """

    def __init__(self, context: Any) -> None:
        super().__init__([context.world.scene.get_object("flip_station_sphere")])
        self.context = context

    def is_obstacle_required(self) -> bool:
        """Check whether the flip station obstacle is needed based on grasp alignment.

        Returns:
            True if the obstacle is required, False otherwise.
        """
        eff_T = self.context.robot.arm.get_fk_T()
        eff_R, eff_p = math_util.unpack_T(eff_T)
        eff_ax, _, _ = math_util.unpack_R(eff_R)

        grasp_p = self.context.active_bin.grasp_T[:3, 3]
        grasp_ax = self.context.active_bin.grasp_T[:3, 0]
        v = eff_p - grasp_p
        dist = v.dot(grasp_ax)
        orth_dist = np.linalg.norm(v - dist * grasp_ax)
        return not (dist < 0.02 and grasp_ax.dot(eff_ax) > 0.75 and orth_dist < 0.03)


class NavigationObstacleMonitor(ObstacleMonitor):
    """Monitor that toggles navigation obstacles based on target-effector side crossing.

    Args:
        context: The bin stacking context.
    """

    def __init__(self, context: Any) -> None:
        obstacles = [
            context.world.scene.get_object("navigation_dome_obs"),
            context.world.scene.get_object("navigation_barrier_obs"),
            context.world.scene.get_object("navigation_flip_station_obs"),
        ]
        super().__init__(obstacles)
        self.context = context

    def is_obstacle_required(self) -> bool:
        """Check whether navigation obstacles are needed based on target and effector sides.

        Returns:
            True if the navigation obstacles are required, False otherwise.
        """
        target_p, _ = self.context.robot.arm.target_prim.get_world_pose()

        ref_p = np.array([0.6, 0.37, -0.99])
        eff_p = self.context.robot.arm.get_fk_p()

        ref_p[2] = 0.0
        eff_p[2] = 0.0
        target_p[2] = 0.0

        s_target = np.sign(np.cross(target_p, ref_p)[2])
        s_eff = np.sign(np.cross(eff_p, ref_p)[2])
        is_required = s_target * s_eff < 0.0
        return is_required


class BinStackingDiagnostic:
    """Store diagnostic information about the current bin stacking state.

    Args:
        bin_name: Name of the active bin.
        bin_base: The bin base prim.
        grasp: The grasp transform.
        grasp_reached: Whether the grasp has been reached.
        attached: Whether the bin is attached to the gripper.
        needs_flip: Whether the bin needs to be flipped.
    """

    def __init__(
        self,
        bin_name: str | None = None,
        bin_base: Any | None = None,
        grasp: np.ndarray | None = None,
        grasp_reached: bool | None = None,
        attached: bool | None = None,
        needs_flip: bool | None = None,
    ) -> None:
        self.bin_name = bin_name
        self.bin_base = bin_base
        self.grasp = grasp
        self.grasp_reached = grasp_reached
        self.attached = attached
        self.needs_flip = needs_flip


class BinStackingDiagnosticsMonitor(DfDiagnosticsMonitor):
    """Diagnostics monitor that reports bin stacking state.

    Args:
        print_dt: Interval between diagnostic prints in seconds.
        diagnostic_fn: Optional callback for receiving diagnostic objects.
    """

    def __init__(self, print_dt: float = 1.0, diagnostic_fn: Any = None) -> None:
        super().__init__(print_dt=print_dt)
        self.diagnostic_fn = diagnostic_fn

    def print_diagnostics(self, context: Any) -> None:
        """Collect and report diagnostic information about the active bin.

        Args:
            context: The bin stacking context providing current state information.
        """
        if context.has_active_bin:
            diagnostic = BinStackingDiagnostic(
                context.active_bin.bin_obj.name,
                context.active_bin.bin_base,
                context.active_bin.grasp_T,
                context.active_bin.is_grasp_reached,
                context.active_bin.is_attached,
                context.active_bin.needs_flip,
            )
        else:
            diagnostic = BinStackingDiagnostic()
        if self.diagnostic_fn:
            self.diagnostic_fn(diagnostic)
        # print("=========== logical state ==========")
        # if context.has_active_bin:
        #     print("active bin info:")
        #     print("- bin_obj.name: {}".format(context.active_bin.bin_obj.name))
        #     print("- bin_base: {}".format(context.active_bin.bin_base))
        #     print("- grasp_T:\n{}".format(context.active_bin.grasp_T))
        #     print("- is_grasp_reached: {}".format(context.active_bin.is_grasp_reached))
        #     print("- is_attached:  {}".format(context.active_bin.is_attached))
        #     print("- needs_flip:  {}".format(context.active_bin.needs_flip))
        # else:
        #     print("<no active bin>")

        # print("------------------------------------")


def get_bin_under(p: np.ndarray, stacked_bins: list) -> Any | None:
    """Find the stacked bin directly under a given position.

    Args:
        p: The 3D position to search under.
        stacked_bins: The list of already-stacked bin states.

    Returns:
        The BinState directly under the given position, or None if not found.
    """
    x, y, z = p
    xy = np.array([x, y])

    for b in reversed(stacked_bins):
        (bin_x, bin_y, bin_z), _ = b.bin_obj.get_world_pose()
        bin_xy = np.array([bin_x, bin_y])
        if np.linalg.norm(bin_xy - xy) < 0.05:
            # We're searching in reverse. Return the first valid candidate.
            return b

    return None


def adjust_about_x_if_opposite(eff_R: np.ndarray, target_R: np.ndarray, threshold: float = -0.9) -> np.ndarray:
    """Rotate target_R by 180 degrees about its X axis if X axes are nearly opposite.

    This avoids a configuration where the end-effector's X axis is nearly anti-parallel
    to the target pose's X axis, which can cause undesirable wrist flips.

    Args:
        eff_R: The current end-effector rotation matrix.
        target_R: The target rotation matrix.
        threshold: Dot product threshold below which axes are considered opposite.

    Returns:
        The adjusted target rotation matrix.
    """
    target_x = target_R[:3, 0]
    eff_x = eff_R[:3, 0]
    if np.dot(target_x, eff_x) < threshold:
        rot180_x = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        return target_R @ rot180_x
    return target_R


class BinStackingContext(ObstacleMonitorContext):
    """Context for the UR10 bin stacking behavior.

    Args:
        robot: The robot API instance.
        monitor_fn: Optional callback for receiving diagnostic information.
    """

    def __init__(self, robot: Any, monitor_fn: Any = None) -> None:
        super().__init__(robot.arm)
        self.robot = robot
        self.world = CortexWorld.instance()
        self.diagnostics_monitor = BinStackingDiagnosticsMonitor(print_dt=1.0, diagnostic_fn=monitor_fn)

        self.flip_station_obs_monitor = FlipStationObstacleMonitor(self)
        self.navigation_obs_monitor = NavigationObstacleMonitor(self)
        self.add_obstacle_monitors([self.flip_station_obs_monitor, self.navigation_obs_monitor])

        h = 0.135
        e = 0.0075
        x_shift = 0.05

        full_stack = True
        if full_stack:
            self.stack_xs = np.array([1.00, 0.79, 0.58]) + x_shift
            self.stack_ys = [-0.62, -0.31, 0]
            self.stack_zs = [-0.59374 + (i * h) + h / 2 + e for i in range(4)]
        else:
            self.stack_xs = np.array([1.00, 0.79]) + x_shift
            self.stack_ys = [-0.62, -0.31]
            self.stack_zs = [-0.59374 + (i * h) + h / 2 + e for i in range(3)]

        self.stack_coordinates = []
        for zi in range(len(self.stack_zs)):
            for yi in range(len(self.stack_ys)):
                for xi in range(len(self.stack_xs)):
                    coords = np.array([self.stack_xs[xi], self.stack_ys[yi], self.stack_zs[zi]])
                    self.stack_coordinates.append(coords)

        self.bins = []
        self.active_bin = None
        self.stacked_bins = []

        self.add_monitors(
            [
                BinStackingContext.monitor_bins,
                BinStackingContext.monitor_active_bin,
                BinStackingContext.monitor_active_bin_grasp_T,
                BinStackingContext.monitor_active_bin_grasp_reached,
                self.diagnostics_monitor.monitor,
            ]
        )

    def reset(self) -> None:
        """Reset the context state including bins and stack."""
        super().reset()

        self.bins.clear()
        self.active_bin = None
        self.stacked_bins.clear()

    @property
    def stack_complete(self) -> bool:
        """Check whether all bins have been stacked."""
        return len(self.stacked_bins) == len(self.stack_coordinates)

    @property
    def elapse_time(self) -> float:
        """Return the elapsed time since the behavior started."""
        return time.time() - self.start_time

    @property
    def has_active_bin(self) -> bool:
        """Check whether there is an active bin being processed."""
        return self.active_bin is not None

    def monitor_bins(self) -> None:
        """Detect new bins and select the active bin from the conveyor region."""
        if self.active_bin is None:
            self.conveyor_bin = None
            min_y = None

            # Check whether there's a new bin in the world.
            bin_obj = self.world.scene.get_object(f"bin_{len(self.bins)}")
            if bin_obj is not None:
                self.bins.append(BinState(bin_obj))

            # Cycle through all bins and find the bin in the active region with smallest y value.
            for bin_state in self.bins:
                p, _ = bin_state.bin_obj.get_world_pose()

                # Check whether it's on the conveyor in the active region.
                x, y, z = p
                if y > 0.0 and y < 0.7 and x > -0.4 and x < 0.4:
                    if self.active_bin is None or y < min_y:
                        self.active_bin = bin_state
                        min_y = y

    def monitor_active_bin(self) -> None:
        """Clear the active bin if it has fallen below the scene."""
        if self.active_bin is not None:
            p, _ = self.active_bin.bin_obj.get_world_pose()
            if p[2] < -1.0:
                self.active_bin = None

    def monitor_active_bin_grasp_T(self) -> None:  # noqa: N802
        """Compute the grasp transform for the active bin based on its orientation."""
        if self.active_bin is not None:
            bin_T = math_util.pq2T(*self.active_bin.bin_base.get_world_pose())
            bin_R, bin_p = math_util.unpack_T(bin_T)
            bin_ax, bin_ay, bin_az = math_util.unpack_R(bin_R)

            self.active_bin.is_rightside_up = False
            up_vec = np.array([0.0, 0.0, 1.0])
            if self.active_bin.is_attached:
                fk_R = self.robot.arm.get_fk_R()
                fk_x, _, _ = math_util.unpack_R(fk_R)
                up_vec = -fk_x

            margin = 0.0
            base_width = 0.00233

            self.active_bin.needs_flip = up_vec.dot(bin_az) > 0.0
            if self.active_bin.needs_flip:
                # The bin is right side up (opens upward)
                target_ax = -bin_az
                margin = 0.0025
            else:
                # The bin is upside down (opens downward)
                target_ax = bin_az
                margin = -0.0025
            if bin_ax[1] < 0.0:
                # x axis is pointing toward the robot
                target_ay = -bin_ax
            else:
                target_ay = bin_ax
            target_az = np.cross(target_ax, target_ay)
            target_p = bin_p + margin * bin_az

            target_T = math_util.pack_Rp(math_util.pack_R(target_ax, target_ay, target_az), target_p)
            self.active_bin.grasp_T = target_T

    def monitor_active_bin_grasp_reached(self) -> None:
        """Update whether the grasp has been reached and whether the bin is attached."""
        if self.has_active_bin:
            fk_T = self.robot.arm.get_fk_T()
            self.active_bin.is_grasp_reached = math_util.transforms_are_close(
                self.active_bin.grasp_T, fk_T, p_thresh=0.01, R_thresh=0.01
            )
            # We can be looser with this proximity check.
            self.active_bin.is_attached = (
                math_util.transforms_are_close(self.active_bin.grasp_T, fk_T, p_thresh=0.1, R_thresh=1.0)
                and self.robot.suction_gripper.is_closed()
            )

    def mark_active_bin_as_complete(self) -> None:
        """Mark the active bin as stacked and clear it."""
        self.stacked_bins.append(self.active_bin)
        self.active_bin = None


class Move(DfState):
    """State that moves the end-effector toward a commanded pose until thresholds are met.

    Args:
        p_thresh: Position threshold for convergence.
        R_thresh: Rotation threshold for convergence.
    """

    def __init__(self, p_thresh: float, R_thresh: float) -> None:
        self.p_thresh = p_thresh
        self.R_thresh = R_thresh
        self.command = None

    def update_command(self, command: Any) -> None:
        """Set the motion command to execute.

        Args:
            command: The motion command to send.
        """
        self.command = command

    def step(self) -> Any:
        """Send the command and check for convergence.

        Returns:
            Self to continue moving, or None when the target is reached.
        """
        self.context.robot.arm.send(self.command)

        fk_T = self.context.robot.arm.get_fk_T()
        if math_util.transforms_are_close(
            self.command.target_pose.to_T(), fk_T, p_thresh=self.p_thresh, R_thresh=self.R_thresh
        ):
            return None
        return self


class MoveWithNavObs(Move):
    """Move state that activates navigation obstacle monitoring during movement."""

    def enter(self) -> None:
        """Activate navigation obstacle auto-toggle on entry."""
        super().enter()
        self.context.navigation_obs_monitor.activate_autotoggle()

    def exit(self) -> None:
        """Deactivate navigation obstacle auto-toggle on exit."""
        super().exit()
        self.context.navigation_obs_monitor.deactivate_autotoggle()


class ReachToPick(MoveWithNavObs):
    """Reach to pick the bin.

    The bin can be anywhere, including on the flip station. On entry, we activate the flip station
    obstacle monitor in case we're picking from the flip station. That obstacle monitor will prevent
    collision with the flip station en route.
    """

    def __init__(self) -> None:
        super().__init__(p_thresh=0.005, R_thresh=2.0)

    def enter(self) -> None:
        """Activate the flip station obstacle monitor on entry."""
        super().enter()
        self.context.flip_station_obs_monitor.activate_autotoggle()

    def step(self) -> Any:
        """Compute and send the pick approach command each cycle.

        Returns:
            Self to continue approaching, or None when the grasp is reached.
        """
        R, p = math_util.unpack_T(self.context.active_bin.grasp_T)
        ax, ay, az = math_util.unpack_R(R)

        posture_config = np.array([-1.2654234, -2.9708025, -2.219733, 0.6445836, 1.5186214, 0.30098662])
        if self.context.active_bin.needs_flip:
            approach_length = 0.3
        else:
            approach_length = 0.1

        # Adjust orientation to avoid X-axis anti-parallel configuration.
        eff_T = self.context.robot.arm.get_fk_T()
        eff_R, eff_p = math_util.unpack_T(eff_T)
        R = adjust_about_x_if_opposite(eff_R, R)

        distance_to_target = np.linalg.norm(p - self.context.robot.arm.get_fk_p())
        if distance_to_target < approach_length:
            approach_length = distance_to_target

        self.update_command(
            MotionCommand(
                target_pose=PosePq(p, math_util.matrix_to_quat(R)),
                approach_params=ApproachParams(direction=approach_length * ax, std_dev=0.005),
                posture_config=posture_config,
            )
        )

        return super().step()

    def exit(self) -> None:
        """Deactivate the flip station obstacle monitor on exit."""
        super().exit()
        self.context.flip_station_obs_monitor.deactivate_autotoggle()


class ReachToPlace(MoveWithNavObs):
    """Move state that reaches to the stacking placement location."""

    def __init__(self) -> None:
        super().__init__(p_thresh=0.005, R_thresh=2.0)

    def enter(self) -> None:
        """Compute the target placement position and orientation on entry."""
        super().enter()

        self.target_p = self.context.stack_coordinates[len(self.context.stacked_bins)]
        self.bin_under = get_bin_under(self.target_p, self.context.stacked_bins)

        target_ax = np.array([0.0, 0.0, -1.0])
        target_az = np.array([0.0, -1.0, 0.0])
        target_ay = np.cross(target_az, target_ax)
        self.target_R = math_util.pack_R(target_ax, target_ay, target_az)

    def step(self) -> Any:
        """Adjust placement for alignment with the bin below, then send the command.

        Returns:
            Self to continue approaching, or None when the target is reached.
        """
        if self.bin_under is not None:
            bin_under_p, _ = self.bin_under.bin_obj.get_world_pose()
            bin_grasped_p, _ = self.context.active_bin.bin_obj.get_world_pose()
            xy_err = bin_under_p[:2] - bin_grasped_p[:2]
            if np.linalg.norm(xy_err) < 0.02:
                self.target_p[:2] += 0.1 * (bin_under_p[:2] - bin_grasped_p[:2])

        p = self.target_p
        R = self.target_R

        # Adjust orientation to avoid X-axis anti-parallel configuration and persist it.
        eff_T = self.context.robot.arm.get_fk_T()
        eff_R, eff_p = math_util.unpack_T(eff_T)
        R = adjust_about_x_if_opposite(eff_R, R)
        self.target_R = R
        target_pose = PosePq(p, math_util.matrix_to_quat(R))

        approach_length = 0.35
        distance_to_target = np.linalg.norm(p - self.context.robot.arm.get_fk_p())
        if distance_to_target < 0.35:
            approach_length = distance_to_target

        approach_params = ApproachParams(direction=approach_length * np.array([0.0, 0.0, -1.0]), std_dev=0.005)

        posture_config = self.context.robot.default_config
        self.update_command(
            MotionCommand(target_pose=target_pose, approach_params=approach_params, posture_config=posture_config)
        )

        return super().step()


class CloseSuctionGripperWithRetries(DfState):
    """State that closes the suction gripper, retrying until successful."""

    def enter(self) -> None:
        """Enter the close gripper with retries state."""

    def step(self) -> Any:
        """Attempt to close the gripper each cycle until successful.

        Returns:
            Self to retry, or None when the gripper is confirmed closed.
        """
        gripper = self.context.robot.suction_gripper
        gripper.close()
        if gripper.is_closed():
            return None
        return self


class CloseSuctionGripper(DfState):
    """State that closes the suction gripper and waits for confirmation."""

    def enter(self) -> None:
        """Close the suction gripper."""
        print("<close gripper>")
        self.context.robot.suction_gripper.close()

    def step(self) -> Any:
        """Wait until the gripper is confirmed closed.

        Returns:
            Self to keep waiting, or None when the gripper is closed.
        """
        if self.context.robot.suction_gripper.is_closed():
            return None
        self.context.robot.suction_gripper.close()
        return self


class OpenSuctionGripper(DfState):
    """State that opens the suction gripper."""

    def enter(self) -> None:
        """Open the suction gripper."""
        print("<open gripper>")
        self.context.robot.suction_gripper.open()

    def step(self) -> None:
        """Exit immediately after opening."""
        return


class DoNothing(DfState):
    """State that clears arm commands and does nothing."""

    def enter(self) -> None:
        """Clear arm commands."""
        self.context.robot.arm.clear()

    def step(self) -> Any:
        """Print the target pose and stay in this state.

        Returns:
            Self to remain in this state indefinitely.
        """
        print(self.context.robot.arm.target_prim.get_world_pose())
        return self


class LiftAndTurn(Move):
    """Move state that lifts the end-effector and turns toward a home-like pose."""

    def __init__(self) -> None:
        super().__init__(p_thresh=0.005, R_thresh=0.1)

    def step(self) -> Any:
        """Compute a lifted target pose based on home config and send the command.

        Returns:
            Self to continue moving, or None when the target is reached.
        """
        home_config = self.context.robot.default_config
        home_T = self.context.robot.arm.get_fk_T(config=home_config)

        p, q = math_util.T2pq(home_T)
        p += 0.5 * normalized(np.array([0.0, -0.5, -1.0]))
        self.target_pose = PosePq(p, q)
        self.update_command(MotionCommand(self.target_pose, posture_config=home_config))

        return super().step()


class PickBin(DfStateMachineDecider):
    """State machine that picks a bin from the conveyor."""

    def __init__(self) -> None:
        super().__init__(
            DfStateSequence(
                [
                    ReachToPick(),
                    DfWaitState(wait_time=1.0),
                    DfSetLockState(set_locked_to=True, decider=self),
                    CloseSuctionGripper(),
                    DfTimedDeciderState(DfLift(0.3), activity_duration=0.1),
                    DfSetLockState(set_locked_to=False, decider=self),
                ]
            )
        )


class FlipBin(DfStateMachineDecider):
    """State machine that flips a bin at the flip station."""

    def __init__(self) -> None:
        super().__init__(
            DfStateSequence(
                [
                    LiftAndTurn(),
                    MoveToFlipStation(),
                    DfSetLockState(set_locked_to=True, decider=self),
                    OpenSuctionGripper(),
                    ReleaseFlipStationBin(),
                    DfSetLockState(set_locked_to=False, decider=self),
                ]
            )
        )


class PlaceBin(DfStateMachineDecider):
    """State machine that places a bin on the stack."""

    def __init__(self) -> None:
        super().__init__(
            DfStateSequence(
                [
                    ReachToPlace(),
                    DfWaitState(wait_time=0.5),
                    DfSetLockState(set_locked_to=True, decider=self),
                    OpenSuctionGripper(),
                    DfTimedDeciderState(DfLift(0.1), activity_duration=0.25),
                    DfWriteContextState(lambda ctx: ctx.mark_active_bin_as_complete()),
                    DfSetLockState(set_locked_to=False, decider=self),
                ]
            )
        )


class MoveToFlipStation(DfState):
    """State that moves the end-effector to the flip station pose."""

    def __init__(self) -> None:
        self.target_pose = PosePq(
            np.array([0.7916634, 0.73902607, -0.02897218]), np.array([0.52239186, 0.6296602, -0.5042411, 0.27636158])
        )

        self.approach_params = ApproachParams(direction=0.4 * normalized(np.array([0.5, -0.3, -0.75])), std_dev=0.05)

        self.posture_config = np.array(
            [
                -2.1273114681243896,
                -3.004627227783203,
                -1.0576069355010986,
                -0.5193580389022827,
                -1.0809129476547241,
                2.0418107509613037,
            ]
        )

    def enter(self) -> None:
        """Send the motion command to move to the flip station."""
        motion_command = MotionCommand(
            target_pose=self.target_pose, approach_params=self.approach_params, posture_config=self.posture_config
        )
        self.context.robot.arm.send(motion_command)

    def step(self) -> Any:
        """Wait until the end-effector reaches the flip station pose.

        Returns:
            Self to keep waiting, or None when the end-effector reaches the pose.
        """
        fk_T = self.context.robot.arm.get_fk_T()
        if math_util.transforms_are_close(self.target_pose.to_T(), fk_T, p_thresh=0.005, R_thresh=2.0, verbose=False):
            return None
        return self


class ReleaseFlipStationBin(DfState):
    """State that releases the bin at the flip station and backs away."""

    def enter(self) -> None:
        """Compute a retraction pose and send the motion command."""
        self.entry_time = time.time()

        # Get some info about the current end-effector transform.
        fk_T = self.context.robot.arm.get_fk_T()
        fk_R, fk_p = math_util.unpack_T(fk_T)
        ax, ay, az = math_util.unpack_R(fk_R)

        v = normalized(np.array([-1.0, -0.3, 0.0]))  # Hard-coded vector pointing approx toward base.
        toward_base_alpha = 0.2
        target_p = fk_p - 0.3 * ax + toward_base_alpha * v
        self.target_p = target_p
        self.ax = ax
        self.v = v

        target_ax = normalized(np.array([1.0, -0.0, 0.0]))
        target_ay = normalized(np.array([0.0, -1.0, 0.0]))
        target_az = np.cross(target_ax, target_ay)
        target_R = math_util.pack_R(target_ax, target_ay, target_az)

        # This target pose is a little below the bin, but off to the side with the end
        # effector angled horizontally. It gets the end-effector out of the flip station
        # collision region and allows us to turn on that obstacle for now moving to pick
        # the bin.
        self.target_pose = PosePq(target_p, math_util.matrix_to_quat(target_R))
        motion_command = MotionCommand(
            target_pose=self.target_pose,
            approach_params=ApproachParams(direction=toward_base_alpha * v, std_dev=0.1),
            posture_config=self.context.robot.get_joint_positions().astype(float),
        )
        self.context.robot.arm.send(motion_command)

    def step(self) -> Any:
        """Wait until the end-effector is close enough to the retraction target.

        Returns:
            Self to keep waiting, or None when the end-effector is within 15 cm of the target.
        """
        # Exit (return None) when the end-effector is within 15cm of the target position.
        fk_p = self.context.robot.arm.get_fk_p()
        dist_to_target = np.linalg.norm(self.target_pose.p - fk_p)
        if dist_to_target < 0.15:
            return None
        return self


class Dispatch(DfDecider):
    """Top-level decider that dispatches between picking, flipping, placing, and going home."""

    def __init__(self) -> None:
        super().__init__()

        self.add_child("flip_bin", FlipBin())
        self.add_child("pick_bin", PickBin())
        self.add_child("place_bin", PlaceBin())
        self.add_child("go_home", make_go_home())
        self.add_child("do_nothing", DfStateMachineDecider(DoNothing()))

    def decide(self) -> Any:
        """Decide the next action based on stack completion and bin state.

        Returns:
            A DfDecision directing to flip_bin, pick_bin, place_bin, or go_home.
        """
        if self.context.stack_complete:
            return DfDecision("go_home")

        if self.context.has_active_bin:
            if not self.context.active_bin.is_attached:
                return DfDecision("pick_bin")
            elif self.context.active_bin.needs_flip:
                return DfDecision("flip_bin")
            else:
                return DfDecision("place_bin")
        else:
            return DfDecision("go_home")


def make_decider_network(robot: Any, monitor_fn: Any) -> Any:
    """Create the bin stacking decider network for the given robot.

    Args:
        robot: The robot API instance.
        monitor_fn: Optional callback for receiving diagnostic information.

    Returns:
        A configured DfNetwork for the bin stacking behavior.
    """
    return DfNetwork(Dispatch(), context=BinStackingContext(robot, monitor_fn))

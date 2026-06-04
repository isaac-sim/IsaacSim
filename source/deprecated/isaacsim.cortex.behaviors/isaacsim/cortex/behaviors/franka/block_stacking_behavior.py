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

"""Franka block stacking behavior using cortex decision framework."""

from __future__ import annotations

import time
from collections import OrderedDict
from typing import Any

import isaacsim.cortex.framework.math_util as math_util
import numpy as np
from isaacsim.cortex.framework.cortex_object import CortexObject
from isaacsim.cortex.framework.df import *
from isaacsim.cortex.framework.dfb import (
    DfApproachGrasp,
    DfCloseGripper,
    DfOpenGripper,
    DfRobotApiContext,
    make_go_home,
)


def make_grasp_T(t: np.ndarray, ay: np.ndarray) -> np.ndarray:  # noqa: N802
    """Construct a 4x4 grasp transform from a translation and approach-y axis.

    Args:
        t: The translation vector defining the grasp position.
        ay: The approach-y axis direction.

    Returns:
        A 4x4 homogeneous transform matrix representing the grasp.
    """
    az = math_util.normalized(-t)
    ax = np.cross(ay, az)

    T = np.eye(4)
    T[:3, 0] = ax
    T[:3, 1] = ay
    T[:3, 2] = az
    T[:3, 3] = t

    return T


def make_block_grasp_Ts(block_pick_height: float) -> list:  # noqa: N802
    """Generate all candidate grasp transforms for a block at the given pick height.

    Args:
        block_pick_height: The height above the block center at which to grasp.

    Returns:
        A list of 4x4 candidate grasp transform matrices.
    """
    R = np.eye(3)

    Ts = []
    for i in range(3):
        t = block_pick_height * R[:, i]
        for j in range(2):
            ay = R[:, (i + j + 1) % 3]
            for s1 in [1, -1]:
                for s2 in [1, -1]:
                    Ts.append(make_grasp_T(s1 * t, s2 * ay))

    return Ts


def get_world_block_grasp_Ts(  # noqa: N802
    obj_T: np.ndarray,
    obj_grasp_Ts: list,
    axis_x_filter: np.ndarray | None = None,
    axis_x_filter_thresh: float = 0.1,
    axis_y_filter: np.ndarray | None = None,
    axis_y_filter_thresh: float = 0.1,
    axis_z_filter: np.ndarray | None = None,
    axis_z_filter_thresh: float = 0.1,
) -> list:
    """Transform object-local grasp transforms to world frame, filtering by axis alignment.

    Args:
        obj_T: The object world transform.
        obj_grasp_Ts: List of grasp transforms in object-local coordinates.
        axis_x_filter: Optional desired x-axis direction to filter grasps.
        axis_x_filter_thresh: Maximum allowed deviation from axis_x_filter.
        axis_y_filter: Optional desired y-axis direction to filter grasps.
        axis_y_filter_thresh: Maximum allowed deviation from axis_y_filter.
        axis_z_filter: Optional desired z-axis direction to filter grasps.
        axis_z_filter_thresh: Maximum allowed deviation from axis_z_filter.

    Returns:
        A list of world-frame grasp transforms that pass the axis filters.
    """
    world_grasp_Ts = []
    for gT in obj_grasp_Ts:
        world_gT = obj_T.dot(gT)
        if axis_x_filter is not None and (
            1.0 - world_gT[:3, 0].dot(math_util.normalized(axis_x_filter)) > axis_x_filter_thresh
        ):
            continue
        if axis_y_filter is not None and (
            1.0 - world_gT[:3, 1].dot(math_util.normalized(axis_y_filter)) > axis_y_filter_thresh
        ):
            continue
        if axis_z_filter is not None and (
            1.0 - world_gT[:3, 2].dot(math_util.normalized(axis_z_filter)) > axis_z_filter_thresh
        ):
            continue

        world_grasp_Ts.append(world_gT)
    return world_grasp_Ts


def get_best_obj_grasp(
    obj_T: np.ndarray, obj_grasp_Ts: list, eff_T: np.ndarray, other_obj_Ts: list
) -> np.ndarray | None:
    """Use a score-based classifier to choose which grasp to use on a given block.

    Choose a grasp that is simultaneously natural for the arm and avoids any nearby blocks.

    Args:
        obj_T: The block object being grasped.
        obj_grasp_Ts: The grasp transforms in coordinates local to the block.
        eff_T: The current end-effector transform.
        other_obj_Ts: The transforms of all other surrounding blocks we want to consider.

    Returns:
        The best grasp transform, or None if no valid grasps are found.
    """
    Ts = get_world_block_grasp_Ts(obj_T, obj_grasp_Ts, axis_z_filter=np.array([0.0, 0.0, -1.0]))

    # This could happen if all the grasps are filtered out.
    if len(Ts) == 0:
        return None

    # Score each grasp based on how well the gripper's x-axis will correlate with the direction to
    # the robot (most natural configuration).
    obj_p = obj_T[:3, 3]
    v = math_util.normalized(-obj_p)

    # Score each of the candidate grasps based on how their world transform is oriented relative to
    # the robot and relative to nearby other blocks.
    scores = np.zeros(len(Ts))
    for i, grasp_T in enumerate(Ts):
        # The base score is a function of how the end-effector would be oriented relative to the
        # base of the robot.
        score = grasp_T[:3, 0].dot(v)

        # For all surrounding objects, if the object is closer than 15cm away, add a proximity cost
        # (negative score) for the grasp based on whether the finger might clip it.
        for obj_T in other_obj_Ts:
            other_obj_p = obj_T[:3, 3]
            toward_other = other_obj_p - obj_p
            dist = np.linalg.norm(toward_other)
            if dist < 0.25:
                # Care about closer blocks more.
                w = np.exp(-0.5 * (dist / 0.15) ** 2)
                prox_score = -10.0 * w * (grasp_T[:3, 1].dot(math_util.normalized(toward_other))) ** 2
                score += prox_score

        scores[i] += score

    # Return the highest scoring transform.
    scored_Ts = zip(scores, Ts)
    T = max(scored_Ts, key=lambda v: v[0])[1]
    return T


def calc_grasp_for_block_T(  # noqa: N802
    context: Any, block_T: np.ndarray, desired_ax: np.ndarray
) -> np.ndarray | None:
    """Calculate the best grasp transform for a block given a desired x-axis direction.

    Args:
        context: The build-tower context providing block and grasp information.
        block_T: The world transform of the block to grasp.
        desired_ax: The desired x-axis direction for the grasp.

    Returns:
        The best grasp transform, or None if no valid grasps are found.
    """
    ct = context
    candidate_Ts = get_world_block_grasp_Ts(block_T, ct.active_block.grasp_Ts, axis_z_filter=np.array([0.0, 0.0, -1.0]))
    if len(candidate_Ts) == 0:
        return None

    scored_candidate_Ts = [(np.dot(desired_ax, T[:3, 0]), T) for T in candidate_Ts]

    grasp_T = max(scored_candidate_Ts, key=lambda v: v[0])[1]
    return grasp_T


def calc_grasp_for_top_of_tower(context: Any) -> np.ndarray | None:
    """Calculate the best grasp for placing a block on top of the tower.

    Args:
        context: The build-tower context providing block and tower information.

    Returns:
        The best grasp transform for the tower top, or None if no valid grasps are found.
    """
    # TODO: use calc_grasp_for_block_T for this calculation
    ct = context
    block_target_T = ct.block_tower.next_block_placement_T
    candidate_Ts = get_world_block_grasp_Ts(
        block_target_T, ct.active_block.grasp_Ts, axis_z_filter=np.array([0.0, 0.0, -1.0])
    )
    if len(candidate_Ts) == 0:
        return None

    desired_ax = np.array([0.0, -1.0, 0.0])
    scored_candidate_Ts = [(np.dot(desired_ax, T[:3, 0]), T) for T in candidate_Ts]

    grasp_T = max(scored_candidate_Ts, key=lambda v: v[0])[1]
    return grasp_T


class BuildTowerContext(DfRobotApiContext):
    """Context for the block tower building behavior.

    Args:
        robot: The robot API instance.
        tower_position: The 3D position for the tower base.
    """

    class Block:
        """Represent a single block with its grasp information and state.

        Args:
            i: Block index.
            obj: The block's cortex object.
            grasp_Ts: List of candidate grasp transforms for the block.
        """

        def __init__(self, i: int, obj: Any, grasp_Ts: list) -> None:
            self.i = i
            self.obj = obj
            self.is_aligned = None
            self.grasp_Ts = grasp_Ts
            self.chosen_grasp = None
            self.collision_avoidance_enabled = True

        @property
        def has_chosen_grasp(self) -> bool:
            """Check whether a grasp has been chosen for this block."""
            return self.chosen_grasp is not None

        @property
        def name(self) -> str:
            """Return the name of the block object."""
            return self.obj.name

        def get_world_grasp_Ts(  # noqa: N802
            self,
            axis_x_filter: np.ndarray | None = None,
            axis_x_filter_thresh: float = 0.1,
            axis_y_filter: np.ndarray | None = None,
            axis_y_filter_thresh: float = 0.1,
            axis_z_filter: np.ndarray | None = None,
            axis_z_filter_thresh: float = 0.1,
        ) -> list:
            """Get grasp transforms in world coordinates with optional axis filtering.

            Args:
                axis_x_filter: Optional desired x-axis direction to filter grasps.
                axis_x_filter_thresh: Maximum allowed deviation from axis_x_filter.
                axis_y_filter: Optional desired y-axis direction to filter grasps.
                axis_y_filter_thresh: Maximum allowed deviation from axis_y_filter.
                axis_z_filter: Optional desired z-axis direction to filter grasps.
                axis_z_filter_thresh: Maximum allowed deviation from axis_z_filter.

            Returns:
                A list of world-frame grasp transforms that pass the axis filters.
            """
            return get_world_block_grasp_Ts(self.obj.get_transform(), self.grasp_Ts)

        def get_best_grasp(self, eff_T: np.ndarray, other_obj_Ts: list) -> np.ndarray | None:
            """Select the best grasp considering end-effector pose and nearby objects.

            Args:
                eff_T: The current end-effector transform.
                other_obj_Ts: The transforms of all surrounding blocks to consider.

            Returns:
                The best grasp transform, or None if no valid grasps are found.
            """
            return get_best_obj_grasp(self.obj.get_transform(), self.grasp_Ts, eff_T, other_obj_Ts)

        def set_aligned(self) -> None:
            """Mark this block as aligned in the tower."""
            self.is_aligned = True

    class BlockTower:
        """Track the state of the block tower being built.

        Args:
            tower_position: The 3D position of the tower base.
            block_height: Height of a single block.
            context: The parent build-tower context.
        """

        def __init__(self, tower_position: np.ndarray, block_height: float, context: Any) -> None:
            self.context = context
            order_preference = ["Blue", "Yellow", "Green", "Red"]
            self.desired_stack = [f"{c}Cube" for c in order_preference]
            self.tower_position = tower_position
            self.block_height = block_height
            self.stack = []
            self.prev_stack = None

        @property
        def height(self) -> int:
            """Return the number of blocks in the tower."""
            return len(self.stack)

        @property
        def top_block(self) -> Any | None:
            """Return the topmost block in the tower, or None if empty."""
            if self.height == 0:
                return None
            return self.stack[-1]

        @property
        def current_stack_in_correct_order(self) -> bool:
            """Return True if the current tower is in the correct order, False otherwise."""
            for pref_name, curr_block in zip(self.desired_stack, self.stack):
                if curr_block.name != pref_name:
                    return False

            return True

        @property
        def is_complete(self) -> bool:
            """Check whether the tower is fully built in the correct order."""
            # TODO: This doesn't account for desired vs actual ordering currently.
            if self.height != len(self.desired_stack):
                return False

            return self.current_stack_in_correct_order

        def stash_stack(self) -> None:
            """Save the current stack and reset it."""
            self.prev_stack = self.stack
            self.stack = []

        def find_new_and_removed(self) -> tuple | list:
            """Find blocks that were added to or removed from the stack since last stash.

            Returns:
                A list of new blocks if prev_stack is None, otherwise a tuple of
                (new_blocks, removed_blocks) since the last stash.
            """
            if self.prev_stack is None:
                return list(self.stack)

            i = 0
            while i < len(self.stack) and i < len(self.prev_stack):
                if self.stack[i] != self.prev_stack[i]:
                    break
                else:
                    i += 1

            new_blocks = self.stack[i:]
            removed_blocks = self.prev_stack[i:]
            return new_blocks, removed_blocks

        def set_top_block_to_aligned(self) -> None:
            """Mark the top block of the tower as aligned."""
            if len(self.stack) > 0:
                self.stack[-1].is_aligned = True

        @property
        def next_block(self) -> Any | None:
            """Return the first name in the desired stack that is not in the current stack.

            This models order preference, but not the strict requirement that the block stack
            be exactly in that order. Use current_stack_in_correct_order to additionally check
            that the current stack is in the correct order.
            """
            stack_names = [b.name for b in self.stack]
            for name in self.desired_stack:
                if name not in stack_names:
                    return self.context.blocks[name]

        @property
        def next_block_placement_T(self) -> np.ndarray:  # noqa: N802
            """Compute the transform for the next block placement on top of the tower."""
            h = self.height
            fractional_margin = 0.025
            dz = (h + 0.5 + fractional_margin) * self.block_height
            p = self.tower_position + np.array([0.0, 0.0, dz])
            R = np.eye(3)
            T = math_util.pack_Rp(R, p)
            return T

    def __init__(self, robot: Any, tower_position: np.ndarray) -> None:
        super().__init__(robot)
        self.robot = robot

        self.block_height = 0.0515
        self.block_pick_height = 0.02
        self.block_grasp_Ts = make_block_grasp_Ts(self.block_pick_height)
        self.tower_position = tower_position
        self.diagnostics_message = ""
        self.reset()

        self.add_monitors(
            [
                BuildTowerContext.monitor_perception,
                BuildTowerContext.monitor_block_tower,
                BuildTowerContext.monitor_gripper_has_block,
                BuildTowerContext.monitor_suppression_requirements,
                BuildTowerContext.monitor_diagnostics,
            ]
        )

    def reset(self) -> None:
        """Reset the context state including blocks, tower, and diagnostics."""
        self.blocks = OrderedDict()
        print("loading blocks")
        for i, (name, cortex_obj) in enumerate(self.robot.registered_obstacles.items()):
            print(f"{i}) {name}")

            # This behavior might be run either with CortexObjects (e.g. when synchronizing with a
            # sim/real world via ROS) or standard core API objects. If it's the latter, add the
            # CortexObject API.
            if not isinstance(cortex_obj, CortexObject):
                cortex_obj = CortexObject(cortex_obj)

            cortex_obj.sync_throttle_dt = 0.25
            self.blocks[name] = BuildTowerContext.Block(i, cortex_obj, self.block_grasp_Ts)

        self.block_tower = BuildTowerContext.BlockTower(self.tower_position, self.block_height, self)

        self.active_block = None
        self.in_gripper = None
        self.placement_target_eff_T = None

        self.print_dt = 0.25
        self.next_print_time = None
        self.start_time = None

    @property
    def has_active_block(self) -> bool:
        """Check whether there is an active block selected."""
        return self.active_block is not None

    def activate_block(self, name: str) -> None:
        """Set the named block as the active block.

        Args:
            name: The name of the block to activate.
        """
        self.active_block = self.blocks[name]

    def reset_active_block(self) -> None:
        """Clear the active block and its chosen grasp."""
        if self.active_block is None:
            return

        self.active_block.chosen_grasp = None
        self.active_block = None

    @property
    def block_names(self) -> list:
        """Return a list of all block names."""
        return list(self.blocks.keys())

    @property
    def num_blocks(self) -> int:
        """Return the total number of blocks."""
        return len(self.blocks)

    def mark_block_in_gripper(self) -> None:
        """Record the closest block to the end-effector as the one in the gripper."""
        eff_p = self.robot.arm.get_fk_p()
        blocks_with_dists = []
        for _, block in self.blocks.items():
            block_p, _ = block.obj.get_world_pose()
            blocks_with_dists.append((block, np.linalg.norm(eff_p - block_p)))

        closest_block, _ = min(blocks_with_dists, key=lambda v: v[1])
        self.in_gripper = closest_block

    def clear_gripper(self) -> None:
        """Clear the record of a block being in the gripper."""
        self.in_gripper = None

    @property
    def is_gripper_clear(self) -> bool:
        """Check whether the gripper is empty."""
        return self.in_gripper is None

    @property
    def gripper_has_block(self) -> bool:
        """Check whether the gripper is holding a block."""
        return not self.is_gripper_clear

    @property
    def has_placement_target_eff_T(self) -> bool:  # noqa: N802
        """Check whether a placement target end-effector transform is set."""
        return self.placement_target_eff_T is not None

    @property
    def next_block_name(self) -> str | None:
        """Return the name of the next block to place based on the desired order."""
        remaining_block_names = [b.name for b in self.find_not_in_tower()]
        if len(remaining_block_names) == 0:
            return None
        for name in self.block_tower.desired_stack:
            if name in remaining_block_names:
                break
        return name

    def find_not_in_tower(self) -> list:
        """Return a list of blocks that are not currently in the tower.

        Returns:
            A list of Block objects that are not part of the current tower stack.
        """
        blocks = list(self.blocks.values())
        for b in self.block_tower.stack:
            blocks[b.i] = None

        return [b for b in blocks if b is not None]

    def print_tower_status(self) -> str:
        """Format and return a string describing the current tower status.

        Returns:
            A formatted multi-line string showing the current and stacked block states.
        """
        in_tower = self.block_tower.stack
        out = "\nin tower:\n"
        for i, b in enumerate(in_tower):
            out += f"{i}) {b.name}, aligned: {b.is_aligned!s}, suppressed: {not b.collision_avoidance_enabled!s}\n"

        not_in_tower = self.find_not_in_tower()
        out += "\nnot in tower:\n"
        for i, b in enumerate(not_in_tower):
            out += f"{i}) {b.name}, aligned: {b.is_aligned!s}, suppressed: {not b.collision_avoidance_enabled!s}\n"
        return out + "\n"

    def monitor_perception(self) -> None:
        """Synchronize block belief poses with measured poses when drift is detected."""
        for _, block in self.blocks.items():
            obj = block.obj
            if not obj.has_measured_pose():
                continue

            measured_T = obj.get_measured_T()
            belief_T = obj.get_T()

            not_in_gripper = block != self.in_gripper

            eff_p = self.robot.arm.get_fk_p()
            sync_performed = False
            if not_in_gripper and np.linalg.norm(belief_T[:3, 3] - eff_p) > 0.05:
                sync_performed = True
                obj.sync_to_measured_pose()
            elif np.linalg.norm(belief_T[:3, 3] - measured_T[:3, 3]) > 0.15:
                sync_performed = True
                obj.sync_to_measured_pose()

    def monitor_block_tower(self) -> None:
        """Monitor the current state of the block tower.

        The block tower is determined as the collection of blocks at the tower location and their
        order by height above the table.
        """
        tower_xy = self.block_tower.tower_position[:2]

        new_block_tower_sequence = []
        for name, block in self.blocks.items():
            if self.gripper_has_block and self.in_gripper.name == block.name:
                # Don't include any blocks currently in the gripper
                continue

            p, _ = block.obj.get_world_pose()
            block_xy = p[:2]
            block_z = p[2]

            dist_to_tower = np.linalg.norm(tower_xy - block_xy)
            thresh = self.block_height / 2
            if dist_to_tower <= thresh:
                new_block_tower_sequence.append((block_z, block))

        if len(new_block_tower_sequence) > 1:
            new_block_tower_sequence.sort(key=lambda v: v[0])

        self.block_tower.stash_stack()
        for _, block in new_block_tower_sequence:
            self.block_tower.stack.append(block)

        new_blocks, removed_blocks = self.block_tower.find_new_and_removed()
        for block in new_blocks:
            block.is_aligned = False

        for block in removed_blocks:
            block.is_aligned = None

    def monitor_gripper_has_block(self) -> None:
        """Clear the gripper if the block in it has drifted too far from the end-effector."""
        if self.gripper_has_block:
            block = self.in_gripper
            _, block_p = math_util.unpack_T(block.obj.get_transform())
            eff_p = self.robot.arm.get_fk_p()
            if np.linalg.norm(block_p - eff_p) > 0.1:
                self.diagnostics_message = "Block lost. Clearing gripper."
                self.clear_gripper()

    def monitor_suppression_requirements(self) -> None:
        """Enable or disable collision avoidance for blocks near the target."""
        arm = self.robot.arm
        eff_T = arm.get_fk_T()
        eff_R, eff_p = math_util.unpack_T(eff_T)
        ax, ay, az = math_util.unpack_R(eff_R)

        target_p, _ = arm.target_prim.get_world_pose()

        toward_target = target_p - eff_p
        dist_to_target = np.linalg.norm(toward_target)

        blocks_to_suppress = []
        if self.gripper_has_block:
            blocks_to_suppress.append(self.in_gripper)

        for name, block in self.blocks.items():
            block_T = block.obj.get_transform()
            block_R, block_p = math_util.unpack_T(block_T)

            # If the block is close to the target and the end-effector is above the block (in z), then
            # suppress it.
            target_dist_to_block = np.linalg.norm(block_p - target_p)
            xy_dist = np.linalg.norm(block_p[:2] - target_p[:2])
            margin = 0.05
            # Add the block if either we're descending on the block, or they're neighboring blocks
            # during the descent.
            if (
                target_dist_to_block < 0.1
                and (xy_dist < 0.02 or eff_p[2] > block_p[2] + margin)
                or target_dist_to_block < 0.15
                and target_dist_to_block > 0.07
                and eff_p[2] > block_p[2] + margin
            ):
                if block not in blocks_to_suppress:
                    blocks_to_suppress.append(block)

        for block in blocks_to_suppress:
            if block.collision_avoidance_enabled:
                try:
                    arm.disable_obstacle(block.obj)
                    block.collision_avoidance_enabled = False
                except Exception as e:
                    print("error disabling obstacle")
                    import traceback

                    traceback.print_exc()

        for name, block in self.blocks.items():
            if block not in blocks_to_suppress:
                if not block.collision_avoidance_enabled:
                    arm.enable_obstacle(block.obj)
                    block.collision_avoidance_enabled = True

    def monitor_diagnostics(self) -> None:
        """Periodically update the diagnostics message with tower and block status."""
        now = time.time()
        if self.start_time is None:
            self.start_time = now
            self.next_print_time = now + self.print_dt

        if now >= self.next_print_time:
            # print("\n==========================================")
            out = f"time since start: {now - self.start_time:f} sec\n"
            out += self.print_tower_status() + "\n"
            self.next_print_time += self.print_dt

            if self.has_active_block:
                out += f"active block:{self.active_block.name}"
            else:
                out += "no active block"
            self.diagnostics_message = out


class OpenGripperRd(DfRldsNode):
    """RLDS node that open the gripper when close to the chosen grasp target.

    Args:
        dist_thresh_for_open: Distance threshold within which to open the gripper.
    """

    def __init__(self, dist_thresh_for_open: float) -> None:
        super().__init__()
        self.dist_thresh_for_open = dist_thresh_for_open
        self.add_child("open_gripper", DfOpenGripper())

    def is_runnable(self) -> bool | None:
        """Check whether the gripper should be opened based on proximity to the grasp.

        Returns:
            True if the end-effector is within the distance threshold of the grasp, else None.
        """
        ct = self.context
        if self.context.is_gripper_clear and not self.context.robot.gripper.is_open():
            if ct.has_active_block and ct.active_block.has_chosen_grasp:
                grasp_T = ct.active_block.chosen_grasp
                eff_T = ct.robot.arm.get_fk_T()
                p1 = grasp_T[:3, 3]
                p2 = eff_T[:3, 3]
                dist_to_target = np.linalg.norm(p1 - p2)
                return dist_to_target < self.dist_thresh_for_open

    def decide(self) -> Any:
        """Decide to open the gripper.

        Returns:
            A DfDecision directing to the open_gripper child.
        """
        return DfDecision("open_gripper")


class ReachToBlockRd(DfRldsNode):
    """RLDS node that dispatches to a linked child when the gripper is clear."""

    def __init__(self) -> None:
        super().__init__()
        self.child_name = None

    def link_to(self, name: str, decider: Any) -> None:
        """Link a named child decider to this node.

        Args:
            name: The name to assign the child decider.
            decider: The child decider to link.
        """
        self.child_name = name
        self.add_child(name, decider)

    def is_runnable(self) -> bool:
        """Check whether the gripper is clear.

        Returns:
            True if the gripper is clear, False otherwise.
        """
        return self.context.is_gripper_clear

    def decide(self) -> Any:
        """Decide to dispatch to the linked child.

        Returns:
            A DfDecision directing to the linked child.
        """
        return DfDecision(self.child_name)


class GoHome(DfDecider):
    """Decider that sends the robot to its home position with the gripper closed."""

    def __init__(self) -> None:
        super().__init__()
        self.add_child("go_home", make_go_home())

    def enter(self) -> None:
        """Close the gripper on entry."""
        self.context.robot.gripper.close()

    def decide(self) -> Any:
        """Decide to go home.

        Returns:
            A DfDecision directing to the go_home child.
        """
        return DfDecision("go_home")


class ChooseNextBlockForTowerBuildUp(DfDecider):
    """Choose the next block for building up the tower and compute its grasp."""

    def __init__(self) -> None:
        super().__init__()
        # If conditions aren't good, we'll just go home.
        self.add_child("go_home", GoHome())
        self.child_name = None

    def link_to(self, name: str, decider: Any) -> None:
        """Link a named child decider to this node.

        Args:
            name: The name to assign the child decider.
            decider: The child decider to link.
        """
        self.child_name = name
        self.add_child(name, decider)

    def decide(self) -> Any:
        """Select the next block and compute the best grasp, or go home if conditions are bad.

        Returns:
            A DfDecision directing to the linked child or go_home.
        """
        ct = self.context
        ct.active_block = ct.blocks[ct.next_block_name]

        # Check exceptions
        block_p, _ = ct.active_block.obj.get_world_pose()
        if np.linalg.norm(block_p) < 0.25:
            print(f"block too close to robot base: {ct.active_block.name}")
            return DfDecision("go_home")
        elif np.linalg.norm(block_p) > 0.81:
            print(f"block too far away: {ct.active_block.name}")
            return DfDecision("go_home")
        elif (
            self.context.block_tower.height >= 2
            and np.linalg.norm(block_p - self.context.block_tower.tower_position) < 0.15
        ):
            print(f"block too close to tower: {ct.active_block.name}")
            return DfDecision("go_home")

        other_obj_Ts = [
            block.obj.get_transform() for block in ct.blocks.values() if ct.next_block_name != block.obj.name
        ]
        ct.active_block.chosen_grasp = ct.active_block.get_best_grasp(ct.robot.arm.get_fk_T(), other_obj_Ts)
        return DfDecision(self.child_name, ct.active_block.chosen_grasp)

    def exit(self) -> None:
        """Clear the chosen grasp on exit."""
        self.context.active_block.chosen_grasp = None


class ChooseNextBlockForTowerTeardown(DfDecider):
    """Choose the top tower block for teardown and compute its grasp."""

    def __init__(self) -> None:
        super().__init__()
        self.child_name = None

    def link_to(self, name: str, decider: Any) -> None:
        """Link a named child decider to this node.

        Args:
            name: The name to assign the child decider.
            decider: The child decider to link.
        """
        self.child_name = name
        self.add_child(name, decider)

    def decide(self) -> Any:
        """Select the top block of the tower and compute its grasp.

        Returns:
            A DfDecision directing to the linked child with the computed grasp.
        """
        ct = self.context
        ct.active_block = ct.block_tower.top_block
        active_block_T = ct.active_block.obj.get_transform()
        ct.active_block.chosen_grasp = calc_grasp_for_block_T(ct, active_block_T, np.array([0.0, -1.0, 0.0]))
        return DfDecision(self.child_name, ct.active_block.chosen_grasp)

    def exit(self) -> None:
        """Clear the chosen grasp on exit."""
        self.context.active_block.chosen_grasp = None


class ChooseNextBlock(DfDecider):
    """Choose the next block, delegating to build-up or teardown as appropriate."""

    def __init__(self) -> None:
        super().__init__()
        self.add_child("choose_next_block_for_tower", ChooseNextBlockForTowerBuildUp())
        self.add_child("choose_tower_block", ChooseNextBlockForTowerTeardown())

    def link_to(self, name: str, decider: Any) -> None:
        """Link a named child decider to all sub-children.

        Args:
            name: The name to assign the child decider.
            decider: The child decider to link.
        """
        for _, child in self.children.items():
            child.link_to(name, decider)

    def decide(self) -> Any:
        """Decide whether to build up or tear down the tower.

        Returns:
            A DfDecision directing to choose_next_block_for_tower or choose_tower_block.
        """
        if self.context.block_tower.current_stack_in_correct_order:
            return DfDecision("choose_next_block_for_tower")
        else:
            return DfDecision("choose_tower_block")


class LiftState(DfState):
    """A simple state which sends a target a distance command_delta_z above the current.

    end-effector position until the end-effector has moved success_delta_z meters up.

    Args:
        command_delta_z: The delta offset up to shift the command away from the current end-effector
            position every cycle.
        success_delta_z: The delta offset up from the original end-effector position measured on
            entry required for exiting the state.
        cautious_command_delta_z: Optional smaller delta offset used when close to another block.
    """

    def __init__(
        self, command_delta_z: float, success_delta_z: float, cautious_command_delta_z: float | None = None
    ) -> None:
        self.command_delta_z = command_delta_z
        self.cautious_command_delta_z = cautious_command_delta_z
        self.success_delta_z = success_delta_z

    def enter(self) -> None:
        """Set posture config to current joint positions and record the success height."""
        # On entry, set the posture config to the current config so the movement is minimal.
        posture_config = self.context.robot.arm.articulation_subset.get_joints_state().positions.astype(float)
        self.context.robot.arm.set_posture_config(posture_config)

        self.success_z = self.context.robot.arm.get_fk_p()[2] + self.success_delta_z

    def closest_non_grasped_block_dist(self, eff_p: np.ndarray) -> float:
        """Return the distance to the closest block that is not currently grasped.

        Args:
            eff_p: The current end-effector position.

        Returns:
            The distance to the closest non-grasped block.
        """
        blocks_with_dists = []
        for name, block in self.context.blocks.items():
            block_p, _ = block.obj.get_world_pose()
            dist = np.linalg.norm(eff_p[:2] - block_p[:2])
            if dist > 0.03:
                # Only consider it if it's not grapsed (i.e. not too close to the gripper).
                blocks_with_dists.append((block, dist))

        closest_block, closest_dist = min(blocks_with_dists, key=lambda v: v[1])
        return closest_dist

    def step(self) -> Any:
        """Lift the end-effector upward until the success height is reached.

        Returns:
            Self to continue lifting, or None when the success height is reached.
        """
        pose = self.context.robot.arm.get_fk_pq()
        if pose.p[2] >= self.success_z:
            return None

        if self.cautious_command_delta_z is not None and self.closest_non_grasped_block_dist(pose.p) < 0.1:
            # Use the cautious command delta-z if it's specified and we're close to another block.
            pose.p[2] += self.cautious_command_delta_z
        else:
            pose.p[2] += self.command_delta_z

        self.context.robot.arm.send_end_effector(target_pose=pose)
        return self

    def exit(self) -> None:
        """Reset the posture config back to the default value."""
        # On exit, reset the posture config back to the default value.
        self.context.robot.arm.set_posture_config_to_default()


class PickBlockRd(DfStateMachineDecider, DfRldsNode):
    """RLDS node that picks a block when the end-effector is at the chosen grasp."""

    def __init__(self) -> None:
        # This behavior uses the locking feature of the decision framework to run a state machine
        # sequence as an atomic unit.
        super().__init__(
            DfStateSequence(
                [
                    DfSetLockState(set_locked_to=True, decider=self),
                    DfTimedDeciderState(DfCloseGripper(), activity_duration=0.5),
                    LiftState(command_delta_z=0.3, cautious_command_delta_z=0.03, success_delta_z=0.075),
                    DfWriteContextState(lambda ctx: ctx.mark_block_in_gripper()),
                    DfSetLockState(set_locked_to=False, decider=self),
                ]
            )
        )
        self.is_locked = False

    def is_runnable(self) -> bool:
        """Check whether the end-effector is close enough to the grasp to begin picking.

        Returns:
            True if the end-effector is within threshold of the chosen grasp, False otherwise.
        """
        ct = self.context
        if ct.has_active_block and ct.active_block.has_chosen_grasp:
            grasp_T = ct.active_block.chosen_grasp
            eff_T = self.context.robot.arm.get_fk_T()

            thresh_met = math_util.transforms_are_close(grasp_T, eff_T, p_thresh=0.005, R_thresh=0.005)
            return thresh_met

        return False


def make_pick_rlds() -> Any:
    """Construct the RLDS decider for the block picking behavior.

    Returns:
        A configured DfRldsDecider for the block picking behavior.
    """
    rlds = DfRldsDecider()

    open_gripper_rd = OpenGripperRd(dist_thresh_for_open=0.15)
    reach_to_block_rd = ReachToBlockRd()
    choose_block = ChooseNextBlock()
    approach_grasp = DfApproachGrasp()

    reach_to_block_rd.link_to("choose_block", choose_block)
    choose_block.link_to("approach_grasp", approach_grasp)

    rlds.append_rlds_node("reach_to_block", reach_to_block_rd)
    rlds.append_rlds_node("pick_block", PickBlockRd())
    rlds.append_rlds_node("open_gripper", open_gripper_rd)  # Always open the gripper if it's not.

    return rlds


class TablePointValidator:
    """Validate and sample random positions on the table avoiding blocks and the tower.

    Args:
        context: The build-tower context.
    """

    def __init__(self, context: Any) -> None:
        ct = context

        block_pts = [b.obj.get_world_pose()[0] for _, b in ct.blocks.items() if b != context.in_gripper]
        block_pts.append(ct.block_tower.tower_position)
        self.avoid_pts_with_dist_threshs = [(p[:2], 0.15) for p in block_pts]
        self.avoid_pts_with_dist_threshs.append((np.zeros(2), 0.35))

        self.center_p = np.array([0.3, 0.0])
        self.std_devs = np.array([0.2, 0.3])

    def validate_point(self, p: np.ndarray) -> bool:
        """Check whether a 2D point is a valid placement location on the table.

        Args:
            p: The 2D point to validate.

        Returns:
            True if the point is a valid placement location, False otherwise.
        """
        for p_avoid, d_thresh in self.avoid_pts_with_dist_threshs:
            d = np.linalg.norm(p_avoid - p)
            if d < d_thresh:
                return False

            # Lateral check
            if p[1] < 0 or p[1] > 0.3:
                return False

            # Depth check
            if p[0] > 0.7 or p[0] < 0.3:
                return False
        return True

    def sample_random_position_2d(self) -> np.ndarray:
        """Sample a random valid 2D position on the table.

        Returns:
            A 2D numpy array representing a valid table position.
        """
        while True:
            p = self.std_devs * (np.random.randn(2) + self.center_p)
            if self.validate_point(p):
                return p


class ReachToPlaceOnTower(DfDecider):
    """Decider that reaches to place a block on top of the tower."""

    def __init__(self) -> None:
        super().__init__()
        self.add_child("approach_grasp", DfApproachGrasp())

    def decide(self) -> Any:
        """Compute the tower-top grasp and dispatch to approach.

        Returns:
            A DfDecision directing to the approach_grasp child.
        """
        ct = self.context
        ct.placement_target_eff_T = calc_grasp_for_top_of_tower(ct)
        return DfDecision("approach_grasp", ct.placement_target_eff_T)

    def exit(self) -> None:
        """Clear the placement target on exit."""
        self.context.placement_target_eff_T = None


class ReachToPlaceOnTable(DfDecider):
    """Decider that reaches to place a block at a random valid position on the table."""

    def __init__(self) -> None:
        super().__init__()
        self.add_child("approach_grasp", DfApproachGrasp())

    def choose_random_T_on_table(self) -> np.ndarray | None:  # noqa: N802
        """Sample a random valid table position and compute the corresponding grasp.

        Returns:
            The grasp transform for the sampled table position, or None if not found.
        """
        ct = self.context

        table_point_validator = TablePointValidator(self.context)
        rp2d = table_point_validator.sample_random_position_2d()
        rp = np.array([rp2d[0], rp2d[1], ct.block_height / 2])

        ax = -math_util.normalized(np.array([rp[0], rp[1], 0.0]))
        az = np.array([0.0, 0.0, 1.0])
        ay = np.cross(az, ax)
        T = math_util.pack_Rp(math_util.pack_R(ax, ay, az), rp)

        return calc_grasp_for_block_T(ct, T, -T[:3, 3])

    def enter(self) -> None:
        """Choose a random table placement on entry."""
        self.context.placement_target_eff_T = self.choose_random_T_on_table()

    def decide(self) -> Any:
        """Re-sample if the current placement is invalid, then dispatch to approach.

        Returns:
            A DfDecision directing to the approach_grasp child.
        """
        ct = self.context

        table_point_validator = TablePointValidator(self.context)
        if not table_point_validator.validate_point(ct.placement_target_eff_T[:2, 3]):
            ct.placement_target_eff_T = self.choose_random_T_on_table()

        return DfDecision("approach_grasp", ct.placement_target_eff_T)

    def exit(self) -> None:
        """Clear the placement target on exit."""
        self.context.placement_target_eff_T = None


class ReachToPlacementRd(DfRldsNode):
    """RLDS node that decides whether to place on the tower or on the table."""

    def __init__(self) -> None:
        super().__init__()
        self.add_child("reach_to_place_on_tower", ReachToPlaceOnTower())
        self.add_child("reach_to_place_table", ReachToPlaceOnTable())

    def is_runnable(self) -> bool:
        """Check whether the gripper is holding a block.

        Returns:
            True if the gripper is holding a block, False otherwise.
        """
        return self.context.gripper_has_block

    def enter(self) -> None:
        """Clear the placement target on entry."""
        self.context.placement_target_eff_T = None

    def decide(self) -> Any:
        """Decide to place on the tower if the block is correct, otherwise on the table.

        Returns:
            A DfDecision directing to reach_to_place_on_tower or reach_to_place_table.
        """
        ct = self.context

        if ct.block_tower.current_stack_in_correct_order and ct.block_tower.next_block == ct.in_gripper:
            return DfDecision("reach_to_place_on_tower")
        else:
            return DfDecision("reach_to_place_table")


def set_top_block_aligned(ct: Any) -> None:
    """Mark the top block of the tower as aligned.

    Args:
        ct: The build-tower context.
    """
    top_block = ct.block_tower.top_block
    if top_block is not None:
        top_block.set_aligned()


class PlaceBlockRd(DfStateMachineDecider, DfRldsNode):
    """RLDS node that places the held block when the end-effector is at the placement target."""

    def __init__(self) -> None:
        # This behavior uses the locking feature of the decision framework to run a state machine
        # sequence as an atomic unit.
        super().__init__(
            DfStateSequence(
                [
                    DfSetLockState(set_locked_to=True, decider=self),
                    DfTimedDeciderState(DfOpenGripper(), activity_duration=0.5),
                    LiftState(command_delta_z=0.1, success_delta_z=0.03),
                    DfWriteContextState(lambda ctx: ctx.clear_gripper()),
                    DfWriteContextState(set_top_block_aligned),
                    DfTimedDeciderState(DfCloseGripper(), activity_duration=0.25),
                    DfSetLockState(set_locked_to=False, decider=self),
                ]
            )
        )
        self.is_locked = False

    def is_runnable(self) -> bool:
        """Check whether the end-effector is close enough to the placement target.

        Returns:
            True if the end-effector is within threshold of the placement target, False otherwise.
        """
        ct = self.context
        if ct.gripper_has_block and ct.has_placement_target_eff_T:
            eff_T = ct.robot.arm.get_fk_T()

            thresh_met = math_util.transforms_are_close(
                ct.placement_target_eff_T, eff_T, p_thresh=0.005, R_thresh=0.005
            )

            if thresh_met:
                print("<placing block>")
            return thresh_met

        return False

    def exit(self) -> None:
        """Reset the active block and clear the placement target on exit."""
        self.context.reset_active_block()
        self.context.placement_target_eff_T = None


def make_place_rlds() -> Any:
    """Construct the RLDS decider for the block placement behavior.

    Returns:
        A configured DfRldsDecider for the block placement behavior.
    """
    rlds = DfRldsDecider()
    rlds.append_rlds_node("reach_to_placement", ReachToPlacementRd())
    rlds.append_rlds_node("place_block", PlaceBlockRd())
    return rlds


class BlockPickAndPlaceDispatch(DfDecider):
    """Top-level decider that dispatches between picking, placing, and going home."""

    def __init__(self) -> None:
        super().__init__()
        self.add_child("pick", make_pick_rlds())
        self.add_child("place", make_place_rlds())
        self.add_child("go_home", GoHome())

    def decide(self) -> Any:
        """Decide to pick, place, or go home based on tower and gripper state.

        Returns:
            A DfDecision directing to pick, place, or go_home.
        """
        ct = self.context
        if ct.block_tower.is_complete:
            return DfDecision("go_home")

        if ct.is_gripper_clear:
            return DfDecision("pick")
        else:
            return DfDecision("place")


def make_decider_network(robot: Any) -> Any:
    """Create the block stacking decider network for the given robot.

    Args:
        robot: The robot API instance.

    Returns:
        A configured DfNetwork for the block stacking behavior.
    """
    return DfNetwork(
        BlockPickAndPlaceDispatch(), context=BuildTowerContext(robot, tower_position=np.array([0.25, 0.3, 0.0]))
    )

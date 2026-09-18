# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Helper functions for robot simulation tests."""

from __future__ import annotations

import math
from dataclasses import dataclass

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.experimental.prims import Articulation


@dataclass(frozen=True)
class ArcMotionResult:
    """Measurements collected while a robot follows a commanded arc."""

    updates: int
    #: Number of application updates needed to reach the target angle.
    yaw_delta: float
    #: Accumulated yaw change in radians.
    position_delta: np.ndarray
    #: Measured XY displacement from the starting pose.
    expected_position_delta: np.ndarray
    #: Ideal differential-drive XY displacement at the measured yaw.
    max_path_error: float
    #: Maximum XY error from the ideal arc.
    max_displacement: float
    #: Maximum planar displacement from the starting pose.
    linear_velocity: float
    #: Final forward velocity reported by odometry.
    angular_velocity: float
    #: Final yaw velocity reported by odometry.


@dataclass(frozen=True)
class OdometryResult:
    """Odometry measured after a commanded motion stabilizes."""

    updates: int
    #: Number of application updates needed to stabilize.
    linear_velocity: float
    #: Final forward velocity.
    angular_velocity: float
    #: Final yaw velocity.


def _quaternion_to_yaw(orientation: object) -> float:
    """Convert a WXYZ quaternion to yaw in radians."""
    euler_angles = transform_utils.quaternion_to_euler_angles(orientation, device="cpu")
    return float(euler_angles.numpy()[2])


def _normalize_angle(angle: float) -> float:
    """Normalize an angle to the half-open interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


async def wait_for_forward_displacement(
    articulation: Articulation,
    start_position: np.ndarray,
    start_orientation: np.ndarray,
    *,
    minimum_distance: float,
    max_updates: int = 60,
) -> tuple[int, float]:
    """Wait until an articulation moves a minimum distance along its starting forward axis.

    Args:
        articulation: Articulation whose world pose is sampled.
        start_position: World position from which displacement is measured.
        start_orientation: Starting WXYZ world orientation.
        minimum_distance: Forward distance that completes the wait successfully.
        max_updates: Maximum number of application updates allowed.

    Returns:
        Number of updates and measured forward displacement at completion.

    Raises:
        ValueError: If a limit is not positive.
        AssertionError: If the articulation does not move far enough before the update limit.
    """
    if minimum_distance <= 0.0:
        raise ValueError("Minimum distance must be positive")
    if max_updates <= 0:
        raise ValueError("Maximum updates must be positive")

    start_position = np.asarray(start_position)[:2]
    start_yaw = _quaternion_to_yaw(start_orientation)
    forward_axis = np.array([math.cos(start_yaw), math.sin(start_yaw)])
    forward_distance = 0.0
    position_delta = np.zeros(2)
    for update in range(1, max_updates + 1):
        await app_utils.update_app_async()
        positions, _ = articulation.get_world_poses()
        position_delta = positions.numpy()[0, :2] - start_position
        forward_distance = float(np.dot(position_delta, forward_axis))
        if forward_distance > minimum_distance:
            print(f"Robot moved forward {forward_distance:.3f} m in {update} updates")
            return update, forward_distance

    raise AssertionError(
        f"Robot moved {forward_distance:.3f} m forward after {max_updates} updates "
        f"(planar delta={position_delta}); expected more than {minimum_distance:.3f} m"
    )


async def wait_for_stable_odometry(
    odom_node: object,
    *,
    expected_linear_velocity: float | None = None,
    expected_angular_velocity: float | None = None,
    linear_tolerance: float = 0.05,
    angular_tolerance: float = 0.05,
    stable_updates: int = 3,
    max_updates: int = 60,
) -> OdometryResult:
    """Wait until requested odometry values remain within tolerance.

    Args:
        odom_node: Odometry OmniGraph node for the robot.
        expected_linear_velocity: Optional target forward velocity.
        expected_angular_velocity: Optional target yaw velocity.
        linear_tolerance: Allowed error in forward velocity.
        angular_tolerance: Allowed error in yaw velocity.
        stable_updates: Consecutive matching updates required for success.
        max_updates: Maximum number of updates allowed to first enter tolerance.

    Returns:
        Final odometry and the number of updates needed to stabilize.

    Raises:
        ValueError: If no target is provided or a limit is invalid.
        AssertionError: If odometry does not stabilize before the update limit.
    """
    if expected_linear_velocity is None and expected_angular_velocity is None:
        raise ValueError("At least one expected velocity must be provided")
    if linear_tolerance < 0.0 or angular_tolerance < 0.0:
        raise ValueError("Velocity tolerances must be nonnegative")
    if stable_updates <= 0 or max_updates <= 0:
        raise ValueError("Update limits must be positive")

    odom_linear_velocity = og.Controller.attribute("outputs:linearVelocity", odom_node)
    odom_angular_velocity = og.Controller.attribute("outputs:angularVelocity", odom_node)
    matching_updates = 0
    linear_velocity = 0.0
    angular_velocity = 0.0

    for update in range(1, max_updates + stable_updates):
        await app_utils.update_app_async()
        linear_velocity = float(og.DataView.get(odom_linear_velocity)[0])
        angular_velocity = float(og.DataView.get(odom_angular_velocity)[2])
        linear_matches = expected_linear_velocity is None or (
            abs(linear_velocity - expected_linear_velocity) <= linear_tolerance
        )
        angular_matches = expected_angular_velocity is None or (
            abs(angular_velocity - expected_angular_velocity) <= angular_tolerance
        )
        matches = linear_matches and angular_matches
        if matches and (matching_updates > 0 or update <= max_updates):
            matching_updates += 1
        else:
            matching_updates = 0
        if matching_updates >= stable_updates:
            result = OdometryResult(update, linear_velocity, angular_velocity)
            print(
                f"Odometry stabilized in {result.updates} updates: linear velocity={result.linear_velocity:.3f}, "
                f"angular velocity={result.angular_velocity:.3f}"
            )
            return result

    raise AssertionError(
        f"Odometry did not enter and remain within tolerance after {max_updates} updates "
        f"(linear velocity={linear_velocity:.3f}, angular velocity={angular_velocity:.3f})"
    )


async def simulate_arc(
    graph_path: str,
    odom_node: object,
    robot_path: str,
    *,
    forward_velocity: float,
    angular_velocity: float,
    target_angle: float = 2.0 * math.pi,
    max_updates: int = 700,
) -> ArcMotionResult:
    """Drive a differential robot through an arc and collect trajectory measurements.

    The robot stops as soon as its accumulated physical yaw reaches ``target_angle``. The update limit is a failure
    guard and does not determine the successful trajectory length.

    Args:
        graph_path: USD path to the action graph containing the differential controller.
        odom_node: Odometry OmniGraph node for the robot.
        robot_path: USD path to the robot articulation prim.
        forward_velocity: Commanded forward velocity.
        angular_velocity: Commanded yaw velocity.
        target_angle: Absolute yaw change at which to finish the trajectory.
        max_updates: Maximum number of application updates allowed.

    Returns:
        Measurements from the completed arc.

    Raises:
        ValueError: If an input cannot describe a turning trajectory.
        AssertionError: If the robot does not reach the target angle before the update limit.
    """
    if angular_velocity == 0.0:
        raise ValueError("Arc angular velocity must be nonzero")
    if target_angle <= 0.0:
        raise ValueError("Target angle must be positive")
    if max_updates <= 0:
        raise ValueError("Maximum updates must be positive")

    odom_linear_velocity = og.Controller.attribute("outputs:linearVelocity", odom_node)
    odom_angular_velocity = og.Controller.attribute("outputs:angularVelocity", odom_node)

    robot = Articulation(robot_path)
    positions, orientations = robot.get_world_poses()
    start_position = positions.numpy()[0, :2].copy()
    start_yaw = _quaternion_to_yaw(orientations.numpy()[0])
    previous_yaw = start_yaw
    yaw_delta = 0.0
    max_path_error = 0.0
    max_displacement = 0.0
    position_delta = np.zeros(2)
    expected_position_delta = np.zeros(2)
    turn_radius = forward_velocity / angular_velocity

    og.Controller.attribute(graph_path + "/DifferentialController.inputs:linearVelocity").set(forward_velocity)
    og.Controller.attribute(graph_path + "/DifferentialController.inputs:angularVelocity").set(angular_velocity)

    for update in range(1, max_updates + 1):
        await app_utils.update_app_async()

        positions, orientations = robot.get_world_poses()
        current_yaw = _quaternion_to_yaw(orientations.numpy()[0])
        yaw_delta += _normalize_angle(current_yaw - previous_yaw)
        previous_yaw = current_yaw

        position_delta = positions.numpy()[0, :2] - start_position
        max_displacement = max(max_displacement, float(np.linalg.norm(position_delta)))
        body_x = turn_radius * math.sin(yaw_delta)
        body_y = turn_radius * (1.0 - math.cos(yaw_delta))
        expected_position_delta = np.array(
            [
                math.cos(start_yaw) * body_x - math.sin(start_yaw) * body_y,
                math.sin(start_yaw) * body_x + math.cos(start_yaw) * body_y,
            ]
        )
        max_path_error = max(
            max_path_error,
            float(np.linalg.norm(position_delta - expected_position_delta)),
        )

        turned_in_commanded_direction = yaw_delta * angular_velocity > 0.0
        if turned_in_commanded_direction and abs(yaw_delta) >= target_angle:
            result = ArcMotionResult(
                updates=update,
                yaw_delta=yaw_delta,
                position_delta=position_delta,
                expected_position_delta=expected_position_delta,
                max_path_error=max_path_error,
                max_displacement=max_displacement,
                linear_velocity=float(og.DataView.get(odom_linear_velocity)[0]),
                angular_velocity=float(og.DataView.get(odom_angular_velocity)[2]),
            )
            print(
                f"Arc completed in {result.updates} updates: yaw={result.yaw_delta:.3f}, "
                f"position={result.position_delta}, expected={result.expected_position_delta}, "
                f"max path error={result.max_path_error:.3f}, max displacement={result.max_displacement:.3f}, "
                f"linear velocity={result.linear_velocity:.3f}, "
                f"angular velocity={result.angular_velocity:.3f}"
            )
            return result

    measured_linear_velocity = float(og.DataView.get(odom_linear_velocity)[0])
    measured_angular_velocity = float(og.DataView.get(odom_angular_velocity)[2])
    raise AssertionError(
        f"Robot reached {yaw_delta:.3f} rad after {max_updates} updates; expected {target_angle:.3f} rad "
        f"in the commanded direction (linear velocity={measured_linear_velocity:.3f}, "
        f"angular velocity={measured_angular_velocity:.3f})"
    )


async def init_robot_sim(art_path: str, graph_path: str = "/ActionGraph") -> None:
    """Initialize robot simulation by resetting pose and velocities.

    Creates an articulation at the given path, resets its position, orientation,
    and velocities, then resets the differential controller inputs.

    Args:
        art_path: USD path to the robot articulation prim.
        graph_path: USD path to the OmniGraph containing the controller.
    """
    art = Articulation(art_path)
    # Wait for physics to be ready (replaces _articulation_view.initialize())
    await app_utils.update_app_async()
    # reset position and orientation (wxyz format for experimental API)
    art.set_world_poses(positions=[[0, 0, 0.1]], orientations=[[1, 0, 0, 0]])
    # reset velocities
    art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
    # reset controller
    og.Controller.attribute(graph_path + "/DifferentialController.inputs:linearVelocity").set(0)
    og.Controller.attribute(graph_path + "/DifferentialController.inputs:angularVelocity").set(0)
    # wait for robot to drop
    for i in range(10):
        await app_utils.update_app_async()

    return


def setup_robot_og(
    graph_path: str,
    lwheel_name: str,
    rwheel_name: str,
    robot_path: str,
    wheel_rad: float,
    wheel_dist: float,
) -> object:
    """Set up OmniGraph for differential drive robot control.

    Creates an action graph with playback tick, differential controller,
    articulation controller, and odometry computation nodes.

    Args:
        graph_path: USD path where the graph will be created.
        lwheel_name: Name of the left wheel joint.
        rwheel_name: Name of the right wheel joint.
        robot_path: USD path to the robot prim.
        wheel_rad: Wheel radius in meters.
        wheel_dist: Distance between wheels in meters.

    Returns:
        Tuple of (graph, odom_node) where graph is the created OmniGraph
        and odom_node is the odometry computation node.
    """
    keys = og.Controller.Keys
    graph, nodes, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                (
                    "DifferentialController",
                    "isaacsim.robot.wheeled_robots.DifferentialController",
                ),
                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                ("computeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
            ],
            keys.SET_VALUES: [
                (
                    "ArticulationController.inputs:jointNames",
                    [lwheel_name, rwheel_name],
                ),
                ("ArticulationController.inputs:robotPath", robot_path),
                ("DifferentialController.inputs:wheelRadius", wheel_rad),
                ("DifferentialController.inputs:wheelDistance", wheel_dist),
                ("computeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(robot_path)]),
            ],
        },
    )

    return graph, nodes[3]

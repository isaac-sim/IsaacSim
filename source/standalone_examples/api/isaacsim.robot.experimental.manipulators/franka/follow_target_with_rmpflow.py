# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Franka follow-target using cuMotion RmpFlowController.

The Franka robot arm tracks a draggable target cube using RMPflow for
real-time motion planning with optional obstacle avoidance.

Usage:
    python follow_target_with_rmpflow.py                     # interactive, GPU physics
    python follow_target_with_rmpflow.py --with-obstacle     # add obstacle cube
    python follow_target_with_rmpflow.py --device cpu        # CPU physics
"""

from __future__ import annotations

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true", help="Run in test mode (exit after N frames)")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cuda", help="Simulation device")
parser.add_argument("--with-obstacle", action="store_true", help="Add static obstacle cube for avoidance demo")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot.experimental.manipulators.examples", True
)

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import warp as wp
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, Mesh
from isaacsim.core.experimental.prims import Articulation, GeomPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.manipulators.examples.franka import FrankaFollowTarget
from isaacsim.robot_motion.cumotion import (
    CumotionRobot,
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_supported_robot,
)

ROBOT_PRIM_PATH = "/World/robot"
TARGET_PATH = "/World/TargetCube"


def get_estimated_state(articulation: Articulation) -> mg.RobotState:
    """Wrap current joint positions and velocities into a RobotState.

    Args:
        articulation: The robot articulation to read state from.

    Returns:
        Robot state containing current joint positions and velocities.
    """
    names = articulation.dof_names
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=names,
            positions=(names, articulation.get_dof_positions()),
            velocities=(names, articulation.get_dof_velocities()),
        )
    )


def create_setpoint_state(cumotion_robot: CumotionRobot, target_object: GeomPrim) -> mg.RobotState:
    """Build the desired end-effector state from the target cube pose.

    Args:
        cumotion_robot: The cuMotion robot description used for tool frame names.
        target_object: The draggable target cube prim.

    Returns:
        Robot state with the desired end-effector site pose.
    """
    tool_frame = cumotion_robot.robot_description.tool_frame_names()[0]
    site_space = cumotion_robot.robot_description.tool_frame_names()

    target_positions, _ = target_object.get_world_poses()

    n = int(target_positions.shape[0])
    target_orientations = wp.array([[0.0, 1.0, 0.0, 0.0]] * n, dtype=wp.float32, device=target_positions.device)

    return mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=site_space,
            positions=([tool_frame], target_positions),
            orientations=([tool_frame], target_orientations),
        ),
    )


def setup_scene_and_controller() -> tuple[RmpFlowController, CumotionRobot, Articulation, mg.WorldBinding, GeomPrim]:
    """Spawn the Franka scene and configure the RMPflow controller.

    Returns:
        Tuple of (controller, cumotion_robot, articulation, world_binding, target_object).
    """
    follow = FrankaFollowTarget()
    follow.setup_scene(target_position=[0.5, 0.0, 0.3])
    articulation = Articulation(ROBOT_PRIM_PATH)
    target_object = follow.target_cube

    if args.with_obstacle:
        Cube("/World/obstacle", sizes=0.05, positions=[0.35, 0.0, 0.45])
        GeomPrim("/World/obstacle", apply_collision_apis=True)

    scene_query = mg.SceneQuery()
    robot_pos, robot_ori = articulation.get_world_poses()
    objects = scene_query.get_prims_in_aabb(
        search_box_origin=robot_pos.numpy()[0],
        search_box_minimum=[-10.0, -10.0, -10.0],
        search_box_maximum=[10.0, 10.0, 10.0],
        tracked_api=mg.TrackableApi.PHYSICS_COLLISION,
        exclude_prim_paths=[ROBOT_PRIM_PATH, TARGET_PATH],
    )

    obstacle_strategy = mg.ObstacleStrategy()
    obstacle_strategy.set_default_configuration(Mesh, mg.ObstacleConfiguration("obb", 0.01))
    obstacle_strategy.set_default_configuration(Cone, mg.ObstacleConfiguration("obb", 0.01))
    obstacle_strategy.set_default_configuration(Cylinder, mg.ObstacleConfiguration("obb", 0.01))

    world_binding = mg.WorldBinding(
        world_interface=CumotionWorldInterface(),
        obstacle_strategy=obstacle_strategy,
        tracked_prims=objects,
        tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
    )
    world_binding.initialize()
    world_binding.get_world_interface().update_world_to_robot_root_transforms(poses=(robot_pos, robot_ori))
    world_binding.synchronize_transforms()

    cumotion_robot = load_cumotion_supported_robot("franka")
    joint_space = articulation.dof_names
    site_space = cumotion_robot.robot_description.tool_frame_names()
    controller = RmpFlowController(
        cumotion_robot=cumotion_robot,
        cumotion_world_interface=world_binding.get_world_interface(),
        robot_joint_space=joint_space,
        robot_site_space=site_space,
        tool_frame=site_space[0],
    )

    return controller, cumotion_robot, articulation, world_binding, target_object


def reset_rmpflow(
    controller: RmpFlowController,
    cumotion_robot: CumotionRobot,
    articulation: Articulation,
    target_object: GeomPrim,
    t: float,
) -> None:
    """Initialize the RMPflow controller with current state and target.

    Args:
        controller: The RMPflow controller to reset.
        cumotion_robot: The cuMotion robot description.
        articulation: The robot articulation.
        target_object: The draggable target cube prim.
        t: Current simulation time in seconds.

    Raises:
        RuntimeError: If the controller reset fails.
    """
    estimated = get_estimated_state(articulation)
    setpoint = create_setpoint_state(cumotion_robot, target_object)
    if not controller.reset(estimated, setpoint, t=t):
        raise RuntimeError("RmpFlowController reset failed.")


def run_step(
    controller: RmpFlowController,
    cumotion_robot: CumotionRobot,
    articulation: Articulation,
    world_binding: mg.WorldBinding,
    target_object: GeomPrim,
    t: float,
) -> None:
    """Execute one simulation step.

    Args:
        controller: The RMPflow controller.
        cumotion_robot: The cuMotion robot description.
        articulation: The robot articulation.
        world_binding: The world binding for obstacle tracking.
        target_object: The draggable target cube prim.
        t: Current simulation time in seconds.
    """
    world_binding.get_world_interface().update_world_to_robot_root_transforms(articulation.get_world_poses())
    world_binding.synchronize_transforms()

    estimated = get_estimated_state(articulation)
    setpoint = create_setpoint_state(cumotion_robot, target_object)
    desired = controller.forward(estimated, setpoint, t)

    if desired is not None and desired.joints.positions is not None:
        articulation.set_dof_position_targets(
            positions=desired.joints.positions,
            dof_indices=desired.joints.position_indices,
        )


def main() -> None:
    """Run the Franka follow-target loop with cuMotion RMPflow."""
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device=args.device)

    controller, cumotion_robot, articulation, world_binding, target_object = setup_scene_and_controller()
    simulation_app.update()

    app_utils.play()
    simulation_app.update()

    physics_dt = 1.0 / 60.0
    t = 0.0

    print("Franka follow-target (cuMotion RMPflow)")
    print("  Select /World/TargetCube and drag it while playing.")
    if args.with_obstacle:
        print("  Obstacle avoidance enabled (/World/obstacle).")

    rmpflow_reset_needed = True
    frame = 0
    while simulation_app.is_running():
        simulation_app.update()
        if app_utils.is_playing() and SimulationManager.is_simulating():
            if rmpflow_reset_needed:
                t = 0.0
                reset_rmpflow(controller, cumotion_robot, articulation, target_object, t)
                rmpflow_reset_needed = False
            else:
                t += physics_dt
                run_step(controller, cumotion_robot, articulation, world_binding, target_object, t)
            frame += 1
            if args.test and frame >= 10:
                break
        elif not app_utils.is_playing():
            rmpflow_reset_needed = True


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        simulation_app.close()

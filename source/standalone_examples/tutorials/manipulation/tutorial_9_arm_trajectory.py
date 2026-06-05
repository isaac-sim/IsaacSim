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

"""Tutorial 9, Part 2: Arm Trajectory Following

Plans and executes a joint-space arm trajectory using
Path.to_minimal_time_joint_trajectory() and TrajectoryFollower from the
Motion Generation API.
"""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
parser.add_argument("--headless", action="store_true")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless, "hide_ui": False})

if args.headless:
    from isaacsim.core.experimental.utils.app import enable_extension

    simulation_app.set_setting("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.app")

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.app
import warp as wp
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path_async

_HOME: np.ndarray = np.array([0.00, -1.57, 1.57, -1.57, -1.57, 0.00], dtype=np.float32)

# ========================================================


async def setup_scene() -> Articulation:
    assets_root_path = await get_assets_root_path_async()
    stage_utils.add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd",
        path="/World/ur",
    )

    GroundPlane("/World/GroundPlane")
    DomeLight("/World/DomeLight").set_intensities(1000)

    await omni.kit.app.get_app().next_update_async()
    set_camera_view(eye=[1.5, 1.5, 1.5], target=[0.5, 0.0, 0.5], camera_prim_path="/OmniverseKit_Persp")
    robot = Articulation("/World/ur")
    await omni.kit.app.get_app().next_update_async()
    return robot


def get_estimated_state(robot: Articulation, joint_space: list[str]) -> mg.RobotState:
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=joint_space,
            positions=(joint_space, robot.get_dof_positions()),
            velocities=(joint_space, robot.get_dof_velocities()),
            efforts=(joint_space, robot.get_dof_efforts()),
        )
    )


def apply_desired_state(robot: Articulation, desired_state: mg.RobotState) -> None:
    if desired_state.joints is None:
        return
    joint_state = desired_state.joints
    if joint_state.positions is not None:
        robot.set_dof_position_targets(joint_state.positions, dof_indices=joint_state.position_indices)
    if joint_state.velocities is not None:
        robot.set_dof_velocity_targets(joint_state.velocities, dof_indices=joint_state.velocity_indices)
    if joint_state.efforts is not None:
        robot.set_dof_efforts(joint_state.efforts, dof_indices=joint_state.effort_indices)


# ========================================================


def main(args: argparse.Namespace, app: SimulationApp) -> None:
    SimulationManager.setup_simulation(dt=1.0 / 60.0)

    robot = app.run_coroutine(setup_scene())
    app.update()

    if args.headless:
        print("Headless mode: simulation is paused. Press Play in the livestream UI to begin.")
        while app.is_running() and not app_utils.is_playing():
            app.update()
    else:
        app_utils.play()
        app.update()

    robot_joint_space = robot.dof_names
    arm_joints = [n for n in robot_joint_space if "finger" not in n and "knuckle" not in n]
    print(f"Arm joints ({len(arm_joints)}): {arm_joints}")

    initial_positions = np.zeros(robot.num_dofs, dtype=np.float32)
    for i, joint_name in enumerate(arm_joints):
        initial_positions[robot_joint_space.index(joint_name)] = _HOME[i]
    robot.set_dof_positions(wp.from_numpy(initial_positions, dtype=wp.float32))
    app.update()

    # <start-arm-trajectory-setup-snippet>
    waypoints = np.array(
        [
            [0.00, -1.57, 1.57, -1.57, -1.57, 0.00],  # home
            [0.50, -1.00, 0.80, -1.30, -1.57, 0.00],  # reach-out
            [0.00, -1.57, 1.57, -1.57, -1.57, 0.00],  # back to home
        ],
    )

    max_velocities = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    max_accelerations = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

    trajectory = mg.Path(waypoints).to_minimal_time_joint_trajectory(
        max_velocities=max_velocities,
        max_accelerations=max_accelerations,
        robot_joint_space=robot_joint_space,
        active_joints=arm_joints,
    )
    print(f"Trajectory duration: {trajectory.duration:.2f} s")

    follower = mg.TrajectoryFollower()
    follower.set_trajectory(trajectory)

    simulation_time = 0.0
    if not follower.reset(get_estimated_state(robot, robot_joint_space), None, simulation_time):
        raise RuntimeError("Failed to reset TrajectoryFollower")
    # <end-arm-trajectory-setup-snippet>

    # <start-arm-trajectory-loop-snippet>
    dt = SimulationManager.get_physics_dt()
    max_steps = int((trajectory.duration + 1.0) / dt)
    frame_count = 0

    while app.is_running():
        app.update()
        if not (app_utils.is_playing() and SimulationManager.is_simulating()):
            continue
        simulation_time = 0.0
        follower.reset(get_estimated_state(robot, robot_joint_space), None, simulation_time)
        for _ in range(max_steps):
            app.update()
            if not (app_utils.is_playing() and SimulationManager.is_simulating()):
                break
            simulation_time += dt
            desired_state = follower.forward(get_estimated_state(robot, robot_joint_space), None, simulation_time)
            if desired_state is None:
                print("Trajectory complete.")
                break
            apply_desired_state(robot, desired_state)
            frame_count += 1
            if args.test and frame_count >= 100:
                return
    # <end-arm-trajectory-loop-snippet>


if __name__ == "__main__":
    try:
        main(args, simulation_app)
    except Exception:
        import traceback

        traceback.print_exc()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        simulation_app.close()

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

"""Tutorial 9, Part 3a: Follow Target

A UR10e arm tracks a draggable target cube using cuMotion RmpFlowController
for real-time motion planning with optional obstacle avoidance.
"""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cuda")
parser.add_argument("--with-obstacle", action="store_true")
parser.add_argument(
    "--xrdf-dir",
    type=str,
    default=None,
    help="Directory containing URDF/XRDF robot config files; omit to use the built-in ur10 model",
)
parser.add_argument("--urdf", type=str, default="robot.urdf")
parser.add_argument("--xrdf", type=str, default="robot.xrdf")
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
import omni.kit.app
import warp as wp
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, DomeLight, GroundPlane, Mesh
from isaacsim.core.experimental.prims import Articulation, GeomPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot_motion.cumotion import (
    CumotionRobot,
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_robot,
    load_cumotion_supported_robot,
)
from isaacsim.storage.native import get_assets_root_path_async

_ROBOT_PRIM_PATH = "/World/ur10e_robot"
_TARGET_PATH = "/World/TargetCube"

# ========================================================


def get_estimated_state(articulation: Articulation) -> mg.RobotState:
    names = articulation.dof_names
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=names,
            positions=(names, articulation.get_dof_positions()),
            velocities=(names, articulation.get_dof_velocities()),
        )
    )


def create_setpoint_state(cumotion_robot: CumotionRobot, target_object: GeomPrim) -> mg.RobotState:
    tool_frame = cumotion_robot.robot_description.tool_frame_names()[0]
    site_space = cumotion_robot.robot_description.tool_frame_names()
    target_positions, _ = target_object.get_world_poses()
    n = int(target_positions.shape[0])
    target_orientations = wp.array([[0.0, 0.0, 1.0, 0.0]] * n, dtype=wp.float32, device=target_positions.device)
    return mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=site_space,
            positions=([tool_frame], target_positions),
            orientations=([tool_frame], target_orientations),
        ),
    )


# <start-follow-target-setup-snippet>
async def setup_scene_and_controller(
    with_obstacle: bool,
) -> tuple[RmpFlowController, CumotionRobot, Articulation, mg.WorldBinding, GeomPrim]:
    assets_root_path = await get_assets_root_path_async()
    stage_utils.add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd",
        path=_ROBOT_PRIM_PATH,
    )

    GroundPlane("/World/GroundPlane")
    DomeLight("/World/DomeLight").set_intensities(1000)

    target_cube = Cube(paths=_TARGET_PATH, positions=[[0.35, 0.25, 0.3]], sizes=1.0, scales=[0.05, 0.05, 0.05])
    target_object = GeomPrim(paths=target_cube.paths)

    await omni.kit.app.get_app().next_update_async()
    set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.5, 0.0, 0.2], camera_prim_path="/OmniverseKit_Persp")

    articulation = Articulation(_ROBOT_PRIM_PATH)
    await omni.kit.app.get_app().next_update_async()

    if with_obstacle:
        Cube("/World/obstacle", sizes=0.05, positions=[0.35, 0.0, 0.55], colors=(1.0, 0.0, 0.0))
        GeomPrim("/World/obstacle", apply_collision_apis=True)

    robot_pos, robot_ori = articulation.get_world_poses()
    objects = mg.SceneQuery().get_prims_in_aabb(
        search_box_origin=robot_pos.numpy()[0],
        search_box_minimum=[-10.0, -10.0, -10.0],
        search_box_maximum=[10.0, 10.0, 10.0],
        tracked_api=mg.TrackableApi.PHYSICS_COLLISION,
        exclude_prim_paths=[_ROBOT_PRIM_PATH, _TARGET_PATH],
    )

    obstacle_strategy = mg.ObstacleStrategy()
    for prim_type in (Mesh, Cone, Cylinder, Cube):
        obstacle_strategy.set_default_configuration(prim_type, mg.ObstacleConfiguration("obb", 0.05))

    world_binding = mg.WorldBinding(
        world_interface=CumotionWorldInterface(),
        obstacle_strategy=obstacle_strategy,
        tracked_prims=objects,
        tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
    )
    world_binding.initialize()
    world_binding.get_world_interface().update_world_to_robot_root_transforms(poses=(robot_pos, robot_ori))
    world_binding.synchronize_transforms()

    if args.xrdf_dir is not None:
        cumotion_robot = load_cumotion_robot(
            directory=args.xrdf_dir,
            urdf_filename=args.urdf,
            xrdf_filename=args.xrdf,
        )
    else:
        cumotion_robot = load_cumotion_supported_robot("ur10")
    site_space = cumotion_robot.robot_description.tool_frame_names()
    controller = RmpFlowController(
        cumotion_robot=cumotion_robot,
        cumotion_world_interface=world_binding.get_world_interface(),
        robot_joint_space=articulation.dof_names,
        robot_site_space=site_space,
        tool_frame=site_space[0],
    )
    controller.get_rmp_flow_config().set_param("cspace_target_rmp/metric_scalar", 1.0)
    controller.get_rmp_flow_config().set_param("collision_rmp/metric_scalar", 10000.0)

    return controller, cumotion_robot, articulation, world_binding, target_object


# <end-follow-target-setup-snippet>


def reset_rmpflow(
    controller: RmpFlowController,
    cumotion_robot: CumotionRobot,
    articulation: Articulation,
    target_object: GeomPrim,
    t: float,
) -> None:
    estimated = get_estimated_state(articulation)
    setpoint = create_setpoint_state(cumotion_robot, target_object)
    if not controller.reset(estimated, setpoint, t=t):
        raise RuntimeError("RmpFlowController reset failed.")


# <start-follow-target-loop-snippet>
def run_step(
    controller: RmpFlowController,
    cumotion_robot: CumotionRobot,
    articulation: Articulation,
    world_binding: mg.WorldBinding,
    target_object: GeomPrim,
    t: float,
) -> None:
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


# <end-follow-target-loop-snippet>


# ========================================================


def main(args: argparse.Namespace, app: SimulationApp) -> None:
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device=args.device)

    controller, cumotion_robot, articulation, world_binding, target_object = app.run_coroutine(
        setup_scene_and_controller(args.with_obstacle)
    )
    app.update()

    if args.headless:
        print("Headless mode: simulation is paused. Press Play in the livestream UI to begin.")
        while app.is_running() and not app_utils.is_playing():
            app.update()
    else:
        app_utils.play()
        app.update()

    print("UR10e follow-target (cuMotion RMPflow)")
    print("  Select /World/TargetCube and drag it while playing.")
    if args.with_obstacle:
        print("  Obstacle avoidance enabled (/World/obstacle).")

    physics_dt = 1.0 / 60.0
    t = 0.0
    reset_needed = True
    frame_count = 0
    while app.is_running():
        app.update()
        if app_utils.is_playing() and SimulationManager.is_simulating():
            if reset_needed:
                t = 0.0
                reset_rmpflow(controller, cumotion_robot, articulation, target_object, t)
                reset_needed = False
            else:
                t += physics_dt
                run_step(controller, cumotion_robot, articulation, world_binding, target_object, t)
            frame_count += 1
            if args.test and frame_count >= 100:
                break
        elif not app_utils.is_playing():
            reset_needed = True


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

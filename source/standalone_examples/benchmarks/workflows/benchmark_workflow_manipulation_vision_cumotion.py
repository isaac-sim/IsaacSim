# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Workflow benchmark: manipulation with RGB + depth cameras; cuMotion or joint replay at 30 Hz."""

from __future__ import annotations

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument(
    "--control",
    choices=["cumotion", "replay"],
    default="cumotion",
    help="cumotion: RmpFlowController; replay: sinusoidal joint targets without planner",
)
parser.add_argument(
    "--rgb-preset",
    choices=["1080p", "realsense_rgb"],
    default="1080p",
    help="RGB render product resolution preset",
)
parser.add_argument("--rgb-fps", type=float, default=30.0, help="Target tick rate for RGB camera (Hz)")
parser.add_argument("--depth-fps", type=float, default=30.0, help="Target tick rate for depth camera (Hz)")
parser.add_argument("--num-frames", type=int, default=300, help="Benchmark frames (app updates)")
parser.add_argument(
    "--planner-warmup-frames",
    type=int,
    default=0,
    help="If >0 and control=cumotion, run this many planner steps in a separate phase without frametime recording",
)
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs")
parser.add_argument("--gpu-frametime", action="store_true", help="Enable GPU frametime measurement")
parser.add_argument("--non-headless", action="store_true", help="Run with GUI")
parser.add_argument(
    "--viewport-updates",
    dest="disable_viewport_updates",
    action="store_false",
    default=True,
    help="Enable viewport updates when headless",
)
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend",
)
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cuda", help="Physics device")
parser.add_argument("--skip-write", action="store_true", help="Do not attach BasicWriter (faster CI)")

args, _ = parser.parse_known_args()

headless = not args.non_headless
n_gpu = args.num_gpus
gpu_frametime = args.gpu_frametime
disable_viewport_updates = args.disable_viewport_updates
num_frames = args.num_frames
control = args.control
planner_warmup_frames = args.planner_warmup_frames
rgb_fps = args.rgb_fps
depth_fps = args.depth_fps

if args.rgb_preset == "1080p":
    rgb_w, rgb_h = 1920, 1080
else:
    rgb_w, rgb_h = 1280, 800

depth_w, depth_h = 848, 480

extra_args = []
if rgb_fps > 0 or depth_fps > 0:
    extra_args.append("--/rtx/hydra/supportMultiTickRate=true")

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": headless,
        "max_gpu_count": n_gpu,
        "disable_viewport_updates": disable_viewport_updates,
        "extra_args": extra_args,
    }
)

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import warp as wp
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, DomeLight, GroundPlane, Mesh
from isaacsim.core.experimental.prims import Articulation, GeomPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.robot_motion.cumotion import (
    CumotionRobot,
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_supported_robot,
)
from isaacsim.storage.native import get_assets_root_path

enable_extension("isaacsim.benchmark.services")

from isaacsim.benchmark.services import DEFAULT_RECORDERS, BaseIsaacBenchmark

ROBOT_PRIM_PATH = "/World/robot"
TARGET_PATH = "/World/TargetCube"
FRANKA_USD_PATH = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
FRANKA_DEFAULT_DOF_POSITIONS = [0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741, 0.04, 0.04]

recorders = list(DEFAULT_RECORDERS) + (["gpu_frametime"] if gpu_frametime else [])
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_workflow_manipulation_vision_cumotion",
    workflow_metadata={
        "metadata": [
            {"name": "control", "data": control},
            {"name": "rgb_preset", "data": args.rgb_preset},
            {"name": "rgb_resolution", "data": f"{rgb_w}x{rgb_h}"},
            {"name": "depth_resolution", "data": f"{depth_w}x{depth_h}"},
            {"name": "rgb_fps", "data": rgb_fps},
            {"name": "depth_fps", "data": depth_fps},
            {"name": "planner_warmup_frames", "data": planner_warmup_frames},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
    recorders=recorders,
)

benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

SimulationManager.setup_simulation(dt=1.0 / 60.0, device=args.device)

GroundPlane("/World/ground_plane")
DomeLight("/World/DomeLight").set_intensities(1000)
assets_root = get_assets_root_path()
if assets_root is None:
    raise RuntimeError("Could not resolve Isaac Sim assets root path.")
stage_utils.add_reference_to_stage(
    usd_path=assets_root + FRANKA_USD_PATH,
    path=ROBOT_PRIM_PATH,
    variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
)
articulation = Articulation(ROBOT_PRIM_PATH)
articulation.set_default_state(dof_positions=FRANKA_DEFAULT_DOF_POSITIONS)
cube_shape = Cube(
    paths=TARGET_PATH,
    positions=[0.5, 0.0, 0.3],
    orientations=[1, 0, 0, 0],
    sizes=0.05,
    colors="red",
)
target_object = GeomPrim(paths=cube_shape.paths)

cam_rgb = rep.create.camera(
    name="workflow_rgb",
    position=[1.2, 1.0, 0.9],
    rotation=[75, 0, 20],
    tick_rate=rgb_fps,
)
cam_depth = rep.create.camera(
    name="workflow_depth",
    position=[1.0, -0.8, 0.85],
    rotation=[80, 0, -15],
    tick_rate=depth_fps,
)
rp_rgb = rep.create.render_product(cam_rgb, (rgb_w, rgb_h), name="rp_workflow_rgb")
rp_depth = rep.create.render_product(cam_depth, (depth_w, depth_h), name="rp_workflow_depth")

if not args.skip_write:
    w_rgb = rep.writers.get("BasicWriter")
    w_rgb.initialize(output_dir=os.path.join(os.getcwd(), "_out_workflow_manip_rgb"), rgb=True)
    w_rgb.attach([rp_rgb])
    w_depth = rep.writers.get("BasicWriter")
    w_depth.initialize(
        output_dir=os.path.join(os.getcwd(), "_out_workflow_manip_depth"),
        distance_to_image_plane=True,
    )
    w_depth.attach([rp_depth])

rep.orchestrator.preview()
omni.kit.app.get_app().update()

controller = None
cumotion_robot = None
world_binding = None

if control == "cumotion":

    def get_estimated_state(art: Articulation) -> mg.RobotState:
        """Build a robot state from the articulation's current joint state."""
        names = art.dof_names
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=names,
                positions=(names, art.get_dof_positions()),
                velocities=(names, art.get_dof_velocities()),
            )
        )

    def create_setpoint_state(cr: CumotionRobot, tgt: GeomPrim) -> mg.RobotState:
        """Build a target robot state from the cuMotion tool frame and target prim."""
        tool_frame = cr.robot_description.tool_frame_names()[0]
        site_space = cr.robot_description.tool_frame_names()
        target_positions, _ = tgt.get_world_poses()
        n = int(target_positions.shape[0])
        target_orientations = wp.array([[0.0, 1.0, 0.0, 0.0]] * n, dtype=wp.float32, device=target_positions.device)
        return mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=site_space,
                positions=([tool_frame], target_positions),
                orientations=([tool_frame], target_orientations),
            ),
        )

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

simulation_app.update()
app_utils.play()
simulation_app.update()
articulation.reset_to_default_state()

physics_dt = 1.0 / 60.0
t = 0.0

if control == "cumotion" and controller is not None and cumotion_robot is not None and world_binding is not None:
    est0 = get_estimated_state(articulation)
    sp0 = create_setpoint_state(cumotion_robot, target_object)
    if not controller.reset(est0, sp0, t=0.0):
        raise RuntimeError("RmpFlowController reset failed.")

dof_count = len(articulation.dof_names)
center = articulation.get_dof_positions().numpy().flatten().astype(np.float32)
amp = np.full(dof_count, 0.08, dtype=np.float32)

benchmark.store_measurements()

if control == "cumotion" and planner_warmup_frames > 0:
    benchmark.set_phase("planner_warmup", start_recording_frametime=False, start_recording_runtime=True)
    for _ in range(planner_warmup_frames):
        if world_binding is not None and controller is not None and cumotion_robot is not None:
            world_binding.get_world_interface().update_world_to_robot_root_transforms(articulation.get_world_poses())
            world_binding.synchronize_transforms()
            est = get_estimated_state(articulation)
            sp = create_setpoint_state(cumotion_robot, target_object)
            desired = controller.forward(est, sp, t)
            t += physics_dt
            if desired is not None and desired.joints.positions is not None:
                articulation.set_dof_position_targets(
                    positions=desired.joints.positions,
                    dof_indices=desired.joints.position_indices,
                )
        simulation_app.update()
    benchmark.store_measurements()
    t = 0.0

benchmark.set_phase("benchmark")

for frame_idx in range(num_frames):
    if control == "replay":
        tau = frame_idx * physics_dt
        targets = center + amp * np.sin(2.0 * np.pi * 0.25 * tau)
        articulation.set_dof_position_targets(np.asarray(targets, dtype=np.float32).reshape(1, -1))
    elif control == "cumotion" and controller is not None and cumotion_robot is not None and world_binding is not None:
        world_binding.get_world_interface().update_world_to_robot_root_transforms(articulation.get_world_poses())
        world_binding.synchronize_transforms()
        est = get_estimated_state(articulation)
        sp = create_setpoint_state(cumotion_robot, target_object)
        desired = controller.forward(est, sp, t)
        t += physics_dt
        if desired is not None and desired.joints.positions is not None:
            articulation.set_dof_position_targets(
                positions=desired.joints.positions,
                dof_indices=desired.joints.position_indices,
            )

    simulation_app.update()

benchmark.store_measurements()
benchmark.stop()

if not args.skip_write:
    rep.orchestrator.wait_until_complete()

rp_rgb.destroy()
rp_depth.destroy()
app_utils.pause()
simulation_app.close()

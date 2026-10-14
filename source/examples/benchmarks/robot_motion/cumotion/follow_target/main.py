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

"""Benchmark cumotion follow_target control and physics throughput."""

from __future__ import annotations

import argparse
import os

import isaacsim.physics_engines.ovphysx  # noqa: F401
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.foundation.objects import Cube, DistantLight, PhysicsScene, PhysxScene, Stage
from isaacsim.foundation.prims import GroundPlane
from isaacsim.performance.benchmarking import benchmark
from isaacsim.physics.entities import ArticulationEntity
from isaacsim.physics.manager import PhysicsManager
from isaacsim.robot_motion.cumotion import CumotionWorldInterface, RmpFlowController, load_cumotion_supported_robot

ENGINE = "ovphysx"
ROBOT_PATH = "/World/Franka"
TARGET_PATH = "/World/Target"
TOOL_FRAME = "panda_leftfingertip"
TARGET_POSITION = np.asarray([0.45, 0.0, 0.5], dtype=np.float32)
DEFAULT_JOINT_POSITIONS = {
    "panda_joint1": 0.012,
    "panda_joint2": -0.568,
    "panda_joint3": 0.0,
    "panda_joint4": -2.811,
    "panda_joint5": 0.0,
    "panda_joint6": 3.037,
    "panda_joint7": 0.741,
    "panda_finger_joint1": 0.04,
    "panda_finger_joint2": 0.04,
}
_DEFAULT_ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
_ASSET_ROOT = (os.environ.get("ISAACSIM_ASSET_ROOT") or _DEFAULT_ASSET_ROOT).rstrip("/\\")
FRANKA_USD = f"{_ASSET_ROOT}/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"


def _author_stage(physics_device: str) -> str:
    """Create, populate, export, and close the standalone Franka stage."""
    from isaacsim.ovgl_viewport.debug import author_viewport
    from pxr import Sdf

    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")
        GroundPlane("/World/Ground", sizes=2.2, colors=[[0.22, 0.24, 0.28]]).set_local_poses(
            translations=[0.3, 0.0, 0.0]
        )
        Cube(TARGET_PATH, sizes=0.04, colors=[[0.85, 0.12, 0.08]]).set_local_poses(
            translations=TARGET_POSITION.tolist()
        )
        PhysicsScene("/World/PhysicsScene").set_gravities([0.0, 0.0, -9.81])
        scene = PhysxScene("/World/PhysicsScene")
        scene.set_enabled_gpu_dynamics(np.array([physics_device == "gpu"]))
        scene.set_broadphase_types("GPU" if physics_device == "gpu" else "MBP")
        light = DistantLight("/World/KeyLight")
        light.set_intensities([[3500.0]])
        light.set_colors([[1.0, 0.93, 0.82]])
        author_viewport(stage)
        authored_stage_text = stage.export_stage_to_string()
    finally:
        stage.close_stage()
    detached_layer = Sdf.Layer.CreateAnonymous("follow_target.usda")
    if not detached_layer.ImportFromString(authored_stage_text):
        raise RuntimeError("OpenUSD could not reconstruct the authored stage layer.")
    robot_spec = Sdf.CreatePrimInLayer(detached_layer, ROBOT_PATH)
    robot_spec.specifier = Sdf.SpecifierDef
    robot_spec.typeName = "Xform"
    robot_spec.referenceList.prependedItems = [Sdf.Reference(FRANKA_USD)]
    return detached_layer.ExportToString()


def _make_setpoint(robot_site_space: list[str], target_position: np.ndarray) -> mg.RobotState:
    """Build the target position requested from RMPflow."""
    return mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=robot_site_space,
            positions=([TOOL_FRAME], wp.array([target_position], dtype=wp.float32, device="cpu")),
        )
    )


def _make_estimated_state(robot: ArticulationEntity) -> mg.RobotState:
    """Build the measured joint state used to reset RMPflow."""
    robot_joint_space = list(robot.dof_names)
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=robot_joint_space,
            positions=(robot_joint_space, robot.get_dof_positions()),
            velocities=(robot_joint_space, robot.get_dof_velocities()),
        )
    )


def _set_natural_robot_pose(robot: ArticulationEntity, manager: PhysicsManager) -> None:
    """Move Franka to a relaxed initial configuration before starting RMPflow."""
    positions = np.asarray([[DEFAULT_JOINT_POSITIONS.get(name, 0.0) for name in robot.dof_names]], dtype=np.float32)
    robot.set_dof_positions(positions)
    robot.set_dof_position_targets(positions)
    manager.step()
    if not manager.publish_transforms_to_stage():
        raise RuntimeError("Physics transform publication failed.")


class Workload:
    """One controller update, completed physics step, and transform publication."""

    def __init__(self, time_step: float = 0.005, physics_device: str = "cpu", render: bool = False) -> None:
        self.time_step = time_step
        self.physics_device = physics_device
        self.render = render
        self.stage = self.manager = self.viewport = None
        self.completed_steps = 0
        self.controller = self.world_interface = None

    def setup(self) -> None:
        """Create the scene and initialize physics and controls."""
        os.environ.setdefault("OVGL_SS", "1")
        self.stage = Stage("ovstage").import_stage_from_string(_author_stage(self.physics_device))
        self.manager = PhysicsManager.get_instance()
        if not self.manager.switch_physics_engine(ENGINE):
            raise RuntimeError("OvPhysX physics engine is unavailable.")
        self.manager.setup(dt=self.time_step)
        if not self.manager.initialize(self.stage.get_stage_ptr(), self.stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")
        self.robot = ArticulationEntity(ENGINE, ROBOT_PATH)
        _set_natural_robot_pose(self.robot, self.manager)
        robot_joint_space = list(self.robot.dof_names)
        robot_positions, robot_orientations_xyzw = self.robot.get_world_poses()
        robot_orientations_wxyz = wp.array(
            np.ascontiguousarray(robot_orientations_xyzw.numpy()[:, [3, 0, 1, 2]]), dtype=wp.float32, device="cpu"
        )
        robot_config = load_cumotion_supported_robot("franka")
        robot_site_space = list(robot_config.robot_description.tool_frame_names())
        if TOOL_FRAME not in robot_site_space:
            raise RuntimeError(f"Franka cuMotion configuration does not expose tool frame {TOOL_FRAME!r}.")
        self.world_interface = CumotionWorldInterface(device="cpu")
        self.world_interface.update_world_to_robot_root_transforms(poses=(robot_positions, robot_orientations_wxyz))
        self.controller = RmpFlowController(
            cumotion_robot=robot_config,
            cumotion_world_interface=self.world_interface,
            robot_joint_space=robot_joint_space,
            robot_site_space=robot_site_space,
            tool_frame=TOOL_FRAME,
        )
        self.setpoint = _make_setpoint(robot_site_space, TARGET_POSITION)
        if not self.controller.reset(_make_estimated_state(self.robot), self.setpoint, 0.0):
            raise RuntimeError("RmpFlowController failed to reset from the measured Franka state.")
        self.initial_time = self.manager.get_simulated_time()
        self.initial_steps = self.manager.get_simulated_physics_steps()
        self.initial_positions = self.robot.get_dof_positions().numpy().copy()
        self.robot_config = robot_config
        self.controlled_indices = self.robot.get_dof_indices(robot_config.controlled_joint_names).numpy()
        initial_tool_position = np.asarray(
            robot_config.kinematics.pose(self.initial_positions[0, self.controlled_indices], TOOL_FRAME).translation
        )
        self.initial_target_error = float(np.linalg.norm(initial_tool_position - TARGET_POSITION))
        if self.render:
            from isaacsim.ovgl_viewport.debug import Camera, Viewport

            self.viewport = Viewport(
                self.stage,
                visible=False,
                width=1280,
                height=720,
                camera=Camera(target=(0.3, 0.15, 0.35), yaw_radians=0.8, pitch_radians=0.3, distance=2.6),
            )
            self.viewport.render()

    def run(self) -> None:
        """Complete one measured simulation frame."""
        desired = self.controller.forward(
            _make_estimated_state(self.robot), self.setpoint, self.manager.get_simulated_time()
        )
        joints = desired.joints
        positions = self.robot.get_dof_positions().numpy().copy()
        positions[:, joints.position_indices.numpy()] = joints.positions.numpy()
        self.robot.set_dof_position_targets(positions)
        self.manager.step()
        if not self.manager.publish_transforms_to_stage():
            raise RuntimeError("Physics transform publication failed.")
        if self.viewport is not None:
            self.viewport.render()
        self.completed_steps += 1

    def validate(self) -> bool:
        """Check that simulation advanced with finite state."""
        positions = self.robot.get_dof_positions().numpy()
        final_tool_position = np.asarray(
            self.robot_config.kinematics.pose(positions[0, self.controlled_indices], TOOL_FRAME).translation
        )
        final_target_error = float(np.linalg.norm(final_tool_position - TARGET_POSITION))
        return bool(
            self.completed_steps > 0
            and self.manager.get_simulated_time() > self.initial_time
            and self.manager.get_simulated_physics_steps() - self.initial_steps == self.completed_steps
            and np.isfinite(positions).all()
            and (not np.array_equal(positions, self.initial_positions))
            and np.isfinite(final_target_error)
            and final_target_error < self.initial_target_error
        )

    def teardown(self) -> None:
        """Release the simulation resources."""
        if self.viewport is not None:
            self.viewport.close()
        self.controller = self.world_interface = None
        if self.manager is not None and self.manager.is_initialized():
            self.manager.invalidate()
        if self.stage is not None:
            self.stage.close_stage()


def main() -> None:
    """Run the workload and reject invalid results."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output")
    parser.add_argument("--physics-device", choices=("cpu", "gpu"), default="cpu")
    parser.add_argument("--time-step", type=float, default=0.005)
    parser.add_argument("--warmup-frames", type=int, default=100)
    parser.add_argument("--measured-frames", type=int, default=1000)
    parser.add_argument("--render", action="store_true", help="Include 1280 x 720 offscreen rendering and readback")
    args = parser.parse_args()
    workload = benchmark.workflow(
        "benchmarks.robot_motion.cumotion.follow_target",
        metrics=("simulation",),
        resources=("cpu", "gpu"),
        simulated_time="time_step",
        warmup_frames=args.warmup_frames,
        measured_frames=args.measured_frames,
        completion="fetched",
    )(Workload)
    result = benchmark.run(
        workload,
        parameters={"time_step": args.time_step, "physics_device": args.physics_device, "render": args.render},
        output=args.output,
    )
    if not result["status"]["valid"]:
        raise RuntimeError(result["status"]["errors"])


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)

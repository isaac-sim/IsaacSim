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

"""Benchmark motion_generation follow_trajectory control and physics throughput."""

from __future__ import annotations

import argparse
import math
import os
from collections.abc import Iterator

import isaacsim.physics_engines.ovphysx  # noqa: F401
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.foundation.objects import Stage
from isaacsim.performance.benchmarking import benchmark
from isaacsim.physics.entities import ArticulationEntity
from isaacsim.physics.manager import PhysicsManager

_ROBOT_PATH = "/World/Franka"
_DEFAULT_ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
_ASSET_ROOT = (os.environ.get("ISAACSIM_ASSET_ROOT") or _DEFAULT_ASSET_ROOT).rstrip("/\\")
_FRANKA_USD = f"{_ASSET_ROOT}/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
_ARM_JOINTS = tuple(f"panda_joint{index}" for index in range(1, 8))
_FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")
_POSE_A = (0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741)
_POSE_B = (0.33, 0.114, 0.314, -2.291, -0.168, 3.022, 0.741)
_FINGER_POSITION = 0.04


def _author_stage(physics_device: str) -> str:
    """Author the Franka scene and return its unresolved root layer as USDA."""
    from isaacsim.foundation.objects import DistantLight, PhysicsScene, PhysxScene
    from isaacsim.foundation.prims import GroundPlane
    from isaacsim.ovgl_viewport.debug import author_viewport
    from pxr import Sdf

    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")
        ground = GroundPlane("/World/Ground", sizes=2.2, colors=[[0.22, 0.24, 0.28]])
        ground.set_local_poses(translations=[0.25, 0.15, 0.0])
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
    detached_layer = Sdf.Layer.CreateAnonymous("follow_trajectory.usda")
    if not detached_layer.ImportFromString(authored_stage_text):
        raise RuntimeError("OpenUSD could not reconstruct the authored stage layer.")
    robot_spec = Sdf.CreatePrimInLayer(detached_layer, _ROBOT_PATH)
    robot_spec.specifier = Sdf.SpecifierDef
    robot_spec.typeName = "Xform"
    robot_spec.referenceList.prependedItems = [Sdf.Reference(_FRANKA_USD)]
    return detached_layer.ExportToString()


def _make_joint_pose(dof_names: list[str], arm_positions: tuple[float, ...]) -> np.ndarray:
    """Build one named Franka pose in articulation order."""
    expected_names = (*_ARM_JOINTS, *_FINGER_JOINTS)
    missing = [name for name in expected_names if name not in dof_names]
    if missing:
        raise RuntimeError(f"Franka articulation is missing joints: {', '.join(missing)}")
    positions_by_name = dict(zip(_ARM_JOINTS, arm_positions, strict=True))
    positions_by_name.update(dict.fromkeys(_FINGER_JOINTS, _FINGER_POSITION))
    return np.asarray([positions_by_name[name] for name in dof_names], dtype=np.float32)


def _make_trajectory(pose_a: np.ndarray, pose_b: np.ndarray, dof_names: list[str]) -> mg.Trajectory:
    """Time-parameterize a two-pose Franka round trip."""
    maximum_velocities = np.asarray([0.55 if name in _ARM_JOINTS else 0.08 for name in dof_names], dtype=np.float32)
    maximum_accelerations = np.asarray([1.2 if name in _ARM_JOINTS else 0.2 for name in dof_names], dtype=np.float32)
    waypoints = np.asarray([pose_a, pose_b, pose_a], dtype=np.float32)
    path = mg.Path(wp.array(waypoints, dtype=wp.float32, device="cpu"))
    return path.to_minimal_time_joint_trajectory(
        max_velocities=maximum_velocities,
        max_accelerations=maximum_accelerations,
        robot_joint_space=dof_names,
        active_joints=dof_names,
    )


def _full_command(values: wp.array, indices: wp.array, dof_count: int) -> np.ndarray:
    """Expand a named joint command into articulation order."""
    command = np.zeros((1, dof_count), dtype=np.float32)
    command[0, indices.numpy()] = values.numpy()
    return command


def _make_robot_state(values: np.ndarray, dof_names: list[str]) -> mg.RobotState:
    """Build the follower's state from measured joint positions."""
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=dof_names, positions=(dof_names, wp.array(values, dtype=wp.float32, device="cpu"))
        )
    )


def _simulation_steps(
    articulation: ArticulationEntity,
    manager: PhysicsManager,
    follower: mg.TrajectoryFollower,
    trajectory: mg.Trajectory,
    cycle_start_state: mg.RobotState,
    time_step: float,
) -> Iterator[None]:
    """Yield after each completed control/physics step, repeating the trajectory."""
    dof_names = list(articulation.dof_names)
    trajectory_step_count = math.ceil(trajectory.duration / time_step) + 1
    while True:
        for trajectory_step in range(trajectory_step_count):
            simulation_time = min(trajectory_step * time_step, trajectory.duration)
            desired = follower.forward(cycle_start_state, None, simulation_time)
            if desired is None or desired.joints is None or desired.joints.positions is None:
                raise RuntimeError("TrajectoryFollower returned no Franka joint command.")
            joints = desired.joints
            for values, indices, setter in (
                (joints.positions, joints.position_indices, articulation.set_dof_position_targets),
                (joints.velocities, joints.velocity_indices, articulation.set_dof_velocity_targets),
                (joints.efforts, joints.effort_indices, articulation.set_dof_efforts),
            ):
                if values is not None:
                    setter(_full_command(values, indices, len(dof_names)))
            manager.step()
            if not manager.publish_transforms_to_stage():
                raise RuntimeError("Physics transform publication failed.")
            yield
        measured_positions = articulation.get_dof_positions().numpy()[0].copy()
        cycle_start_state = _make_robot_state(measured_positions, dof_names)
        if not follower.reset(cycle_start_state, None, 0.0):
            raise RuntimeError("TrajectoryFollower could not restart the Franka trajectory.")


class Workload:
    """One controller update, completed physics step, and transform publication."""

    def __init__(self, time_step: float = 0.005, physics_device: str = "cpu", render: bool = False) -> None:
        self.time_step = time_step
        self.physics_device = physics_device
        self.render = render
        self.stage = self.manager = self.viewport = None
        self.completed_steps = 0
        self.steps = None

    def setup(self) -> None:
        """Create the scene and initialize physics and controls."""
        os.environ.setdefault("OVGL_SS", "1")
        self.stage = Stage("ovstage").import_stage_from_string(_author_stage(self.physics_device))
        self.manager = PhysicsManager.get_instance()
        if not self.manager.switch_physics_engine("ovphysx"):
            raise RuntimeError("OvPhysX physics engine is unavailable.")
        self.manager.setup(dt=self.time_step)
        if not self.manager.initialize(self.stage.get_stage_ptr(), self.stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")
        self.robot = ArticulationEntity("ovphysx", _ROBOT_PATH)
        if self.robot.num_prims != 1:
            raise RuntimeError(f"The Franka articulation was not found at {_ROBOT_PATH}.")
        dof_names = list(self.robot.dof_names)
        pose_a = _make_joint_pose(dof_names, _POSE_A)
        pose_b = _make_joint_pose(dof_names, _POSE_B)
        trajectory = _make_trajectory(pose_a, pose_b, dof_names)
        self.robot.set_dof_positions(pose_a.reshape(1, -1))
        self.robot.set_dof_position_targets(pose_a.reshape(1, -1))
        self.manager.step()
        if not self.manager.publish_transforms_to_stage():
            raise RuntimeError("Physics transform publication failed.")
        follower = mg.TrajectoryFollower()
        follower.set_trajectory(trajectory)
        cycle_start_state = _make_robot_state(pose_a, dof_names)
        if not follower.reset(cycle_start_state, None, 0.0):
            raise RuntimeError("TrajectoryFollower rejected the Franka trajectory.")
        self.steps = _simulation_steps(
            self.robot, self.manager, follower, trajectory, cycle_start_state, self.time_step
        )
        self.initial_time = self.manager.get_simulated_time()
        self.initial_steps = self.manager.get_simulated_physics_steps()
        self.initial_positions = self.robot.get_dof_positions().numpy().copy()
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
        next(self.steps)
        if self.viewport is not None:
            self.viewport.render()
        self.completed_steps += 1

    def validate(self) -> bool:
        """Check that simulation advanced with finite state."""
        positions = self.robot.get_dof_positions().numpy()
        return bool(
            self.completed_steps > 0
            and self.manager.get_simulated_time() > self.initial_time
            and self.manager.get_simulated_physics_steps() - self.initial_steps == self.completed_steps
            and np.isfinite(positions).all()
            and (not np.array_equal(positions, self.initial_positions))
        )

    def teardown(self) -> None:
        """Release the simulation resources."""
        self.steps = None
        if self.viewport is not None:
            self.viewport.close()
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
        "benchmarks.robot_motion.motion_generation.follow_trajectory",
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

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

"""Benchmark Kaya physics and publication with preset holonomic wheel targets."""

from __future__ import annotations

import argparse
import os

import isaacsim.physics_engines.ovphysx  # noqa: F401
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.foundation.objects import Stage, Xform
from isaacsim.performance.benchmarking import benchmark
from isaacsim.physics.entities import ArticulationEntity
from isaacsim.physics.manager import PhysicsManager
from isaacsim.robot_motion import controllers

ENGINE = "ovphysx"
ROBOT_PATH = "/kaya"
WHEEL_JOINTS = ["axle_0_joint", "axle_1_joint", "axle_2_joint"]
COMMAND = (0.2, 0.1, 0.5)
WHEEL_RADIUS = 0.04
WHEEL_POSITIONS = [
    [-0.09804319, 0.00063677, -0.05050102],
    [0.04934748, -0.08452497, -0.05050102],
    [0.04952906, 0.08569367, -0.05050102],
]
WHEEL_ORIENTATIONS = [[-5e-08, 0.0, 0.0, 1.0], [0.8660267, 0.0, 0.0, -0.4999977], [0.86602294, 0.0, 0.0, 0.50000423]]
MECANUM_ANGLE = 90.0
_DEFAULT_ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
_ASSET_ROOT = (os.environ.get("ISAACSIM_ASSET_ROOT") or _DEFAULT_ASSET_ROOT).rstrip("/\\")
KAYA_USD = f"{_ASSET_ROOT}/Isaac/Robots_Multiphysics/NVIDIA/Kaya/kaya.usda"


def _author_stage(physics_device: str) -> str:
    """Author the NVIDIA Kaya environment and return its unresolved root layer as USDA."""
    from isaacsim.foundation.objects import DistantLight, PhysicsScene, PhysxScene
    from isaacsim.foundation.prims import GroundPlane
    from isaacsim.ovgl_viewport.debug import author_viewport
    from pxr import Sdf

    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")
        Xform(ROBOT_PATH, reset_xform_op_properties=True).set_world_poses(positions=[0.0, 0.0, 0.02])
        GroundPlane("/World/Ground", sizes=4.0, colors=[[0.22, 0.24, 0.28]])
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
    detached_layer = Sdf.Layer.CreateAnonymous("holonomic.usda")
    if not detached_layer.ImportFromString(authored_stage_text):
        raise RuntimeError("OpenUSD could not reconstruct the authored stage layer.")
    robot_spec = Sdf.CreatePrimInLayer(detached_layer, ROBOT_PATH)
    robot_spec.specifier = Sdf.SpecifierDef
    robot_spec.typeName = "Xform"
    robot_spec.referenceList.prependedItems = [Sdf.Reference(KAYA_USD)]
    return detached_layer.ExportToString()


def _make_setpoint() -> mg.RobotState:
    """Build a command with forward speed, lateral speed, and yaw rate."""
    site = "control_point"
    return mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=[site],
            linear_velocities=([site], wp.array([[COMMAND[0], COMMAND[1], 0.0]], dtype=wp.float32, device="cpu")),
            angular_velocities=([site], wp.array([[0.0, 0.0, COMMAND[2]]], dtype=wp.float32, device="cpu")),
        )
    )


class Workload:
    """One completed physics step and publication with preset holonomic wheel targets."""

    def __init__(self, time_step: float = 0.005, physics_device: str = "cpu", render: bool = False) -> None:
        self.time_step = time_step
        self.physics_device = physics_device
        self.render = render
        self.stage = self.manager = self.viewport = None
        self.completed_steps = 0

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
        self.controller = controllers.HolonomicController(
            robot_joint_space=list(self.robot.dof_names),
            wheel_joint_names=WHEEL_JOINTS,
            wheel_radius=WHEEL_RADIUS,
            wheel_positions=WHEEL_POSITIONS,
            wheel_orientations=WHEEL_ORIENTATIONS,
            mecanum_angles=MECANUM_ANGLE,
            max_linear_speed=0.6,
            max_angular_speed=3.0,
            device="cpu",
        )
        self.controller.reset(mg.RobotState(), None, 0.0)
        self.setpoint = _make_setpoint()
        self.velocity_targets = np.zeros((1, len(self.robot.dof_names)), dtype=np.float32)
        desired = self.controller.forward(mg.RobotState(), self.setpoint, self.manager.get_simulated_time())
        self.velocity_targets[0, desired.joints.velocity_indices.numpy()] = desired.joints.velocities.numpy()
        self.robot.set_dof_velocity_targets(wp.array(self.velocity_targets, dtype=wp.float32, device="cpu"))
        self.initial_time = self.manager.get_simulated_time()
        self.initial_steps = self.manager.get_simulated_physics_steps()
        self.initial_positions = self.robot.get_dof_positions().numpy().copy()
        self.initial_xy = self.robot.get_world_poses()[0].numpy()[0, :2].copy()
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
        self.manager.step()
        if not self.manager.publish_transforms_to_stage():
            raise RuntimeError("Physics transform publication failed.")
        if self.viewport is not None:
            self.viewport.render()
        self.completed_steps += 1

    def validate(self) -> bool:
        """Check that simulation advanced with finite state."""
        positions = self.robot.get_dof_positions().numpy()
        final_xy = self.robot.get_world_poses()[0].numpy()[0, :2]
        return bool(
            self.completed_steps > 0
            and self.manager.get_simulated_time() > self.initial_time
            and self.manager.get_simulated_physics_steps() - self.initial_steps == self.completed_steps
            and np.isfinite(positions).all()
            and (not np.array_equal(positions, self.initial_positions))
            and np.isfinite(final_xy).all()
            and np.linalg.norm(final_xy - self.initial_xy) > np.finfo(np.float32).eps
        )

    def teardown(self) -> None:
        """Release the simulation resources."""
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
        "benchmarks.robot_motion.controllers.holonomic_drive",
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

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

"""Drive the NVIDIA Kaya robot with the holonomic-drive controller and an OVGL viewport."""

from __future__ import annotations

import os
import time

import isaacsim.physics_engines.ovphysx  # noqa: F401
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.foundation.objects import Stage, Xform
from isaacsim.physics.entities import ArticulationEntity
from isaacsim.physics.manager import PhysicsManager
from isaacsim.robot_motion import controllers

ENGINE = "ovphysx"
ROBOT_PATH = "/kaya"
WHEEL_JOINTS = ["axle_0_joint", "axle_1_joint", "axle_2_joint"]
COMMAND = (0.20, 0.10, 0.50)
TIME_STEP = 1.0 / 60.0
STEP_COUNT = 180
MAX_STEPS_PER_FRAME = 15
MAX_CATCHUP_TIME = MAX_STEPS_PER_FRAME * TIME_STEP

# Controller geometry authored in the versioned NVIDIA Kaya asset below.
WHEEL_RADIUS = 0.04
WHEEL_POSITIONS = [
    [-0.09804319, 0.00063677, -0.05050102],
    [0.04934748, -0.08452497, -0.05050102],
    [0.04952906, 0.08569367, -0.05050102],
]
WHEEL_ORIENTATIONS = [
    [-0.00000005, 0.0, 0.0, 1.0],
    [0.8660267, 0.0, 0.0, -0.4999977],
    [0.86602294, 0.0, 0.0, 0.50000423],
]
MECANUM_ANGLE = 90.0

_DEFAULT_ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
_ASSET_ROOT = (os.environ.get("ISAACSIM_ASSET_ROOT") or _DEFAULT_ASSET_ROOT).rstrip("/\\")
KAYA_USD = f"{_ASSET_ROOT}/Isaac/Robots_Multiphysics/NVIDIA/Kaya/kaya.usda"


def _author_stage() -> str:
    """Author the NVIDIA Kaya environment and return its unresolved root layer as USDA.

    Returns:
        USDA text containing the environment and unresolved robot reference.
    """
    from isaacsim.foundation.objects import DistantLight, PhysicsScene
    from isaacsim.foundation.prims import GroundPlane
    from isaacsim.ovgl_viewport.debug import author_viewport
    from pxr import Sdf

    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")
        Xform(ROBOT_PATH, reset_xform_op_properties=True).set_world_poses(positions=[0.0, 0.0, 0.02])

        GroundPlane("/World/Ground", sizes=4.0, colors=[[0.22, 0.24, 0.28]])

        PhysicsScene("/World/PhysicsScene").set_gravities([0.0, 0.0, -9.81])

        light = DistantLight("/World/KeyLight")
        light.set_intensities([[3500.0]])
        light.set_colors([[1.0, 0.93, 0.82]])
        author_viewport(stage)
        authored_stage_text = stage.export_stage_to_string()
    finally:
        stage.close_stage()

    # `Stage.add_reference()` resolves assets before authoring, so Sdf is required for this unresolved remote URI.
    detached_layer = Sdf.Layer.CreateAnonymous("holonomic.usda")
    if not detached_layer.ImportFromString(authored_stage_text):
        raise RuntimeError("OpenUSD could not reconstruct the authored stage layer.")
    robot_spec = Sdf.CreatePrimInLayer(detached_layer, ROBOT_PATH)
    robot_spec.specifier = Sdf.SpecifierDef
    robot_spec.typeName = "Xform"
    robot_spec.referenceList.prependedItems = [Sdf.Reference(KAYA_USD)]
    return detached_layer.ExportToString()


def _make_setpoint() -> mg.RobotState:
    """Build a command with forward speed, lateral speed, and yaw rate.

    Returns:
        Named control-point velocity state.
    """
    site = "control_point"
    return mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=[site],
            linear_velocities=([site], wp.array([[COMMAND[0], COMMAND[1], 0.0]], dtype=wp.float32, device="cpu")),
            angular_velocities=([site], wp.array([[0.0, 0.0, COMMAND[2]]], dtype=wp.float32, device="cpu")),
        )
    )


def _report_result(
    robot: ArticulationEntity,
    initial_position: np.ndarray,
    wheel_indices: np.ndarray,
    wheel_velocities: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Report the completed controller run.

    Args:
        robot: Simulated NVIDIA Kaya articulation.
        initial_position: Robot position before the controller run.
        wheel_indices: Articulation indices of the controlled wheels.
        wheel_velocities: Commanded wheel speeds in radians per second.

    Returns:
        Measured wheel velocities and planar distance traveled in meters.
    """
    measured_wheel_velocities = robot.get_dof_velocities().numpy()[0][wheel_indices]
    final_position = robot.get_world_poses()[0].numpy()[0]
    displacement = final_position[:2] - initial_position[:2]
    distance = float(np.linalg.norm(displacement))
    print(f"Body command: {list(COMMAND)} [forward speed (m/s), lateral speed (m/s), yaw rate (rad/s)].")
    print(f"Wheel angular velocities: {np.round(wheel_velocities, 3).tolist()} rad/s.")
    print(f"Measured wheel velocities: {np.round(measured_wheel_velocities, 3).tolist()} rad/s.")
    print(f"Robot planar displacement: {distance:.3f} m.")
    return measured_wheel_velocities, distance


def main(*, visible: bool = True) -> None:
    """Run holonomic-drive control without loading Kit.

    Args:
        visible: Whether to present an interactive viewport.
    """
    os.environ.setdefault("OVGL_SS", "1")

    stage = None
    physics_manager = None
    viewport = None
    try:
        # Load the authored scene and initialize its physics simulation.
        stage = Stage("ovstage").import_stage_from_string(_author_stage())
        physics_manager = PhysicsManager.get_instance()
        if not physics_manager.switch_physics_engine(ENGINE):
            raise RuntimeError("OvPhysX physics engine is unavailable.")
        physics_manager.setup(dt=TIME_STEP)
        if not physics_manager.initialize(stage.get_stage_ptr(), stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")

        # Configure the controller from the NVIDIA Kaya wheel geometry and joint names.
        robot = ArticulationEntity(ENGINE, ROBOT_PATH)
        controller = controllers.HolonomicController(
            robot_joint_space=list(robot.dof_names),
            wheel_joint_names=WHEEL_JOINTS,
            wheel_radius=WHEEL_RADIUS,
            wheel_positions=WHEEL_POSITIONS,
            wheel_orientations=WHEEL_ORIENTATIONS,
            mecanum_angles=MECANUM_ANGLE,
            max_linear_speed=0.6,
            max_angular_speed=3.0,
            device="cpu",
        )
        controller.reset(mg.RobotState(), None, 0.0)

        # Convert the desired planar body velocity into named wheel-velocity targets.
        desired_state = controller.forward(mg.RobotState(), _make_setpoint(), 0.0)
        if desired_state is None or desired_state.joints is None or desired_state.joints.velocities is None:
            raise RuntimeError("HolonomicController did not produce wheel velocity targets.")
        if desired_state.joints.velocity_names != WHEEL_JOINTS:
            raise RuntimeError(f"Unexpected wheel command order: {desired_state.joints.velocity_names}")

        wheel_velocities = desired_state.joints.velocities.numpy()
        wheel_indices = robot.get_dof_indices(desired_state.joints.velocity_names).numpy()
        all_dof_targets = np.zeros((1, len(robot.dof_names)), dtype=np.float32)
        all_dof_targets[0, wheel_indices] = wheel_velocities
        velocity_targets = wp.array(all_dof_targets, dtype=wp.float32, device="cpu")
        robot.set_dof_velocity_targets(velocity_targets)
        initial_position = robot.get_world_poses()[0].numpy()[0]

        # Create the interactive or offscreen viewport used to render the simulation.
        from isaacsim.ovgl_viewport.debug import Camera, Viewport

        viewport = Viewport(
            stage,
            title="Holonomic Drive Controller",
            visible=visible,
            camera=Camera(target=(0.2, 0.15, 0.12), yaw_radians=0.8, pitch_radians=0.28, distance=1.5),
        )
        if not viewport.poll_events():
            raise RuntimeError("OVGL viewport closed before its initial render.")
        viewport.render()

        # Advance physics at a fixed rate while keeping visible rendering responsive.
        step = 0
        reported = False
        last_frame_time = time.monotonic()
        accumulated_simulation_time = 0.0
        while viewport.poll_events():
            if visible:
                current_time = time.monotonic()
                accumulated_simulation_time += min(current_time - last_frame_time, MAX_CATCHUP_TIME)
                last_frame_time = current_time
                steps_this_frame = min(
                    int(accumulated_simulation_time / TIME_STEP),
                    MAX_STEPS_PER_FRAME,
                    STEP_COUNT - step,
                )
                accumulated_simulation_time -= steps_this_frame * TIME_STEP
            else:
                steps_this_frame = STEP_COUNT - step

            step += physics_manager.step(steps=steps_this_frame)
            if steps_this_frame and not physics_manager.publish_transforms_to_stage():
                raise RuntimeError("Physics transform publication failed.")
            viewport.render()

            if step == STEP_COUNT and not reported:
                _report_result(robot, initial_position, wheel_indices, wheel_velocities)
                reported = True
            if not visible:
                break

        if not reported:
            print(f"Viewport closed after {step} of {STEP_COUNT} simulation steps.")
    finally:
        if viewport is not None:
            viewport.close()
        if physics_manager is not None and physics_manager.is_initialized():
            physics_manager.invalidate()
        if stage is not None:
            stage.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

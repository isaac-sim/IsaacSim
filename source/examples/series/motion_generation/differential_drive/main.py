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

"""Drive a two-wheeled robot with the differential-drive controller and an OVGL viewport."""

from __future__ import annotations

import math
import os
import time

import isaacsim.physics_engines.ovphysx  # noqa: F401
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.foundation.objects import Stage
from isaacsim.physics.entities import ArticulationEntity
from isaacsim.physics.manager import PhysicsManager
from isaacsim.robot_motion import controllers

ENGINE = "ovphysx"
ROBOT_PATH = "/turtlebot3"
ARTICULATION_PATH = f"{ROBOT_PATH}/Geometry/base_footprint/base_link"
LEFT_WHEEL_JOINT = "wheel_left_joint"
RIGHT_WHEEL_JOINT = "wheel_right_joint"
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.16
TIME_STEP = 1.0 / 60.0
STEP_COUNT = 420
MAX_STEPS_PER_FRAME = 15
MAX_CATCHUP_TIME = MAX_STEPS_PER_FRAME * TIME_STEP
LINEAR_SPEED = 0.15
YAW_RATE = 1.0
MAX_YAW_RATE = 1.0

_DEFAULT_ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
_ASSET_ROOT = (os.environ.get("ISAACSIM_ASSET_ROOT") or _DEFAULT_ASSET_ROOT).rstrip("/\\")
TURTLEBOT_USD = (
    f"{_ASSET_ROOT}/Isaac/Robots_Multiphysics/Turtlebot/Turtlebot3/" "turtlebot3_burger/turtlebot3_burger.usda"
)


def _author_stage() -> str:
    """Author the TurtleBot3 Burger environment and return its unresolved root layer as USDA.

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
    detached_layer = Sdf.Layer.CreateAnonymous("differential_drive.usda")
    if not detached_layer.ImportFromString(authored_stage_text):
        raise RuntimeError("OpenUSD could not reconstruct the authored stage layer.")
    robot_spec = Sdf.CreatePrimInLayer(detached_layer, ROBOT_PATH)
    robot_spec.specifier = Sdf.SpecifierDef
    robot_spec.typeName = "Xform"
    robot_spec.referenceList.prependedItems = [Sdf.Reference(TURTLEBOT_USD)]
    robot_spec.variantSelections["Physics"] = "physics"
    return detached_layer.ExportToString()


def _make_setpoint(linear_speed: float, yaw_rate: float) -> mg.RobotState:
    """Build the control-point velocity requested by DifferentialDriveController.

    Args:
        linear_speed: Requested forward speed in meters per second.
        yaw_rate: Requested yaw rate in radians per second.

    Returns:
        Named control-point velocity state.
    """
    return mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=["control_point"],
            linear_velocities=(
                ["control_point"],
                wp.array([[linear_speed, 0.0, 0.0]], dtype=wp.float32, device="cpu"),
            ),
            angular_velocities=(
                ["control_point"],
                wp.array([[0.0, 0.0, yaw_rate]], dtype=wp.float32, device="cpu"),
            ),
        )
    )


def _report_result(
    robot: ArticulationEntity,
    initial_position: np.ndarray,
    commanded_left: float,
    commanded_right: float,
) -> tuple[float, float]:
    """Report the completed controller run.

    Args:
        robot: Simulated TurtleBot3 articulation.
        initial_position: Robot position before the controller run.
        commanded_left: Commanded left-wheel speed in radians per second.
        commanded_right: Commanded right-wheel speed in radians per second.

    Returns:
        Planar distance traveled and absolute vertical displacement, in meters.
    """
    final_position = robot.get_world_poses()[0].numpy()[0]
    measured_wheel_velocities = robot.get_dof_velocities().numpy()[0]
    displacement = final_position[:2] - initial_position[:2]
    distance = math.hypot(float(displacement[0]), float(displacement[1]))
    vertical_displacement = abs(float(final_position[2] - initial_position[2]))
    print(f"Wheel angular velocities: left={commanded_left:.3f}, right={commanded_right:.3f} rad/s.")
    print(f"Measured wheel velocities: {measured_wheel_velocities.tolist()} rad/s.")
    print(f"Robot position: {initial_position.tolist()} -> {final_position.tolist()}.")
    print(f"Robot planar displacement: {distance:.3f} m.")
    print(f"Robot vertical displacement: {vertical_displacement:.3f} m.")
    print(f"Commanded turn: {math.degrees(YAW_RATE * TIME_STEP * STEP_COUNT):.1f} degrees.")
    return distance, vertical_displacement


def main(*, visible: bool = True) -> None:
    """Run differential-drive control.

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

        # Configure the controller from the TurtleBot3 wheel geometry and joint names.
        robot = ArticulationEntity(ENGINE, ARTICULATION_PATH)
        controller = controllers.DifferentialDriveController(
            robot_joint_space=list(robot.dof_names),
            left_wheel_joint=LEFT_WHEEL_JOINT,
            right_wheel_joint=RIGHT_WHEEL_JOINT,
            wheel_radius=WHEEL_RADIUS,
            wheel_base=WHEEL_BASE,
            max_linear_speed=LINEAR_SPEED,
            max_angular_speed=MAX_YAW_RATE,
            device="cpu",
        )
        controller.reset(mg.RobotState(), None, 0.0)

        # Convert the desired body velocity into named wheel-velocity targets.
        initial_position = robot.get_world_poses()[0].numpy()[0]
        setpoint = _make_setpoint(LINEAR_SPEED, YAW_RATE)
        desired_state = controller.forward(mg.RobotState(), setpoint, 0.0)
        if desired_state is None or desired_state.joints is None or desired_state.joints.velocities is None:
            raise RuntimeError("DifferentialDriveController did not produce wheel velocity targets.")

        wheel_velocities = desired_state.joints.velocities.numpy()
        wheel_velocities_by_name = dict(zip(desired_state.joints.velocity_names, wheel_velocities, strict=True))
        expected_names = {LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT}
        if set(wheel_velocities_by_name) != expected_names:
            raise RuntimeError(
                f"Unexpected wheel velocity names {sorted(wheel_velocities_by_name)}; expected {sorted(expected_names)}."
            )
        actual_left = float(wheel_velocities_by_name[LEFT_WHEEL_JOINT])
        actual_right = float(wheel_velocities_by_name[RIGHT_WHEEL_JOINT])
        wheel_indices = robot.get_dof_indices(desired_state.joints.velocity_names)
        wheel_targets = wp.array(wheel_velocities.reshape(1, -1), dtype=wp.float32, device="cpu")
        robot.set_dof_velocity_targets(wheel_targets, dof_indices=wheel_indices)

        # Create the interactive or offscreen viewport used to render the simulation.
        from isaacsim.ovgl_viewport.debug import Camera, Viewport

        viewport = Viewport(
            stage,
            title="Differential Drive Controller",
            visible=visible,
            camera=Camera(target=(0.2, 0.15, 0.08), yaw_radians=0.8, pitch_radians=0.28, distance=1.3),
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
                steps_this_frame = min(int(accumulated_simulation_time / TIME_STEP), MAX_STEPS_PER_FRAME)
                accumulated_simulation_time -= steps_this_frame * TIME_STEP
            else:
                steps_this_frame = STEP_COUNT - step

            step += physics_manager.step(steps=steps_this_frame)
            if steps_this_frame and not physics_manager.publish_transforms_to_stage():
                raise RuntimeError("Physics transform publication failed.")
            viewport.render()

            if step >= STEP_COUNT and not reported:
                _report_result(robot, initial_position, actual_left, actual_right)
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

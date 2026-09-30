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

"""Drive a Franka end effector to a target with cuMotion RMPflow and an OVGL viewport."""

from __future__ import annotations

import os
import time

import isaacsim.physics_engines.ovphysx  # noqa: F401
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.foundation.objects import Cube, DistantLight, PhysicsScene, Stage, Xform
from isaacsim.foundation.prims import GroundPlane
from isaacsim.physics.entities import ArticulationEntity
from isaacsim.physics.manager import PhysicsManager
from isaacsim.robot_motion.cumotion import (
    CumotionRobot,
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_supported_robot,
)

ENGINE = "ovphysx"
ROBOT_PATH = "/World/Franka"
TARGET_PATH = "/World/Target"
TOOL_FRAME = "panda_hand"
TARGET_POSITION = np.asarray([0.45, 0.0, 0.5], dtype=np.float32)
TARGET_MARKER_OFFSET = np.asarray([0.08, 0.0, -0.08], dtype=np.float32)
TARGET_MARKER_SIZE = 0.025
TARGET_ANGULAR_SPEED = 0.8
TARGET_X_AMPLITUDE = 0.06
TARGET_Y_AMPLITUDE = 0.14
TARGET_Z_AMPLITUDE = 0.06
TIME_STEP = 1.0 / 60.0
STEP_COUNT = 360
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


def _author_stage() -> str:
    """Create, populate, export, and close the standalone Franka stage.

    Returns:
        USDA text containing the scene and unresolved Franka reference.
    """
    from isaacsim.ovgl_viewport.debug import author_viewport
    from pxr import Sdf

    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")
        GroundPlane("/World/Ground", sizes=2.2, colors=[[0.22, 0.24, 0.28]]).set_local_poses(
            translations=[0.3, 0.0, 0.0]
        )
        Cube(TARGET_PATH, sizes=TARGET_MARKER_SIZE, colors=[[0.85, 0.12, 0.08]]).set_local_poses(
            translations=(TARGET_POSITION + TARGET_MARKER_OFFSET).tolist()
        )
        PhysicsScene("/World/PhysicsScene").set_gravities([0.0, 0.0, -9.81])

        light = DistantLight("/World/KeyLight")
        light.set_intensities([[3500.0]])
        light.set_colors([[1.0, 0.93, 0.82]])
        author_viewport(stage)
        authored_stage_text = stage.export_stage_to_string()
    finally:
        stage.close_stage()

    # `Stage.add_reference()` resolves assets before authoring, so Sdf is required for this unresolved remote URI.
    detached_layer = Sdf.Layer.CreateAnonymous("follow_target.usda")
    if not detached_layer.ImportFromString(authored_stage_text):
        raise RuntimeError("OpenUSD could not reconstruct the authored stage layer.")
    robot_spec = Sdf.CreatePrimInLayer(detached_layer, ROBOT_PATH)
    robot_spec.specifier = Sdf.SpecifierDef
    robot_spec.typeName = "Xform"
    robot_spec.referenceList.prependedItems = [Sdf.Reference(FRANKA_USD)]
    return detached_layer.ExportToString()


def _compute_moving_target(simulation_time: float) -> np.ndarray:
    """Compute the slowly moving viewport target position.

    Args:
        simulation_time: Elapsed simulation time in seconds.

    Returns:
        World-space target position.
    """
    phase = TARGET_ANGULAR_SPEED * simulation_time
    offset = np.asarray(
        [
            TARGET_X_AMPLITUDE * (np.cos(phase) - 1.0),
            TARGET_Y_AMPLITUDE * np.sin(phase),
            TARGET_Z_AMPLITUDE * np.sin(0.5 * phase),
        ],
        dtype=np.float32,
    )
    return TARGET_POSITION + offset


def _make_setpoint(robot_site_space: list[str], target_position: np.ndarray) -> mg.RobotState:
    """Build the target position requested from RMPflow.

    Args:
        robot_site_space: Site names exposed by the robot description.
        target_position: Requested world-space tool position.

    Returns:
        Named site-position setpoint.
    """
    return mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=robot_site_space,
            positions=([TOOL_FRAME], wp.array([target_position], dtype=wp.float32, device="cpu")),
        )
    )


def _make_estimated_state(robot: ArticulationEntity) -> mg.RobotState:
    """Build the measured joint state used to reset RMPflow.

    Args:
        robot: Simulated Franka articulation.

    Returns:
        Named measured joint state.
    """
    robot_joint_space = list(robot.dof_names)
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=robot_joint_space,
            positions=(robot_joint_space, robot.get_dof_positions()),
            velocities=(robot_joint_space, robot.get_dof_velocities()),
        )
    )


def _set_natural_robot_pose(robot: ArticulationEntity, manager: PhysicsManager) -> None:
    """Move Franka to a relaxed initial configuration before starting RMPflow.

    Args:
        robot: Simulated Franka articulation.
        manager: Active physics manager.
    """
    positions = np.asarray([[DEFAULT_JOINT_POSITIONS.get(name, 0.0) for name in robot.dof_names]], dtype=np.float32)
    robot.set_dof_positions(positions)
    robot.set_dof_position_targets(positions)
    manager.step()
    if not manager.publish_transforms_to_stage():
        raise RuntimeError("Physics transform publication failed.")


def _advance_simulation(
    robot: ArticulationEntity,
    manager: PhysicsManager,
    controller: RmpFlowController,
    setpoint: mg.RobotState,
    simulation_time: float,
) -> float:
    """Apply one RMPflow command, advance physics, and publish its transforms to OVStage.

    Args:
        robot: Simulated Franka articulation.
        manager: Active physics manager.
        controller: Initialized RMPflow controller.
        setpoint: Desired robot state for this step.
        simulation_time: Current simulation time in seconds.

    Returns:
        Simulation time after the step.
    """
    desired_state = controller.forward(_make_estimated_state(robot), setpoint, simulation_time)
    if desired_state is None or desired_state.joints is None or desired_state.joints.positions is None:
        raise RuntimeError("RmpFlowController did not produce joint position targets.")
    position_targets = robot.get_dof_positions().numpy().copy()
    position_targets[:, desired_state.joints.position_indices.numpy()] = desired_state.joints.positions.numpy()
    robot.set_dof_position_targets(position_targets)
    manager.step()
    if not manager.publish_transforms_to_stage():
        raise RuntimeError("Physics transform publication failed.")
    return simulation_time + TIME_STEP


def _report_result(
    robot: ArticulationEntity,
    robot_config: CumotionRobot,
    target_position: np.ndarray,
) -> float:
    """Report the measured tool-position error.

    Args:
        robot: Simulated Franka articulation.
        robot_config: cuMotion robot model used to compute forward kinematics.
        target_position: Requested world-space tool position.

    Returns:
        Euclidean tool-position error in meters.
    """
    joint_positions = robot.get_dof_positions().numpy()[0]
    controlled_indices = robot.get_dof_indices(robot_config.controlled_joint_names).numpy()
    controlled_positions = joint_positions[controlled_indices]
    measured_position = np.asarray(robot_config.kinematics.pose(controlled_positions, TOOL_FRAME).translation)
    position_error = float(np.linalg.norm(measured_position - target_position))
    print(f"Target tool position: {np.round(target_position, 4).tolist()} m.")
    print(f"Measured {TOOL_FRAME} position: {np.round(measured_position, 4).tolist()} m.")
    print(f"RMPflow target error: {position_error:.3f} m.")
    return position_error


def main(*, visible: bool = True) -> None:
    """Run cuMotion RMPflow without loading Kit.

    Args:
        visible: Whether to present an interactive viewport.
    """
    os.environ.setdefault("OVGL_SS", "1")

    stage = None
    physics_manager = None
    viewport = None
    controller = None
    world_interface = None
    try:
        # Load the authored scene and initialize its physics simulation.
        stage = Stage("ovstage").import_stage_from_string(_author_stage())
        physics_manager = PhysicsManager.get_instance()
        if not physics_manager.switch_physics_engine(ENGINE):
            raise RuntimeError("OvPhysX physics engine is unavailable.")
        physics_manager.setup(dt=TIME_STEP)
        if not physics_manager.initialize(stage.get_stage_ptr(), stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")

        # Initialize Franka from a measured, natural starting configuration.
        robot = ArticulationEntity(ENGINE, ROBOT_PATH)
        _set_natural_robot_pose(robot, physics_manager)
        robot_joint_space = list(robot.dof_names)
        robot_positions, robot_orientations_xyzw = robot.get_world_poses()
        robot_orientations_wxyz = wp.array(
            np.ascontiguousarray(robot_orientations_xyzw.numpy()[:, [3, 0, 1, 2]]),
            dtype=wp.float32,
            device="cpu",
        )

        # Configure RMPflow with the robot model, tool frames, and world transform.
        robot_config = load_cumotion_supported_robot("franka")
        robot_site_space = list(robot_config.robot_description.tool_frame_names())
        if TOOL_FRAME not in robot_site_space:
            raise RuntimeError(f"Franka cuMotion configuration does not expose tool frame {TOOL_FRAME!r}.")
        world_interface = CumotionWorldInterface(device="cpu")
        world_interface.update_world_to_robot_root_transforms(poses=(robot_positions, robot_orientations_wxyz))
        controller = RmpFlowController(
            cumotion_robot=robot_config,
            cumotion_world_interface=world_interface,
            robot_joint_space=robot_joint_space,
            robot_site_space=robot_site_space,
            tool_frame=TOOL_FRAME,
        )
        setpoint = _make_setpoint(robot_site_space, TARGET_POSITION)
        if not controller.reset(_make_estimated_state(robot), setpoint, 0.0):
            raise RuntimeError("RmpFlowController failed to reset from the measured Franka state.")

        # Create the interactive or offscreen viewport used to render the simulation.
        from isaacsim.ovgl_viewport.debug import Camera, Viewport

        target_xform = Xform(TARGET_PATH, resolve_paths=False, reset_xform_op_properties=False)
        viewport = Viewport(
            stage,
            title="cuMotion RMPflow Follow Target",
            visible=visible,
            camera=Camera(target=(0.3, 0.0, 0.45), yaw_radians=0.75, pitch_radians=0.35, distance=1.8),
        )
        if not viewport.poll_events():
            raise RuntimeError("OVGL viewport closed before its initial render.")
        viewport.render()

        # Follow a moving target interactively or a fixed target during a bounded offscreen run.
        simulation_time = 0.0
        if visible:
            step = 0
            reported = False
            current_target_position = TARGET_POSITION
            last_frame_time = time.monotonic()
            accumulated_simulation_time = 0.0
            while viewport.poll_events():
                current_time = time.monotonic()
                # Limit catch-up to 15 fixed physics steps (0.25 seconds) per rendered frame.
                accumulated_simulation_time += min(current_time - last_frame_time, 0.25)
                last_frame_time = current_time
                steps_this_frame = 0
                while accumulated_simulation_time >= TIME_STEP and steps_this_frame < 15:
                    current_target_position = _compute_moving_target(simulation_time)
                    target_xform.set_world_poses(positions=current_target_position + TARGET_MARKER_OFFSET)
                    setpoint = _make_setpoint(robot_site_space, current_target_position)
                    simulation_time = _advance_simulation(
                        robot,
                        physics_manager,
                        controller,
                        setpoint,
                        simulation_time,
                    )
                    accumulated_simulation_time -= TIME_STEP
                    step += 1
                    steps_this_frame += 1
                viewport.render()
                if step >= STEP_COUNT and not reported:
                    _report_result(robot, robot_config, current_target_position)
                    reported = True
            if not reported:
                print(f"Viewport closed after {step} of {STEP_COUNT} reporting steps.")
        else:
            for _ in range(STEP_COUNT):
                simulation_time = _advance_simulation(
                    robot,
                    physics_manager,
                    controller,
                    setpoint,
                    simulation_time,
                )
            _report_result(robot, robot_config, TARGET_POSITION)
            viewport.render()
    finally:
        if viewport is not None:
            viewport.close()
        controller = None
        world_interface = None
        if physics_manager is not None and physics_manager.is_initialized():
            physics_manager.invalidate()
        if stage is not None:
            stage.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

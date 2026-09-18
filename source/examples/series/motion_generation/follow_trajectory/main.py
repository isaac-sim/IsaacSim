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

"""Move a Franka articulation back and forth along a joint-space trajectory."""

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

_ROBOT_PATH = "/World/Franka"
_DEFAULT_ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
_ASSET_ROOT = (os.environ.get("ISAACSIM_ASSET_ROOT") or _DEFAULT_ASSET_ROOT).rstrip("/\\")
_FRANKA_USD = f"{_ASSET_ROOT}/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
_ARM_JOINTS = tuple(f"panda_joint{index}" for index in range(1, 8))
_FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")
_POSE_A = (0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741)
_POSE_B = (0.330, 0.114, 0.314, -2.291, -0.168, 3.022, 0.741)
_FINGER_POSITION = 0.04
_PHYSICS_DT = 1.0 / 60.0


def _author_stage() -> str:
    """Author the Franka scene and return its unresolved root layer as USDA.

    Returns:
        Serialized USDA text for the unresolved root layer.
    """
    from isaacsim.foundation.objects import DistantLight, PhysicsScene
    from isaacsim.foundation.prims import GroundPlane
    from isaacsim.ovgl_viewport.debug import author_viewport
    from pxr import Sdf

    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")
        ground = GroundPlane("/World/Ground", sizes=2.2, colors=[[0.22, 0.24, 0.28]])
        ground.set_local_poses(translations=[0.25, 0.15, 0.0])
        PhysicsScene("/World/PhysicsScene").set_gravities([0.0, 0.0, -9.81])

        light = DistantLight("/World/KeyLight")
        light.set_intensities([[3500.0]])
        light.set_colors([[1.0, 0.93, 0.82]])
        author_viewport(stage)
        authored_stage_text = stage.export_stage_to_string()
    finally:
        stage.close_stage()

    # `Stage.add_reference()` resolves assets before authoring, so Sdf is required for this unresolved remote URI.
    detached_layer = Sdf.Layer.CreateAnonymous("follow_trajectory.usda")
    if not detached_layer.ImportFromString(authored_stage_text):
        raise RuntimeError("OpenUSD could not reconstruct the authored stage layer.")
    robot_spec = Sdf.CreatePrimInLayer(detached_layer, _ROBOT_PATH)
    robot_spec.specifier = Sdf.SpecifierDef
    robot_spec.typeName = "Xform"
    robot_spec.referenceList.prependedItems = [Sdf.Reference(_FRANKA_USD)]
    return detached_layer.ExportToString()


def _make_joint_pose(dof_names: list[str], arm_positions: tuple[float, ...]) -> np.ndarray:
    """Build one named Franka pose in articulation order.

    Args:
        dof_names: Ordered degree-of-freedom names.
        arm_positions: Positions for the seven arm joints.

    Returns:
        Joint positions in articulation order.
    """
    expected_names = (*_ARM_JOINTS, *_FINGER_JOINTS)
    missing = [name for name in expected_names if name not in dof_names]
    if missing:
        raise RuntimeError(f"Franka articulation is missing joints: {', '.join(missing)}")
    positions_by_name = dict(zip(_ARM_JOINTS, arm_positions, strict=True))
    positions_by_name.update(dict.fromkeys(_FINGER_JOINTS, _FINGER_POSITION))
    return np.asarray([positions_by_name[name] for name in dof_names], dtype=np.float32)


def _make_trajectory(pose_a: np.ndarray, pose_b: np.ndarray, dof_names: list[str]) -> mg.Trajectory:
    """Time-parameterize a two-pose Franka round trip.

    Args:
        pose_a: Starting joint pose.
        pose_b: Opposite joint pose.
        dof_names: Ordered degree-of-freedom names.

    Returns:
        Minimal-time trajectory from pose A to pose B and back.
    """
    maximum_velocities = np.asarray([0.55 if name in _ARM_JOINTS else 0.08 for name in dof_names], dtype=np.float32)
    maximum_accelerations = np.asarray([1.2 if name in _ARM_JOINTS else 0.20 for name in dof_names], dtype=np.float32)
    waypoints = np.asarray([pose_a, pose_b, pose_a], dtype=np.float32)
    # The follower consumes CPU robot state, so do not inherit Warp's CUDA default on GPU workers.
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


def main(*, visible: bool = True) -> None:
    """Run a Franka joint trajectory, repeating while the interactive viewport remains open.

    Args:
        visible: Whether to present an interactive viewport.
    """
    os.environ.setdefault("OVGL_SS", "1")
    ovstage_stage = None
    physics_manager = None
    viewport = None

    try:
        from isaacsim.ovgl_viewport.debug import Camera, Viewport

        ovstage_stage = Stage("ovstage").import_stage_from_string(_author_stage())
        physics_manager = PhysicsManager.get_instance()
        if not physics_manager.switch_physics_engine("ovphysx"):
            raise RuntimeError("OvPhysX physics engine is unavailable.")
        physics_manager.setup(dt=_PHYSICS_DT)
        if not physics_manager.initialize(ovstage_stage.get_stage_ptr(), ovstage_stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")

        articulation = ArticulationEntity("ovphysx", _ROBOT_PATH)
        if articulation.num_prims != 1:
            raise RuntimeError(f"The Franka articulation was not found at {_ROBOT_PATH}.")

        dof_names = list(articulation.dof_names)
        pose_a = _make_joint_pose(dof_names, _POSE_A)
        pose_b = _make_joint_pose(dof_names, _POSE_B)
        trajectory = _make_trajectory(pose_a, pose_b, dof_names)

        articulation.set_dof_positions(pose_a.reshape(1, -1))
        articulation.set_dof_position_targets(pose_a.reshape(1, -1))
        physics_manager.step()
        if not physics_manager.publish_transforms_to_stage():
            raise RuntimeError("Physics transform publication failed.")

        follower = mg.TrajectoryFollower()
        follower.set_trajectory(trajectory)

        def make_robot_state(values: np.ndarray) -> mg.RobotState:
            return mg.RobotState(
                joints=mg.JointState.from_name(
                    robot_joint_space=dof_names,
                    positions=(
                        dof_names,
                        wp.array(values, dtype=wp.float32, device="cpu"),
                    ),
                )
            )

        cycle_start_state = make_robot_state(pose_a)
        if not follower.reset(cycle_start_state, None, 0.0):
            raise RuntimeError("TrajectoryFollower rejected the Franka trajectory.")

        trajectory_step = 0
        trajectory_step_count = math.ceil(trajectory.duration / _PHYSICS_DT) + 1
        completed_cycles = 0
        simulation_complete = False
        print(f"Following a {trajectory.duration:.2f} second Franka round-trip trajectory.")

        def advance_simulation() -> None:
            nonlocal completed_cycles
            nonlocal cycle_start_state
            nonlocal simulation_complete
            nonlocal trajectory_step

            simulation_time = min(trajectory_step * _PHYSICS_DT, trajectory.duration)
            # `TrajectoryFollower` samples an open-loop trajectory, so the state is refreshed only when a cycle resets.
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
            physics_manager.step()
            if not physics_manager.publish_transforms_to_stage():
                raise RuntimeError("Physics transform publication failed.")
            trajectory_step += 1

            if trajectory_step >= trajectory_step_count:
                completed_cycles += 1
                print(f"Completed Franka trajectory cycle {completed_cycles}.")
                if not visible:
                    simulation_complete = True
                    return
                trajectory_step = 0
                measured_positions = articulation.get_dof_positions().numpy()[0].copy()
                cycle_start_state = make_robot_state(measured_positions)
                if not follower.reset(cycle_start_state, None, 0.0):
                    raise RuntimeError("TrajectoryFollower could not restart the Franka trajectory.")

        viewport = Viewport(
            ovstage_stage,
            title="Franka Motion Generation Trajectory",
            visible=visible,
            camera=Camera(
                target=(0.30, 0.15, 0.45),
                yaw_radians=0.80,
                pitch_radians=0.30,
                distance=2.6,
            ),
        )
        if not viewport.poll_events():
            raise RuntimeError("OVGL viewport closed before its initial render.")
        frame = viewport.render()

        if not visible:
            midpoint_step = trajectory_step_count // 2
            while not simulation_complete:
                advance_simulation()
                if trajectory_step == midpoint_step:
                    frame = viewport.render()
            frame = viewport.render()
        else:
            last_frame_time = time.monotonic()
            accumulated_simulation_time = 0.0
            print(
                "Controls: drag with the left mouse button to look, use WASD to move, use Q/E for down/up, use the "
                "wheel to dolly, press R to reset, and press Escape to quit."
            )
            while viewport.poll_events():
                current_time = time.monotonic()
                accumulated_simulation_time += min(current_time - last_frame_time, 0.25)
                last_frame_time = current_time
                steps_this_frame = 0
                while accumulated_simulation_time >= _PHYSICS_DT and steps_this_frame < 15:
                    advance_simulation()
                    accumulated_simulation_time -= _PHYSICS_DT
                    steps_this_frame += 1
                frame = viewport.render()
        print(f"Viewport rendered {frame.frame_number} frame(s).")
    finally:
        if viewport is not None:
            viewport.close()
        try:
            if physics_manager is not None and physics_manager.is_initialized():
                physics_manager.invalidate()
        finally:
            if ovstage_stage is not None:
                ovstage_stage.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

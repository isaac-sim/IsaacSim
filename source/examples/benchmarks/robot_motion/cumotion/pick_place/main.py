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

"""Benchmark cumotion pick_place control and physics throughput."""

from __future__ import annotations

import argparse
import os
from collections.abc import Iterator

import isaacsim.physics_engines.ovphysx  # noqa: F401
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.foundation.objects import Stage
from isaacsim.performance.benchmarking import benchmark
from isaacsim.physics.entities import ArticulationEntity, RigidBodyEntity
from isaacsim.physics.manager import PhysicsManager
from isaacsim.robot_motion.cumotion import CumotionWorldInterface, RmpFlowController
from state_machine import GuardInput, PickPlaceStateMachine

_ROBOT_PATH = "/World/Franka"
_GROUND_COLLISION_PATH = "/World/Ground/Plane"
_BLOCK_PATHS = ("/World/Block0", "/World/Block1")
_SUPPORT_BLOCK, _MOVING_BLOCK = (0, 1)
_DEFAULT_ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
_ASSET_ROOT = (os.environ.get("ISAACSIM_ASSET_ROOT") or _DEFAULT_ASSET_ROOT).rstrip("/\\")
_FRANKA_USD = f"{_ASSET_ROOT}/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
_TOOL_FRAME = "panda_hand"
_ROOT_LINK = "panda_link0"
_FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")
_BLOCK_SIZE = 0.05
_INITIAL_BLOCK_POSITIONS = np.asarray(
    ((0.45, 0.32, _BLOCK_SIZE / 2.0), (0.45, 0.02, _BLOCK_SIZE / 2.0)), dtype=np.float32
)
_UNSTACK_DESTINATION = np.asarray((0.64, 0.34, _BLOCK_SIZE / 2.0), dtype=np.float64)
_TRANSFERS = ("stack blue cube on orange cube", "unstack blue cube")
_HOME_POSITIONS = {
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
_GRASP_ORIENTATION = np.asarray((0.0, 0.0, 1.0, 0.0), dtype=np.float64)
_HAND_TO_GRASP_POSITION = np.asarray((0.0, 0.0, 0.1034), dtype=np.float64)


def _rotate(vector: np.ndarray, orientation: np.ndarray) -> np.ndarray:
    """Rotate a vector by a normalized WXYZ quaternion."""
    xyz = orientation[1:]
    return vector + 2.0 * (orientation[0] * np.cross(xyz, vector) + np.cross(xyz, np.cross(xyz, vector)))


def _link_pose(transforms: np.ndarray, index: int) -> tuple[np.ndarray, np.ndarray]:
    """Read one link pose as a position and normalized WXYZ orientation."""
    link = transforms[0, index]
    orientation = link[[6, 3, 4, 5]].astype(np.float64, copy=True)
    orientation /= np.linalg.norm(orientation)
    return (link[:3].astype(np.float64, copy=True), orientation)


def _author_stage(physics_device: str) -> tuple[str, Stage]:
    """Author the scene and return its USDA text with the live authoring stage."""
    from isaacsim.foundation.objects import Cube, DistantLight, Mesh, PhysicsScene, PhysxScene
    from isaacsim.foundation.prims import ColliderBody, GroundPlane, RigidBody
    from isaacsim.foundation.utils import stage as stage_utils
    from isaacsim.ovgl_viewport.debug import author_viewport
    from pxr import Sdf

    authoring_stage = Stage("openusd").create_stage(make_default=False)
    try:
        with stage_utils.use_stage(authoring_stage):
            authoring_stage.define_prim("/World")
            GroundPlane("/World/Ground", sizes=2.2).set_local_poses(translations=[0.25, 0.15, 0.0])
            Mesh("/World/Ground/Mesh", reset_xform_op_properties=False).set_attribute_values(
                "primvars:displayColor", np.asarray([[[0.22, 0.24, 0.28]]], dtype=np.float32)
            )
            for path, position, color in zip(
                _BLOCK_PATHS, _INITIAL_BLOCK_POSITIONS, ((0.95, 0.32, 0.05), (0.05, 0.65, 0.95)), strict=True
            ):
                cube = Cube(path, sizes=_BLOCK_SIZE)
                cube.set_local_poses(translations=position.tolist())
                cube.set_attribute_values("primvars:displayColor", np.asarray([[color]], dtype=np.float32))
                ColliderBody(path)
                RigidBody(path)
            PhysicsScene("/World/PhysicsScene").set_gravities([0.0, 0.0, -9.81])
            scene = PhysxScene("/World/PhysicsScene")
            scene.set_enabled_gpu_dynamics(np.array([physics_device == "gpu"]))
            scene.set_broadphase_types("GPU" if physics_device == "gpu" else "MBP")
            light = DistantLight("/World/KeyLight")
            light.set_intensities([[3500.0]])
            light.set_colors([[1.0, 0.93, 0.82]])
            author_viewport(authoring_stage)
            stage_text = authoring_stage.export_stage_to_string()
    except Exception:
        authoring_stage.close_stage()
        raise
    layer = Sdf.Layer.CreateAnonymous("pick_place_state_machine.usda")
    if not layer.ImportFromString(stage_text):
        authoring_stage.close_stage()
        raise RuntimeError("OpenUSD could not reconstruct the authored stage layer.")
    robot = Sdf.CreatePrimInLayer(layer, _ROBOT_PATH)
    robot.specifier = Sdf.SpecifierDef
    robot.typeName = "Xform"
    robot.referenceList.prependedItems = [Sdf.Reference(_FRANKA_USD)]
    return (layer.ExportToString(), authoring_stage)


def _target_state(site_space: list[str], grasp_position: np.ndarray) -> tuple[mg.RobotState, np.ndarray]:
    """Build the hand-frame setpoint for a world-space grasp position."""
    hand_position = grasp_position - _rotate(_HAND_TO_GRASP_POSITION, _GRASP_ORIENTATION)
    target = mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=site_space,
            positions=([_TOOL_FRAME], wp.array([hand_position], dtype=wp.float32, device="cpu")),
            orientations=([_TOOL_FRAME], wp.array([_GRASP_ORIENTATION], dtype=wp.float32, device="cpu")),
        )
    )
    return (target, hand_position)


def _full_command(values: wp.array, indices: wp.array, dof_count: int) -> np.ndarray:
    """Expand a sparse joint command into articulation order."""
    command = np.zeros((1, dof_count), dtype=np.float32)
    command[0, indices.numpy()] = values.numpy()
    return command


def _simulation_steps(
    articulation: ArticulationEntity,
    blocks: RigidBodyEntity,
    manager: PhysicsManager,
    fsm: PickPlaceStateMachine,
    controller: RmpFlowController,
    world_interface: CumotionWorldInterface,
    world_binding: mg.WorldBinding,
    site_space: list[str],
    finger_states: dict[bool, mg.RobotState],
    time_step: float,
) -> Iterator[PickPlaceStateMachine]:
    """Complete one control/physics step and yield the current state machine."""
    dof_names = list(articulation.dof_names)
    link_names = list(articulation.link_names)
    finger_indices = [dof_names.index(name) for name in _FINGER_JOINTS]
    hand_link_index = link_names.index(_TOOL_FRAME)
    root_link_index = link_names.index(_ROOT_LINK)
    simulation_time = 0.0
    for transfer_index in range(len(_TRANSFERS)):
        block_positions = blocks.get_data("transforms").numpy()[:, :3].astype(np.float64, copy=True)
        pick_position = block_positions[_MOVING_BLOCK]
        if transfer_index == 0:
            place_position = block_positions[_SUPPORT_BLOCK].copy()
            place_position[2] += _BLOCK_SIZE
        else:
            place_position = _UNSTACK_DESTINATION.copy()
        fsm.reset(
            transfer_index=transfer_index,
            pick_position=pick_position,
            place_position=place_position,
            time=simulation_time,
        )
        controller_phase = None
        while not fsm.is_done or transfer_index == len(_TRANSFERS) - 1:
            joint_positions = articulation.get_dof_positions().numpy()[0].copy()
            link_transforms = articulation.get_data("link-transforms").numpy()
            hand_position, hand_orientation = _link_pose(link_transforms, hand_link_index)
            root_position, root_orientation = _link_pose(link_transforms, root_link_index)
            block_positions = blocks.get_data("transforms").numpy()[:, :3].astype(np.float64, copy=True)
            block_linear_velocities, block_angular_velocities = blocks.get_velocities()
            moving_position = block_positions[_MOVING_BLOCK]
            grasp_position = hand_position + _rotate(_HAND_TO_GRASP_POSITION, hand_orientation)
            estimated = mg.RobotState(
                joints=mg.JointState.from_name(
                    robot_joint_space=dof_names,
                    positions=(dof_names, wp.array(joint_positions, dtype=wp.float32, device="cpu")),
                )
            )
            world_interface.update_world_to_robot_root_transforms(
                (
                    wp.array([root_position], dtype=wp.float32, device="cpu"),
                    wp.array([root_orientation], dtype=wp.float32, device="cpu"),
                )
            )
            world_binding.synchronize_transforms()
            target, hand_target = _target_state(site_space, np.asarray(fsm.target_position, dtype=np.float64))
            orientation_dot = abs(float(np.dot(hand_orientation, _GRASP_ORIENTATION)))
            phase_changed = fsm.tick(
                GuardInput(
                    position_error=float(np.linalg.norm(hand_position - hand_target)),
                    orientation_error=2.0 * float(np.arccos(np.clip(orientation_dot, 0.0, 1.0))),
                    finger_positions=tuple(float(value) for value in joint_positions[finger_indices]),
                    object_height=float(moving_position[2]),
                    grasp_distance=float(np.linalg.norm(grasp_position - moving_position)),
                    placement_error=float(np.linalg.norm(moving_position - place_position)),
                    object_linear_speed=float(np.linalg.norm(block_linear_velocities.numpy()[_MOVING_BLOCK])),
                    object_angular_speed=float(np.linalg.norm(block_angular_velocities.numpy()[_MOVING_BLOCK])),
                ),
                simulation_time,
            )
            if phase_changed:
                target, _ = _target_state(site_space, np.asarray(fsm.target_position))
            if controller_phase is None or phase_changed:
                if not controller.reset(estimated, target, simulation_time):
                    raise RuntimeError(f"cuMotion RMPflow reset failed in {fsm.phase}.")
                controller_phase = fsm.phase
            arm_state = controller.forward(estimated, target, simulation_time)
            desired = mg.combine_robot_states(arm_state, finger_states[fsm.close_fingers])
            if desired is None or desired.joints is None:
                raise RuntimeError("Could not combine RMPflow and finger RobotState commands.")
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
            simulation_time += time_step
            yield fsm


class Workload:
    """One controller update, completed physics step, and transform publication."""

    def __init__(self, time_step: float = 0.005, physics_device: str = "cpu", render: bool = False) -> None:
        self.time_step = time_step
        self.physics_device = physics_device
        self.render = render
        self.stage = self.manager = self.viewport = None
        self.completed_steps = 0
        self.steps = None
        self.controller = self.world_interface = None
        self.authoring_stage = None
        self.phase_progressed = False

    def setup(self) -> None:
        """Create the scene and initialize physics and controls."""
        os.environ.setdefault("OVGL_SS", "1")
        import isaacsim.core.experimental.utils.stage as compatibility_stage_utils
        from isaacsim.foundation.utils import stage as foundation_stage_utils
        from isaacsim.physics.entities import ArticulationEntity, RigidBodyEntity
        from isaacsim.physics.manager import PhysicsManager
        from isaacsim.robot_motion.cumotion import (
            CumotionWorldInterface,
            RmpFlowController,
            load_cumotion_supported_robot,
        )

        try:
            self.previous_foundation = foundation_stage_utils.get_default_stage()
        except RuntimeError:
            self.previous_foundation = None
        self.previous_compatibility_id = compatibility_stage_utils.get_default_stage_id()
        stage_text, self.authoring_stage = _author_stage(self.physics_device)
        self.stage = Stage("ovstage").import_stage_from_string(stage_text, make_default=False)
        self.manager = PhysicsManager.get_instance()
        if not self.manager.switch_physics_engine("ovphysx"):
            raise RuntimeError("OvPhysX physics engine is unavailable.")
        self.manager.setup(dt=self.time_step)
        if not self.manager.initialize(self.stage.get_stage_ptr(), self.stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")
        with foundation_stage_utils.use_stage(self.stage):
            self.robot = ArticulationEntity("ovphysx", _ROBOT_PATH)
            blocks = RigidBodyEntity("ovphysx", list(_BLOCK_PATHS))
        dof_names = list(self.robot.dof_names)
        link_names = list(self.robot.link_names)
        try:
            hand_link_index = link_names.index(_TOOL_FRAME)
            root_link_index = link_names.index(_ROOT_LINK)
            home = np.asarray([_HOME_POSITIONS[name] for name in dof_names], dtype=np.float32)
        except (KeyError, ValueError) as error:
            raise RuntimeError("The Franka articulation does not match the expected joint/link names.") from error
        self.robot.set_dof_positions(home.reshape(1, -1))
        self.robot.set_dof_position_targets(home.reshape(1, -1))
        self.robot.set_dof_velocities(np.zeros((1, len(dof_names)), dtype=np.float32))
        blocks.set_world_poses(
            positions=_INITIAL_BLOCK_POSITIONS.copy(),
            orientations=np.tile(np.asarray((0.0, 0.0, 0.0, 1.0), dtype=np.float32), (len(_BLOCK_PATHS), 1)),
        )
        blocks.set_velocities(
            linear_velocities=np.zeros((len(_BLOCK_PATHS), 3), dtype=np.float32),
            angular_velocities=np.zeros((len(_BLOCK_PATHS), 3), dtype=np.float32),
        )
        self.manager.step()
        if not self.manager.publish_transforms_to_stage():
            raise RuntimeError("Physics transform publication failed after reset.")
        link_transforms = self.robot.get_data("link-transforms").numpy()
        initial_hand_position, _ = _link_pose(link_transforms, hand_link_index)
        root_position, root_orientation = _link_pose(link_transforms, root_link_index)
        self.world_interface = CumotionWorldInterface(device="cpu")
        self.world_interface.update_world_to_robot_root_transforms(
            (
                wp.array([root_position], dtype=wp.float32, device="cpu"),
                wp.array([root_orientation], dtype=wp.float32, device="cpu"),
            )
        )
        world_binding = mg.WorldBinding(
            world_interface=self.world_interface,
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[_GROUND_COLLISION_PATH],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )
        foundation_stage_utils.set_default_stage(self.authoring_stage)
        compatibility_stage_utils.set_default_stage_id(self.authoring_stage.get_stage_id())
        world_binding.initialize()
        world_binding.synchronize_transforms()
        robot = load_cumotion_supported_robot("franka")
        site_space = list(robot.robot_description.tool_frame_names())
        block_positions = blocks.get_data("transforms").numpy()[:, :3].astype(np.float64, copy=True)
        place_position = block_positions[_SUPPORT_BLOCK].copy()
        place_position[2] += _BLOCK_SIZE
        self.fsm = PickPlaceStateMachine()
        self.fsm.reset(
            transfer_index=0,
            pick_position=block_positions[_MOVING_BLOCK],
            place_position=place_position,
            time=0.0,
        )
        _, self.initial_hand_target = _target_state(site_space, np.asarray(self.fsm.target_position, dtype=np.float64))
        self.initial_approach_error = float(np.linalg.norm(initial_hand_position - self.initial_hand_target))
        self.hand_link_index = hand_link_index
        self.controller = RmpFlowController(
            cumotion_robot=robot,
            cumotion_world_interface=self.world_interface,
            robot_joint_space=dof_names,
            robot_site_space=site_space,
            tool_frame=_TOOL_FRAME,
        )
        finger_states = {}
        for closed, position in ((False, 0.04), (True, 0.0)):
            finger_states[closed] = mg.RobotState(
                joints=mg.JointState.from_name(
                    robot_joint_space=dof_names,
                    positions=(list(_FINGER_JOINTS), wp.array([position, position], dtype=wp.float32, device="cpu")),
                )
            )
        self.steps = _simulation_steps(
            self.robot,
            blocks,
            self.manager,
            self.fsm,
            self.controller,
            self.world_interface,
            world_binding,
            site_space,
            finger_states,
            self.time_step,
        )
        self.initial_time = self.manager.get_simulated_time()
        self.initial_step_count = self.manager.get_simulated_physics_steps()
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
        self.fsm = next(self.steps)
        self.phase_progressed |= self.fsm.phase != "approach_pick"
        if self.viewport is not None:
            self.viewport.render()
        self.completed_steps += 1

    def validate(self) -> bool:
        """Check that simulation advanced with finite state."""
        positions = self.robot.get_dof_positions().numpy()
        link_transforms = self.robot.get_data("link-transforms").numpy()
        final_hand_position, _ = _link_pose(link_transforms, self.hand_link_index)
        final_approach_error = float(np.linalg.norm(final_hand_position - self.initial_hand_target))
        return bool(
            self.completed_steps > 0
            and self.manager.get_simulated_physics_steps() - self.initial_step_count == self.completed_steps
            and self.manager.get_simulated_time() > self.initial_time
            and np.isfinite(positions).all()
            and np.isfinite(final_approach_error)
            and (self.phase_progressed or final_approach_error < self.initial_approach_error)
        )

    def teardown(self) -> None:
        """Release the simulation resources."""
        self.steps = None
        if self.viewport is not None:
            self.viewport.close()
        self.controller = self.world_interface = None
        if self.manager is not None and self.manager.is_initialized():
            self.manager.invalidate()
        if self.stage is not None:
            self.stage.close_stage()
        if self.authoring_stage is not None:
            self.authoring_stage.close_stage()
        import isaacsim.core.experimental.utils.stage as compatibility_stage_utils
        from isaacsim.foundation.utils import stage as foundation_stage_utils

        compatibility_stage_utils.set_default_stage_id(self.previous_compatibility_id)
        if self.previous_foundation is not None:
            foundation_stage_utils.set_default_stage(self.previous_foundation)


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
        "benchmarks.robot_motion.cumotion.pick_place",
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

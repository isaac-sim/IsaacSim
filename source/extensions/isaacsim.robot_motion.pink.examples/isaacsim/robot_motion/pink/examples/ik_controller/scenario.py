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

"""Example demonstrating PINK IK controller for reactive end-effector tracking."""

from __future__ import annotations

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils import transform as transform_utils
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.pink import PinkIKController, load_pink_supported_robot
from isaacsim.storage.native import get_assets_root_path_async

# Franka Panda URDF tool frame name as it appears in Pinocchio after parsing.
_FRANKA_TOOL_FRAME = "panda_hand"
_FRANKA_HAND_TO_FINGERTIP_MIDPOINT = np.array([0.0, 0.0, 0.1034], dtype=np.float32)
_FRANKA_REACH_POSTURE = {
    "panda_joint1": 0.012,
    "panda_joint2": -0.568,
    "panda_joint3": 0.0,
    "panda_joint4": -2.811,
    "panda_joint5": 0.0,
    "panda_joint6": 3.037,
    "panda_joint7": 0.741,
    "panda_finger_joint1": 0.035,
    "panda_finger_joint2": 0.035,
}


def _compute_hand_target_positions(target_positions: wp.array, target_orientations: wp.array) -> wp.array:
    """Compute `panda_hand` targets from visible fingertip target poses."""
    positions = target_positions.numpy() if hasattr(target_positions, "numpy") else np.asarray(target_positions)
    rotations = transform_utils.quaternion_to_rotation_matrix(target_orientations).numpy()
    offsets = rotations @ _FRANKA_HAND_TO_FINGERTIP_MIDPOINT
    return wp.from_numpy((positions - offsets).astype(np.float32), dtype=wp.float32, device=target_positions.device)


def _get_reach_posture(dof_names: list[str]) -> np.ndarray:
    """Get the nominal Franka reach posture in articulation DOF order.

    The posture seeds the articulation away from the fully extended home pose
    and gives the controller's posture task a bent-arm regularization target.
    """
    return np.array([_FRANKA_REACH_POSTURE.get(name, 0.0) for name in dof_names], dtype=np.float32)


class FrankaPinkIKExample:
    """Example demonstrating PINK differential IK controller with a Franka Panda.

    Loads a Franka Panda robot and a movable target cube. On each physics step
    the PinkIKController solves a weighted QP to track the target pose with the
    end-effector while regularizing the posture toward a bent reaching configuration.
    """

    def __init__(self) -> None:
        self._controller: PinkIKController | None = None
        self._articulation: Articulation | None = None
        self._target: Cube | None = None
        self._sim_time: float = 0.0
        self._controller_reset: bool = False
        self._initial_posture_applied: bool = False
        self._reach_posture: np.ndarray | None = None
        self._robot_joint_space: list[str] = []
        self._robot_site_space: list[str] = [_FRANKA_TOOL_FRAME]

    async def load_example_assets(self) -> tuple:
        """Load robot and target assets to the stage.

        Returns:
            Tuple of (articulation, target_cube) prims.
        """
        self._robot_prim_path = "/panda"
        path_to_robot_usd = await get_assets_root_path_async() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

        add_reference_to_stage(path_to_robot_usd, self._robot_prim_path)
        self._articulation = Articulation(self._robot_prim_path)

        self._target = Cube(paths="/World/target", sizes=0.04, positions=[0.5, 0.0, 0.25])

        return self._articulation, self._target

    def setup(self) -> None:
        """Set up the PINK IK controller after assets are loaded."""
        pink_robot = load_pink_supported_robot("franka")

        self._robot_joint_space = self._articulation.dof_names
        self._reach_posture = _get_reach_posture(self._robot_joint_space)

        self._controller = PinkIKController(
            pink_robot=pink_robot,
            robot_joint_space=self._robot_joint_space,
            robot_site_space=self._robot_site_space,
            tool_frame=_FRANKA_TOOL_FRAME,
            position_cost=5.0,
            orientation_cost=0.05,
            posture_cost=5e-3,
            solver="osqp",
            dt=1.0 / 60.0,
        )

        # Set initial target position near the nominal reaching posture.
        quat = transform_utils.euler_angles_to_quaternion([0, np.pi, 0]).numpy()
        self._target.set_world_poses(positions=np.array([[0.45, 0.0, 0.55]]), orientations=np.array([quat]))

        self._sim_time = 0.0
        self._controller_reset = False
        self._initial_posture_applied = False

    def update(self, step: float) -> None:
        """Update controller on each physics step.

        Args:
            step: Physics timestep in seconds.
        """
        if self._controller is None:
            return

        if not self._articulation.is_physics_tensor_entity_valid():
            return

        if not self._initial_posture_applied:
            self._articulation.set_dof_positions(self._reach_posture)
            self._articulation.set_dof_position_targets(self._reach_posture)
            self._initial_posture_applied = True
            self._controller_reset = False
            return

        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self._robot_joint_space,
                positions=(self._robot_joint_space, self._articulation.get_dof_positions()),
                velocities=(self._robot_joint_space, self._articulation.get_dof_velocities()),
            )
        )

        target_positions, target_orientations = self._target.get_world_poses()
        hand_target_positions = _compute_hand_target_positions(target_positions, target_orientations)

        setpoint_state = mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=self._robot_site_space,
                positions=([_FRANKA_TOOL_FRAME], hand_target_positions),
                orientations=([_FRANKA_TOOL_FRAME], target_orientations),
            ),
        )

        if not self._controller_reset:
            self._controller_reset = self._controller.reset(estimated_state, setpoint_state, self._sim_time)

        desired_state = self._controller.forward(estimated_state, setpoint_state, self._sim_time)

        if desired_state is not None and desired_state.joints.positions is not None:
            self._articulation.set_dof_position_targets(
                positions=desired_state.joints.positions,
                dof_indices=desired_state.joints.position_indices,
            )

        self._sim_time += step

    def reset(self) -> None:
        """Reset the example."""
        self.setup()

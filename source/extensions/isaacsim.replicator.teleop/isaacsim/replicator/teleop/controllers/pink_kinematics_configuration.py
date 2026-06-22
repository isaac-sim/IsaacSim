# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Dual-model PINK configuration for selective joint control.

Wraps Pinocchio with a **full model** (all joints) and a **reduced model**
(controlled joints only).  The IK solver operates on the reduced model;
FK uses the full model so non-controlled joints are accounted for.

Zero Isaac Sim / Isaac Lab dependencies -- only requires pinocchio, pink,
and numpy.

Based on Isaac Lab's ``PinkKinematicsConfiguration``.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin
from pink.configuration import Configuration
from pink.exceptions import FrameNotFound
from pinocchio.robot_wrapper import RobotWrapper


class PinkKinematicsConfiguration(Configuration):
    """Configuration maintaining both a full and a reduced (controlled) model.

    The **reduced model** contains only the actively-controlled joints and is
    passed to PINK's ``solve_ik``.  The **full model** contains every joint
    and is used for forward kinematics so that non-controlled joint positions
    (e.g. gripper fingers, waist, lower body) are properly accounted for when
    computing end-effector poses and Jacobians.

    Args:
        controlled_joint_names: Joint names to optimise (IK decision variables).
        urdf_path: Path to robot URDF file.
        mesh_path: Optional path to mesh directory for collision geometry.
        copy_data: If True, work on an internal copy of the data.
        forward_kinematics: If True, run FK at construction time.
    """

    def __init__(
        self,
        controlled_joint_names: list[str],
        urdf_path: str,
        mesh_path: str | None = None,
        copy_data: bool = True,
        forward_kinematics: bool = True,
    ) -> None:
        self._controlled_joint_names = controlled_joint_names

        if mesh_path:
            self.robot_wrapper = RobotWrapper.BuildFromURDF(urdf_path, mesh_path)
        else:
            self.robot_wrapper = RobotWrapper.BuildFromURDF(urdf_path)
        self.full_model = self.robot_wrapper.model
        self.full_data = self.robot_wrapper.data
        self.full_q = self.robot_wrapper.q0

        # All joint names in Pinocchio traversal order (skip "universe" at [0])
        self._all_joint_names = self.full_model.names.tolist()[1:]

        # Indices of controlled joints within the full model's joint list
        self._controlled_joint_indices = [
            idx for idx, name in enumerate(self._all_joint_names) if name in self._controlled_joint_names
        ]

        # Lock every joint that is NOT controlled
        joints_to_lock = [
            self.full_model.getJointId(name)
            for name in self._all_joint_names
            if name not in self._controlled_joint_names
        ]

        if len(joints_to_lock) == 0:
            self.controlled_model = self.full_model
            self.controlled_data = self.full_data
            self.controlled_q = self.full_q
        else:
            self.controlled_model = pin.buildReducedModel(self.full_model, joints_to_lock, self.full_q)
            self.controlled_data = self.controlled_model.createData()
            self.controlled_q = self.full_q[self._controlled_joint_indices]

        # PINK's Configuration base class operates on the reduced model
        super().__init__(
            self.controlled_model,
            self.controlled_data,
            self.controlled_q,
            copy_data,
            forward_kinematics,
        )

    # ------------------------------------------------------------------
    # Overrides
    # ------------------------------------------------------------------

    def update(self, q: np.ndarray | None = None) -> None:
        """Update both models with a new joint-position vector.

        Args:
            q: ALL joints in Pinocchio order (length == number of joints in the full model).  Controlled joints are extracted automatically. Pass ``None`` to re-run FK on the current configuration.

        Raises:
            ValueError: If *q* length does not match the full model.
        """
        if q is not None and len(q) != len(self._all_joint_names):
            raise ValueError(
                f"q must have {len(self._all_joint_names)} elements " f"(one per joint in the full model), got {len(q)}"
            )
        if q is not None:
            # Reduced model (IK solver)
            super().update(q[self._controlled_joint_indices])
            # Full model (FK for frame poses and Jacobians)
            q_readonly = q.copy()
            q_readonly.setflags(write=False)
            self.full_q = q_readonly
            pin.computeJointJacobians(self.full_model, self.full_data, q)
            pin.updateFramePlacements(self.full_model, self.full_data)
        else:
            super().update()
            pin.computeJointJacobians(self.full_model, self.full_data, self.full_q)
            pin.updateFramePlacements(self.full_model, self.full_data)

    def get_frame_jacobian(self, frame: str) -> np.ndarray:
        """Frame Jacobian from the full model, columns for controlled joints only.

        Args:
            frame: Value for frame.

        Returns:
            The requested value.
        """
        if not self.full_model.existFrame(frame):
            raise FrameNotFound(frame, self.full_model.frames)
        frame_id = self.full_model.getFrameId(frame)
        J: np.ndarray = pin.getFrameJacobian(self.full_model, self.full_data, frame_id, pin.ReferenceFrame.LOCAL)
        return J[:, self._controlled_joint_indices]

    def get_transform_frame_to_world(self, frame: str) -> pin.SE3:
        """Frame pose using the full model (non-controlled joints accounted for).

        Overrides the default PINK implementation which only uses the reduced
        model and therefore assumes non-controlled joints stay at q0.

        Args:
            frame: Value for frame.

        Returns:
            The requested value.
        """
        frame_id = self.full_model.getFrameId(frame)
        try:
            return self.full_data.oMf[frame_id].copy()
        except IndexError as exc:
            raise FrameNotFound(frame, self.full_model.frames) from exc

    def check_limits(self, tol: float = 1e-6, safety_break: bool = True) -> None:
        """Only enforce limits when *safety_break* is enabled.

        Args:
            tol: Value for tol.
            safety_break: Value for safety break.
        """
        if safety_break:
            super().check_limits(tol, safety_break)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def controlled_joint_names_pinocchio_order(self) -> list[str]:
        """Controlled joint names in Pinocchio traversal order.

        Returns:
            The requested value.
        """
        return [self._all_joint_names[i] for i in self._controlled_joint_indices]

    @property
    def all_joint_names_pinocchio_order(self) -> list[str]:
        """All joint names in Pinocchio traversal order.

        Returns:
            The requested value.
        """
        return list(self._all_joint_names)

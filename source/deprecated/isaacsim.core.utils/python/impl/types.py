# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Deprecated type definitions for simulation data structures."""

from __future__ import annotations

import numpy as np
from isaacsim.core.deprecation_manager import import_module
from pxr import Usd

torch = import_module("torch")


class DataFrame(object):
    """Container for simulation data at a specific time step.

    Args:
        current_time_step: The current simulation time step index.
        current_time: The current simulation time in seconds.
        data: Dictionary containing the simulation data.
    """

    def __init__(self, current_time_step: int, current_time: float, data: dict) -> None:
        self.current_time_step = current_time_step
        self.current_time = current_time
        self.data = data

    def get_dict(self) -> dict:
        """Convert the DataFrame to a dictionary representation.

        Returns:
            Dictionary with time step, time, and data fields.
        """
        return {"current_time": self.current_time, "current_time_step": self.current_time_step, "data": self.data}

    def __str__(self) -> str:
        """String representation of the DataFrame.

        Returns:
            Dictionary representation of the DataFrame as a string.
        """
        return str(self.get_dict())

    @classmethod
    def init_from_dict(cls, dict_representation: dict) -> "DataFrame":
        """Create a DataFrame instance from a dictionary.

        Args:
            dict_representation: Dictionary containing time_step, time, and data.

        Returns:
            A new DataFrame instance initialized from the dictionary.
        """
        frame = object.__new__(cls)
        frame.current_time_step = dict_representation["current_time_step"]
        frame.current_time = dict_representation["current_time"]
        frame.data = dict_representation["data"]
        return frame


class DOFInfo(object):
    """Information about a degree of freedom in an articulation.

    Args:
        prim_path: The USD prim path for this DOF.
        handle: The physics handle for this DOF.
        prim: The USD prim object for this DOF.
        index: The index of this DOF in the articulation.
    """

    def __init__(self, prim_path: str, handle: int, prim: Usd.Prim, index: int) -> None:
        self.prim_path = prim_path
        self.handle = handle
        self.prim = prim
        self.index = index
        return


class XFormPrimState(object):
    """State of an XFormPrim containing position and orientation.

    Args:
        position: The position as a numpy array of shape (3,).
        orientation: The orientation quaternion (w, x, y, z) as a numpy array of shape (4,).
    """

    def __init__(self, position: np.ndarray, orientation: np.ndarray) -> None:
        self.position = position
        self.orientation = orientation


class XFormPrimViewState(object):
    """State of multiple XFormPrims containing positions and orientations.

    Args:
        positions: Positions with shape (N, 3).
        orientations: Quaternion orientations (scalar first) with shape (N, 4).
    """

    def __init__(self, positions: np.ndarray | torch.Tensor, orientations: np.ndarray | torch.Tensor) -> None:
        self.positions = positions
        self.orientations = orientations


class DynamicState(object):
    """State of a dynamic rigid body including pose and velocities.

    Args:
        position: The position as a numpy array of shape (3,).
        orientation: The orientation quaternion (w, x, y, z) as a numpy array of shape (4,).
        linear_velocity: The linear velocity as a numpy array of shape (3,).
        angular_velocity: The angular velocity as a numpy array of shape (3,).
    """

    def __init__(
        self, position: np.ndarray, orientation: np.ndarray, linear_velocity: np.ndarray, angular_velocity: np.ndarray
    ) -> None:
        self.position = position
        self.orientation = orientation
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity


class DynamicsViewState(object):
    """State of multiple dynamic rigid bodies including poses and velocities.

    Args:
        positions: Positions with shape (N, 3).
        orientations: Quaternion orientations (scalar first) with shape (N, 4).
        linear_velocities: Linear velocities with shape (N, 3).
        angular_velocities: Angular velocities with shape (N, 3).
    """

    def __init__(
        self,
        positions: np.ndarray | torch.Tensor,
        orientations: np.ndarray | torch.Tensor,
        linear_velocities: np.ndarray | torch.Tensor,
        angular_velocities: np.ndarray | torch.Tensor,
    ) -> None:
        self.positions = positions
        self.orientations = orientations
        self.linear_velocities = linear_velocities
        self.angular_velocities = angular_velocities


class JointsState(object):
    """State of articulation joints including positions, velocities, and efforts.

    Args:
        positions: Joint positions array.
        velocities: Joint velocities array.
        efforts: Joint efforts (torques/forces) array.
    """

    def __init__(self, positions: np.ndarray, velocities: np.ndarray, efforts: np.ndarray) -> None:
        self.positions = positions
        self.velocities = velocities
        self.efforts = efforts


class ArticulationAction(object):
    """Action to apply to an articulation's joints.

    Args:
        joint_positions: Target joint positions.
        joint_velocities: Target joint velocities.
        joint_efforts: Target joint efforts (torques/forces).
        joint_indices: Joint indices to specify which joints to manipulate.
    """

    def __init__(
        self,
        joint_positions: list | np.ndarray | None = None,
        joint_velocities: list | np.ndarray | None = None,
        joint_efforts: list | np.ndarray | None = None,
        joint_indices: list | np.ndarray | None = None,
    ) -> None:
        self.joint_positions = joint_positions
        self.joint_velocities = joint_velocities
        self.joint_efforts = joint_efforts
        self.joint_indices = joint_indices

    def get_dof_action(self, index: int) -> dict:
        """Get the action for a specific DOF as a dictionary.

        Args:
            index: The index of the DOF to get the action for.

        Returns:
            Dictionary containing the position, velocity, or effort for this DOF.
        """
        if self.joint_efforts is not None and self.joint_efforts[index] is not None:
            return {"effort": self.joint_efforts[index]}
        else:
            dof_action = {}
            if self.joint_velocities is not None and self.joint_velocities[index] is not None:
                dof_action["velocity"] = self.joint_velocities[index]
            if self.joint_positions is not None and self.joint_positions[index] is not None:
                dof_action["position"] = self.joint_positions[index]
            return dof_action

    def get_dict(self) -> dict:
        """Convert the ArticulationAction to a dictionary representation.

        Returns:
            Dictionary with joint_positions, joint_velocities, and joint_efforts.
        """
        result = {}
        if self.joint_positions is not None:
            if isinstance(self.joint_positions, np.ndarray):
                result["joint_positions"] = self.joint_positions.tolist()
            else:
                result["joint_positions"] = self.joint_positions
        else:
            result["joint_positions"] = None
        if self.joint_velocities is not None:
            if isinstance(self.joint_velocities, np.ndarray):
                result["joint_velocities"] = self.joint_velocities.tolist()
            else:
                result["joint_velocities"] = self.joint_velocities
        else:
            result["joint_velocities"] = None
        if self.joint_efforts is not None:
            if isinstance(self.joint_efforts, np.ndarray):
                result["joint_efforts"] = self.joint_efforts.tolist()
            else:
                result["joint_efforts"] = self.joint_efforts
        else:
            result["joint_efforts"] = None
        return result

    def __str__(self) -> str:
        """String representation of the ArticulationAction.

        Returns:
            String representation of the action's dictionary.
        """
        return str(self.get_dict())

    def get_length(self) -> int | None:
        """Get the number of joints this action applies to.

        Returns:
            The length of the action arrays, or None if all arrays are None.
        """
        size = None
        if self.joint_positions is not None:
            if size is None:
                size = 0
            if isinstance(self.joint_positions, np.ndarray):
                size = max(size, self.joint_positions.shape[0])
            else:
                size = max(size, len(self.joint_positions))
        if self.joint_velocities is not None:
            if size is None:
                size = 0
            if isinstance(self.joint_velocities, np.ndarray):
                size = max(size, self.joint_velocities.shape[0])
            else:
                size = max(size, len(self.joint_velocities))
        if self.joint_efforts is not None:
            if size is None:
                size = 0
            if isinstance(self.joint_efforts, np.ndarray):
                size = max(size, self.joint_efforts.shape[0])
            else:
                size = max(size, len(self.joint_efforts))
        return size


class ArticulationActions(object):
    """Actions to apply to multiple articulations' joints.

    Args:
        joint_positions: Target joint positions.
        joint_velocities: Target joint velocities.
        joint_efforts: Target joint efforts (torques/forces).
        joint_indices: Joint indices to specify which joints to manipulate. Shape (K,).
            Where K <= num of dofs.
        joint_names: Joint names to specify which joints to manipulate
            (cannot be specified together with joint_indices). Shape (K,).
            Where K <= num of dofs.
    """

    def __init__(
        self,
        joint_positions: list | np.ndarray | None = None,
        joint_velocities: list | np.ndarray | None = None,
        joint_efforts: list | np.ndarray | None = None,
        joint_indices: list | np.ndarray | None = None,
        joint_names: list[str] | None = None,
    ) -> None:
        if joint_names is not None and joint_indices is not None:
            raise Exception("joint indices and joint names can't be both specified")
        self.joint_positions = joint_positions
        self.joint_velocities = joint_velocities
        self.joint_efforts = joint_efforts
        self.joint_indices = joint_indices
        self.joint_names = joint_names


SDF_type_to_Gf = {
    "matrix3d": "Gf.Matrix3d",
    "matrix3f": "Gf.Matrix3f",
    "matrix4d": "Gf.Matrix4d",
    "matrix4f": "Gf.Matrix4f",
    "range1d": "Gf.Range1d",
    "range1f": "Gf.Range1f",
    "range2d": "Gf.Range2d",
    "range2f": "Gf.Range2f",
    "range3d": "Gf.Range3d",
    "range3f": "Gf.Range3f",
    "rect2i": "Gf.Rect2i",
    "vec2d": "Gf.Vec2d",
    "vec2f": "Gf.Vec2f",
    "vec2h": "Gf.Vec2h",
    "vec2i": "Gf.Vec2i",
    "vec3d": "Gf.Vec3d",
    "double3": "Gf.Vec3d",
    "vec3f": "Gf.Vec3f",
    "vec3h": "Gf.Vec3h",
    "vec3i": "Gf.Vec3i",
    "vec4d": "Gf.Vec4d",
    "vec4f": "Gf.Vec4f",
    "vec4h": "Gf.Vec4h",
    "vec4i": "Gf.Vec4i",
}

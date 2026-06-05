# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""A utility class for working with a subset of joints in a robot articulation."""

from __future__ import annotations

import functools
from collections.abc import Callable
from typing import Any

import carb
import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction, JointsState


def require_initialized(func: callable) -> Callable:
    """Decorator that ensures the underlying articulation is initialized before function execution.

    Prints a warning if the underlying articulation isn't initialized and returns None. If it
    is initialized, the function is called as usual and the value returned.

    Args:
        func: The function to decorate.

    Returns:
        The decorated function that checks initialization before execution.

    """

    @functools.wraps(func)
    def decorator(self: Any, *args: Any, **kwargs: Any) -> Any:
        if not self.is_initialized:
            carb.log_warn("Attempting to access an uninitialized robot Articulation.")
            return None
        return func(self, *args, **kwargs)

    return decorator


class ArticulationSubset:
    """A utility class for viewing a subset of the joints in a robot Articulation object.

    This class can be helpful in two ways:

    1) The order of joints returned by a robot Articulation may not match the order of joints
       expected by a function

    2) A function may only care about a subset of the joint states that are returned by a robot
       Articulation.

    Example:

        Suppose the robot Articulation returns positions [0,1,2] for joints ["A","B","C"], and
        suppose that we pass joint_names = ["B","A"].

        ArticulationSubset.get_joint_positions() -> [1,0]
        ArticulationSubset.map_to_articulation_order([1,0]) -> [0,1,None]

    Args:
        articulation: An initialized SingleArticulation object representing the simulated robot.
        joint_names: A list of joint names whose order determines the order of the joints returned by
            functions like get_joint_positions().

    """

    def __init__(self, articulation: SingleArticulation, joint_names: list[str]) -> None:
        self.articulation = articulation
        self.joint_names = joint_names
        self._joint_indices = None

    @property
    def is_initialized(self) -> bool:
        """Whether or not the underlying articulation object has been initialized.

        Returns:
            True if the articulation is initialized, False otherwise.

        """
        return self.articulation.handles_initialized

    @property
    def num_joints(self) -> int:
        """Number of joints in this subset.

        Returns:
            The count of joint names specified during construction.

        """
        return len(self.joint_names)

    @property
    def joint_indices(self) -> np.array:
        """Joint indices for this subset within the full articulation.

        Returns:
            Array of indices into the articulation's degrees of freedom.

        """
        return self._get_joint_indices()

    @require_initialized
    def get_joint_positions(self) -> np.array:
        """Get joint positions for the joint names that were passed into this articulation view on.

            initialization. The indices of the joint positions returned correspond to the indices of
            the joint names.

        Returns:
            Joint positions.

        """
        return self.articulation.get_joint_positions()[self._get_joint_indices()]

    @require_initialized
    def get_joint_velocities(self) -> np.array:
        """Get joint velocities for the joint names that were passed into this articulation view on.

            initialization. The indices of the joint velocities returned correspond to the indices of
            the joint names.

        Returns:
            Joint velocities.

        """
        return self.articulation.get_joint_velocities()[self._get_joint_indices()]

    @require_initialized
    def get_joint_efforts(self) -> np.array:
        """Get joint efforts for the joint names that were passed into this articulation view on.

            initialization. The indices of the joint efforts returned correspond to the indices of the
            joint names.

        Returns:
            Joint efforts.

        """
        return self.get_measured_joint_efforts()

    @require_initialized
    def get_measured_joint_efforts(self) -> np.array:
        """Get measured joint efforts for this subset's joints.

        Returns:
            np.array: measured joint efforts

        """
        return self.articulation.get_measured_joint_efforts()[self._get_joint_indices()]

    @require_initialized
    def get_applied_joint_efforts(self) -> np.array:
        """Get applied joint efforts for this subset's joints.

        Returns:
            np.array: applied joint efforts

        """
        return self.articulation.get_applied_joint_efforts()[self._get_joint_indices()]

    @require_initialized
    def set_joint_positions(self, positions: np.array) -> None:
        """Set the joint positions for this view.

        Args:
            positions: The position values, one for each view joint in the order specified on
                construction.

        Returns:
            The result of setting joint positions on the articulation.

        """
        return self.articulation.set_joint_positions(positions, self._get_joint_indices())

    @require_initialized
    def set_joint_velocities(self, velocities: np.array) -> None:
        """Set the joint velocities for this view.

        Args:
            velocities: The velocity values, one for each view joint in the order specified on
                construction.

        Returns:
            The result of setting joint velocities on the articulation.

        """
        return self.articulation.set_joint_velocities(velocities, self._get_joint_indices())

    @require_initialized
    def set_joint_efforts(self, efforts: np.array) -> None:
        """Set the joint efforts for this view.

        Args:
            efforts: The effort values, one for each view joint in the order specified on
                construction.

        Returns:
            The result of setting joint efforts on the articulation.

        """
        return self.articulation.set_joint_efforts(efforts, self._get_joint_indices())

    @require_initialized
    def map_to_articulation_order(self, joint_values: np.array) -> np.array:
        """Map a set of joint values to a format consumable by the robot Articulation.

        Args:
            joint_values: A set of joint values corresponding to the joint_names used to initialize this class.
                joint_values may be either one or two dimensional.

                If one dimensional with shape (k,): A vector will be returned with length (self.articulation.num_dof) that may
                be consumed by the robot Articulation in an ArticulationAction.

                If two dimensional with shape (N, k): A matrix will be returned with shape (N, self.articulation.num_dof) that may be
                converted to N ArticulationActions

        Returns:
            A set of joint values that is padded with None to match the shape and order expected by the robot Articulation.

        """
        joint_values = np.asarray(joint_values)
        joint_indices = self._get_joint_indices()

        is_single_action = joint_values.ndim == 1
        if is_single_action:
            joint_values = joint_values.reshape((1, joint_values.size))

        actions = np.full((joint_values.shape[0], self.articulation.num_dof), None)
        actions[:, joint_indices] = joint_values

        if is_single_action:
            return actions[0]
        return actions

    @require_initialized
    def make_articulation_action(self, joint_positions: np.array, joint_velocities: np.array) -> ArticulationAction:
        """Make an articulation action for only this subset's joints using the given target.

        position and velocity values.

        Args:
            joint_positions: Target joint positions for this subset's joints.
            joint_velocities: Target joint velocities for this subset's joints.

        Returns:
            An ArticulationAction object specifying the action for this subset's joints.

        """
        return ArticulationAction(
            joint_positions=joint_positions, joint_velocities=joint_velocities, joint_indices=self._get_joint_indices()
        )

    @require_initialized
    def apply_action(self, joint_positions: np.array | None = None, joint_velocities: np.array | None = None) -> None:
        """Apply the specified control actions to this views joints.

        Args:
            joint_positions: Target joint positions for this subset's joints.
            joint_velocities: Target joint velocities for this subset's joints.

        """
        self.articulation.apply_action(self.make_articulation_action(joint_positions, joint_velocities))

    @require_initialized
    def get_applied_action(self) -> ArticulationAction:
        """Retrieves the latest applied action for this subset.

        Returns:
            The ArticulationAction for this subset. Each commanded entry is either None or
            contains one value for each of the subset's joints. The joint_indices is set to this
            subset's joint indices.

        """
        joint_indices = self._get_joint_indices()
        action = self.articulation.get_applied_action()

        if action.joint_positions is not None:
            action.joint_positions = action.joint_positions[joint_indices]

        if action.joint_velocities is not None:
            action.joint_velocities = action.joint_velocities[joint_indices]

        if action.joint_efforts is not None:
            action.joint_efforts = action.joint_efforts[joint_indices]

        action.joint_indices = joint_indices
        return action

    @require_initialized
    def get_joints_state(self) -> JointsState:
        """Joint state for this subset's joints.

        Returns:
            The joint state with positions, velocities, and efforts filtered to this subset's joints.

        """
        joint_indices = self._get_joint_indices()
        joints_state = self.articulation.get_joints_state()

        if joints_state.positions is not None:
            joints_state.positions = joints_state.positions[joint_indices]

        if joints_state.velocities is not None:
            joints_state.velocities = joints_state.velocities[joint_indices]

        if joints_state.efforts is not None:
            joints_state.efforts = joints_state.efforts[joint_indices]

        return joints_state

    def get_joint_subset_indices(self) -> np.array:
        """Accessor for the joint indices for this subset. These are the indices into the full.

        articulation degrees of freedom corresponding to this subset of joints.

        Returns:
            An array of joint indices defining the subset.

        """
        return self._get_joint_indices()

    def _get_joint_indices(self) -> np.array:
        """Internal member which initializes the subset's joint indices from the specified names.

        the first time through and returns that from then on out.

        Returns:
            The cached joint indices for this subset.

        """
        if self._joint_indices is not None:
            return self._joint_indices

        # If the member isn't set, initialize it with all the joints in the order specified on
        # construction and cache the result.The next time this method is called, it'll retrieve the
        # cached values.
        if self.articulation.handles_initialized:
            self._joint_indices = [self.articulation.get_dof_index(joint) for joint in self.joint_names]

        return self._joint_indices

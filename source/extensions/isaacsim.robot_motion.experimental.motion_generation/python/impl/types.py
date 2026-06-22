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

"""Provides container classes for representing robot states including joints, spatial frames, and root links."""

from __future__ import annotations

from enum import Enum, auto
from typing import Literal, TypeAlias

import numpy as np
import warp as wp


class _LazyFieldUnset(Enum):
    """Lazily-built row slice not yet materialised.

    ``None`` is reserved for a valid materialised state ("no data in this row"), so we
    use a dedicated enum member instead of ``object()`` for clearer static analysis.
    """

    UNSET = auto()


_UNSET: _LazyFieldUnset = _LazyFieldUnset.UNSET

# Row views that may be absent (``None``), not yet built (``_UNSET``), or materialised.
# These aliases are quoted (forward references) because ``wp.array`` is a Warp
# C-extension type whose metaclass does not implement ``__or__``; an unquoted
# ``wp.array | None`` is a runtime assignment expression (``from __future__ import
# annotations`` only defers annotations, not assignment values) and would raise
# ``TypeError`` at import. The string keeps the modern ``|`` syntax for type checkers.
_LazyOptionalWpArray: TypeAlias = "wp.array | None | _LazyFieldUnset"
# Index arrays: always ``wp.array`` once built; ``_UNSET`` until first property read.
_LazyWpIndices: TypeAlias = "wp.array | _LazyFieldUnset"


class JointState:
    """Container for the state of robot joints.

    User should almost never call this constructor directly. Instead, use the from_name or from_index methods.

    Args:
        robot_joint_space: The ordered list of joint names defining the joint space.
        data_array: Pre-allocated data array with shape (3, N) where N is the length of robot_joint_space.
            Row 0 contains positions, row 1 contains velocities, row 2 contains efforts.
        valid_array: Boolean array with shape (3, N) indicating which fields are valid for each joint.

    Raises:
        ValueError: If data_array is not a 2D wp.array with dtype wp.float32.
        ValueError: If valid_array is not a 2D wp.array with dtype wp.bool.
        ValueError: If data_array or valid_array shapes don't match (3, len(robot_joint_space)).
    """

    def __init__(
        self,
        robot_joint_space: list[str],
        data_array: wp.array,
        valid_array: wp.array,
    ) -> None:
        if not isinstance(data_array, wp.array) or not (data_array.dtype is wp.float32) or (data_array.ndim != 2):
            raise ValueError(f"data_array must be a 2D wp.array with data-type wp.float32")

        if not isinstance(valid_array, wp.array) or not (valid_array.dtype is wp.bool) or (valid_array.ndim != 2):
            raise ValueError(f"valid_array must be a 2D wp.array with data-type wp.bool")

        if not (data_array.shape == valid_array.shape == (3, len(robot_joint_space))):
            raise ValueError(f"One of data array or valid array is not correctly sized for this joint-space.")

        if not len(set(robot_joint_space)) == len(robot_joint_space):
            raise ValueError("robot_joint_space is not valid: cannot have duplicated joint names.")

        # Store full matrices, which makes for very efficient combining
        # of JointState objects which are intended for the same robot joint space.
        self.__robot_joint_space = robot_joint_space
        self.__data_array = data_array
        self.__valid_array = valid_array

        # Pre-compute which joints are valid per row once, as numpy index arrays.
        # This avoids repeated valid_array.numpy() + np.where() calls.
        valid_np = valid_array.numpy()
        self.__position_valid_idx = np.where(valid_np[0, :].flatten())[0]
        self.__velocity_valid_idx = np.where(valid_np[1, :].flatten())[0]
        self.__effort_valid_idx = np.where(valid_np[2, :].flatten())[0]

        # Names are cheap (list comprehensions) so we materialise them eagerly.
        self.__position_names = [self.__robot_joint_space[i] for i in self.__position_valid_idx]
        self.__velocity_names = [self.__robot_joint_space[i] for i in self.__velocity_valid_idx]
        self.__effort_names = [self.__robot_joint_space[i] for i in self.__effort_valid_idx]

        # Per-row wp.arrays (data + indices) are built lazily on first property access.
        # The previous eager construction issued an H2D ``wp.from_numpy`` per row
        # (six wp.copy calls per JointState), even when the caller never read the
        # corresponding property — which is the common case for ``efforts`` and for
        # ``velocities`` returned by RmpFlowController.forward().
        self.__valid_positions: _LazyOptionalWpArray = _UNSET
        self.__valid_velocities: _LazyOptionalWpArray = _UNSET
        self.__valid_efforts: _LazyOptionalWpArray = _UNSET
        self.__position_indices: _LazyWpIndices = _UNSET
        self.__velocity_indices: _LazyWpIndices = _UNSET
        self.__effort_indices: _LazyWpIndices = _UNSET

    @classmethod
    def from_name(
        cls,
        robot_joint_space: list[str],
        positions: tuple[list[str], wp.array] | None = None,
        velocities: tuple[list[str], wp.array] | None = None,
        efforts: tuple[list[str], wp.array] | None = None,
    ) -> "JointState":
        """Create a JointState from joint names and data arrays.

        At least one of positions, velocities, or efforts must be provided. Each tuple
        contains a list of joint names and a corresponding warp array of values.

        **Array Shape Support:**
        The data arrays can be either:
        - 1D arrays (shape: ``(N,)``) where N is the number of joints
        - 2D arrays (shape: ``(1, N)``) where the first dimension must be size 1

        Args:
            robot_joint_space: The ordered list of joint names defining the joint space
                of the controlled robot (for example, `Articulation.dof_names`).
            positions: Tuple of (joint_names, position_array) where joint_names is a list
                of joint names and position_array is a 1D or 2D warp array (with first
                dimension size 1) of position values.
            velocities: Tuple of (joint_names, velocity_array) where joint_names is a list
                of joint names and velocity_array is a 1D or 2D warp array (with first
                dimension size 1) of velocity values.
            efforts: Tuple of (joint_names, effort_array) where joint_names is a list
                of joint names and effort_array is a 1D or 2D warp array (with first
                dimension size 1) of effort values.

        Returns:
            A JointState instance with the provided joint data.

        Raises:
            ValueError: If all of positions, velocities, and efforts are None.
            ValueError: If any provided array is not a 1D or 2D warp array of float types.
            ValueError: If a 2D array is provided and the first dimension is not size 1.
            ValueError: If any provided array has length less than 1.
            ValueError: If the first element of any tuple is not a list of joint names.
            ValueError: If any array length doesn't match its corresponding name list length.
            ValueError: If any joint names are duplicated.
            ValueError: If any joint names are not in the robot_joint_space.

        Example:
            .. code-block:: python

                # Direct usage with Articulation (no flatten/reshape needed):
                robot_joint_space = articulation.dof_names
                joint_state = JointState.from_name(
                    robot_joint_space=robot_joint_space,
                    positions=(robot_joint_space, articulation.get_dof_positions()),
                    velocities=(robot_joint_space, articulation.get_dof_velocities()),
                )

                # Or with 1D arrays:
                positions_1d = wp.array([0.0, 1.0, 2.0], dtype=wp.float32)
                joint_state = JointState.from_name(
                    robot_joint_space=["joint_0", "joint_1", "joint_2"],
                    positions=(["joint_0", "joint_1", "joint_2"], positions_1d),
                )
        """
        if (positions is None) and (velocities is None) and (efforts is None):
            raise ValueError("One of positions, velocities, or efforts must be defined.")

        for vector_tuple in [positions, velocities, efforts]:
            if vector_tuple is None:
                continue

            names, vector = vector_tuple

            if not isinstance(vector, wp.array):
                raise ValueError("All defined [positions, velocities, efforts] must be 1D-warp arrays of float-types.")

            if vector.ndim == 2:
                if vector.shape[0] != 1:
                    raise ValueError(
                        "All defined [positions, velocities, efforts] must only be defined for a single robot."
                    )
                # flatten the array:
                vector = vector.reshape([-1])

            # Enforce that these are warp array inputs:
            if (vector.ndim != 1) or (  # can still occur if non-2D array passed in.
                vector.dtype not in (wp.float32, wp.float64, float)
            ):
                raise ValueError("All defined [positions, velocities, efforts] must be 1D-warp arrays of float-types.")

            if len(vector) < 1:
                raise ValueError("Any defined [positions, velocities, efforts] must have at least len of 1.")

            if not isinstance(names, list):
                raise ValueError(
                    f"Expected a list of joint names as the first element of the tuple, but got {type(names).__name__}."
                )

            if len(vector) != len(names):
                raise ValueError(
                    "Any defined [positions, velocities, efforts] must have the same length as their corresponding name list"
                )

            if len(set(names)) != len(names):
                raise ValueError("Joint names must all be unique.")

            if not (set(names).issubset(robot_joint_space)):
                raise ValueError("All joint names must be in the robot joint space.")

        # all inputs are valid, construct the joint-data and valid-arrays:
        # create full-sized warp array for the joint-data based on the joint-space
        # described by the robot_joint_space:
        n_joint_dims = len(robot_joint_space)
        joint_name_to_index = {name: idx for idx, name in enumerate(robot_joint_space)}

        # Storing on the CPU since we often have to do indexing operations.
        data = wp.zeros(shape=[3, n_joint_dims], dtype=wp.float32, device="cpu")
        valid = wp.zeros(shape=[3, n_joint_dims], dtype=wp.bool, device="cpu")

        for i, vector_tuple in enumerate([positions, velocities, efforts]):
            if vector_tuple is not None:
                joint_names, joint_data = vector_tuple
                joint_data = joint_data.numpy()
                joint_indices = [joint_name_to_index[name] for name in joint_names]
                data.numpy()[i, joint_indices] = joint_data
                valid.numpy()[i, joint_indices] = True

        return cls(robot_joint_space, data, valid)

    @classmethod
    def from_index(
        cls,
        robot_joint_space: list[str],
        positions: tuple[wp.array, wp.array] | None = None,
        velocities: tuple[wp.array, wp.array] | None = None,
        efforts: tuple[wp.array, wp.array] | None = None,
    ) -> "JointState":
        """Create a JointState from joint indices and data arrays.

        At least one of positions, velocities, or efforts must be provided. Each tuple
        contains a 1D warp array of joint indices and a corresponding warp array of values.

        **Array Shape Support:**
        The data arrays can be either:
        - 1D arrays (shape: ``(N,)``) where N is the number of joints
        - 2D arrays (shape: ``(1, N)``) where the first dimension must be size 1

        Args:
            robot_joint_space: The ordered list of joint names defining the joint space.
            positions: Tuple of (indices, position_array) where indices is a 1D warp array
                of joint indices and position_array is a 1D or 2D warp array (with first
                dimension size 1) of position values.
            velocities: Tuple of (indices, velocity_array) where indices is a 1D warp array
                of joint indices and velocity_array is a 1D or 2D warp array (with first
                dimension size 1) of velocity values.
            efforts: Tuple of (indices, effort_array) where indices is a 1D warp array
                of joint indices and effort_array is a 1D or 2D warp array (with first
                dimension size 1) of effort values.

        Returns:
            A JointState instance with the provided joint data.

        Raises:
            ValueError: If all of positions, velocities, and efforts are None.
            ValueError: If any provided data array is not a 1D or 2D warp array of float types.
            ValueError: If a 2D array is provided and the first dimension is not size 1.
            ValueError: If any provided indices array is not a 1D warp array of int types.
            ValueError: If any provided array has length less than 1.
            ValueError: If any index is out of range for the robot_joint_space.
            ValueError: If any index values are duplicated.
            ValueError: If any array length doesn't match its corresponding index array length.

        Example:
            .. code-block:: python

                # Direct usage with Articulation (no flatten/reshape needed):
                robot_joint_space = articulation.dof_names
                dof_indices = wp.array([0, 1, 2], dtype=wp.int32)
                joint_state = JointState.from_index(
                    robot_joint_space=robot_joint_space,
                    positions=(dof_indices, articulation.get_dof_positions()),
                    velocities=(dof_indices, articulation.get_dof_velocities()),
                )

                # Or with 1D arrays:
                indices = wp.array([0, 1, 2], dtype=wp.int32)
                positions_1d = wp.array([0.0, 1.0, 2.0], dtype=wp.float32)
                joint_state = JointState.from_index(
                    robot_joint_space=["joint_0", "joint_1", "joint_2"],
                    positions=(indices, positions_1d),
                )
        """
        if (positions is None) and (velocities is None) and (efforts is None):
            raise ValueError("One of positions, velocities, or efforts must be defined.")

        for vector_tuple in [positions, velocities, efforts]:
            if vector_tuple is None:
                continue

            indices, vector = vector_tuple

            if not isinstance(vector, wp.array):
                raise ValueError("All defined [positions, velocities, efforts] must be 1D-warp arrays of float-types.")

            if vector.ndim == 2:
                if vector.shape[0] != 1:
                    raise ValueError(
                        "All defined [positions, velocities, efforts] must only be defined for a single robot."
                    )
                # flatten the array:
                vector = vector.reshape([-1])

            # Enforce that these are warp array inputs:
            if (vector.ndim != 1) or (  # can still occur if non-2D array passed in.
                vector.dtype not in (wp.float32, wp.float64, float)
            ):
                raise ValueError("All defined [positions, velocities, efforts] must be 1D-warp arrays of float-types.")

            if len(vector) < 1:
                raise ValueError("Any defined [positions, velocities, efforts] must have at least len of 1.")

            if not isinstance(indices, wp.array) or (indices.ndim != 1) or (indices.dtype not in (wp.int32, int)):
                raise ValueError(
                    "All defined [positions, velocities, efforts] indices must be 1D-warp arrays of int types."
                )

            if (indices.numpy() < 0).any() or (indices.numpy() >= len(robot_joint_space)).any():
                raise ValueError(
                    "All indices must be greater than or equal to 0 and less than the length of the robot joint space."
                )

            if len(set(indices.numpy())) != len(indices):
                raise ValueError("Index values must all be unique.")

            if len(vector) != len(indices):
                raise ValueError(
                    "Any defined [positions, velocities, efforts] must have the same length as their corresponding index list."
                )

        # all inputs are valid, construct the joint-data and valid-arrays:
        # create full-sized warp array for the joint-data based on the joint-space
        # described by the robot_joint_space:
        n_joint_dims = len(robot_joint_space)

        # Storing on the CPU since we often have to do indexing operations.
        data = wp.zeros(shape=[3, n_joint_dims], dtype=wp.float32, device="cpu")
        valid = wp.zeros(shape=[3, n_joint_dims], dtype=wp.bool, device="cpu")

        for i, vector_tuple in enumerate([positions, velocities, efforts]):
            if vector_tuple is not None:
                joint_indices, joint_data = vector_tuple
                joint_data = joint_data.numpy()
                data.numpy()[i, joint_indices.numpy()] = joint_data
                valid.numpy()[i, joint_indices.numpy()] = True

        return cls(robot_joint_space, data, valid)

    @property
    def robot_joint_space(self) -> list[str]:
        """List of joints defining the joint-space of this JointState.

        Returns:
            The ordered list of joint names defining the joint space.
        """
        return self.__robot_joint_space

    @property
    def position_names(self) -> list[str]:
        """List of joint names that have valid positions.

        Returns:
            The list of joint names with valid position data.
        """
        return self.__position_names

    @property
    def position_indices(self) -> wp.array:
        """List of joint indices that have valid positions.

        Returns:
            A 1D warp array of joint indices with valid position data.
        """
        if self.__position_indices is _UNSET:
            self.__position_indices = self.__build_indices_array(self.__position_valid_idx)
        return self.__position_indices

    @property
    def positions(self) -> wp.array | None:
        """Valid positions as a warp array.

        Returns:
            A 1D warp array containing position values for joints with valid positions.
            Returns None if no positions are valid.
        """
        if self.__valid_positions is _UNSET:
            self.__valid_positions = self.__build_data_array(row=0, valid_idx=self.__position_valid_idx)
        return self.__valid_positions

    @property
    def velocity_names(self) -> list[str]:
        """List of joint names that have valid velocities.

        Returns:
            The list of joint names with valid velocity data.
        """
        return self.__velocity_names

    @property
    def velocity_indices(self) -> wp.array:
        """List of joint indices that have valid velocities.

        Returns:
            A 1D warp array of joint indices with valid velocity data.
        """
        if self.__velocity_indices is _UNSET:
            self.__velocity_indices = self.__build_indices_array(self.__velocity_valid_idx)
        return self.__velocity_indices

    @property
    def velocities(self) -> wp.array | None:
        """Valid velocities as a warp array.

        Returns:
            A 1D warp array containing velocity values for joints with valid velocities.
            Returns None if no velocities are valid.
        """
        if self.__valid_velocities is _UNSET:
            self.__valid_velocities = self.__build_data_array(row=1, valid_idx=self.__velocity_valid_idx)
        return self.__valid_velocities

    @property
    def effort_names(self) -> list[str]:
        """List of joint names that have valid efforts.

        Returns:
            The list of joint names with valid effort data.
        """
        return self.__effort_names

    @property
    def effort_indices(self) -> wp.array:
        """List of joint indices that have valid efforts.

        Returns:
            A 1D warp array containing the indices of joints with valid efforts.
        """
        if self.__effort_indices is _UNSET:
            self.__effort_indices = self.__build_indices_array(self.__effort_valid_idx)
        return self.__effort_indices

    @property
    def efforts(self) -> wp.array | None:
        """Valid efforts as a warp array.

        Returns:
            A 1D warp array containing effort values for joints with valid efforts.
            Returns None if no efforts are valid.
        """
        if self.__valid_efforts is _UNSET:
            self.__valid_efforts = self.__build_data_array(row=2, valid_idx=self.__effort_valid_idx)
        return self.__valid_efforts

    @property
    def data_array(self) -> wp.array:
        """Full data array.

        Returns:
            A 2D warp array with shape (3, N) where N is the length of robot_joint_space.
            Row 0 contains positions, row 1 contains velocities, row 2 contains efforts.
        """
        return self.__data_array

    @property
    def valid_array(self) -> wp.array:
        """Valid flags array.

        Returns:
            A 2D boolean warp array with shape (3, N) indicating which fields are valid
            for each joint. Row 0 corresponds to positions, row 1 to velocities, row 2 to efforts.
        """
        return self.__valid_array

    def __build_indices_array(self, valid_idx: np.ndarray) -> wp.array:
        """Wrap a numpy index array as a CPU wp.array without an H2D copy.

        ``wp.array(np_data, copy=False, device="cpu")`` references the numpy buffer
        directly via ``_init_from_ptr`` and skips the ``warp.copy`` call that
        ``wp.from_numpy`` would otherwise issue. The returned array is read-only in
        practice — callers that mutate it would alias ``valid_idx`` — but the
        existing API treats these arrays as immutable views.

        Args:
            valid_idx: Numpy array of valid joint indices.

        Returns:
            A 1D CPU Warp array view of ``valid_idx`` with int32 elements.
        """
        # ``_init_from_data`` will run ``np.asarray(valid_idx, dtype=np.int32)``
        # internally, performing the int64->int32 conversion if needed (without
        # invoking ``wp.copy``).
        return wp.array(valid_idx, dtype=wp.int32, copy=False, device="cpu")

    def __build_data_array(self, row: int, valid_idx: np.ndarray) -> wp.array | None:
        """Wrap a numpy slice of the data array as a CPU wp.array without an H2D copy.

        Args:
            row: Row in ``self.__data_array`` to gather from.
            valid_idx: Numpy array of valid joint indices to gather.

        Returns:
            A 1D CPU Warp array containing the selected data values, or None if
            ``valid_idx`` is empty.
        """
        if len(valid_idx) == 0:
            return None
        # ``self.__data_array`` is built CPU-resident in ``from_name`` / ``from_index``,
        # so ``.numpy()`` is a zero-copy view of the underlying buffer. Fancy-indexing
        # produces a new contiguous numpy array; wrapping it with ``copy=False``
        # avoids a redundant ``wp.copy`` for the second hop.
        slice_np = self.__data_array.numpy()[row, valid_idx]
        return wp.array(slice_np, dtype=wp.float32, copy=False, device="cpu")

    def _update_data_in_place(self, positions_np: "np.ndarray | None", velocities_np: "np.ndarray | None") -> None:
        """Update position and velocity data in-place, invalidating cached property values.

        Intended for hot-path use with pre-allocated JointState objects built via the
        direct constructor. Avoids the full allocation cost of ``from_name()`` — no new
        ``wp.zeros`` arrays, no validation, no index recomputation. Cached ``positions``
        / ``velocities`` wp.arrays are invalidated and rebuilt lazily on next access
        (and skipped entirely when never read).

        Args:
            positions_np: New position values as a numpy array whose length equals
                ``len(position_names)``, written into the valid position slots.
                Pass ``None`` to leave positions unchanged.
            velocities_np: New velocity values as a numpy array whose length equals
                ``len(velocity_names)``.  Pass ``None`` to leave velocities unchanged.
        """
        data_np = self.__data_array.numpy()
        if positions_np is not None:
            data_np[0, self.__position_valid_idx] = positions_np
            self.__valid_positions = _UNSET
        if velocities_np is not None:
            data_np[1, self.__velocity_valid_idx] = velocities_np
            self.__valid_velocities = _UNSET


class SpatialState:
    """Container for the spatial state (pose and twist) of frames.

    A spatial state represents the pose (position and orientation) and twist (linear and angular
    velocities) of a set of frames. This can be used for robot links, sites (tool frames), or any
    other frames of interest.

    At least one of positions, orientations, linear_velocities, or angular_velocities must be
    provided. All provided arrays must be 2D warp arrays with shape (N, K) where N equals the
    length of names and K is the appropriate dimension for the field.

    User should almost never call this constructor directly. Instead, use the from_name or from_index methods.

    Args:
        spatial_space: The ordered list of frame names (e.g., link names or site names).
        position_data: Pre-allocated position data array with shape (N, 3).
        linear_velocity_data: Pre-allocated linear velocity data array with shape (N, 3).
        orientation_data: Pre-allocated orientation data array with shape (N, 4).
        angular_velocity_data: Pre-allocated angular velocity data array with shape (N, 3).
        valid_array: Boolean array with shape (N, 4) indicating which fields are valid for each frame.

    Raises:
        ValueError: If all of positions, orientations, linear_velocities, and angular_velocities
            are None.
        ValueError: If any provided array is not a 2D warp array.
        ValueError: If any provided array has shape[0] not equal to the length of spatial_space.
        ValueError: If positions, linear_velocities, or angular_velocities has shape[1] != 3.
        ValueError: If orientations has shape[1] != 4.
    """

    def __init__(
        self,
        spatial_space: list[str],
        position_data: wp.array,
        linear_velocity_data: wp.array,
        orientation_data: wp.array,
        angular_velocity_data: wp.array,
        valid_array: wp.array,
    ) -> None:

        if not len(set(spatial_space)) == len(spatial_space):
            raise ValueError("spatial_space is not valid: cannot have duplicated reference frame names.")

        if (
            not isinstance(position_data, wp.array)
            or not (position_data.dtype is wp.float32)
            or (position_data.ndim != 2)
        ):
            raise ValueError(f"position_data must be a 2D wp.array with data-type wp.float32")

        if (
            not isinstance(linear_velocity_data, wp.array)
            or not (linear_velocity_data.dtype is wp.float32)
            or (linear_velocity_data.ndim != 2)
        ):
            raise ValueError(f"linear_velocity_data must be a 2D wp.array with data-type wp.float32")

        if (
            not isinstance(orientation_data, wp.array)
            or not (orientation_data.dtype is wp.float32)
            or (orientation_data.ndim != 2)
        ):
            raise ValueError(f"orientation_data must be a 2D wp.array with data-type wp.float32")

        if (
            not isinstance(angular_velocity_data, wp.array)
            or not (angular_velocity_data.dtype is wp.float32)
            or (angular_velocity_data.ndim != 2)
        ):
            raise ValueError(f"angular_velocity_data must be a 2D wp.array with data-type wp.float32")

        if not isinstance(valid_array, wp.array) or not (valid_array.dtype is wp.bool) or (valid_array.ndim != 2):
            raise ValueError(f"valid_array must be a 2D wp.array with data-type wp.bool")

        if not (
            position_data.shape == linear_velocity_data.shape == angular_velocity_data.shape == (len(spatial_space), 3)
        ):
            raise ValueError(
                f"One of position_data, linear_velocity_data, or angular_velocity_data is not correctly sized for this spatial-space."
            )

        if not (valid_array.shape == (len(spatial_space), 4)):
            raise ValueError(f"valid_array is not correctly sized for this spatial-space.")

        if not (orientation_data.shape == (len(spatial_space), 4)):
            raise ValueError(f"orientation_data is not correctly sized for this spatial-space.")

        self.__spatial_space = spatial_space
        self.__position_data = position_data
        self.__linear_velocity_data = linear_velocity_data
        self.__orientation_data = orientation_data
        self.__angular_velocity_data = angular_velocity_data
        self.__valid_array = valid_array

        # Pre-compute which frames are valid per dimension once. ``valid_array`` is
        # built CPU-resident inside ``from_name`` / ``from_index`` so ``.numpy()`` is
        # a zero-copy view; fancy column reads + ``np.where`` then run purely on the
        # host without any wp.copy traffic.
        valid_np = valid_array.numpy()
        self.__position_valid_idx = np.where(valid_np[:, 0].flatten())[0]
        self.__orientation_valid_idx = np.where(valid_np[:, 1].flatten())[0]
        self.__linear_velocity_valid_idx = np.where(valid_np[:, 2].flatten())[0]
        self.__angular_velocity_valid_idx = np.where(valid_np[:, 3].flatten())[0]

        # Names are cheap (list comprehensions) so we materialise them eagerly.
        self.__position_names = [self.__spatial_space[i] for i in self.__position_valid_idx]
        self.__orientation_names = [self.__spatial_space[i] for i in self.__orientation_valid_idx]
        self.__linear_velocity_names = [self.__spatial_space[i] for i in self.__linear_velocity_valid_idx]
        self.__angular_velocity_names = [self.__spatial_space[i] for i in self.__angular_velocity_valid_idx]

        # Per-dimension wp.arrays (data + indices) are built lazily on first property
        # access. The previous eager construction issued an H2D ``wp.from_numpy``
        # per dimension (up to eight wp.copy calls per SpatialState) — wasteful for
        # benchmark workloads where ``*_indices`` are typically never read and
        # ``positions`` / ``orientations`` are immediately re-downloaded via
        # ``.numpy()`` by the consumer.
        self.__valid_positions: _LazyOptionalWpArray = _UNSET
        self.__valid_orientations: _LazyOptionalWpArray = _UNSET
        self.__valid_linear_velocities: _LazyOptionalWpArray = _UNSET
        self.__valid_angular_velocities: _LazyOptionalWpArray = _UNSET
        self.__position_indices: _LazyWpIndices = _UNSET
        self.__orientation_indices: _LazyWpIndices = _UNSET
        self.__linear_velocity_indices: _LazyWpIndices = _UNSET
        self.__angular_velocity_indices: _LazyWpIndices = _UNSET

    @classmethod
    def from_name(
        cls,
        spatial_space: list[str],
        positions: tuple[list[str], wp.array] | None = None,
        orientations: tuple[list[str], wp.array] | None = None,
        linear_velocities: tuple[list[str], wp.array] | None = None,
        angular_velocities: tuple[list[str], wp.array] | None = None,
    ) -> "SpatialState":
        """Create a SpatialState from frame names and data arrays.

        At least one of positions, orientations, linear_velocities, or angular_velocities
        must be provided. Each tuple contains a list of frame names and a corresponding
        2D warp array of values.

        Args:
            spatial_space: The ordered list of frame names (e.g., link names or site names).
            positions: Tuple of (frame_names, position_array) where frame_names is a list
                of frame names and position_array is a 2D warp array with shape (N, 3).
            orientations: Tuple of (frame_names, orientation_array) where frame_names is a list
                of frame names and orientation_array is a 2D warp array with shape (N, 4).
            linear_velocities: Tuple of (frame_names, velocity_array) where frame_names is a list
                of frame names and velocity_array is a 2D warp array with shape (N, 3).
            angular_velocities: Tuple of (frame_names, angular_velocity_array) where frame_names
                is a list of frame names and angular_velocity_array is a 2D warp array with shape (N, 3).

        Returns:
            A SpatialState instance with the provided frame data.

        Raises:
            ValueError: If all of positions, orientations, linear_velocities, and angular_velocities
                are None.
            ValueError: If any provided array is not a 2D warp array of float types.
            ValueError: If any provided array has shape[0] less than 1.
            ValueError: If the first element of any tuple is not a list of frame names.
            ValueError: If any array shape[0] doesn't match its corresponding name list length.
            ValueError: If any frame names are duplicated.
            ValueError: If any frame names are not in the spatial_space.
            ValueError: If positions, linear_velocities, or angular_velocities has shape[1] != 3.
            ValueError: If orientations has shape[1] != 4.
        """
        if (
            (positions is None)
            and (orientations is None)
            and (linear_velocities is None)
            and (angular_velocities is None)
        ):
            raise ValueError(
                "One of positions, orientations, linear_velocities, or angular_velocities must be defined."
            )

        for vector_tuple in [positions, orientations, linear_velocities, angular_velocities]:
            if vector_tuple is None:
                continue

            names, vector = vector_tuple

            # Enforce that these are warp array inputs:
            if (
                not isinstance(vector, wp.array)
                or (vector.ndim != 2)
                or (vector.dtype not in (wp.float32, wp.float64, float))
            ):
                raise ValueError(
                    "Any defined [positions, orientations, linear or angular velocity] must be a 2D warp array of float types."
                )

            if vector.shape[0] < 1:
                raise ValueError(
                    "Any defined [positions, orientations, linear or angular velocity] must have shape[0] >= 1"
                )

            if not isinstance(names, list):
                raise ValueError(
                    f"Expected a list of frame names as the first element of the tuple, but got {type(names).__name__}."
                )

            if len(set(names)) != len(names):
                raise ValueError("Reference frame names must all be unique.")

            if vector.shape[0] != len(names):
                raise ValueError(
                    "Any defined [positions, orientations, linear or angular velocity] must have the same length as their corresponding name list"
                )

            if not (set(names).issubset(spatial_space)):
                raise ValueError("All frame names must be in the spatial space.")

        if (positions is not None) and (positions[1].shape[1] != 3):
            raise ValueError("position data array must have shape[1] == 3.")
        if (linear_velocities is not None) and (linear_velocities[1].shape[1] != 3):
            raise ValueError("linear velocity data array must have shape[1] == 3.")
        if (orientations is not None) and (orientations[1].shape[1] != 4):
            raise ValueError("orientation data array must have shape[1] == 4.")
        if (angular_velocities is not None) and (angular_velocities[1].shape[1] != 3):
            raise ValueError("angular velocity data array must have shape[1] == 3.")

        # create warp arrays for the positions, linear velocities, orientations and angular velocities.
        # fill in whether a given value is set:
        n_frames = len(spatial_space)
        frame_name_to_index = {name: idx for idx, name in enumerate(spatial_space)}
        position_data = wp.zeros(shape=[n_frames, 3], dtype=wp.float32, device="cpu")
        linear_velocity_data = wp.zeros(shape=[n_frames, 3], dtype=wp.float32, device="cpu")
        orientation_data = wp.zeros(shape=[n_frames, 4], dtype=wp.float32, device="cpu")
        angular_velocity_data = wp.zeros(shape=[n_frames, 3], dtype=wp.float32, device="cpu")
        valid = wp.zeros(shape=[n_frames, 4], dtype=wp.bool, device="cpu")

        # Fill in position data (column 0 of valid)
        if positions is not None:
            names, pos_array = positions
            pos_array = pos_array.numpy()
            indices = [frame_name_to_index[name] for name in names]
            position_data.numpy()[indices, :] = pos_array
            valid.numpy()[indices, 0] = True

        # Fill in orientation data (column 1 of valid)
        if orientations is not None:
            names, orient_array = orientations
            orient_array = orient_array.numpy()
            indices = [frame_name_to_index[name] for name in names]
            orientation_data.numpy()[indices, :] = orient_array
            valid.numpy()[indices, 1] = True

        # Fill in linear velocity data (column 2 of valid)
        if linear_velocities is not None:
            names, vel_array = linear_velocities
            vel_array = vel_array.numpy()
            indices = [frame_name_to_index[name] for name in names]
            linear_velocity_data.numpy()[indices, :] = vel_array
            valid.numpy()[indices, 2] = True

        # Fill in angular velocity data (column 3 of valid)
        if angular_velocities is not None:
            names, ang_vel_array = angular_velocities
            ang_vel_array = ang_vel_array.numpy()
            indices = [frame_name_to_index[name] for name in names]
            angular_velocity_data.numpy()[indices, :] = ang_vel_array
            valid.numpy()[indices, 3] = True

        return cls(spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid)

    @classmethod
    def from_index(
        cls,
        spatial_space: list[str],
        positions: tuple[wp.array, wp.array] | None = None,
        orientations: tuple[wp.array, wp.array] | None = None,
        linear_velocities: tuple[wp.array, wp.array] | None = None,
        angular_velocities: tuple[wp.array, wp.array] | None = None,
    ) -> "SpatialState":
        """Create a SpatialState from frame indices and data arrays.

        At least one of positions, orientations, linear_velocities, or angular_velocities
        must be provided. Each tuple contains a 1D warp array of frame indices and a
        corresponding 2D warp array of values.

        Args:
            spatial_space: The ordered list of frame names (e.g., link names or site names).
            positions: Tuple of (indices, position_array) where indices is a 1D warp array
                of frame indices and position_array is a 2D warp array with shape (N, 3).
            orientations: Tuple of (indices, orientation_array) where indices is a 1D warp array
                of frame indices and orientation_array is a 2D warp array with shape (N, 4).
            linear_velocities: Tuple of (indices, velocity_array) where indices is a 1D warp array
                of frame indices and velocity_array is a 2D warp array with shape (N, 3).
            angular_velocities: Tuple of (indices, angular_velocity_array) where indices is a 1D
                warp array of frame indices and angular_velocity_array is a 2D warp array with shape (N, 3).

        Returns:
            A SpatialState instance with the provided frame data.

        Raises:
            ValueError: If all of positions, orientations, linear_velocities, and angular_velocities
                are None.
            ValueError: If any provided data array is not a 2D warp array of float types.
            ValueError: If any provided indices array is not a 1D warp array of int types.
            ValueError: If any provided array has shape[0] less than 1.
            ValueError: If any index is out of range for the spatial_space.
            ValueError: If any index values are duplicated.
            ValueError: If any array shape[0] doesn't match its corresponding index array length.
            ValueError: If positions, linear_velocities, or angular_velocities has shape[1] != 3.
            ValueError: If orientations has shape[1] != 4.
        """
        if (
            (positions is None)
            and (orientations is None)
            and (linear_velocities is None)
            and (angular_velocities is None)
        ):
            raise ValueError(
                "One of positions, orientations, linear_velocities, or angular_velocities must be defined."
            )

        for vector_tuple in [positions, orientations, linear_velocities, angular_velocities]:
            if vector_tuple is None:
                continue

            indices, vector = vector_tuple

            # Enforce that these are warp array inputs:
            if (
                not isinstance(vector, wp.array)
                or (vector.ndim != 2)
                or (vector.dtype not in (wp.float32, wp.float64, float))
            ):
                raise ValueError(
                    "Any defined [positions, orientations, linear or angular velocity] must be a 2D warp array of float types."
                )

            if vector.shape[0] < 1:
                raise ValueError(
                    "Any defined [positions, orientations, linear or angular velocity] must have shape[0] >= 1"
                )

            if not isinstance(indices, wp.array) or (indices.ndim != 1) or (indices.dtype not in (wp.int32, int)):
                raise ValueError(
                    "All defined [positions, orientations, linear or angular velocity] indices must be 1D-warp arrays of int types."
                )

            if (indices.numpy() < 0).any() or (indices.numpy() >= len(spatial_space)).any():
                raise ValueError(
                    "All indices must be greater than or equal to 0 and less than the length of the spatial space."
                )

            if len(set(indices.numpy())) != len(indices):
                raise ValueError("Index values must all be unique.")

            if vector.shape[0] != len(indices):
                raise ValueError(
                    "Any defined [positions, orientations, linear or angular velocity] must have the same length as their corresponding name list"
                )

        if (positions is not None) and (positions[1].shape[1] != 3):
            raise ValueError("position data array must have shape[1] == 3.")
        if (linear_velocities is not None) and (linear_velocities[1].shape[1] != 3):
            raise ValueError("linear velocity data array must have shape[1] == 3.")
        if (orientations is not None) and (orientations[1].shape[1] != 4):
            raise ValueError("orientation data array must have shape[1] == 4.")
        if (angular_velocities is not None) and (angular_velocities[1].shape[1] != 3):
            raise ValueError("angular velocity data array must have shape[1] == 3.")

        # create warp arrays for the positions, linear velocities, orientations and angular velocities.
        # fill in whether a given value is set:
        n_frames = len(spatial_space)
        frame_name_to_index = {name: idx for idx, name in enumerate(spatial_space)}
        position_data = wp.zeros(shape=[n_frames, 3], dtype=wp.float32, device="cpu")
        linear_velocity_data = wp.zeros(shape=[n_frames, 3], dtype=wp.float32, device="cpu")
        orientation_data = wp.zeros(shape=[n_frames, 4], dtype=wp.float32, device="cpu")
        angular_velocity_data = wp.zeros(shape=[n_frames, 3], dtype=wp.float32, device="cpu")
        valid = wp.zeros(shape=[n_frames, 4], dtype=wp.bool, device="cpu")

        # Fill in position data (column 0 of valid)
        if positions is not None:
            indices, pos_array = positions
            pos_array = pos_array.numpy()
            indices = indices.numpy()
            position_data.numpy()[indices, :] = pos_array
            valid.numpy()[indices, 0] = True

        # Fill in orientation data (column 1 of valid)
        if orientations is not None:
            indices, orient_array = orientations
            orient_array = orient_array.numpy()
            indices = indices.numpy()
            orientation_data.numpy()[indices, :] = orient_array
            valid.numpy()[indices, 1] = True

        # Fill in linear velocity data (column 2 of valid)
        if linear_velocities is not None:
            indices, vel_array = linear_velocities
            vel_array = vel_array.numpy()
            indices = indices.numpy()
            linear_velocity_data.numpy()[indices, :] = vel_array
            valid.numpy()[indices, 2] = True

        # Fill in angular velocity data (column 3 of valid)
        if angular_velocities is not None:
            indices, ang_vel_array = angular_velocities
            ang_vel_array = ang_vel_array.numpy()
            indices = indices.numpy()
            angular_velocity_data.numpy()[indices, :] = ang_vel_array
            valid.numpy()[indices, 3] = True

        return cls(spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid)

    @property
    def spatial_space(self) -> list[str]:
        """List of frame names.

        Returns:
            The ordered list of frame names defining the spatial space.
        """
        return self.__spatial_space

    @property
    def position_names(self) -> list[str]:
        """List of frame names that have valid positions.

        Returns:
            The list of frame names that have valid positions.
        """
        return self.__position_names

    @property
    def position_indices(self) -> wp.array:
        """List of frame indices that have valid positions.

        Returns:
            A 1D warp array of frame indices that have valid positions.
        """
        if self.__position_indices is _UNSET:
            self.__position_indices = self.__build_indices_array(self.__position_valid_idx)
        return self.__position_indices

    @property
    def positions(self) -> wp.array | None:
        """Valid positions as a warp array.

        Returns:
            A 2D warp array with shape (N, 3) containing position values for frames with valid
            positions. Returns None if no positions are valid.
        """
        if self.__valid_positions is _UNSET:
            self.__valid_positions = self.__build_data_array(self.__position_data, self.__position_valid_idx)
        return self.__valid_positions

    @property
    def orientation_names(self) -> list[str]:
        """List of frame names that have valid orientations.

        Returns:
            The list of frame names that have valid orientations.
        """
        return self.__orientation_names

    @property
    def orientation_indices(self) -> wp.array:
        """List of frame indices that have valid orientations.

        Returns:
            A 1D warp array of frame indices that have valid orientations.
        """
        if self.__orientation_indices is _UNSET:
            self.__orientation_indices = self.__build_indices_array(self.__orientation_valid_idx)
        return self.__orientation_indices

    @property
    def orientations(self) -> wp.array | None:
        """Valid orientations as a warp array.

        Returns:
            A 2D warp array with shape (N, 4) containing orientation values (quaternions) for
            frames with valid orientations. Returns None if no orientations are valid.
        """
        if self.__valid_orientations is _UNSET:
            self.__valid_orientations = self.__build_data_array(self.__orientation_data, self.__orientation_valid_idx)
        return self.__valid_orientations

    @property
    def linear_velocity_names(self) -> list[str]:
        """List of frame names that have valid linear velocities.

        Returns:
            The list of frame names that have valid linear velocities.
        """
        return self.__linear_velocity_names

    @property
    def linear_velocity_indices(self) -> wp.array:
        """List of frame indices that have valid linear velocities.

        Returns:
            A 1D Warp array containing the indices of frames with valid linear velocities.
        """
        if self.__linear_velocity_indices is _UNSET:
            self.__linear_velocity_indices = self.__build_indices_array(self.__linear_velocity_valid_idx)
        return self.__linear_velocity_indices

    @property
    def linear_velocities(self) -> wp.array | None:
        """Valid linear velocities as a Warp array.

        Returns:
            A 2D Warp array with shape (N, 3) containing linear velocity values for frames
            with valid linear velocities. Returns None if no linear velocities are valid.
        """
        if self.__valid_linear_velocities is _UNSET:
            self.__valid_linear_velocities = self.__build_data_array(
                self.__linear_velocity_data, self.__linear_velocity_valid_idx
            )
        return self.__valid_linear_velocities

    @property
    def angular_velocity_names(self) -> list[str]:
        """List of frame names that have valid angular velocities.

        Returns:
            A list of frame names that have valid angular velocities.
        """
        return self.__angular_velocity_names

    @property
    def angular_velocity_indices(self) -> wp.array:
        """List of frame indices that have valid angular velocities.

        Returns:
            A 1D Warp array containing the indices of frames with valid angular velocities.
        """
        if self.__angular_velocity_indices is _UNSET:
            self.__angular_velocity_indices = self.__build_indices_array(self.__angular_velocity_valid_idx)
        return self.__angular_velocity_indices

    @property
    def angular_velocities(self) -> wp.array | None:
        """Valid angular velocities as a Warp array.

        Returns:
            A 2D Warp array with shape (N, 3) containing angular velocity values for frames
            with valid angular velocities. Returns None if no angular velocities are valid.
        """
        if self.__valid_angular_velocities is _UNSET:
            self.__valid_angular_velocities = self.__build_data_array(
                self.__angular_velocity_data, self.__angular_velocity_valid_idx
            )
        return self.__valid_angular_velocities

    @property
    def position_data(self) -> wp.array:
        """Full position data array.

        Returns:
            A 2D Warp array with shape (N, 3) containing position data for all frames in
            the spatial space, regardless of validity flags.
        """
        return self.__position_data

    @property
    def orientation_data(self) -> wp.array:
        """Full orientation data array.

        Returns:
            A 2D Warp array with shape (N, 4) containing orientation data (quaternions) for
            all frames in the spatial space, regardless of validity flags.
        """
        return self.__orientation_data

    @property
    def linear_velocity_data(self) -> wp.array:
        """Full linear velocity data array.

        Returns:
            A 2D Warp array with shape (N, 3) containing linear velocity data for all frames
            in the spatial space, regardless of validity flags.
        """
        return self.__linear_velocity_data

    @property
    def angular_velocity_data(self) -> wp.array:
        """Full angular velocity data array.

        Returns:
            A 2D Warp array with shape (N, 3) containing angular velocity data for all frames
            in the spatial space, regardless of validity flags.
        """
        return self.__angular_velocity_data

    @property
    def valid_array(self) -> wp.array:
        """Valid flags array.

        Returns:
            A 2D boolean Warp array with shape (N, 4) indicating which fields are valid
            for each frame. Column 0 corresponds to positions, column 1 to orientations,
            column 2 to linear velocities, column 3 to angular velocities.
        """
        return self.__valid_array

    def __build_indices_array(self, valid_idx: np.ndarray) -> wp.array:
        """Wrap a numpy index array as a CPU wp.array without an H2D copy.

        ``wp.array(np_data, copy=False, device="cpu")`` references the numpy buffer
        directly via ``_init_from_ptr`` and skips the ``warp.copy`` call that
        ``wp.from_numpy`` would otherwise issue. The returned array is treated as
        immutable by the existing API.

        Args:
            valid_idx: Numpy array of valid frame indices.

        Returns:
            A 1D CPU Warp array view of ``valid_idx`` with int32 elements.
        """
        return wp.array(valid_idx, dtype=wp.int32, copy=False, device="cpu")

    def __build_data_array(self, data_array: wp.array, valid_idx: np.ndarray) -> wp.array | None:
        """Wrap a numpy slice of the per-dimension data array as a CPU wp.array without an H2D copy.

        ``data_array`` is built CPU-resident in ``from_name`` / ``from_index``, so
        ``.numpy()`` is a zero-copy view. Fancy-indexing produces a new contiguous
        numpy array; wrapping it with ``copy=False`` avoids the redundant
        ``wp.copy`` for the second hop.

        Args:
            data_array: Per-dimension data array to gather from.
            valid_idx: Numpy array of valid frame indices to gather.

        Returns:
            A 2D CPU Warp array containing the selected rows of ``data_array``,
            or None if ``valid_idx`` is empty.
        """
        if len(valid_idx) == 0:
            return None
        slice_np = data_array.numpy()[valid_idx, :]
        return wp.array(slice_np, dtype=wp.float32, copy=False, device="cpu")


class RootState:
    """Container for the state of the robot's root link.

    At least one of position, orientation, linear_velocity, or angular_velocity must be provided.
    All provided arrays must be 1D warp arrays with the correct number of elements.

    All properties are read-only after construction.

    Args:
        position: The position of the root as a 1D warp array with 3 elements (x, y, z).
        orientation: The orientation of the root as a 1D warp array with 4 elements (w, x, y, z).
        linear_velocity: The linear velocity of the root as a 1D warp array with 3 elements (x, y, z).
        angular_velocity: The angular velocity of the root as a 1D warp array with 3 elements (x, y, z).

    Raises:
        ValueError: If all of position, orientation, linear_velocity, and angular_velocity are None.
        ValueError: If any provided array is not a 1D warp array with the correct number of elements.
    """

    def __init__(
        self,
        position: wp.array | None = None,
        orientation: wp.array | None = None,
        linear_velocity: wp.array | None = None,
        angular_velocity: wp.array | None = None,
    ) -> None:
        if (position is None) and (orientation is None) and (linear_velocity is None) and (angular_velocity is None):
            raise ValueError("One of position, orientation, linear_velocity, or angular_velocity must be defined.")

        if position is not None:
            if (
                not isinstance(position, wp.array)
                or (position.ndim != 1)
                or (len(position) != 3)
                or (position.dtype not in (wp.float32, wp.float64, float))
            ):
                raise ValueError("position must be a 1D warp array with 3 elements (x, y, z) and a 32-bit float type.")

        if orientation is not None:
            if (
                not isinstance(orientation, wp.array)
                or (orientation.ndim != 1)
                or (len(orientation) != 4)
                or (orientation.dtype not in (wp.float32, wp.float64, float))
            ):
                raise ValueError(
                    "orientation must be a 1D warp array with 4 elements (w, x, y, z) and a 32-bit float type."
                )

        if linear_velocity is not None:
            if (
                not isinstance(linear_velocity, wp.array)
                or (linear_velocity.ndim != 1)
                or (len(linear_velocity) != 3)
                or (linear_velocity.dtype not in (wp.float32, wp.float64, float))
            ):
                raise ValueError(
                    "linear_velocity must be a 1D warp array with 3 elements (x, y, z) and a 32-bit float type."
                )

        if angular_velocity is not None:
            if (
                not isinstance(angular_velocity, wp.array)
                or (angular_velocity.ndim != 1)
                or (len(angular_velocity) != 3)
                or (angular_velocity.dtype not in (wp.float32, wp.float64, float))
            ):
                raise ValueError(
                    "angular_velocity must be a 1D warp array with 3 elements (x, y, z) and a 32-bit float type."
                )

        # Note, we don't need to do indexing operations on these warp arrays, so we allow them to
        # be on any device.
        self.__position = (
            wp.from_numpy(position.numpy(), dtype=wp.float32, device=position.device) if position is not None else None
        )
        self.__orientation = (
            wp.from_numpy(orientation.numpy(), dtype=wp.float32, device=orientation.device)
            if orientation is not None
            else None
        )
        self.__linear_velocity = (
            wp.from_numpy(linear_velocity.numpy(), dtype=wp.float32, device=linear_velocity.device)
            if linear_velocity is not None
            else None
        )
        self.__angular_velocity = (
            wp.from_numpy(angular_velocity.numpy(), dtype=wp.float32, device=angular_velocity.device)
            if angular_velocity is not None
            else None
        )

    @property
    def position(self) -> wp.array | None:
        """Root position.

        Returns:
            A 1D warp array with 3 elements (x, y, z) representing the root position, or None
            if not set.
        """
        return self.__position

    @property
    def orientation(self) -> wp.array | None:
        """Root orientation.

        Returns:
            A 1D warp array with 4 elements (w, x, y, z quaternion) representing the root
            orientation, or None if not set.
        """
        return self.__orientation

    @property
    def linear_velocity(self) -> wp.array | None:
        """Root linear velocity.

        Returns:
            A 1D warp array with 3 elements (x, y, z) representing the root linear velocity,
            or None if not set.
        """
        return self.__linear_velocity

    @property
    def angular_velocity(self) -> wp.array | None:
        """Root angular velocity.

        Returns:
            A 1D warp array with 3 elements (x, y, z) representing the root angular velocity,
            or None if not set.
        """
        return self.__angular_velocity


class RobotState:
    """Composite container for the complete state of a robot.

    A RobotState aggregates the state of all robot components: joints, root link, rigid bodies,
    and sites. All components are optional, allowing partial state representations.

    Args:
        joints: The state of the robot's joints.
        root: The state of the robot's root link.
        links: The state of the robot's non-root rigid bodies.
        sites: The state of non-link reference frames (tools, sensors, etc.).

    Raises:
        ValueError: If joints is not None and not of type JointState.
        ValueError: If root is not None and not of type RootState.
        ValueError: If links is not None and not of type SpatialState.
        ValueError: If sites is not None and not of type SpatialState.
    """

    def __init__(
        self,
        joints: JointState | None = None,
        root: RootState | None = None,
        links: SpatialState | None = None,
        sites: SpatialState | None = None,
    ) -> None:
        if (joints is not None) and not (isinstance(joints, JointState)):
            raise ValueError("joints must be of type JointState")

        if (root is not None) and not (isinstance(root, RootState)):
            raise ValueError("root must be of type RootState")

        if (links is not None) and not (isinstance(links, SpatialState)):
            raise ValueError("links must be of type SpatialState")

        if (sites is not None) and not (isinstance(sites, SpatialState)):
            raise ValueError("sites must be of type SpatialState")

        self.__joints = joints
        self.__root = root
        self.__links = links
        self.__sites = sites

    @property
    def joints(self) -> JointState | None:
        """Joint state containing joint positions, velocities, and efforts.

        Returns:
            The JointState instance containing joint positions, velocities, and efforts,
            or None if not set.
        """
        return self.__joints

    @property
    def root(self) -> RootState | None:
        """Root state containing root position, orientation, and velocities.

        Returns:
            The RootState instance containing root position, orientation, and velocities,
            or None if not set.
        """
        return self.__root

    @property
    def links(self) -> SpatialState | None:
        """Link state containing link positions, orientations, and velocities.

        Returns:
            The SpatialState instance containing link positions, orientations, and velocities,
            or None if not set.
        """
        return self.__links

    @property
    def sites(self) -> SpatialState | None:
        """Site state containing site positions, orientations, and velocities.

        Returns:
            The SpatialState instance containing site positions, orientations, and velocities,
            or None if not set.
        """
        return self.__sites


def _combine_joint_states(
    joint_state_1: JointState | None, joint_state_2: JointState | None
) -> JointState | Literal[False] | None:
    """Combine two joint states into one.

    Two joint states can be combined only if:
    - They have exactly the same robot_joint_space (ensuring they are for the same robot
      configuration).
    - They have non-overlapping valid flags (i.e., no joint+field combination is set in both
      states).

    Args:
        joint_state_1: The first joint state, or None.
        joint_state_2: The second joint state, or None.

    Returns:
        The combined JointState if successful, None if both inputs are None, or False if the
        states cannot be combined due to mismatched names or overlapping valid flags.
    """
    if joint_state_1 is None:
        return joint_state_2

    if joint_state_2 is None:
        return joint_state_1

    # States must have exactly the same robot_joint_space to be combined
    # This ensures they are for the same robot configuration
    if joint_state_1.robot_joint_space != joint_state_2.robot_joint_space:
        return False

    # Check for overlap in valid flags - if any joint+field combination is set in both states, cannot combine
    joint_states_overlap = np.any(joint_state_1.valid_array.numpy() & joint_state_2.valid_array.numpy())
    if joint_states_overlap:
        return False

    # There is no overlap, we can combine these values:
    combined_data_array = np.where(
        joint_state_1.valid_array.numpy(), joint_state_1.data_array.numpy(), joint_state_2.data_array.numpy()
    )
    combined_valid_array = np.where(
        joint_state_1.valid_array.numpy(),
        joint_state_1.valid_array.numpy(),
        joint_state_2.valid_array.numpy(),
    )
    return JointState(
        robot_joint_space=joint_state_1.robot_joint_space,
        data_array=wp.from_numpy(combined_data_array, dtype=wp.float32, device="cpu"),
        valid_array=wp.from_numpy(combined_valid_array, dtype=wp.bool, device="cpu"),
    )


def _combine_spatial_states(
    spatial_state_1: SpatialState | None, spatial_state_2: SpatialState | None
) -> SpatialState | Literal[False] | None:
    """Combine two spatial states into one.

    Two spatial states can be combined only if:
    - They have exactly the same spatial_space (ensuring they are for the same robot
      configuration).
    - They have non-overlapping valid flags (i.e., no frame+field combination is set in both
      states).

    Args:
        spatial_state_1: The first spatial state, or None.
        spatial_state_2: The second spatial state, or None.

    Returns:
        The combined SpatialState if successful, None if both inputs are None, or False if the
        states cannot be combined due to mismatched names or overlapping valid flags.
    """
    if spatial_state_1 is None:
        return spatial_state_2

    if spatial_state_2 is None:
        return spatial_state_1

    # States must have exactly the same spatial_space to be combined
    # This ensures they are for the same spatial configuration
    if spatial_state_1.spatial_space != spatial_state_2.spatial_space:
        return False

    # Check if there's any overlap in valid flags (same frame, same field)
    spatial_states_overlap = np.any(spatial_state_1.valid_array.numpy() & spatial_state_2.valid_array.numpy())
    if spatial_states_overlap:
        return False

    # There is no overlap, we can combine these values:
    # For each data array, use np.where to combine based on valid flags
    # Fill in position data (column 0 of valid)
    position_data = np.where(
        spatial_state_1.valid_array.numpy()[:, 0],  # Broadcast to match shape [n_frames, 3]
        spatial_state_1.position_data.numpy(),
        spatial_state_2.position_data.numpy(),
    )
    # Fill in orientation data (column 1 of valid)
    orientation_data = np.where(
        spatial_state_1.valid_array.numpy()[:, 1],  # Broadcast to match shape [n_frames, 4]
        spatial_state_1.orientation_data.numpy(),
        spatial_state_2.orientation_data.numpy(),
    )

    # Fill in linear velocity data (column 2 of valid)
    linear_velocity_data = np.where(
        spatial_state_1.valid_array.numpy()[:, 2],  # Broadcast to match shape [n_frames, 3]
        spatial_state_1.linear_velocity_data.numpy(),
        spatial_state_2.linear_velocity_data.numpy(),
    )

    # Fill in angular velocity data (column 3 of valid)
    angular_velocity_data = np.where(
        spatial_state_1.valid_array.numpy()[:, 3],  # Broadcast to match shape [n_frames, 3]
        spatial_state_1.angular_velocity_data.numpy(),
        spatial_state_2.angular_velocity_data.numpy(),
    )

    # Combine valid flags
    valid = np.where(
        spatial_state_1.valid_array.numpy(),
        spatial_state_1.valid_array.numpy(),
        spatial_state_2.valid_array.numpy(),
    )

    return SpatialState(
        spatial_space=spatial_state_1.spatial_space,
        position_data=wp.array(position_data, dtype=wp.float32),
        linear_velocity_data=wp.array(linear_velocity_data, dtype=wp.float32),
        orientation_data=wp.array(orientation_data, dtype=wp.float32),
        angular_velocity_data=wp.array(angular_velocity_data, dtype=wp.float32),
        valid_array=wp.from_numpy(valid, dtype=wp.bool),
    )


def _combine_root_states(
    root_state_1: RootState | None, root_state_2: RootState | None
) -> RootState | Literal[False] | None:
    """Combine two root states into one.

    Two root states can be combined if they define non-overlapping fields. For example, one
    defines position and the other defines orientation.

    Args:
        root_state_1: The first root state, or None.
        root_state_2: The second root state, or None.

    Returns:
        The combined RootState if successful, None if both inputs are None, or False if the
        states cannot be combined due to field conflicts.
    """
    if root_state_1 is None:
        return root_state_2
    if root_state_2 is None:
        return root_state_1

    # If there is no overlap in the root_state fields, then they can be combined:
    if (root_state_1.position is not None) and (root_state_2.position is not None):
        return False
    if (root_state_1.orientation is not None) and (root_state_2.orientation is not None):
        return False
    if (root_state_1.linear_velocity is not None) and (root_state_2.linear_velocity is not None):
        return False
    if (root_state_1.angular_velocity is not None) and (root_state_2.angular_velocity is not None):
        return False

    # Combine the states:
    return RootState(
        position=root_state_1.position if root_state_1.position is not None else root_state_2.position,
        orientation=root_state_1.orientation if root_state_1.orientation is not None else root_state_2.orientation,
        linear_velocity=(
            root_state_1.linear_velocity if root_state_1.linear_velocity is not None else root_state_2.linear_velocity
        ),
        angular_velocity=(
            root_state_1.angular_velocity
            if root_state_1.angular_velocity is not None
            else root_state_2.angular_velocity
        ),
    )


def combine_robot_states(robot_state_1: RobotState | None, robot_state_2: RobotState | None) -> RobotState | None:
    """Combine two robot states into a single robot state.

    This function merges two RobotState objects by combining their respective joint states,
    root states, link states (spatial states), and site states (spatial states). The combination succeeds only if the
    component states are compatible (i.e., they don't define conflicting values for the same
    joints, frames, or root fields).

    Args:
        robot_state_1: The first robot state to combine, or None.
        robot_state_2: The second robot state to combine, or None.

    Returns:
        The combined RobotState if successful, or None if either input is None or if the
        states cannot be combined due to conflicts.

    Example:

        .. code-block:: python

            import warp as wp
            from isaacsim.robot_motion.experimental.motion_generation import (
                JointState, RobotState, combine_robot_states
            )

            # Create two robot states with different joints
            state_1 = RobotState(
                joints=JointState.from_name(
                    robot_joint_space=["joint_0", "joint_1"],
                    positions=(["joint_0"], wp.array([0.0]))
                )
            )
            state_2 = RobotState(
                joints=JointState.from_name(
                    robot_joint_space=["joint_0", "joint_1"],
                    positions=(["joint_1"], wp.array([1.0]))
                )
            )

            # Combine them into a single state
            combined = combine_robot_states(state_1, state_2)
            # combined.joints.position_names == ["joint_0", "joint_1"]
    """
    # If either robot state is undefined, the entire robot state should be undefined.
    if (robot_state_1 is None) or (robot_state_2 is None):
        return None

    joints = _combine_joint_states(robot_state_1.joints, robot_state_2.joints)
    if joints is False:
        return None

    root = _combine_root_states(robot_state_1.root, robot_state_2.root)
    if root is False:
        return None

    sites = _combine_spatial_states(robot_state_1.sites, robot_state_2.sites)

    if sites is False:
        return None

    links = _combine_spatial_states(robot_state_1.links, robot_state_2.links)
    if links is False:
        return None

    return RobotState(joints=joints, root=root, links=links, sites=sites)

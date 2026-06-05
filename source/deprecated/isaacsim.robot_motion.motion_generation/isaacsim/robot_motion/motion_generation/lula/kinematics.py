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

"""Provides a Lula-based implementation of the KinematicsSolver interface for robot kinematics calculations."""

from __future__ import annotations

from typing import Optional

import lula
import numpy as np
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices
from isaacsim.core.utils.stage import get_stage_units

from ...motion_generation.kinematics_interface import KinematicsSolver
from . import utils as lula_utils
from .interface_helper import LulaInterfaceHelper


class LulaKinematicsSolver(KinematicsSolver):
    """A Lula-based implementation of the KinematicsSolver interface. Lula uses a URDF file describing the robot and.

    a custom yaml file that specifies the cspace of the robot and other parameters.

    This class provides functions beyond the KinematicsSolver interface for getting and setting solver parameters.
    Inverse kinematics is solved quickly by first approximating a solution with cyclic coordinate descent (CCD) and then
    refining the solution with a second-order method (bfgs). As such, parameters for both solvers are available and changeable
    as properties of this class.

    Args:
        robot_description_path: Path to a robot description yaml file describing the cspace of the robot and other
            relevant parameters.
        urdf_path: Path to a URDF file describing the robot.
        robot_description: An initialized lula.RobotDescription object. Other Lula-based classes such as RmpFlow may use
            a lula.RobotDescription object that they have already created to initialize a LulaKinematicsSolver. When
            specified, the provided file paths are unused.
    """

    def __init__(
        self, robot_description_path: str, urdf_path: str, robot_description: Optional[lula.RobotDescription] = None
    ) -> None:
        # Other Lula classes may initialize a KinematicsSolver using a pre-existing lula robot_description

        if robot_description is None:
            self._robot_description = lula.load_robot(robot_description_path, urdf_path)
        else:
            self._robot_description = robot_description
        self._kinematics = self._robot_description.kinematics()
        self._ik_config = lula.CyclicCoordDescentIkConfig()

        LulaInterfaceHelper.__init__(self, self._robot_description)  # for tracking robot base

        self._meters_per_unit = get_stage_units()

        self._default_orientation_tolerance = self._lula_orientation_tol_to_rad_tol(
            self._ik_config.orientation_tolerance
        )
        self._default_position_tolerance = self._ik_config.position_tolerance

        self._default_bfgs_orientation_weight = self._ik_config.bfgs_orientation_weight
        self._default_ccd_orientation_weight = self._ik_config.ccd_orientation_weight

        self._default_cspace_seeds = []

    @property
    def bfgs_cspace_limit_biasing(self) -> bool:
        """Whether configuration space limit biasing is enabled for the BFGS solver.

        Returns:
            True if BFGS cspace limit biasing is enabled.
        """
        return self._ik_config.bfgs_cspace_limit_biasing

    @bfgs_cspace_limit_biasing.setter
    def bfgs_cspace_limit_biasing(self, value: bool) -> None:
        self._ik_config.bfgs_cspace_limit_biasing = value

    @property
    def bfgs_cspace_limit_biasing_weight(self) -> float:
        """Weight applied to configuration space limit biasing in the BFGS solver.

        Returns:
            The BFGS cspace limit biasing weight value.
        """
        return self._ik_config.bfgs_cspace_limit_biasing_weight

    @bfgs_cspace_limit_biasing_weight.setter
    def bfgs_cspace_limit_biasing_weight(self, value: float) -> None:
        self._ik_config.bfgs_cspace_limit_biasing_weight = value

    @property
    def bfgs_cspace_limit_penalty_region(self) -> float:
        """Size of the penalty region for configuration space limits in the BFGS solver.

        Returns:
            The BFGS cspace limit penalty region value.
        """
        return self._ik_config.bfgs_cspace_limit_penalty_region

    @bfgs_cspace_limit_penalty_region.setter
    def bfgs_cspace_limit_penalty_region(self, value: float) -> None:
        self._ik_config.bfgs_cspace_limit_penalty_region = value

    @property
    def bfgs_gradient_norm_termination(self) -> float:
        """Gradient norm threshold for terminating the BFGS solver.

        Returns:
            The BFGS gradient norm termination threshold.
        """
        return self._ik_config.bfgs_gradient_norm_termination

    @bfgs_gradient_norm_termination.setter
    def bfgs_gradient_norm_termination(self, value: float) -> None:
        self._ik_config.bfgs_gradient_norm_termination = value

    @property
    def bfgs_gradient_norm_termination_coarse_scale_factor(self) -> float:
        """Scale factor applied to the gradient norm termination threshold for coarse BFGS iterations.

        Returns:
            The BFGS gradient norm termination coarse scale factor.
        """
        return self._ik_config.bfgs_gradient_norm_termination_coarse_scale_factor

    @bfgs_gradient_norm_termination_coarse_scale_factor.setter
    def bfgs_gradient_norm_termination_coarse_scale_factor(self, value: float) -> None:
        self._ik_config.bfgs_gradient_norm_termination_coarse_scale_factor = value

    @property
    def bfgs_max_iterations(self) -> int:
        """Maximum number of iterations allowed for the BFGS solver.

        Returns:
            The maximum number of BFGS iterations.
        """
        return self._ik_config.bfgs_max_iterations

    @bfgs_max_iterations.setter
    def bfgs_max_iterations(self, value: int) -> None:
        self._ik_config.bfgs_max_iterations = value

    @property
    def bfgs_orientation_weight(self) -> float:
        """Weight applied to orientation error in the BFGS solver cost function.

        Returns:
            The BFGS orientation weight value.
        """
        return self._default_bfgs_orientation_weight

    @bfgs_orientation_weight.setter
    def bfgs_orientation_weight(self, value: float) -> None:
        self._default_bfgs_orientation_weight = value

    @property
    def bfgs_position_weight(self) -> float:
        """Weight applied to position error in the BFGS solver cost function.

        Returns:
            The BFGS position weight value.
        """
        return self._ik_config.bfgs_position_weight

    @bfgs_position_weight.setter
    def bfgs_position_weight(self, value: float) -> None:
        self._ik_config.bfgs_position_weight = value

    @property
    def ccd_bracket_search_num_uniform_samples(self) -> int:
        """Number of uniform samples used in the bracket search for the CCD solver.

        Returns:
            The number of CCD bracket search uniform samples.
        """
        return self._ik_config.ccd_bracket_search_num_uniform_samples

    @ccd_bracket_search_num_uniform_samples.setter
    def ccd_bracket_search_num_uniform_samples(self, value: int) -> None:
        self._ik_config.ccd_bracket_search_num_uniform_samples = value

    @property
    def ccd_descent_termination_delta(self) -> float:
        """Termination threshold for the descent phase of the CCD solver.

        Returns:
            The CCD descent termination delta value.
        """
        return self._ik_config.ccd_descent_termination_delta

    @ccd_descent_termination_delta.setter
    def ccd_descent_termination_delta(self, value: float) -> None:
        self._ik_config.ccd_descent_termination_delta = value

    @property
    def ccd_max_iterations(self) -> int:
        """Maximum number of iterations for the cyclic coordinate descent (CCD) solver.

        Returns:
            Maximum number of CCD iterations.
        """
        return self._ik_config.ccd_max_iterations

    @ccd_max_iterations.setter
    def ccd_max_iterations(self, value: int) -> None:
        self._ik_config.ccd_max_iterations = value

    @property
    def ccd_orientation_weight(self) -> float:
        """Weight applied to orientation error in the cyclic coordinate descent (CCD) solver cost function.

        Returns:
            CCD orientation weight value.
        """
        return self._default_ccd_orientation_weight

    @ccd_orientation_weight.setter
    def ccd_orientation_weight(self, value: float) -> None:
        self._default_ccd_orientation_weight = value

    @property
    def ccd_position_weight(self) -> float:
        """Weight applied to position error in the cyclic coordinate descent (CCD) solver cost function.

        Returns:
            CCD position weight value.
        """
        return self._ik_config.ccd_position_weight

    @ccd_position_weight.setter
    def ccd_position_weight(self, value: float) -> None:
        self._ik_config.ccd_position_weight = value

    @property
    def irwin_hall_sampling_order(self) -> int:
        """Order for Irwin-Hall sampling used in inverse kinematics initial seed generation.

        Returns:
            Irwin-Hall sampling order.
        """
        return self._ik_config.irwin_hall_sampling_order

    @irwin_hall_sampling_order.setter
    def irwin_hall_sampling_order(self, value: int) -> None:
        self._ik_config.irwin_hall_sampling_order = value

    @property
    def max_num_descents(self) -> int:
        """Maximum number of descent iterations allowed during inverse kinematics solving.

        Returns:
            Maximum number of descent iterations.
        """
        return self._ik_config.max_num_descents

    @max_num_descents.setter
    def max_num_descents(self, value: int) -> None:
        self._ik_config.max_num_descents = value

    @property
    def sampling_seed(self) -> int:
        """Random seed used for sampling during inverse kinematics solving.

        Returns:
            Random sampling seed value.
        """
        return self._ik_config.sampling_seed

    @sampling_seed.setter
    def sampling_seed(self, value: int) -> None:
        self._ik_config.sampling_seed = value

    def set_robot_base_pose(self, robot_position: np.array, robot_orientation: np.array) -> None:
        """Sets the robot base pose for kinematics calculations.

        Args:
            robot_position: 3D position vector of the robot base in stage units.
            robot_orientation: Quaternion representing the robot base orientation.
        """
        LulaInterfaceHelper.set_robot_base_pose(self, robot_position, robot_orientation)

    def get_joint_names(self) -> list[str]:
        """Joint names of the active joints in the robot.

        Returns:
            List of active joint names in the order used by the solver.
        """
        return LulaInterfaceHelper.get_active_joints(self)

    def get_all_frame_names(self) -> list[str]:
        """All available frame names in the robot kinematics model.

        Returns:
            List of all frame names that can be used for forward kinematics.
        """
        return self._kinematics.frame_names()

    def compute_forward_kinematics(
        self, frame_name: str, joint_positions: np.array, position_only: Optional[bool] = False
    ) -> tuple[np.array, np.array]:
        """Compute the position of a given frame in the robot relative to the USD stage global frame.

        Args:
            frame_name: Name of robot frame on which to calculate forward kinematics
            joint_positions: Joint positions for the joints returned by get_joint_names()
            position_only: Lula Kinematics ignore this flag and always computes both position and orientation

        Returns:
            A tuple of (frame_positions, frame_rotation) where frame_positions is a (3x1) vector describing the
            translation of the frame relative to the USD stage origin and frame_rotation is a (3x3) rotation matrix
            describing the rotation of the frame relative to the USD stage global frame.
        """
        return LulaInterfaceHelper.get_end_effector_pose(self, joint_positions, frame_name)

    def compute_inverse_kinematics(
        self,
        frame_name: str,
        target_position: np.array,
        target_orientation: np.array = None,
        warm_start: np.array = None,
        position_tolerance: float = None,
        orientation_tolerance: float = None,
    ) -> tuple[np.array, bool]:
        """Compute joint positions such that the specified robot frame will reach the desired translations and rotations.

        Lula Kinematics interpret the orientation tolerance as being the maximum rotation separating any standard axes.
        e.g. For a tolerance of .1: The X axes, Y axes, and Z axes of the rotation matrices may independently be as far as .1 radians apart.

        Default values for position and orientation tolerances may be seen and changed with setter and getter functions.

        Args:
            frame_name: name of the target frame for inverse kinematics
            target_position: target translation of the target frame (in stage units) relative to the USD stage origin
            target_orientation: target orientation of the target frame relative to the USD stage global frame.
            warm_start: a starting position that will be used when solving the IK problem.  If default cspace seeds have been set,
                the warm start will be given priority, but the default seeds will still be used.
            position_tolerance: l-2 norm of acceptable position error (in stage units) between the target and achieved translations.
            orientation_tolerance: magnitude of rotation (in radians) separating the target orientation from the achieved orienatation.
                orientation_tolerance is well defined for values between 0 and pi.

        Returns:
            A tuple containing (joint_positions, success) where joint_positions are in the order specified by get_joint_names() which result in the target frame achieving the desired position and success is True if the solver converged to a solution within the given tolerances.
        """
        if position_tolerance is None:
            self._ik_config.position_tolerance = self._default_position_tolerance
        else:
            self._ik_config.position_tolerance = position_tolerance * self._meters_per_unit

        if orientation_tolerance is None:
            self._ik_config.orientation_tolerance = self._rad_tol_to_lula_orientation_tol(
                self._default_orientation_tolerance
            )
        else:
            self._ik_config.orientation_tolerance = self._rad_tol_to_lula_orientation_tol(orientation_tolerance)

        if target_orientation is None:
            target_orientation = np.array([1, 0, 0, 0])
            self._ik_config.orientation_tolerance = 2.0
            self._ik_config.ccd_orientation_weight = 0.0
            self._ik_config.bfgs_orientation_weight = 0.0
        else:
            self._ik_config.ccd_orientation_weight = self._default_ccd_orientation_weight
            self._ik_config.bfgs_orientation_weight = self._default_bfgs_orientation_weight

        rot = quats_to_rot_matrices(target_orientation).astype(np.float64)
        pos = target_position.astype(np.float64) * self._meters_per_unit

        pos, rot = LulaInterfaceHelper._get_pose_rel_robot_base(self, pos, rot)

        target_pose = lula_utils.get_pose3(pos, rot)

        if warm_start is not None:
            seeds = [warm_start]
            seeds.extend(self._default_cspace_seeds)
            self._ik_config.cspace_seeds = seeds
        else:
            self._ik_config.cspace_seeds = self._default_cspace_seeds

        results = lula.compute_ik_ccd(self._kinematics, target_pose, frame_name, self._ik_config)

        return results.cspace_position, results.success

    def supports_collision_avoidance(self) -> bool:
        """Lula Inverse Kinematics do not support collision avoidance with USD obstacles.

        Returns:
            Always False
        """
        return False

    def set_default_orientation_tolerance(self, tolerance: float) -> None:
        """Default orientation tolerance to be used when calculating IK when none is specified.

        Args:
            tolerance: magnitude of rotation (in radians) separating the target orientation from the achieved orienatation.
                orientation_tolerance is well defined for values between 0 and pi.
        """
        self._default_orientation_tolerance = tolerance

    def set_default_position_tolerance(self, tolerance: float) -> None:
        """Default position tolerance to be used when calculating IK when none is specified.

        Args:
            tolerance: l-2 norm of acceptable position error (in stage units) between the target and achieved translations
        """
        self._default_position_tolerance = tolerance * self._meters_per_unit

    def set_default_cspace_seeds(self, seeds: np.array) -> None:
        """Set a list of cspace seeds that the solver may use as starting points for solutions.

        Args:
            seeds: An N x num_dof list of cspace seeds
        """
        self._default_cspace_seeds = seeds

    def get_default_orientation_tolerance(self) -> float:
        """Default orientation tolerance to be used when calculating IK when none is specified.

        Returns:
            magnitude of rotation (in radians) separating the target orientation from the achieved orienatation.
            orientation_tolerance is well defined for values between 0 and pi.
        """
        return self._default_orientation_tolerance

    def get_default_position_tolerance(self) -> float:
        """Default position tolerance to be used when calculating IK when none is specified.

        Returns:
            l-2 norm of acceptable position error (in stage units) between the target and achieved translations
        """
        return self._default_position_tolerance / self._meters_per_unit

    def get_default_cspace_seeds(self) -> list[np.array]:
        """List of cspace seeds that the solver may use as starting points for solutions.

        Returns:
            An N x num_dof list of cspace seeds
        """
        return self._default_cspace_seeds

    def get_cspace_position_limits(self) -> tuple[np.array, np.array]:
        """Default upper and lower joint limits of the active joints.

        Returns:
            A tuple containing (default_lower_joint_position_limits, default_upper_joint_position_limits) where default_lower_joint_position_limits are Default lower position limits of active joints and default_upper_joint_position_limits are Default upper position limits of active joints
        """
        num_coords = self._kinematics.num_c_space_coords()

        lower = []
        upper = []
        for i in range(num_coords):
            limits = self._kinematics.c_space_coord_limits(i)
            lower.append(limits.lower)
            upper.append(limits.upper)

        c_space_position_upper_limits = np.array(upper, dtype=np.float64)
        c_space_position_lower_limits = np.array(lower, dtype=np.float64)

        return c_space_position_lower_limits, c_space_position_upper_limits

    def get_cspace_velocity_limits(self) -> np.array:
        """Default velocity limits of the active joints.

        Returns:
            Default velocity limits of the active joints
        """
        num_coords = self._kinematics.num_c_space_coords()

        c_space_velocity_limits = np.array(
            [self._kinematics.c_space_coord_velocity_limit(i) for i in range(num_coords)], dtype=np.float64
        )
        return c_space_velocity_limits

    def get_cspace_acceleration_limits(self) -> np.array:
        """Get the default acceleration limits of the active joints.

        Default acceleration limits are read from the robot_description YAML file.
        Any acceleration limits that are not specified in the robot_description YAML file will
        be None.

        Returns:
            Default acceleration limits of the active joints.
        """
        num_coords = self._kinematics.num_c_space_coords()

        c_space_acceleration_limits = [None] * num_coords
        for i in range(num_coords):
            if self._kinematics.has_c_space_acceleration_limit(i):
                c_space_acceleration_limits[i] = self._kinematics.c_space_coord_acceleration_limit(i)

        return np.array(c_space_acceleration_limits)

    def get_cspace_jerk_limits(self) -> np.array:
        """Get the default jerk limits of the active joints.

        Default jerk limits are read from the robot_description YAML file.
        Any jerk limits that are not specified in the robot_description YAML file will
        be None.

        Returns:
            Default jerk limits of the active joints.
        """
        num_coords = self._kinematics.num_c_space_coords()

        c_space_jerk_limits = [None] * num_coords
        for i in range(num_coords):
            if self._kinematics.has_c_space_jerk_limit(i):
                c_space_jerk_limits[i] = self._kinematics.c_space_coord_jerk_limit(i)

        return np.array(c_space_jerk_limits)

    def _lula_orientation_tol_to_rad_tol(self, tol: float) -> float:
        """Convert from lula IK orientation tolerance to radian magnitude tolerance.

        This function is the inverse of _rad_tol_to_lula_orientation_tol.

        Args:
            tol: Lula orientation tolerance value to convert.

        Returns:
            The equivalent radian magnitude tolerance value.
        """
        # convert from lula IK orientation tolerance to radian magnitude tolerance
        # This function is the inverse of _rad_tol_to_lula_orientation_tol

        return np.arccos(1 - tol**2 / 2)

    def _rad_tol_to_lula_orientation_tol(self, tol: float) -> float:
        """Convert from radian magnitude tolerance to lula IK orientation tolerance.

        Orientation tolerance in Lula is defined as the maximum l2-norm between rotation matrix columns
        paired by index. For example, rotating pi rad about the z axis maps to a norm of 2.0 when
        comparing the x columns.

        Args:
            tol: Radian magnitude tolerance value to convert.

        Returns:
            The equivalent lula orientation tolerance value.
        """
        # convert from radian magnitude tolerance to lula IK orientation tolerance

        # Orientation tolerance in Lula is defined as the maximum l2-norm between rotation matrix columns paired by index.
        # e.g. rotating pi rad about the z axis maps to a norm of 2.0 when comparing the x columns

        return np.linalg.norm(np.subtract([1, 0], [np.cos(tol), np.sin(tol)]))


LulaKinematicsSolver.bfgs_cspace_limit_biasing.__doc__ = (
    lula.CyclicCoordDescentIkConfig.bfgs_cspace_limit_biasing.__doc__
)

LulaKinematicsSolver.bfgs_cspace_limit_biasing_weight.__doc__ = (
    lula.CyclicCoordDescentIkConfig.bfgs_cspace_limit_biasing_weight.__doc__
)

LulaKinematicsSolver.bfgs_cspace_limit_penalty_region.__doc__ = (
    lula.CyclicCoordDescentIkConfig.bfgs_cspace_limit_penalty_region.__doc__
)

LulaKinematicsSolver.bfgs_gradient_norm_termination.__doc__ = (
    lula.CyclicCoordDescentIkConfig.bfgs_gradient_norm_termination.__doc__
)

LulaKinematicsSolver.bfgs_gradient_norm_termination_coarse_scale_factor.__doc__ = (
    lula.CyclicCoordDescentIkConfig.bfgs_gradient_norm_termination_coarse_scale_factor.__doc__
)

LulaKinematicsSolver.bfgs_max_iterations.__doc__ = lula.CyclicCoordDescentIkConfig.bfgs_max_iterations.__doc__

LulaKinematicsSolver.bfgs_orientation_weight.__doc__ = lula.CyclicCoordDescentIkConfig.bfgs_orientation_weight.__doc__

LulaKinematicsSolver.bfgs_position_weight.__doc__ = lula.CyclicCoordDescentIkConfig.bfgs_position_weight.__doc__

LulaKinematicsSolver.ccd_bracket_search_num_uniform_samples.__doc__ = (
    lula.CyclicCoordDescentIkConfig.ccd_bracket_search_num_uniform_samples.__doc__
)

LulaKinematicsSolver.ccd_descent_termination_delta.__doc__ = (
    lula.CyclicCoordDescentIkConfig.ccd_descent_termination_delta.__doc__
)

LulaKinematicsSolver.ccd_max_iterations.__doc__ = lula.CyclicCoordDescentIkConfig.ccd_max_iterations.__doc__

LulaKinematicsSolver.ccd_orientation_weight.__doc__ = lula.CyclicCoordDescentIkConfig.ccd_orientation_weight.__doc__

LulaKinematicsSolver.ccd_position_weight.__doc__ = lula.CyclicCoordDescentIkConfig.ccd_position_weight.__doc__

LulaKinematicsSolver.irwin_hall_sampling_order.__doc__ = (
    lula.CyclicCoordDescentIkConfig.irwin_hall_sampling_order.__doc__
)

LulaKinematicsSolver.max_num_descents.__doc__ = lula.CyclicCoordDescentIkConfig.max_num_descents.__doc__

LulaKinematicsSolver.sampling_seed.__doc__ = lula.CyclicCoordDescentIkConfig.sampling_seed.__doc__

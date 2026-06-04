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

"""Provides obstacle representation strategies and configurations for robot motion planning."""

from enum import StrEnum, auto
from typing import Type

from isaacsim.core.experimental.objects import (
    Capsule,
    Cone,
    Cube,
    Cylinder,
    Mesh,
    Plane,
    Shape,
    Sphere,
)
from isaacsim.core.experimental.prims import Prim

from .utils.prims import get_shape_type


class ObstacleRepresentation(StrEnum):
    """Enumerate supported obstacle representations for planning.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_motion.experimental.motion_generation import ObstacleRepresentation
        >>>
        >>> ObstacleRepresentation.SPHERE.name
        'SPHERE'
    """

    SPHERE = auto()
    """Spherical obstacle representation for planning."""
    CONE = auto()
    """Conical obstacle representation for planning."""
    CUBE = auto()
    """Cubic obstacle representation for planning."""
    PLANE = auto()
    """Planar obstacle representation for planning."""
    CAPSULE = auto()
    """Capsular obstacle representation for planning."""
    CYLINDER = auto()
    """Cylindrical obstacle representation for planning."""
    MESH = auto()
    """Mesh obstacle representation for planning."""
    TRIANGULATED_MESH = auto()
    """Triangulated mesh obstacle representation for planning."""
    OBB = auto()
    """Oriented bounding box obstacle representation for planning."""

    # TODO:
    # CONVEX_HULL = auto()
    # CONVEX_DECOMPOSITION = auto()
    # BOUNDING_SPHERE = auto()
    # SIGNED_DISTANCE_FIELD = auto()
    # OCTREE = auto()
    # VOXEL_GRID = auto()


# This is the set of legal conversions:
# TODO: SHOULD ALLOW FOR ADDITIONAL CONVERSIONS. FOR EXAMPLE,
# OBJECTS COULD BE GENERALLY CONVERTABLE TO A MESH REPRESENTATION.
_LEGAL_REPRESENTATIONS: dict[Type[Shape], list[ObstacleRepresentation]] = {
    Sphere: [
        ObstacleRepresentation.SPHERE,
        ObstacleRepresentation.OBB,
    ],
    Cone: [
        ObstacleRepresentation.CONE,
        ObstacleRepresentation.OBB,
    ],
    Cube: [
        ObstacleRepresentation.CUBE,
        ObstacleRepresentation.OBB,
    ],
    Plane: [
        ObstacleRepresentation.PLANE,
    ],
    Capsule: [
        ObstacleRepresentation.CAPSULE,
        ObstacleRepresentation.OBB,
    ],
    Cylinder: [
        ObstacleRepresentation.CYLINDER,
        ObstacleRepresentation.OBB,
    ],
    Mesh: [
        ObstacleRepresentation.MESH,
        ObstacleRepresentation.TRIANGULATED_MESH,
        ObstacleRepresentation.OBB,
    ],
}


class ObstacleConfiguration:
    """Define the planning representation and safety tolerance for a shape.

    Args:
        representation: Representation type used in planning.
        safety_tolerance: Safety distance buffer applied to the obstacle.

    Raises:
        ValueError: if representation is a str that does not map to an ObstacleRepresentation.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_motion.experimental.motion_generation import (
        ...     ObstacleConfiguration,
        ...     ObstacleRepresentation,
        ... )
        >>>
        >>> # Can use string representation
        >>> config = ObstacleConfiguration(representation="sphere", safety_tolerance=0.1)
        >>> config.representation is ObstacleRepresentation.SPHERE
        True
        >>> config.safety_tolerance == 0.1
        True
        >>>
        >>> # Or use enum directly
        >>> config2 = ObstacleConfiguration(
        ...     representation=ObstacleRepresentation.SPHERE,
        ...     safety_tolerance=0.1,
        ... )
        >>> config2.representation is ObstacleRepresentation.SPHERE
        True
    """

    def __init__(self, representation: ObstacleRepresentation | str, safety_tolerance: float):
        if not isinstance(representation, (ObstacleRepresentation, str)):
            raise ValueError("representation must be either ObstacleRepresentation or str")

        if isinstance(representation, str):
            # will raise value error if not an option.
            representation = ObstacleRepresentation(representation)

        self.representation = representation
        self.safety_tolerance = safety_tolerance


class ObstacleStrategy:
    """Manage obstacle representation defaults and overrides.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_motion.experimental.motion_generation import ObstacleStrategy
        >>> strategy = ObstacleStrategy()
    """

    def __init__(self):
        self.__default_configurations = {
            Sphere: ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=0.0),
            Cone: ObstacleConfiguration(representation=ObstacleRepresentation.CONE, safety_tolerance=0.0),
            Cube: ObstacleConfiguration(representation=ObstacleRepresentation.CUBE, safety_tolerance=0.0),
            Plane: ObstacleConfiguration(representation=ObstacleRepresentation.PLANE, safety_tolerance=0.0),
            Capsule: ObstacleConfiguration(representation=ObstacleRepresentation.CAPSULE, safety_tolerance=0.0),
            Cylinder: ObstacleConfiguration(representation=ObstacleRepresentation.CYLINDER, safety_tolerance=0.0),
            Mesh: ObstacleConfiguration(representation=ObstacleRepresentation.MESH, safety_tolerance=0.0),
        }

        # store overrides for an entire shape-type which are set by the user:
        # these are higher priority than the default configurations.
        self.__shape_configuration_overrides = {}

        # store overrides for a specific prim which are set by the user:
        # these are higher priority than the shape-type overrides.
        self.__prim_configuration_overrides = {}

    def set_default_configuration(
        self, prim_type: Type[Shape], configuration: ObstacleConfiguration, allow_negative_tolerance: bool = False
    ):
        """Set the default configuration for a given prim type.

        Args:
            prim_type: The type of shape for which we want to update the default obstacle configuration.
            configuration: Configuration to store for the shape type.
            allow_negative_tolerance: An override flag to allow negative safety tolerances.

        Raises:
            ValueError: If the representation is invalid for the shape type.
            ValueError: If safety tolerance is negative and allow_negative_tolerance is False.

        Example:

        .. code-block:: python

            >>> from isaacsim.robot_motion.experimental.motion_generation import (
            ...     ObstacleConfiguration,
            ...     ObstacleRepresentation,
            ...     ObstacleStrategy,
            ... )
            >>> from isaacsim.core.experimental.objects import Sphere
            >>>
            >>> strategy = ObstacleStrategy()
            >>> strategy.set_default_configuration(
            ...     Sphere,
            ...     ObstacleConfiguration(
            ...         representation=ObstacleRepresentation.OBB,
            ...         safety_tolerance=0.05,
            ...     ),
            ... )
        """
        # verify that the representation is legal for this type:
        if not (configuration.representation in _LEGAL_REPRESENTATIONS[prim_type]):
            raise ValueError(f"{configuration.representation} is not a valid obstacle representation for {prim_type}.")

        if configuration.safety_tolerance < 0 and not allow_negative_tolerance:
            raise ValueError(
                f"Safety tolerance cannot be negative. Got {configuration.safety_tolerance}. Use allow_negative_tolerance=True to allow negative tolerances."
            )

        self.__shape_configuration_overrides[prim_type] = configuration

    def set_default_safety_tolerance(self, safety_tolerance: float, allow_negative_tolerance: bool = False):
        """Set the safety tolerance on the default configuration for all prim types.

        Args:
            safety_tolerance: Safety tolerance to apply to all defaults.
            allow_negative_tolerance: An override flag to allow negative safety tolerances.

        Raises:
            ValueError: If safety tolerance is negative and allow_negative_tolerance is False.

        Example:

        .. code-block:: python

            >>> from isaacsim.robot_motion.experimental.motion_generation import ObstacleStrategy
            >>>
            >>> strategy = ObstacleStrategy()
            >>> strategy.set_default_safety_tolerance(0.2)
        """
        if safety_tolerance < 0 and not allow_negative_tolerance:
            raise ValueError(
                f"Safety tolerance cannot be negative. Got {safety_tolerance}. Use allow_negative_tolerance=True to allow negative tolerances."
            )

        for prim_type in self.__default_configurations:
            self.__default_configurations[prim_type].safety_tolerance = safety_tolerance

    def set_configuration_overrides(
        self, configurations: dict[str, ObstacleConfiguration], allow_negative_tolerance: bool = False
    ):
        """Set the configuration overrides for a given set of prim paths.

        Args:
            configurations: Mapping of prim path to override configuration.
            allow_negative_tolerance: An override flag to allow negative safety tolerances.

        Raises:
            RuntimeError: If any prim paths do not correspond to valid prims on the stage.
            ValueError: If any representation is invalid for its shape type.
            ValueError: If any safety tolerance is negative and allow_negative_tolerance is False.

        Example:

        .. code-block:: python

            >>> from isaacsim.robot_motion.experimental.motion_generation import (
            ...     ObstacleConfiguration,
            ...     ObstacleRepresentation,
            ...     ObstacleStrategy,
            ... )
            >>> from isaacsim.core.experimental.objects import Sphere
            >>>
            >>> strategy = ObstacleStrategy()
            >>> _ = Sphere(paths="/World/Sphere")
            >>> strategy.set_configuration_overrides(
            ...     {
            ...         "/World/Sphere": ObstacleConfiguration(
            ...             representation=ObstacleRepresentation.OBB,
            ...             safety_tolerance=0.1,
            ...         )
            ...     }
            ... )
        """
        # Validate that all prims we want to set overrides on exist in the stage:
        prim_paths = list(configurations.keys())
        _, nonexistent_paths = Prim.resolve_paths(prim_paths, raise_on_mixed_paths=False)
        if nonexistent_paths:
            msg = (
                f"Failed to set configuration overrides: one or more prim paths do not exist on the stage.\n"
                f"Invalid prim paths: {nonexistent_paths}"
            )
            raise RuntimeError(msg)

        # First, confirm that ALL configurations are valid before setting any overrides:
        for prim_path, configuration in configurations.items():
            prim_type = get_shape_type(prim_path)

            # verify that the representation is legal for this type:
            if not (configuration.representation in _LEGAL_REPRESENTATIONS[prim_type]):
                raise ValueError(
                    f"{configuration.representation} is not a valid obstacle representation for {prim_type}. No overrides will be applied."
                )

            if configuration.safety_tolerance < 0 and not allow_negative_tolerance:
                raise ValueError(
                    f"Safety tolerance cannot be negative. Got {configuration.safety_tolerance}. Use allow_negative_tolerance=True to allow negative tolerances."
                )

        # All configurations are valid, so set the overrides:
        for prim_path, configuration in configurations.items():
            self.__prim_configuration_overrides[prim_path] = configuration

    def get_obstacle_configuration(self, prim_path: str) -> ObstacleConfiguration:
        """Get the obstacle configuration for a given prim path.

        Args:
            prim_path: Prim path to resolve to a configuration.

        Returns:
            Obstacle configuration for the prim path.

        Raises:
            RuntimeError: If the prim path: does not exist on the stage, is unsupported, or configuration is invalid.

        Example:

        .. code-block:: python

            >>> from isaacsim.robot_motion.experimental.motion_generation import ObstacleStrategy
            >>> from isaacsim.core.experimental.objects import Sphere
            >>>
            >>> strategy = ObstacleStrategy()
            >>> _ = Sphere(paths="/World/Sphere")
            >>> config = strategy.get_obstacle_configuration("/World/Sphere")
            >>> config.safety_tolerance
            0.0
        """
        # Validate that our prim exists on the stage:
        _, nonexistent_paths = Prim.resolve_paths(prim_path, raise_on_mixed_paths=False)
        if nonexistent_paths:
            msg = (
                f"Failed to get obstacle configuration: prim path does not exist on the stage.\n"
                f"Invalid prim path: {prim_path}"
            )
            raise RuntimeError(msg)

        # get the defined obstacle configuration for this object:
        shape_type = get_shape_type(prim_path)
        if prim_path in self.__prim_configuration_overrides:
            obstacle_configuration = self.__prim_configuration_overrides[prim_path]
        elif shape_type in self.__shape_configuration_overrides:
            obstacle_configuration = self.__shape_configuration_overrides[shape_type]
        else:
            obstacle_configuration = self.__default_configurations[shape_type]

        # verify that the representation is legal for this type:
        if not (obstacle_configuration.representation in _LEGAL_REPRESENTATIONS[shape_type]):
            raise RuntimeError(
                f"{obstacle_configuration.representation} is not a valid obstacle representation for {shape_type}."
            )

        return obstacle_configuration

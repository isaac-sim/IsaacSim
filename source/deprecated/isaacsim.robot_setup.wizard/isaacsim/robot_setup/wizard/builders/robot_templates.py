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

"""Provides robot template classes for the Isaac Sim robot setup wizard, including base abstractions and specific implementations for different robot types like manipulators, wheeled robots, humanoids, and quadrupeds."""

from abc import ABC
from typing import Any

from ..utils.utils import Singleton


#######################
# Information saved in the templates and registry are meant for continuity for the wizard UI only.
# Robot definition and API/setting implementation are saved to the usd files at each step of the wizard.
#######################
@Singleton
class RobotRegistry:
    """A singleton class that keeps track of a single robot instance."""

    _instance = None
    """Singleton instance of the RobotRegistry class."""
    _robot = None
    """The currently registered robot instance."""

    @classmethod
    def register(cls, name: str, instance: object) -> None:
        """Register a robot instance with the registry.

        Args:
            name: Name of the robot to register.
            instance: The robot instance to register.

        Raises:
            ValueError: If a robot is already registered.
        """
        if cls._robot is not None:
            raise ValueError("A robot is already registered. Only one robot instance is allowed.")
        cls._robot = instance
        cls._robot_name = name

    @classmethod
    def get(cls) -> None:
        """Get the currently registered robot instance.

        Returns:
            The registered robot instance, or None if no robot is registered.
        """
        return cls._robot

    @classmethod
    def update(cls, instance: object) -> None:
        """Update the currently registered robot instance.

        Args:
            instance: The robot instance to update to.
        """
        cls._robot = instance

    @classmethod
    def get_name(cls) -> None:
        """Get the name of the currently registered robot.

        Returns:
            The robot name, or None if no robot is registered.
        """
        return cls._robot_name if cls._robot is not None else None

    @classmethod
    def reset(cls) -> None:
        """Reset the registry by clearing the registered robot instance and name."""
        cls._robot = None
        cls._robot_name = None
        cls._instance = None


class RobotTemplate(ABC):
    """Abstract base class for robot templates in the Isaac Sim robot setup wizard.

    This class serves as the foundation for defining different types of robots (manipulators, grippers, wheeled robots,
    etc.) within the wizard framework. It provides a common interface and shared functionality for robot configuration,
    file management, and property handling.

    The class automatically registers itself with the RobotRegistry singleton upon instantiation and dynamically adds
    properties for storing robot configuration data such as file paths, link information, and save options. Subclasses
    implement specific robot types by adding their own specialized properties and configurations.

    Args:
        name: The name identifier for the robot template instance.
    """

    @staticmethod
    def add_property(cls: type, name: str, default: object) -> None:
        """Dynamically adds a property `name` to `cls`.

        Stores `default` as the initial value, getter returns it, and setter rebinds it on the instance.
        Only supports simple data types like string, list, or dict.

        Args:
            cls: The class to add the property to.
            name: The name of the property to add.
            default: The initial value for the property.
        """
        private = f"_{name}"

        # Store the default value on the class
        setattr(cls, private, default)

        def getter(self: Any) -> None:
            return getattr(self, private)

        def setter(self: Any, value: Any) -> None:
            setattr(self, private, value)

        doc = f"Dynamically added simple property '{name}' for storing basic data types"

        # Create and install the property
        prop = property(getter, setter, doc=doc)
        setattr(cls, name, prop)

    def __init__(self, name: str) -> None:
        # the properties default to any robot template
        self.add_property(self.__class__, "name", name)
        self.add_property(self.__class__, "parent_prim_path", None)  # robot prim at the origianl stage

        self.add_property(self.__class__, "robot_root_folder", None)
        self.add_property(self.__class__, "base_file_path", None)
        self.add_property(self.__class__, "physics_file_path", None)
        self.add_property(self.__class__, "robot_schema_file_path", None)
        self.add_property(self.__class__, "robot_file_path", None)  # the main robot file usd with all the variants
        self.add_property(
            self.__class__, "original_stage_path", None
        )  # path to the original stage where editing started
        self.add_property(
            self.__class__, "original_robot_path", None
        )  # path to the original robot even if the robot on editing stage is a reference or payload
        self.add_property(self.__class__, "save_stage_copy", False)  # save a copy of the stage in the robot root folder
        self.add_property(
            self.__class__, "save_stage_original", False
        )  # save (likely overwrites) the original stage in the original_stage_path

        # register the robot
        RobotRegistry().register(name, self)


class CustomRobot(RobotTemplate):
    """A robot template for custom robot types.

    This class extends RobotTemplate to provide a flexible foundation for robots that don't fit into standard categories like manipulators, wheeled robots, or humanoids. It maintains the core robot template functionality while allowing for custom link configurations and specialized properties.

    The custom robot template initializes with default links and registers itself with the RobotRegistry for tracking within the robot setup wizard. It inherits all standard robot properties including file paths for physics, schema, and robot definitions, as well as stage management capabilities.

    Args:
        name: Name identifier for the custom robot instance.
    """

    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.add_property(self.__class__, "robot_type", "Custom")
        self.add_property(self.__class__, "links", ["link1", "link2"])
        # register the robot
        RobotRegistry().update(self)

    def __repr__(self) -> str:
        """Return a string representation of the CustomRobot instance.

        Returns:
            A formatted string containing the class name and all instance attributes.
        """
        return f"CustomRobot: {self.__dict__}"


class WheeledRobot(RobotTemplate):
    """A robot template class for creating wheeled robots with chassis and two wheels.

    This class extends RobotTemplate to provide specific configurations for wheeled robots,
    including predefined link structures and robot type identification. It automatically
    registers itself with the RobotRegistry upon instantiation and sets up default
    properties for wheeled robot configurations.

    The class dynamically adds properties for storing robot metadata such as file paths,
    stage information, and save options. It defines a standard wheeled robot structure
    with chassis, left wheel, and right wheel links.

    Args:
        name: The name identifier for the wheeled robot instance.
    """

    def __init__(self, name: str) -> None:

        super().__init__(name)

        # manipulator specific defaults
        self.add_property(self.__class__, "links", ["chassis", "wheel_left", "wheel_right"])
        self.add_property(self.__class__, "robot_type", "Wheeled Robot")
        # register the robot
        RobotRegistry().update(self)

    def __repr__(self) -> str:
        """Returns a string representation of the WheeledRobot instance.

        Returns:
            A string containing the class name and all instance attributes.
        """
        return f"WheeledRobot: {self.__dict__}"


class Manipulator(RobotTemplate):
    """A robot template class for manipulator robots.

    This class represents a manipulator robot type in the robot setup wizard. It inherits from RobotTemplate
    and provides default configuration for manipulator robots, including predefined links and robot type
    classification. Upon initialization, it registers itself with the RobotRegistry singleton to track
    the robot instance throughout the wizard workflow.

    The manipulator template includes default links for a typical robotic arm structure with a tooltip
    end effector. This template serves as a foundation for configuring manipulator robots in the Isaac
    Sim robot setup wizard.

    Args:
        name: The name of the manipulator robot.
    """

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.add_property(self.__class__, "links", ["link1", "link2", "link3", "tooltip"])
        self.add_property(self.__class__, "robot_type", "Manipulator")
        # register the robot
        RobotRegistry().update(self)

    def __repr__(self) -> str:
        """Return a string representation of the Manipulator instance.

        Returns:
            String representation showing the class name and instance attributes.
        """
        return f"Manipulator: {self.__dict__}"


class Gripper(RobotTemplate):
    """A robot template for gripper-type robots.

    This class represents a gripper robot configuration with predefined links including base_link, right_finger,
    and left_finger. It inherits from RobotTemplate and provides gripper-specific defaults for robot setup and
    configuration within the robot wizard.

    The class automatically registers itself with the RobotRegistry upon instantiation and sets up default
    properties common to gripper robots. It uses dynamic property addition to store robot configuration data
    such as file paths, links, and robot type information.

    Args:
        name: The name identifier for the gripper robot instance.
    """

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.add_property(self.__class__, "links", ["base_link", "right_finger", "left_finger"])
        self.add_property(self.__class__, "robot_type", "Gripper")

        # register the robot
        RobotRegistry().update(self)

    def __repr__(self) -> str:
        """String representation of the Gripper instance.

        Returns:
            A string showing the class name and instance attributes.
        """
        return f"Gripper: {self.__dict__}"


class Humanoid(RobotTemplate):
    """A robot template class for humanoid robots in the Robot Setup Wizard.

    This class provides a structured template for configuring humanoid robots with typical anatomical links
    including arms and legs. It inherits from RobotTemplate and automatically sets up default properties
    specific to humanoid robot configurations.

    The class initializes with predefined links representing a basic humanoid structure: base_link,
    right_leg, left_leg, right_arm, and left_arm. It sets the robot_type to "Humanoid" and registers
    itself with the RobotRegistry for use throughout the wizard workflow.

    This template serves as a starting point for humanoid robot setup and can be customized through
    the wizard interface to match specific robot configurations.

    Args:
        name: The name identifier for the humanoid robot instance.
    """

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.add_property(self.__class__, "links", ["base_link", "right_leg", "left_leg", "right_arm", "left_arm"])
        self.add_property(self.__class__, "robot_type", "Humanoid")

        # register the robot
        RobotRegistry().update(self)

    def __repr__(self) -> str:
        """String representation of the Humanoid robot template.

        Returns:
            A string containing the class name and instance attributes.
        """
        return f"Humanoid: {self.__dict__}"


class Quadruped(RobotTemplate):
    """A template class for quadruped robot configuration in the Isaac Sim robot setup wizard.

    This class provides a specialized template for four-legged robots, inheriting from RobotTemplate
    to provide common robot setup functionality. It automatically configures default properties
    specific to quadruped robots, including typical link names and robot type classification.

    The class dynamically adds properties for managing robot file paths, stage information,
    and robot-specific configuration data. Upon initialization, it registers itself with the
    RobotRegistry singleton to maintain a single active robot instance throughout the wizard workflow.

    Args:
        name: The name identifier for the quadruped robot instance.
    """

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.add_property(self.__class__, "links", ["base_link", "right_leg", "left_leg", "right_leg", "left_leg"])
        self.add_property(self.__class__, "robot_type", "Quadruped")

        # register the robot
        RobotRegistry().update(self)

    def __repr__(self) -> str:
        """String representation of the Quadruped robot instance.

        Returns:
            A formatted string containing the robot type and all instance attributes.
        """
        return f"Quadruped: {self.__dict__}"

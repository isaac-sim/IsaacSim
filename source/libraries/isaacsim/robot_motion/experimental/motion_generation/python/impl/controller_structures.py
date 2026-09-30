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

"""Controller structures for composing and organizing multiple robot motion controllers."""

from enum import Enum

from ._logging import log
from .base_controller import BaseController
from .types import RobotState, combine_robot_states


# Class to contain any number of controllers. Because they all have the same API,
# we can swap any two controllers at run-time.
class SelectableController(BaseController):
    """Controller class which contains any number of controllers.

    Can switch between controllers at run-time using a selection key.

    Args:
        controller_options: Mapping of selection keys to controllers.
        initial_controller_selection: Selection key for the initial controller.

    Raises:
        ValueError: If no controllers are provided.

    Example:

    .. code-block:: python

        >>> selectable_controller = SelectableController(
        ...     controller_options=controller_options,
        ...     initial_controller_selection=initial_selection,
        ... )
        >>> selectable_controller.get_active_controller_enum() == initial_selection
        True
    """

    def __init__(self, controller_options: dict[Enum, BaseController], initial_controller_selection: Enum) -> None:
        self._controller_options = controller_options
        if len(controller_options) == 0:
            raise ValueError("SelectableController must have at least one controller.")
        self._active_controller = controller_options[initial_controller_selection]

        # storing the initial, active, and next controller:
        self._initial_controller_selection = initial_controller_selection
        self._active_controller_selection = initial_controller_selection
        self._next_controller_selection = initial_controller_selection

    def reset(self, estimated_state: RobotState, setpoint_state: RobotState | None, t: float, **kwargs: object) -> bool:
        """Set the initial controller to be active again and call reset on it.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional setpoint state of the robot.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments.

        Returns:
            True if reset succeeded, False otherwise.

        Example:

        .. code-block:: python

            >>> ok = selectable_controller.reset(estimated_state, setpoint_state, t)
            >>> if not ok:
            ...     raise RuntimeError("Controller reset failed")
        """
        self._active_controller_selection = self._initial_controller_selection
        self._next_controller_selection = self._initial_controller_selection
        self._active_controller = self._controller_options[self._initial_controller_selection]
        return self._active_controller.reset(estimated_state, setpoint_state, t, **kwargs)

    def forward(
        self, estimated_state: RobotState, setpoint_state: RobotState | None, t: float, **kwargs: object
    ) -> RobotState | None:
        """Run the active controller.

        If the active controller has changed since the last forward call, the controller is
        switched to the next controller, and then the reset function is called on the new
        controller.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional setpoint state of the robot.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments.

        Returns:
            Desired robot state for the robot to track, or None if the active controller fails
            to produce a valid output.

        Raises:
            RuntimeError: If resetting the new controller fails during the switch.

        Example:

        .. code-block:: python

            >>> desired = selectable_controller.forward(estimated_state, setpoint_state, t)
            >>> if desired is None:
            ...     print("Controller failed")
        """
        if self._next_controller_selection != self._active_controller_selection:
            self._active_controller = self._controller_options[self._next_controller_selection]

            # save which controller is active.
            self._active_controller_selection = self._next_controller_selection

            if not self._active_controller.reset(estimated_state, setpoint_state, t, **kwargs):
                raise RuntimeError(
                    f"Error resetting controller {self._next_controller_selection} during call to forward."
                )

        # run the controller and return its result.
        return self._active_controller.forward(estimated_state, setpoint_state, t, **kwargs)

    def set_next_controller(self, next_controller_selection: Enum) -> None:
        """Set the controller which will be running starting at the next time-step.

        This function is typically called by a higher-level behavior control scheme,
        such as a state-machine or a behavior tree.

        Args:
            next_controller_selection: Selection key for the controller to switch to.

        Raises:
            LookupError: If the requested enum does not correspond to any controller.

        Example:

        .. code-block:: python

            >>> selectable_controller.set_next_controller(next_controller_selection)
        """
        if next_controller_selection not in self._controller_options:
            raise LookupError("There is no controller which corresponds to the requested enum.")
        self._next_controller_selection = next_controller_selection

    def get_active_controller_enum(self) -> Enum:
        """Get the enum of the currently active controller.

        This function is typically called by a higher-level behavior control scheme,
        such as a state-machine or a behavior tree.

        Returns:
            Selection key for the currently active controller.

        Example:

        .. code-block:: python

            >>> active = selectable_controller.get_active_controller_enum()
        """
        return self._active_controller_selection

    def get_controller(self, controller_selection: Enum) -> BaseController:
        """Get a controller object.

        This is useful to call custom methods on a controller which are not part of the BaseController interface.
        For example, many controllers may require long-term goals (setpoints). These types
        of functions are not part of the BaseController interface, and must be called on
        the controller object directly.

        Args:
            controller_selection: Selection key for the controller to get.

        Returns:
            The controller instance.

        Raises:
            LookupError: If the selected controller is not part of the available options.

        Example:

        .. code-block:: python

            >>> controller = selectable_controller.get_controller(controller_selection)
        """
        if controller_selection not in self._controller_options:
            raise LookupError("The selected controller is not a part of the available options.")
        return self._controller_options[controller_selection]


class CombinedController(BaseController):
    """Controller class which runs multiple controllers in parallel.

    This controller runs each controller in parallel, and returns the desired state of the robot
    as a combination of the desired states of the individual controllers.

    Args:
        controllers: Controllers to run in parallel.

    Raises:
        ValueError: If fewer than two controllers are provided.

    Example:

    .. code-block:: python

        >>> combined_controller = CombinedController(controllers)
    """

    def __init__(self, controllers: list[BaseController]) -> None:
        if len(controllers) < 2:
            raise ValueError("CombinedController must have at least two controllers.")
        self._controllers = controllers

    def reset(self, estimated_state: RobotState, setpoint_state: RobotState | None, t: float, **kwargs: object) -> bool:
        """Reset all contained controllers.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional setpoint state of the robot.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments for the controllers.

        Returns:
            True if all resets succeed, False otherwise.

        Example:

        .. code-block:: python

            >>> ok = combined_controller.reset(estimated_state, setpoint_state, t)
            >>> if not ok:
            ...     raise RuntimeError("Parallel reset failed")
        """
        for controller in self._controllers:
            if not controller.reset(estimated_state, setpoint_state, t, **kwargs):
                return False
        return True

    def forward(
        self, estimated_state: RobotState, setpoint_state: RobotState | None, t: float, **kwargs: object
    ) -> RobotState | None:
        """Run all controllers and combine their outputs.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional setpoint state of the robot.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments for the controllers.

        Returns:
            Combined robot state, or None if any controller fails to produce an output
            that can be combined (for example, multiple controllers writing to the same part
            of the state, or a controller failing to produce a valid output).

        Example:

        .. code-block:: python

            >>> desired = combined_controller.forward(estimated_state, setpoint_state, t)
            >>> if desired is None:
            ...     print("Parallel controller failed")
        """
        out_state = RobotState()
        for controller_index, controller in enumerate(self._controllers):
            out_state = combine_robot_states(
                out_state, controller.forward(estimated_state, setpoint_state, t, **kwargs)
            )
            if out_state is None:
                log.warning(
                    f"CombinedController: failed to combine output from controller at index {controller_index} "
                    "(child returned None or state conflict).",
                    capture_source=True,
                )
                return None
        return out_state


class ChainedController(BaseController):
    """Controller class which runs multiple controllers in a chained-sequence.

    This controller runs each controller in sequence, chaining the desired state of the previous controller
    as the setpoint to the next controller. The output of the final controller is returned.

    Args:
        controllers: Controllers to run in sequence.

    Raises:
        ValueError: If fewer than two controllers are provided.

    Example:

    .. code-block:: python

        >>> chained_controller = ChainedController(controllers)
    """

    def __init__(self, controllers: list[BaseController]) -> None:
        if len(controllers) < 2:
            raise ValueError("ChainedController must have at least two controllers.")
        self._controllers = controllers

    def reset(self, estimated_state: RobotState, setpoint_state: RobotState | None, t: float, **kwargs: object) -> bool:
        """Reset all contained controllers.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional setpoint state of the robot.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments for the controllers.

        Returns:
            True if all resets succeed, False otherwise.

        Example:

        .. code-block:: python

            >>> ok = chained_controller.reset(estimated_state, setpoint_state, t)
            >>> if not ok:
            ...     raise RuntimeError("Chained reset failed")
        """
        for controller in self._controllers:
            if not controller.reset(estimated_state, setpoint_state, t, **kwargs):
                return False
        return True

    def forward(
        self, estimated_state: RobotState, setpoint_state: RobotState | None, t: float, **kwargs: object
    ) -> RobotState | None:
        """Run controllers sequentially, passing outputs as setpoints.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Initial setpoint state for the sequence.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments for the controllers.

        Returns:
            Robot state produced by the final controller, or None if any controller fails.

        Example:

        .. code-block:: python

            >>> desired = chained_controller.forward(estimated_state, setpoint_state, t)
            >>> if desired is None:
            ...     print("Chained controller failed")
        """
        for controller in self._controllers:
            # Every controller gets access to the estimated state, time and kwargs.
            # The output of one controller is treated as the setpoint to the next controller.
            setpoint_state = controller.forward(estimated_state, setpoint_state, t, **kwargs)
            if setpoint_state is None:
                return None
        return setpoint_state

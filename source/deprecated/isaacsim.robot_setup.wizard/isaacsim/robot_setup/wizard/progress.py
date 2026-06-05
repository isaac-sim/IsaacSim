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

"""Progress tracking and state management for the robot setup wizard."""

from enum import Enum
from typing import Any

from .utils.utils import Singleton


class ProgressColorState(Enum):
    """Enumeration defining the visual states for progress tracking in the robot setup wizard.

    This enum provides standardized state values used to indicate the current status of steps
    in a multi-step process, with each state corresponding to a different visual representation
    in the user interface.
    """

    REMAINING = 0
    """Indicates a step that has not yet been started."""
    IN_PROGRESS = 1
    """Indicates a step that is currently being processed."""
    ACTIVE = 2
    """Indicates the currently active step in the progress sequence."""
    COMPLETE = 3
    """Indicates a step that has been finished successfully."""


def Singleton(class_: type) -> None:  # noqa: N802
    """A singleton decorator that ensures only one instance of a class exists.

    When applied to a class, this decorator maintains a single instance across all instantiation attempts.
    Subsequent calls to create the class return the same instance.

    Args:
        class_: The class to be made singleton.

    Returns:
        A wrapper function that returns the singleton instance of the class.
    """
    instances = {}

    def getinstance(*args: Any, **kwargs: Any) -> None:
        if class_ not in instances:
            instances[class_] = class_(*args, **kwargs)
        return instances[class_]

    return getinstance


@Singleton
class ProgressRegistry:
    """A singleton registry for tracking and managing step-based progress states.

    This class provides a centralized system for managing progress across multiple steps in a workflow.
    It maintains step states, notifies subscribers of progress changes, and tracks the currently active step.
    Being a singleton, only one instance exists throughout the application lifecycle.

    The registry supports four progress states: REMAINING, IN_PROGRESS, ACTIVE, and COMPLETE.
    Subscribers can register callbacks to receive notifications when step progress changes.
    """

    class _Event(set):
        """A list of callable objects. Calling an instance of this will cause a.

        call to each item in the list in ascending order by index.
        """

        def __call__(self, *args: object, **kwargs: object) -> None:
            """Called when the instance is "called" as a function.

            Calls all saved functions with the provided arguments.

            Args:
                *args: Variable length argument list passed to each callback function.
                **kwargs: Arbitrary keyword arguments passed to each callback function.
            """
            # Call all the saved functions
            for f in self:
                f(*args, **kwargs)

        def __repr__(self) -> str:
            """Called by the repr() built-in function to compute the "official" string representation of an object.

            Returns:
                String representation of the Event object.
            """
            return f"Event({set.__repr__(self)})"

    class _EventSubscription:
        """Event subscription.

        _Event has callback while this object exists.

        Args:
            event: The event object to subscribe to.
            fn: The callback function to register with the event.
        """

        def __init__(self, event: object, fn: callable) -> None:
            self._fn = fn
            self._event = event
            event.add(self._fn)

        def __del__(self) -> None:
            """Called by GC.

            Removes the callback function from the event when the subscription is garbage collected.
            """
            self._event.remove(self._fn)

    def __init__(self) -> None:
        # TODO: begin_edit/end_edit
        self.__on_progress_changed = self._Event()
        self._steps_dict = {}

    def destroy(self) -> None:
        """Called to cancel current search."""

    def set_steps(self, steps: dict) -> None:
        """Set the steps.

        Args:
            steps: The steps to set in the registry.
        """
        self._steps_dict = steps

    def get_progress_by_name(self, step_name: str) -> None:
        """Get the steps.

        Args:
            step_name: Name of the step to retrieve.

        Returns:
            The progress state for the specified step name, or None if not found.
        """
        if step_name in self._steps_dict:
            return self._steps_dict[step_name]
        return None

    def _progress_changed(self, step_name: str, state: object) -> None:
        """Call the event object that has the list of functions.

        Args:
            step_name: Name of the step that changed.
            state: New progress state of the step.
        """
        self.__on_progress_changed(step_name, state)

    def subscribe_progress_changed(self, fn: callable) -> None:
        """Return the object that will automatically unsubscribe when destroyed.

        Args:
            fn: Callback function to subscribe to progress changes.

        Returns:
            Event subscription object that unsubscribes when destroyed.
        """
        return self._EventSubscription(self.__on_progress_changed, fn)

    def set_step_progress(self, step_name: str, state: object) -> None:
        """Updates the progress state of a step and triggers change notifications.

        Args:
            step_name: Name of the step to update.
            state: New progress state to set for the step.
        """
        if step_name in self._steps_dict:
            if self._steps_dict[step_name] != state:
                self._steps_dict[step_name] = state
                self._progress_changed(step_name, state)

    def get_active_step(self) -> None:
        """Return the name of the currently active step.

        Returns:
            Name of the step with ACTIVE state, or None if no step is active.
        """
        for step_name, state in self._steps_dict.items():
            if state == ProgressColorState.ACTIVE:
                return step_name
        return None

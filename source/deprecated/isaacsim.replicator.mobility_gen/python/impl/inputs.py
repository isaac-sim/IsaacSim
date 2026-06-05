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

"""Input modules for capturing keyboard and gamepad inputs in mobility generation scenarios."""

import carb
import numpy as np
import omni

from .common import Buffer, Module

# =========================================================
#  IMPLEMENTATION
# =========================================================


class KeyboardButton:
    """Represents a keyboard button that tracks press and release states.

    This class monitors a specific keyboard key and maintains its current state (pressed or released).
    It responds to keyboard events and updates its internal value accordingly. The button is considered
    pressed during KEY_PRESS and KEY_REPEAT events, and released during KEY_RELEASE events.

    Args:
        key: The keyboard input key to monitor for press and release events.
    """

    def __init__(self, key: carb.input.KeyboardInput) -> None:
        self._key = key
        self._value = False

    @property
    def value(self) -> bool:
        """Current pressed state of the keyboard button.

        Returns:
            True if the button is currently pressed, False otherwise.
        """
        return self._value

    def _event_callback(self, event: carb.input.KeyboardEvent, *args: object, **kwargs: object) -> None:
        """Handle keyboard events to update the button state.

        Args:
            event: The keyboard event containing input information and event type.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.
        """
        if event.input == self._key:
            if (
                event.type == carb.input.KeyboardEventType.KEY_PRESS
                or event.type == carb.input.KeyboardEventType.KEY_REPEAT
            ):
                self._value = True
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                self._value = False


class KeyboardDriver(object):
    """Singleton driver for capturing keyboard input events in Isaac Sim.

    This class provides a centralized interface for monitoring WASD keyboard inputs commonly used for navigation
    and control in robotics applications. It implements the singleton pattern to ensure only one keyboard driver
    instance exists throughout the application lifecycle.

    The driver monitors four specific keys:
    - W: Forward movement
    - A: Left movement
    - S: Backward movement
    - D: Right movement

    Use the static methods to manage the driver lifecycle and access the singleton instance. The driver must be
    connected before it can capture keyboard events and disconnected when no longer needed to clean up resources.

    Raises:
        RuntimeError: If attempting to create multiple instances directly. Use KeyboardDriver.instance() instead.
    """

    _instance = None
    """Singleton instance of the KeyboardDriver class."""

    def __init__(self) -> None:
        if self._instance is not None:
            raise RuntimeError("Keyboard singleton already instantiated.  Please call Keyboard.instance() instead.")
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()

        key_input_types = [
            carb.input.KeyboardInput.W,
            carb.input.KeyboardInput.A,
            carb.input.KeyboardInput.S,
            carb.input.KeyboardInput.D,
        ]

        self.buttons = [KeyboardButton(key) for key in key_input_types]

    def _event_callback(self, event: carb.input.KeyboardEvent, *args: object, **kwargs: object) -> None:
        """Handle keyboard events and forward them to all keyboard buttons.

        Args:
            event: The keyboard event containing input information.
            *args: Variable length argument list.
            **kwargs: Additional keyword arguments.
        """
        for button in self.buttons:
            button._event_callback(event, *args, **kwargs)

    def _connect(self) -> None:
        """Subscribe to keyboard events from the application window's keyboard."""
        self._event_handle = self._input.subscribe_to_keyboard_events(self._keyboard, self._event_callback)

    def _disconnect(self) -> None:
        """Unsubscribes from keyboard events and clears the event handle."""
        self._input.unsubscribe_to_keyboard_events(self._keyboard, self._event_handle)
        self._event_handle = None

    @staticmethod
    def connect() -> "KeyboardDriver":
        """Create and connect the keyboard driver singleton instance.

        Returns:
            The connected KeyboardDriver instance.
        """
        instance = KeyboardDriver.instance()
        instance._connect()
        return instance

    @staticmethod
    def disconnect() -> None:
        """Disconnects the keyboard driver singleton instance if it exists."""
        if KeyboardDriver._instance is None:
            return
        KeyboardDriver.instance()._disconnect()

    @staticmethod
    def instance() -> "KeyboardDriver":
        """Get or create the singleton KeyboardDriver instance.

        Returns:
            The KeyboardDriver singleton instance.
        """
        if KeyboardDriver._instance is None:
            KeyboardDriver._instance = KeyboardDriver()
        return KeyboardDriver._instance

    def get_button_values(self) -> np.ndarray:
        """Get the current state of all keyboard buttons.

        Returns:
            Array of boolean values representing the pressed state of each button.
        """
        return np.array([b.value for b in self.buttons])


class GamepadAxis:
    """Represents a single axis of a gamepad controller.

    This class manages a bidirectional gamepad axis by combining two opposing inputs (positive and negative)
    into a single axis value. It applies deadzone filtering to eliminate small unintentional movements and
    provides the current axis value through the value property.

    The axis value ranges from -1.0 to 1.0, where positive values indicate input from the positive direction
    and negative values indicate input from the negative direction. Values within the deadzone threshold
    are filtered out and reported as 0.0.

    Args:
        gamepad: The parent gamepad driver instance that manages this axis.
        carb_pos_input: The Carbonite input for the positive direction of the axis.
        carb_neg_input: The Carbonite input for the negative direction of the axis.
        deadzone: Threshold below which input values are ignored to prevent drift.
    """

    def __init__(
        self,
        gamepad: "Gamepad",
        carb_pos_input: carb.input.GamepadInput,
        carb_neg_input: carb.input.GamepadInput,
        deadzone: bool = 0.01,
    ) -> None:
        self.carb_pos_input = carb_pos_input
        self.carb_neg_input = carb_neg_input
        self.deadzone = deadzone
        self._gamepad = gamepad
        self._pos_val = 0.0
        self._neg_val = 0.0

    @property
    def value(self) -> float:
        """Current axis value with deadzone applied.

        Returns:
            The axis value ranging from -1.0 to 1.0, or 0.0 if within the deadzone.
        """
        if self._pos_val > self._neg_val:
            return self._pos_val if self._pos_val > self.deadzone else 0.0
        else:
            return -self._neg_val if self._neg_val > self.deadzone else 0.0

    def _event_callback(self, event: carb.input.GamepadEvent, *args: object, **kwargs: object) -> None:
        """Handle gamepad input events to update axis values.

        Args:
            event: The gamepad input event containing axis value data.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.
        """
        cur_val = event.value
        if event.input == self.carb_pos_input:
            self._pos_val = cur_val
        if event.input == self.carb_neg_input:
            self._neg_val = cur_val


class GamepadDriver(object):
    """Singleton driver for gamepad input handling in Isaac Sim.

    This class provides a centralized interface for capturing and processing gamepad input events.
    It implements a singleton pattern to ensure only one instance manages gamepad connectivity and
    event handling throughout the application.

    The driver automatically configures four analog stick axes:
    - Left stick vertical (up/down)
    - Left stick horizontal (left/right)
    - Right stick vertical (up/down)
    - Right stick horizontal (left/right)

    Each axis includes deadzone filtering to eliminate noise from slight stick movements.
    The driver connects to the first available gamepad (index 0) and subscribes to gamepad
    events through the Carbonite input interface.

    Use the static methods to manage the driver lifecycle:
    - ``connect()`` to establish gamepad event subscription
    - ``disconnect()`` to clean up event handlers
    - ``instance()`` to access the singleton instance

    The driver provides axis values as a NumPy array through ``get_axis_values()``, with
    positive values indicating up/right directions and negative values indicating down/left
    directions. Button values are available through ``get_button_values()`` which returns
    an empty array in the current implementation.

    Raises:
        RuntimeError: If attempting to create multiple instances directly instead of using
            the singleton pattern.
    """

    _instance = None
    """Singleton instance of the GamepadDriver class."""

    def __init__(self) -> None:
        if self._instance is not None:
            raise RuntimeError("Gamepad singleton already instantiated.  Please call Gamepad.instance() instead.")
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._gamepad = self._appwindow.get_gamepad(0)
        self.axes = [
            GamepadAxis(
                gamepad=self,
                carb_pos_input=carb.input.GamepadInput.LEFT_STICK_UP,
                carb_neg_input=carb.input.GamepadInput.LEFT_STICK_DOWN,
            ),
            GamepadAxis(
                gamepad=self,
                carb_pos_input=carb.input.GamepadInput.LEFT_STICK_RIGHT,
                carb_neg_input=carb.input.GamepadInput.LEFT_STICK_LEFT,
            ),
            GamepadAxis(
                gamepad=self,
                carb_pos_input=carb.input.GamepadInput.RIGHT_STICK_UP,
                carb_neg_input=carb.input.GamepadInput.RIGHT_STICK_DOWN,
            ),
            GamepadAxis(
                gamepad=self,
                carb_pos_input=carb.input.GamepadInput.RIGHT_STICK_RIGHT,
                carb_neg_input=carb.input.GamepadInput.RIGHT_STICK_LEFT,
            ),
        ]

    def _event_callback(self, event: carb.input.GamepadEvent, *args: object, **kwargs: object) -> None:
        """Handle gamepad events and update axis values.

        Args:
            event: The gamepad event containing input and value data.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.
        """
        for axis in self.axes:
            axis._event_callback(event, *args, **kwargs)
        # carb.log_warn(f"{self.axes[0].value}, {self.axes[1].value}, {self.axes[2].value}, {self.axes[3].value}")

    def _connect(self) -> None:
        """Subscribe to gamepad events to start receiving input data."""
        self._event_handle = self._input.subscribe_to_gamepad_events(self._gamepad, self._event_callback)

    def _disconnect(self) -> None:
        """Unsubscribes from gamepad events and cleans up the event handle."""
        self._input.unsubscribe_to_gamepad_events(self._gamepad, self._event_handle)
        self._event_handle = None

    @staticmethod
    def connect() -> "GamepadDriver":
        """Create and connect a gamepad driver instance.

        Returns:
            The connected GamepadDriver instance.
        """
        instance = GamepadDriver.instance()
        instance._connect()
        return instance

    @staticmethod
    def disconnect() -> None:
        """Disconnects the gamepad driver instance if it exists."""
        if GamepadDriver._instance is None:
            return
        GamepadDriver.instance()._disconnect()

    @staticmethod
    def instance() -> "GamepadDriver":
        """Get the singleton GamepadDriver instance, creating it if needed.

        Returns:
            The GamepadDriver singleton instance.
        """
        if GamepadDriver._instance is None:
            GamepadDriver._instance = GamepadDriver()
        return GamepadDriver._instance

    def get_axis_values(self) -> np.ndarray:
        """Current values of all gamepad axes.

        Returns:
            Array containing axis values for left stick vertical, left stick horizontal, right stick vertical, and right stick horizontal.
        """
        return np.array([axis.value for axis in self.axes])

    def get_button_values(self) -> np.ndarray:
        """Current values of all gamepad buttons.

        Returns:
            Empty array as no button inputs are currently implemented.
        """
        return np.ndarray([])


# =========================================================
#  MODULES
# =========================================================


class Keyboard(Module):
    """A keyboard input module for capturing WASD key presses in mobility generation scenarios.

    This module provides a high-level interface for accessing keyboard input states, specifically monitoring
    the W, A, S, and D keys commonly used for movement control. It integrates with the underlying keyboard
    driver system to capture key press and release events, storing the current state of each monitored key
    in a buffer for easy access.

    The module automatically manages the keyboard driver instance and provides real-time updates of button
    states through its buffer system. Key states are represented as boolean values where True indicates
    a key is currently pressed (including key repeat events) and False indicates the key is released.

    The keyboard input data is accessible through the ``buttons`` buffer attribute, which contains an array
    of boolean values corresponding to the W, A, S, D keys in that order. This makes it suitable for
    integration with mobility generation systems that require directional input commands.

    The module follows the standard Module interface, requiring periodic calls to ``update_state()`` to
    refresh the input data and maintain synchronization with the underlying input system.
    """

    def __init__(self) -> None:
        self._keyboard = KeyboardDriver.instance()
        self.buttons = Buffer()

    def update_state(self) -> None:
        """Update the current state of keyboard button values.

        Retrieves the latest button states from the keyboard driver and stores them in the internal buffer.

        Returns:
            The result of the parent class's update_state method.
        """
        self.buttons.set_value(self._keyboard.get_button_values())
        return super().update_state()


class Gamepad(Module):
    """A module for capturing and managing gamepad input data.

    This module provides access to gamepad controller input through a standardized interface. It tracks both analog
    stick axes and button states, making gamepad data available for use in simulations, training scenarios, or
    interactive applications.

    The module automatically connects to the first available gamepad (index 0) and monitors four analog axes:
    left stick vertical (up/down), left stick horizontal (left/right), right stick vertical (up/down), and right
    stick horizontal (left/right). Axis values are normalized with configurable deadzone handling to filter out
    controller drift.

    Gamepad input data is stored in two main buffers:

    - ``buttons``: A Buffer containing button press states
    - ``axes``: A Buffer containing analog stick position values

    The module follows a singleton pattern through the underlying GamepadDriver, ensuring consistent gamepad state
    across the application. Input values are updated each time ``update_state()`` is called, capturing the current
    gamepad state at that moment.

    This module is particularly useful for creating human-in-the-loop training scenarios, manual control interfaces,
    or data collection workflows where human input needs to be captured alongside simulation data.
    """

    def __init__(self) -> None:
        self._gamepad = GamepadDriver.instance()
        self.buttons = Buffer()
        self.axes = Buffer()

    def update_state(self) -> None:
        """Update the gamepad state by refreshing button and axis values from the gamepad driver.

        Returns:
            The result of the parent class update_state method.
        """
        self.buttons.set_value(self._gamepad.get_button_values())
        self.axes.set_value(self._gamepad.get_axis_values())
        return super().update_state()

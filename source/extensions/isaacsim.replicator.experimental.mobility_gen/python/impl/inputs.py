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

"""Keyboard and gamepad input modules for robot teleoperation."""

import carb
import numpy as np
import omni

from .common import Buffer, Module

# =========================================================
#  IMPLEMENTATION
# =========================================================


class KeyboardButton:
    """A single keyboard button tracker that monitors press, repeat, and release events.

    Args:
        key: The carb keyboard input to track.
    """

    def __init__(self, key: carb.input.KeyboardInput) -> None:
        self._key = key
        self._value = False

    @property
    def value(self) -> bool:
        """Return True if the button is currently pressed."""
        return self._value

    def _event_callback(self, event: carb.input.KeyboardEvent, *args: object, **kwargs: object) -> bool:
        # Only consume events for this button's key and the press/repeat/release types we
        # actually act on; otherwise return False so Kit can keep propagating the event
        # (e.g. to UI text fields).
        if event.input != self._key:
            return False
        if (
            event.type == carb.input.KeyboardEventType.KEY_PRESS
            or event.type == carb.input.KeyboardEventType.KEY_REPEAT
        ):
            self._value = True
            return True
        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._value = False
            return True
        return False


class KeyboardDriver(object):
    """A singleton driver that subscribes to keyboard events and tracks button states."""

    _instance = None

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

    def _event_callback(self, event: carb.input.KeyboardEvent, *args: object, **kwargs: object) -> bool:
        # Consume the event only if one of the tracked WASD buttons handled it; otherwise
        # return False so Kit continues propagating to other listeners and UI widgets.
        consumed = False
        for button in self.buttons:
            if button._event_callback(event, *args, **kwargs):
                consumed = True
        return consumed

    def _connect(self) -> None:
        self._event_handle = self._input.subscribe_to_keyboard_events(self._keyboard, self._event_callback)

    def _disconnect(self) -> None:
        self._input.unsubscribe_to_keyboard_events(self._keyboard, self._event_handle)
        self._event_handle = None

    @staticmethod
    def connect() -> "KeyboardDriver":
        """Connect the keyboard driver and return the singleton instance.

        Returns:
            The connected singleton keyboard driver.
        """
        instance = KeyboardDriver.instance()
        instance._connect()
        return instance

    @staticmethod
    def disconnect() -> None:
        """Disconnect the keyboard driver if it is connected."""
        if KeyboardDriver._instance is None:
            return
        KeyboardDriver.instance()._disconnect()
        KeyboardDriver._instance = None

    @staticmethod
    def instance() -> "KeyboardDriver":
        """Return the singleton KeyboardDriver instance, creating it if needed.

        Returns:
            The singleton keyboard driver.
        """
        if KeyboardDriver._instance is None:
            KeyboardDriver._instance = KeyboardDriver()
        return KeyboardDriver._instance

    def get_button_values(self) -> np.ndarray:
        """Return a boolean array of current button states.

        Returns:
            Boolean array of current button states.
        """
        return np.array([b.value for b in self.buttons])


class GamepadAxis:
    """A single gamepad axis tracker using positive and negative carb inputs.

    Args:
        gamepad: The parent Gamepad driver instance.
        carb_pos_input: The carb input for the positive axis direction.
        carb_neg_input: The carb input for the negative axis direction.
        deadzone: The deadzone threshold below which the axis reads as zero.
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
        """Return the current signed axis value, applying the deadzone."""
        if self._pos_val > self._neg_val:
            return self._pos_val if self._pos_val > self.deadzone else 0.0
        else:
            return -self._neg_val if self._neg_val > self.deadzone else 0.0

    def _event_callback(self, event: carb.input.GamepadEvent, *args: object, **kwargs: object) -> None:
        cur_val = event.value
        if event.input == self.carb_pos_input:
            self._pos_val = cur_val
        if event.input == self.carb_neg_input:
            self._neg_val = cur_val


class GamepadDriver(object):
    """A singleton driver that subscribes to gamepad events and tracks axis states."""

    _instance = None

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
        for axis in self.axes:
            axis._event_callback(event, *args, **kwargs)
        # carb.log_warn(f"{self.axes[0].value}, {self.axes[1].value}, {self.axes[2].value}, {self.axes[3].value}")

    def _connect(self) -> None:
        self._event_handle = self._input.subscribe_to_gamepad_events(self._gamepad, self._event_callback)

    def _disconnect(self) -> None:
        self._input.unsubscribe_to_gamepad_events(self._gamepad, self._event_handle)
        self._event_handle = None

    @staticmethod
    def connect() -> "GamepadDriver":
        """Connect the gamepad driver and return the singleton instance.

        Returns:
            The connected singleton gamepad driver.
        """
        instance = GamepadDriver.instance()
        instance._connect()
        return instance

    @staticmethod
    def disconnect() -> None:
        """Disconnect the gamepad driver if it is connected."""
        if GamepadDriver._instance is None:
            return
        GamepadDriver.instance()._disconnect()
        GamepadDriver._instance = None

    @staticmethod
    def instance() -> "GamepadDriver":
        """Return the singleton GamepadDriver instance, creating it if needed.

        Returns:
            The singleton gamepad driver.
        """
        if GamepadDriver._instance is None:
            GamepadDriver._instance = GamepadDriver()
        return GamepadDriver._instance

    def get_axis_values(self) -> np.ndarray:
        """Return an array of current axis values.

        Returns:
            Array of current axis values.
        """
        return np.array([axis.value for axis in self.axes])

    def get_button_values(self) -> np.ndarray:
        """Return an array of current button values.

        Returns:
            Empty array because gamepad button tracking is not implemented.
        """
        return np.array([])


# =========================================================
#  MODULES
# =========================================================


class Keyboard(Module):
    """A module that exposes keyboard button states as a Buffer."""

    def __init__(self) -> None:
        self._keyboard = KeyboardDriver.instance()
        self.buttons = Buffer()

    def update_state(self) -> None:
        """Update the buttons buffer with the current keyboard state."""
        self.buttons.set_value(self._keyboard.get_button_values())
        return super().update_state()


class Gamepad(Module):
    """A module that exposes gamepad axis and button states as Buffers."""

    def __init__(self) -> None:
        self._gamepad = GamepadDriver.instance()
        self.buttons = Buffer()
        self.axes = Buffer()

    def update_state(self) -> None:
        """Update the buttons and axes buffers with the current gamepad state."""
        self.buttons.set_value(self._gamepad.get_button_values())
        self.axes.set_value(self._gamepad.get_axis_values())
        return super().update_state()

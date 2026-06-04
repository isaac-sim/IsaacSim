# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Scene management for sensor icon display and manipulation in the viewport."""

__all__ = ["IconScene"]

import carb.eventdispatcher
import carb.input
import carb.settings
import omni.timeline
from omni.ui import scene as sc

from .manipulator import IconManipulator, PreventOthers
from .model import IconModel

VISIBLE_SETTING = "/persistent/exts/isaacsim.gui.sensors.icon/visible_on_startup"


class IconScene:  # pragma: no cover
    """The window with the manipulator.

    Args:
        title: The window title.
        icon_scale: The scale factor for icons.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, title: str | None = None, icon_scale: float = 1.0, **kwargs: object) -> None:
        self._sensor_icon = SensorIcon.get_instance()
        self._manipulater = IconManipulator(
            model=self._sensor_icon.get_model(),
            aspect_ratio_policy=sc.AspectRatioPolicy.PRESERVE_ASPECT_HORIZONTAL,
            icon_scale=icon_scale,
        )
        prevent_others = PreventOthers()

        # we don't want the prim icon show outside the viewport area
        # so we need do check on end drag,
        # don't check it on change(dragging) to get a better performance
        right_drag_gesture = sc.DragGesture(
            name="SensorIcon_look_drag",
            on_ended_fn=lambda sender: self._end_drag(sender),
            mouse_button=1,  # hook right button
            manager=prevent_others,
        )

        left_drag_gesture = sc.DragGesture(
            name="SensorIcon_navigation_drag",  # naigation bar use left button to drag
            on_ended_fn=lambda sender: self._end_drag(sender),
            mouse_button=0,  # hook left button
            manager=prevent_others,
        )

        # TODO: This would cause crash when click the prim icon,
        # note it until find a better solution
        # self._screen = sc.Screen(gestures=[
        #     right_drag_gesture,
        #     left_drag_gesture
        # ])
        self.visible = True

    def _end_drag(self, sender: object) -> None:
        """Handles the end of a drag gesture on sensor icons.

        Rebuilds the icons with position checking to ensure they remain within the viewport area.

        Args:
            sender: The gesture sender that triggered the drag end event.
        """
        self._manipulater.rebuild_icons(need_check=True)

    @property
    def visible(self) -> bool:
        """Visibility state of the sensor icon manipulator.

        Returns:
            True if the manipulator is visible, False otherwise.
        """
        return self._manipulater.visible

    @visible.setter
    def visible(self, value: bool) -> None:
        value = bool(value)
        # self._model.beam_visible = value
        self._manipulater.visible = value

    def destroy(self) -> None:
        """Destroys the icon scene and releases all resources.

        Clears all icons and sets the manipulator to None to prevent further operations.
        """
        self.clear()
        self._manipulater = None

    def clear(self) -> None:
        """Clears all icons from the manipulator.

        Removes all sensor icons currently displayed in the scene without destroying the manipulator itself.
        """
        if not self._manipulater:
            return
        self._manipulater.clear()

    def __del__(self) -> None:
        """Destructor that ensures proper cleanup when the object is garbage collected.

        Calls destroy() to release all resources and prevent memory leaks.
        """
        self.destroy()


class SensorIcon:
    """Singleton class for managing sensor icons in the viewport.

    Provides functionality to add, remove, show, and hide sensor icons that are displayed as overlay graphics
    in the 3D viewport. Icons are positioned at sensor prim locations and can be clicked to trigger custom
    callbacks. The class manages global visibility settings and responds to timeline events to refresh icon
    visuals.

    Key features:
    - Add/remove sensor icons at specific prim paths
    - Show/hide individual or all sensor icons
    - Set click callbacks for interactive sensor icons
    - Automatic refresh of icon visuals on timeline stop
    - Persistent visibility settings via carb.settings

    Args:
        test: Whether to initialize in test mode.
    """

    _instance = None
    """Singleton instance of the SensorIcon class."""

    def __init__(self, test: bool = False) -> None:
        self.model = IconModel()
        self._settings = carb.settings.get_settings()
        self._visible_sub = self._settings.subscribe_to_node_change_events(VISIBLE_SETTING, self._on_visible_changed)
        self.toggle_all_fn = []
        self._timeline = omni.timeline.get_timeline_interface()

        if self._settings.get(VISIBLE_SETTING) is False:
            self.model.hide_all()

        self.timeline_event_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._on_timeline_stop,
            observer_name="isaacsim.gui.sensors.icon.SensorIcon._on_timeline_stop",
        )

    def _on_timeline_stop(self, event: object) -> None:
        """Timeline stop event callback - refresh icon visuals.

        Args:
            event: The timeline stop event.
        """
        if self.model:
            self.model.refresh_all_icon_visuals()

    def _on_visible_changed(self, *args: object) -> None:
        """Handles visibility setting changes for sensor icons.

        Args:
            *args: Variable arguments passed from the settings change event.
        """
        if not self.model:
            return

        visible = self._settings.get(VISIBLE_SETTING)
        if visible:
            self.model.show_all()
        else:
            self.model.hide_all()
        for fn in self.toggle_all_fn:
            fn(visible)

    def register_toggle_all_fn(self, fn: callable) -> None:
        """Registers a callback function to be called when all sensor icons are toggled.

        Args:
            fn: Callback function to register for toggle all events.
        """
        self.toggle_all_fn.append(fn)

    def unregister_toggle_all_fn(self, fn: callable) -> None:
        """Unregisters a previously registered toggle all callback function.

        Args:
            fn: Callback function to unregister from toggle all events.
        """
        self.toggle_all_fn.remove(fn)

    @staticmethod
    def get_instance() -> "SensorIcon":
        """Gets the singleton instance of SensorIcon.

        Returns:
            The singleton SensorIcon instance.
        """
        if not SensorIcon._instance:
            SensorIcon._instance = SensorIcon()
        return SensorIcon._instance

    def destroy(self) -> None:
        """Destroys the SensorIcon instance and cleans up resources."""
        self.timeline_event_sub = None
        self.clear()
        self.model = None
        SensorIcon._instance = None

    def get_model(self) -> IconModel | None:
        """The IconModel instance used by this SensorIcon.

        Returns:
            The IconModel instance.
        """
        return self.model

    def add_sensor_icon(self, prim_path: str, icon_url: str | None = None) -> None:
        """Adds a sensor icon for the specified prim path.

        Args:
            prim_path: The USD prim path to add the sensor icon for.
            icon_url: Optional URL for the icon image.
        """
        if not self.model:
            return
        self.model.add_sensor_icon(prim_path, icon_url)

    def remove_sensor_icon(self, prim_path: str) -> None:
        """Removes the sensor icon for the specified prim path.

        Args:
            prim_path: The USD prim path to remove the sensor icon from.
        """
        if not self.model:
            return
        self.model.remove_sensor_icon(prim_path)

    def set_icon_click_fn(self, prim_path: str, call_back: callable) -> None:
        """Sets a callback function to be called when the sensor icon is clicked.

        Args:
            prim_path: The USD prim path of the sensor icon.
            call_back: Callback function to execute when the icon is clicked.
        """
        if not self.model:
            return
        self.model.set_icon_click_fn(prim_path, call_back)

    def show_sensor_icon(self, prim_path: str) -> None:
        """Shows the sensor icon for the specified prim.

        Args:
            prim_path: Path to the prim whose sensor icon should be shown.
        """
        if not self.model:
            return
        self.model.show_sensor_icon(prim_path)

    def hide_sensor_icon(self, prim_path: str) -> None:
        """Hides the sensor icon for the specified prim.

        Args:
            prim_path: Path to the prim whose sensor icon should be hidden.
        """
        if not self.model:
            return
        self.model.hide_sensor_icon(prim_path)

    def clear(self) -> None:
        """Clears all sensor icons from the model."""
        if not self.model:
            return
        self.model.clear()

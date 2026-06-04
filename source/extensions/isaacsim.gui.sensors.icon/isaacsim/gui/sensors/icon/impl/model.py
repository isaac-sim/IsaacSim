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

"""Sensor icon model for viewport scene management."""

__all__ = ["IconModel"]

from collections.abc import Callable
from pathlib import Path

import carb
import omni.kit.app
import omni.usd
import usdrt.Usd
from omni.ui import scene as sc
from pxr import Gf, Sdf, Trace, Usd, UsdGeom


class IconModel(sc.AbstractManipulatorModel):
    """Manages sensor icons within the viewport scene as manipulator items.

    This class automatically detects and visualizes sensor prims in the USD stage as interactive icons.
    It monitors various sensor types including OmniLidar, IsaacContactSensor, IsaacImuSensor,
    IsaacRaycastSensor, and deprecated types (Lidar, IsaacLightBeamSensor, Generic). The icons
    are dynamically updated based on prim visibility, activation status, and stage changes.

    The model maintains a persistent connection to both USD and USDRT stages for efficient querying
    and real-time updates. Icons are positioned based on the sensor prim's world transform and respond
    to stage events such as opening, closing, and frame updates.

    Icon visibility is controlled by both the prim's USD visibility attribute and activation state.
    The model provides methods to show, hide, add, and remove sensor icons programmatically, as well
    as set click callbacks for interactive behavior.
    """

    SENSOR_TYPES = [
        "OmniLidar",
        "IsaacContactSensor",
        "IsaacImuSensor",
        "IsaacRaycastSensor",
        # DEPRECATED types (kept for backward compatibility with existing scenes)
        "Lidar",
        "IsaacLightBeamSensor",
        "Generic",
    ]
    """List of sensor type names that the icon model recognizes and displays icons for."""

    class IconItem(sc.AbstractManipulatorItem):
        """Represents a sensor icon item in the 3D viewport.

        This class encapsulates the visual representation of a sensor prim in Isaac Sim's 3D scene. Each IconItem
        corresponds to a specific sensor prim and displays an icon at the sensor's location in the viewport. The
        item tracks its visibility state, click handlers, and removal status for proper UI management.

        Args:
            prim_path: The USD prim path of the sensor.
            icon_url: The file path or URL to the icon image to display for this sensor.
        """

        def __init__(self, prim_path: object, icon_url: str) -> None:
            super().__init__()
            self.icon_url = icon_url
            self.prim_path = prim_path
            self.on_click = None
            self.removed = False
            self.visible = True

    def __init__(self) -> None:
        super().__init__()
        self._usd_listening_active = True
        self._sensor_icon_dir = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        self._sensor_icon_path = str(Path(self._sensor_icon_dir).joinpath("icons/icoSensors.svg"))
        self._usd_context = omni.usd.get_context()
        self._usdrt_stage = None
        self._world_unit = 0.1
        self._icons = {}
        self._frame_sub = None
        # Persistent XformCache used across position queries
        self._xform_cache = None
        self._hidden_paths = set()

        bus = carb.eventdispatcher.get_eventdispatcher()

        self._stage_open_sub = bus.observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=self._on_stage_opened,
            observer_name="IconModel._on_stage_opened",
        )

        self._stage_close_sub = bus.observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.CLOSED),
            on_event=self._on_stage_closed,
            observer_name="IconModel._on_stage_closed",
        )

        self._connect_to_stage()

    def _connect_to_stage(self) -> None:
        """Connects the icon model to the current USD stage and initializes icon population."""
        stage = self._usd_context.get_stage()
        if stage:
            stage_id = self._usd_context.get_stage_id()
            try:
                self._usdrt_stage = usdrt.Usd.Stage.Attach(stage_id)
            except Exception as e:
                print(f"[Error] Failed to attach usdrt stage: {e}")
                self._usdrt_stage = None

            self._world_unit = UsdGeom.GetStageMetersPerUnit(stage)

            if self._world_unit == 0.0:
                self._world_unit = 0.1

            if self._usd_listening_active:
                self._frame_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                    on_event=self._on_frame_update,
                    observer_name="IsaacSensorIconGUI.__update_event_callback",
                )
                self._populate_initial_icons()
            else:
                if self._frame_sub:
                    self._frame_sub = None

                # populate icons with hidden state
                self._populate_initial_icons()
                for item in self._icons.values():
                    item.visible = False
                self._item_changed(None)
        else:
            self._usdrt_stage = None
            if self._frame_sub:
                self._frame_sub = None
            self.clear()

    @Trace.TraceFunction
    def _populate_initial_icons(self) -> None:
        """Populate icons by querying the USDrt stage. Skip if icon visibility is disabled."""
        # Do not populate any icons when global visibility is turned off.
        if not self._usd_listening_active:
            return

        self.clear()
        if not self._usdrt_stage:
            return

        stage = self._usd_context.get_stage()
        if not stage:
            return

        all_sensor_paths = set()
        for sensor_type_str in self.SENSOR_TYPES:
            try:
                paths = self._usdrt_stage.GetPrimsWithTypeName(sensor_type_str)
                all_sensor_paths.update(paths)
            except Exception as e:
                carb.log_warn(f"Failed querying usdrt for type {sensor_type_str}: {e}")

        for prim_path_obj in all_sensor_paths:
            prim_path = Sdf.Path(str(prim_path_obj))
            prim = stage.GetPrimAtPath(prim_path)
            if not prim:
                continue

            # Create icon for all sensors and compute initial visibility
            item = IconModel.IconItem(prim_path, self._sensor_icon_path)

            # Check visibility and activation status
            is_active = prim.IsActive()
            should_be_visible = False
            try:
                if prim.IsA(UsdGeom.Imageable):
                    visibility = UsdGeom.Imageable(prim).ComputeVisibility()
                    should_be_visible = is_active and visibility != UsdGeom.Tokens.invisible
                else:
                    should_be_visible = is_active
            except Exception as e:
                carb.log_warn(f"[Warning] Failed to compute visibility/activation for {prim_path}: {e}")

            item.visible = should_be_visible if self._usd_listening_active else False
            self._icons[prim_path] = item

        self._item_changed(None)

    def _on_stage_opened(self, event: object) -> None:
        """Handles stage opened events by reconnecting to the new stage.

        Args:
            event: The stage opened event.
        """
        self._connect_to_stage()

    def _on_stage_closed(self, event: object) -> None:
        """Handles stage closed events by clearing icons and disconnecting from the stage.

        Args:
            event: The stage closed event.
        """
        self.clear()
        self._usdrt_stage = None

    def get_world_unit(self) -> float:
        """World unit scale for the current stage.

        Returns:
            The world unit scale, with a minimum value of 0.1.
        """
        return max(self._world_unit, 0.1)

    def __del__(self) -> None:
        """Destructor that cleans up event subscriptions and destroys the model."""
        self._stage_open_sub = None
        self._stage_close_sub = None
        self._frame_sub = None
        self.destroy()

    def destroy(self) -> None:
        """Destroys the icon model and cleans up resources."""
        self._icons = {}
        if self._frame_sub:
            self._frame_sub = None
        self._hidden_paths.clear()

    def get_item(self, identifier: object) -> IconItem | None:
        """Icon item for the specified identifier.

        Args:
            identifier: Prim path as string or Sdf.Path to get the icon item for.

        Returns:
            IconItem | None: The IconItem instance, or None if not found.
        """
        if isinstance(identifier, str):
            identifier = Sdf.Path(identifier)
        return self._icons.get(identifier)

    def get_prim_paths(self) -> list[Sdf.Path]:
        """Prim paths of all icons in the model.

        Returns:
            List of prim paths for all tracked icons.
        """
        return list(self._icons.keys())

    @Trace.TraceFunction
    def get_position(self, prim_path: object) -> Gf.Vec3d | None:
        """World position of the icon at the specified prim path.

        Args:
            prim_path: Path to the prim to get the position for.

        Returns:
            Gf.Vec3d | None: The world position as a 3D vector, or None if the position cannot be computed.
        """
        if not isinstance(prim_path, Sdf.Path):
            prim_path = Sdf.Path(prim_path)

        if prim_path in self._icons:
            stage = self._usd_context.get_stage()
            if not stage:
                return None
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid():
                try:
                    # Use the persistent XformCache to avoid per-call allocation
                    if self._xform_cache is None:
                        self._xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
                    worldTransform = self._xform_cache.GetLocalToWorldTransform(prim)
                    translation = worldTransform.ExtractTranslation()
                    return Gf.Vec3d(translation[0], translation[1], translation[2])
                except Exception as e:
                    carb.log_warn(f"Failed to compute transform for {prim_path}: {e}")
                    return None
        return None

    def get_on_click(self, prim_path: object) -> Callable | None:
        """Gets the click callback function for a sensor icon.

        Args:
            prim_path: The USD prim path of the sensor.

        Returns:
            Callable | None: The click callback function if it exists, None otherwise.
        """
        if not isinstance(prim_path, Sdf.Path):
            prim_path = Sdf.Path(prim_path)
        item = self._icons.get(prim_path)
        return item.on_click if item else None

    def get_icon_url(self, prim_path: object) -> str:
        """Gets the icon URL for a sensor icon.

        Args:
            prim_path: The USD prim path of the sensor.

        Returns:
            str: The icon URL if the sensor exists, empty string otherwise.
        """
        if not isinstance(prim_path, Sdf.Path):
            prim_path = Sdf.Path(prim_path)
        item = self._icons.get(prim_path)
        return item.icon_url if item else ""

    @Trace.TraceFunction
    def _on_frame_update(self, e: object) -> None:
        """Updates sensor icon visibility and tracks sensor prims on each frame.

        This method is called on every frame update to synchronize the icon model with the current USD stage state.
        It detects new sensor prims, removes deleted ones, and updates visibility based on USD prim properties.

        Args:
            e: The frame update event.
        """
        # Clear the transform cache so position queries are up-to-date.
        self._xform_cache = None

        if not self._usd_listening_active or not self._usdrt_stage:
            return

        current_sensor_paths = set()
        for sensor_type in self.SENSOR_TYPES:
            try:
                paths = self._usdrt_stage.GetPrimsWithTypeName(sensor_type)
                current_sensor_paths.update(Sdf.Path(str(p)) for p in paths if p)
            except Exception as err:
                carb.log_warn(f"[SensorIcon] usdrt query failed for {sensor_type}: {err}")

        cached_paths = set(self._icons.keys())
        added_paths = current_sensor_paths - cached_paths
        removed_paths = cached_paths - current_sensor_paths

        for prim_path in added_paths:
            if prim_path not in self._hidden_paths:
                self.add_sensor_icon(prim_path)

        for prim_path in removed_paths:
            self.remove_sensor_icon(prim_path)
            self._hidden_paths.discard(prim_path)

        stage = self._usd_context.get_stage()
        if not stage:
            return

        for prim_path in current_sensor_paths:
            item = self._icons.get(prim_path)
            if not item:
                continue

            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                continue

            # Visibility check
            should_be_visible = item.visible
            try:
                if prim.IsA(UsdGeom.Imageable):
                    is_active = prim.IsActive()
                    visibility = UsdGeom.Imageable(prim).ComputeVisibility()
                    should_be_visible = is_active and visibility != UsdGeom.Tokens.invisible
            except Exception:
                pass

            visibility_changed = should_be_visible != item.visible
            if visibility_changed:
                item.visible = should_be_visible

            self._item_changed(item)

    def clear(self) -> None:
        """Clears all sensor icons from the model and notifies observers of the change."""
        if self._icons:
            self._icons = {}
            self._item_changed(None)

    def add_sensor_icon(self, prim_path: object, icon_url: str = None) -> None:
        """Adds a sensor icon to the model if the prim is a recognized sensor type.

        Args:
            prim_path: The USD prim path of the sensor.
            icon_url: The URL of the icon image. Uses default sensor icon if not provided.
        """
        # Skip registering icons when visibility is disabled
        if not self._usd_listening_active:
            return

        if not isinstance(prim_path, Sdf.Path):
            prim_path = Sdf.Path(prim_path)

        if prim_path in self._icons:
            return

        is_sensor = False
        if self._usdrt_stage:
            try:
                prim_type_token = self._usdrt_stage.GetPrimAtPath(str(prim_path)).GetTypeName()
                if str(prim_type_token) in self.SENSOR_TYPES:
                    is_sensor = True
            except Exception:
                pass

        if not is_sensor:
            stage = self._usd_context.get_stage()
            if stage:
                prim = stage.GetPrimAtPath(prim_path)
                if prim and str(prim.GetTypeName()) in self.SENSOR_TYPES:
                    is_sensor = True

        if is_sensor:
            icon_url = icon_url or self._sensor_icon_path
            item = IconModel.IconItem(prim_path, icon_url)

            # Check initial visibility and activation state
            should_be_visible = True  # Default to visible
            stage = self._usd_context.get_stage()
            if stage:
                prim = stage.GetPrimAtPath(prim_path)
                if prim:
                    try:
                        is_active = prim.IsActive()
                        # Use default timecode for initial check
                        visibility = UsdGeom.Imageable(prim).ComputeVisibility()
                        should_be_visible = is_active and visibility != UsdGeom.Tokens.invisible
                    except Exception as e:
                        should_be_visible = False  # Default to hidden if check fails
                else:
                    should_be_visible = False  # Prim doesn't exist? Hide.
            else:
                should_be_visible = False  # No stage? Hide.

            item.visible = should_be_visible and self._usd_listening_active
            self._icons[prim_path] = item
            self._item_changed(item)

    def remove_sensor_icon(self, prim_path: object) -> None:
        """Removes a sensor icon from the model and marks the path as hidden.

        Args:
            prim_path: The USD prim path of the sensor to remove.
        """
        if not isinstance(prim_path, Sdf.Path):
            prim_path = Sdf.Path(prim_path)

        if prim_path in self._icons:
            self._icons[prim_path].removed = True
            self._item_changed(self._icons[prim_path])
            self._icons.pop(prim_path)
            # Mark as hidden
            self._hidden_paths.add(prim_path)

    def set_icon_click_fn(self, prim_path: object, call_back: callable) -> None:
        """Sets the click callback function for a sensor icon.

        Args:
            prim_path: The USD prim path of the sensor.
            call_back: The callback function to execute when the icon is clicked.
        """
        if not isinstance(prim_path, Sdf.Path):
            prim_path = Sdf.Path(prim_path)
        item = self._icons.get(prim_path)
        if item:
            item.on_click = call_back

    @Trace.TraceFunction
    def show_sensor_icon(self, prim_path: object) -> None:
        """Show a sensor icon by setting the USD prim visibility to visible and immediately updating internal state.

        Args:
            prim_path: The USD prim path of the sensor to show.
        """
        if not isinstance(prim_path, Sdf.Path):
            prim_path = Sdf.Path(prim_path)

        stage = self._usd_context.get_stage()
        if stage:
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsA(UsdGeom.Imageable):
                imageable = UsdGeom.Imageable(prim)
                imageable.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited)

        item = self._icons.get(prim_path)
        if item and not item.visible:
            item.visible = True
            self._item_changed(item)

    def hide_sensor_icon(self, prim_path: object) -> None:
        """Hide a sensor icon by setting the USD prim visibility to invisible and immediately updating internal state.

        Args:
            prim_path: The USD prim path of the sensor to hide.
        """
        if not isinstance(prim_path, Sdf.Path):
            prim_path = Sdf.Path(prim_path)

        stage = self._usd_context.get_stage()
        if stage:
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsA(UsdGeom.Imageable):
                imageable = UsdGeom.Imageable(prim)
                imageable.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)

        item = self._icons.get(prim_path)
        if item and item.visible:
            item.visible = False
            self._item_changed(item)

    def show_all(self) -> None:
        """Shows all sensor icons by activating USD listening and repopulating icons from the current stage state."""
        # Activate USD listening
        self._usd_listening_active = True

        # Re-register the USD listener if needed
        stage = self._usd_context.get_stage()
        if stage and not self._frame_sub:
            self._frame_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self._on_frame_update,
                observer_name="IsaacSensorIconGUI.__update_event_callback",
            )

        # Refresh all icons from the current USD state
        self._populate_initial_icons()

    def hide_all(self) -> None:
        """Hides all sensor icons by deactivating USD listening and clearing the icon model."""
        # Deactivate USD listening
        self._usd_listening_active = False
        self._frame_sub = None

        # Forcefully clear all icons from the model
        if self._icons:
            self._icons = {}
            self._item_changed(None)

    @Trace.TraceFunction
    def refresh_all_icon_visuals(self) -> None:
        """Force a refresh notification for all currently tracked icon items."""
        if not self._usd_listening_active:
            return

        # carb.log_info("Forcing refresh of all sensor icon visuals due to simulation state change.")
        # Destroy existing icons and repopulate
        if self._icons:
            self._icons = {}
            self._item_changed(None)
            self._populate_initial_icons()

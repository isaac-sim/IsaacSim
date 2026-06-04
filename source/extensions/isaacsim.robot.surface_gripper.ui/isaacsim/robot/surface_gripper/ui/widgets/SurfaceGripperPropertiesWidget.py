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

"""Properties widget for Surface Gripper prims providing specialized controls and attribute display for gripper functionality in the USD property panel."""

import asyncio
import typing

import carb
import omni
import omni.ui as ui
from isaacsim.robot.surface_gripper import _surface_gripper as surface_gripper
from omni.kit.property.usd.usd_property_widget import UiDisplayGroup, UsdPropertiesWidget
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder
from pxr import Tf, Usd
from usd.schema.isaac import robot_schema


class SurfaceGripperPropertiesWidget(UsdPropertiesWidget):
    """Properties widget for Surface Gripper prims in the USD stage.

    This widget provides a specialized property interface for ``SurfaceGripper`` prims, enabling users to view and
    modify gripper attributes and relationships through the Omniverse Kit property panel. It displays gripper-specific
    properties such as status, grip limits, force constraints, and attachment points in a customized layout.

    The widget includes interactive controls for opening and closing the gripper, automatically refreshes when USD
    changes occur, and filters properties to show only those relevant to Surface Gripper functionality. Properties
    are displayed in a logical order with custom display names and tooltips for better user experience.

    Key features include:
    - Interactive open/close gripper buttons
    - Custom property ordering and display names
    - Real-time USD change monitoring and refresh
    - Filtered view showing only Surface Gripper properties
    - Support for both attributes (status, limits, offsets) and relationships (attachment points, gripped objects)

    Args:
        *args: Variable positional arguments passed to the parent UsdPropertiesWidget.
        **kwargs: Additional keyword arguments passed to the parent UsdPropertiesWidget.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._old_payload = []
        self._listener = None
        self.frames = []
        self.wrapped_ui_elements = []
        self.reset()
        self._request_refresh()

        self.property_metadata = {
            # Attributes
            robot_schema.Attributes.STATUS.name: {
                "displayName": robot_schema.Attributes.STATUS.display_name,
                "tooltip": "Current status of the gripper",
                "allowedTokens": ["Open", "Closed", "Closing"],
            },
            robot_schema.Attributes.MAX_GRIP_DISTANCE.name: {
                "displayName": robot_schema.Attributes.MAX_GRIP_DISTANCE.display_name,
                "tooltip": "Maximum allowed grip distance",
            },
            robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name: {
                "displayName": robot_schema.Attributes.COAXIAL_FORCE_LIMIT.display_name,
                "tooltip": "Maximum force that can be applied along the gripper axis",
            },
            robot_schema.Attributes.SHEAR_FORCE_LIMIT.name: {
                "displayName": robot_schema.Attributes.SHEAR_FORCE_LIMIT.display_name,
                "tooltip": "Maximum force that can be applied perpendicular to the gripper axis",
            },
            robot_schema.Attributes.CLEARANCE_OFFSET.name: {
                "displayName": robot_schema.Attributes.CLEARANCE_OFFSET.display_name,
                "tooltip": "Clearance offset for the gripper",
            },
            robot_schema.Attributes.RETRY_INTERVAL.name: {
                "displayName": robot_schema.Attributes.RETRY_INTERVAL.display_name,
                "tooltip": "Retry interval for the gripper",
            },
            # Relationships
            robot_schema.Relations.ATTACHMENT_POINTS.name: {
                "displayName": robot_schema.Relations.ATTACHMENT_POINTS.display_name,
                "tooltip": "Points where the gripper can attach",
                "relationshipTargetPaths": True,
            },
            robot_schema.Relations.GRIPPED_OBJECTS.name: {
                "displayName": robot_schema.Relations.GRIPPED_OBJECTS.display_name,
                "tooltip": "Objects currently gripped",
                "relationshipTargetPaths": True,
            },
        }

    def _request_refresh(self) -> None:
        """Refreshes the entire property window."""
        selection = omni.usd.get_context().get_selection()
        selected_paths = selection.get_selected_prim_paths()
        window = omni.kit.window.property.get_window()._window  # noqa: PLW0212 what does this mean?

        selection.clear_selected_prim_paths()
        window.frame.rebuild()
        selection.set_selected_prim_paths(selected_paths, True)
        window.frame.rebuild()

    def _on_usd_changed(self, notice: Usd.Notice.ObjectsChanged, stage: Usd.Stage) -> None:
        """Handles USD change notifications.

        Args:
            notice: The USD notice containing change information.
            stage: The USD stage that was modified.
        """
        notice.GetChangedInfoOnlyPaths()
        if self._old_payload != self.on_new_payload(
            self._payload
        ):  # if selection didn't change, check if attribute still exists, and force rebuild if so
            self._old_payload = self._prims
            self._request_refresh()
        else:
            super()._on_usd_changed(notice, stage)

    def _get_prim(self, prim_path: str) -> Usd.Prim | None:
        """Gets a Surface Gripper prim from the specified path.

        Args:
            prim_path: Path to the prim.

        Returns:
            The Surface Gripper prim if valid, otherwise None.
        """
        if prim_path:
            stage = self._payload.get_stage()
            if stage:
                prim = stage.GetPrimAtPath(prim_path)
                if prim and prim.GetTypeName() == robot_schema.Classes.SURFACE_GRIPPER.value:
                    return prim
        return None

    def on_new_payload(self, payload: list) -> "list | bool":
        """See PropertyWidget.on_new_payload.

        Args:
            payload: The new payload containing prim selection data.

        Returns:
            List of valid Surface Gripper prims if found, otherwise False.
        """
        if self._listener:
            self._listener.Revoke()
            self._listener = None

        if not super().on_new_payload(payload):
            return False

        prim_paths = self._payload.get_paths()
        prims = [self._get_prim(prim_path) for prim_path in prim_paths]
        self._prims = [p for p in prims if p is not None]
        if not self._prims:
            return False

        return self._prims

    def build_items(self) -> None:
        """Build the property widget items for the Surface Gripper."""
        # Reset the widget state
        self.reset()

        if not self._payload or len(self._payload) == 0:
            return

        last_prim = self._get_prim(self._payload[-1])
        stage = last_prim.GetStage() if last_prim else None

        if not stage:  # pragma: no cover
            return

        self._listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_usd_changed, stage)

        # Get shared properties from selected prims
        shared_props = self._get_shared_properties_from_selected_prims(last_prim)
        if not shared_props:  # pragma: no cover
            return

        # Add custom UI controls for open/close
        with ui.HStack():
            ui.Button("Close", clicked_fn=self.close)
            ui.Button("Open", clicked_fn=self.open)

        # Filter and customize properties
        shared_props = self._customize_props_layout(shared_props)
        filtered_props = shared_props
        if self._filter.name:

            def display_name(ui_prop: typing.Any) -> str:
                return UsdPropertiesWidgetBuilder.get_display_name(ui_prop.prop_name, ui_prop.metadata)

            print([display_name(ui_prop) for ui_prop in shared_props])
            filtered_props = [ui_prop for ui_prop in shared_props if self._filter.matches(display_name(ui_prop))]
            if not filtered_props:
                return

            # Define the desired order of properties
        property_order = [
            robot_schema.Attributes.STATUS.name,
            robot_schema.Relations.GRIPPED_OBJECTS.name,
            robot_schema.Attributes.RETRY_INTERVAL.name,
            robot_schema.Attributes.MAX_GRIP_DISTANCE.name,
            robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name,
            robot_schema.Attributes.SHEAR_FORCE_LIMIT.name,
            robot_schema.Attributes.CLEARANCE_OFFSET.name,
            robot_schema.Relations.ATTACHMENT_POINTS.name,
        ]

        # Sort the filtered properties according to the defined order
        def get_sort_key(prop: typing.Any) -> int:
            try:
                return property_order.index(prop.prop_name)
            except ValueError:
                return len(property_order)  # Put unordered properties at the end

        filtered_props.sort(key=get_sort_key)

        self._any_item_visible = True

        # Add property change listeners
        attr_names = [prop.prop_name for prop in filtered_props]
        self.add_listener_adapters(attr_names)

        # Create property group and build frames
        grouped_props = UiDisplayGroup("", self._maintain_property_order, filtered_props)
        self.build_nested_group_frames(stage, grouped_props)

        # Add schema API frame if enabled
        if (
            hasattr(self, "show_schemas")
            and self.show_schemas()
            and carb.settings.get_settings().get("ext/omni.kit.property.usd/showSchemaAPI")
        ):
            frame = ui.Frame(visible=False)
            asyncio.ensure_future(self._build_schema_group_frames(frame, stage, grouped_props, last_prim))

    def _customize_props_layout(self, props: list) -> list:
        """Customize the properties layout with specific display names and tooltips.

        Args:
            props: The properties to customize.

        Returns:
            The customized properties with applied metadata.
        """
        # Apply metadata to properties
        for prop in props:
            if prop.prop_name in self.property_metadata:
                for key, value in self.property_metadata[prop.prop_name].items():
                    prop.metadata[key] = value
                prop.override_display_name(self.property_metadata[prop.prop_name]["displayName"])

        return props

    def close(self) -> None:
        """Closes the gripper for all selected prims."""
        gripper_interface = surface_gripper.acquire_surface_gripper_interface()
        for prim in self._prims:
            # Define the prim path for your gripper
            gripper_path = prim.GetPath().pathString

            # Close the gripper
            gripper_interface.close_gripper(gripper_path)

    def open(self) -> None:
        """Opens the gripper for all selected prims."""
        gripper_interface = surface_gripper.acquire_surface_gripper_interface()
        for prim in self._prims:
            gripper_path = prim.GetPath().pathString
            gripper_interface.open_gripper(gripper_path)

    def _on_forward_axis_selection(self, item: str) -> None:
        """Handles forward axis selection changes.

        Args:
            item: The selected axis item.
        """

    def _on_coaxial_force_limit_changed(self, value: float) -> None:
        """Handles coaxial force limit value changes.

        Args:
            value: The new coaxial force limit value.
        """
        print("Coaxial Force Limit Changed was called!", value)

    def _on_grip_distance_changed(self, value: float) -> None:
        """Handles changes to the grip distance value.

        Args:
            value: The new grip distance value.
        """

    def _on_max_grip_distance_changed(self, value: float) -> None:
        """Handles changes to the maximum grip distance value.

        Args:
            value: The new maximum grip distance value.
        """

    def _on_retry_interval_changed(self, value: float) -> None:
        """Handles changes to the retry interval value.

        Args:
            value: The new retry interval value.
        """

    def _on_shear_force_limit_changed(self, value: float) -> None:
        """Handles changes to the shear force limit value.

        Args:
            value: The new shear force limit value.
        """

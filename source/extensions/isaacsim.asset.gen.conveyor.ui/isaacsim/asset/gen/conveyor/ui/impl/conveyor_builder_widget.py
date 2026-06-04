"""Widget for the conveyor builder UI, providing interactive track layout tools."""

# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

import copy
import json
from typing import Any

import carb
import carb.eventdispatcher
import omni
import omni.ui as ui
import omni.usd
from pxr import Gf, UsdGeom
from pxr.Usd import EditContext, Stage

from .conveyor_builder.conveyor_system import ConveyorBuilder, ConveyorFilter, ConveyorSelector, set_pose_from_transform
from .conveyor_builder.conveyor_track import Angle, ConveyorTrack, Ramp, Style, Type
from .preferences import ConveyorBuilderPreferences
from .preview_highlight import ConveyorPreviewHighlight
from .selected_conveyor import SelectedConveyorWidget


def singleton(class_: type) -> object:
    """A singleton decorator.

    Ensures only one instance of the decorated class exists by caching instances and returning the same
    instance on subsequent instantiations.

    Args:
        class_: The class to apply singleton pattern to.

    Returns:
        A wrapper function that returns the singleton instance of the class.
    """
    instances = {}

    def getinstance(*args: Any, **kwargs: Any) -> Any:
        if class_ not in instances:
            instances[class_] = class_(*args, **kwargs)
        return instances[class_]

    return getinstance


class ConveyorBuilderWidget:
    """A widget for building conveyor systems in USD stages.

    Provides an interactive UI for creating and managing conveyor tracks with different styles,
    types, angles, and ramps. The widget allows users to select conveyor pieces, configure their
    properties, and connect them to build complete conveyor systems. It handles real-time preview
    of conveyor placement and maintains visual feedback through temporary materials and geometry.

    The widget integrates with USD stage selection events to update the available connection points
    and track configurations based on the currently selected conveyor pieces. It supports various
    conveyor styles including belt, roller, and dual configurations, with different track types
    such as straight sections, T-merges, Y-merges, starts, and ends.

    Args:
        style: UI style configuration dictionary for the widget's appearance and layout.
    """

    def on_display(self) -> None:
        """Initializes the conveyor builder display and sets up the stage environment.

        Creates necessary USD prims for the conveyor builder system, including the temporary
        preview geometry. The preview is tinted via a viewport selection group (see
        ``ConveyorPreviewHighlight``); no MDL material binding is authored on the preview
        prim. Sets up stage event listeners for selection and stage closure events.
        """
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            carb.log_error(
                "A stage is required in the context to use the conveyor builder tool.  No stage open in the current context."
            )
            return
        self.builder.clear_system(stage)
        with EditContext(stage, stage.GetSessionLayer()):
            omni.kit.commands.execute(
                "CreatePrim", prim_path="/ConveyorBuilder", prim_type="Scope", attributes={}, select_new_prim=False
            )
            prim = stage.GetPrimAtPath("/ConveyorBuilder")
            omni.usd.editor.set_hide_in_stage_window(prim, True)
            self.temp_prim = stage.DefinePrim("/ConveyorBuilder/conveyorBuilder_Temp", "Xform")
            omni.usd.get_context().set_pickable("/ConveyorBuilder/conveyorBuilder_Temp", False)
        self._usd_context = omni.usd.get_context()
        self._stage_event_sub_selection = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SELECTION_CHANGED),
            on_event=self._on_selection_changed,
            observer_name="isaacsim.asset.gen.conveyor.ui._on_selection_changed",
        )
        self._stage_event_sub_closed = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
            on_event=self._on_stage_closed,
            observer_name="isaacsim.asset.gen.conveyor.ui._on_stage_closed",
        )

    def __init__(self, style: Any) -> None:
        self._frame = ui.Frame(style=style)
        self.style = style
        self.style_buttons = []
        self.direction_buttons = []
        self.slope_buttons = []
        self.type_buttons = []
        self.temp_prim = None
        self._selection = None
        self.current_track = None
        self.next_track = None
        self.flip_placement = False
        self.part_usd = None
        self.previous_selection = []
        self._preview_highlight = ConveyorPreviewHighlight()
        preferences = ConveyorBuilderPreferences()
        with open(preferences.config_file) as f:
            self.config = json.load(f)
        self._conveyor_selector = ConveyorSelector(self.config, thumb_loaded_callback=self.on_thumb_loaded)

        self._filter = ConveyorFilter(
            styles=[Style.BELT], types=[Type.STRAIGHT], angles=[Angle.NONE], ramps=[Ramp.FLAT]
        )
        self.builder = ConveyorBuilder(omni.usd.get_context().get_stage(), self._conveyor_selector)
        self.available_tiles = self._conveyor_selector.list_tracks(self._filter)
        self._selection = omni.usd.get_context().get_selection()
        self._events = omni.usd.get_context().get_stage_event_stream()

        self.button_to_style = {
            "conveyor_roller": [Style.ROLLER],
            "conveyor_belt": [Style.BELT],
            "conveyor_combine": [Style.DUAL],
        }
        self.button_to_type = {
            "type_start": [Type.START],
            "type_straight": [Type.STRAIGHT],
            "type_t": [Type.T_MERGE],
            "type_y": [Type.Y_MERGE, Type.FORK_MERGE],
            "type_end": [Type.END],
        }
        self.button_to_angle = {
            "arrow_180_left": [Angle.FULL],
            "arrow_180_right": [Angle.FULL],
            "arrow_90_left": [Angle.HALF],
            "arrow_90_right": [Angle.HALF],
            "arrow_up": [Angle.NONE],
        }
        self.button_to_ramp = {
            "ramp_2l": [Ramp.THREE, Ramp.FOUR],
            "ramp_2r": [Ramp.THREE, Ramp.FOUR],
            "ramp_1l": [Ramp.ONE, Ramp.TWO],
            "ramp_1r": [Ramp.ONE, Ramp.TWO],
            "ramp_flat": [Ramp.FLAT],
        }
        self.build_ui()
        self._on_kit_selection_changed()

    def shutdown(self) -> None:
        """Cleans up the conveyor builder widget resources.

        Removes event subscriptions, releases the preview highlight selection group,
        clears the conveyor system from the stage, and removes the temporary USD prims
        created during the conveyor building process.
        """
        self._stage_event_sub_selection = None
        self._stage_event_sub_closed = None
        self._preview_highlight.shutdown()
        stage = omni.usd.get_context().get_stage()
        self.builder.clear_system(stage)
        if stage:
            for pth in ["/ConveyorBuilder"]:
                temp_prim = stage.GetPrimAtPath(pth)
                if temp_prim:
                    with EditContext(stage, stage.GetSessionLayer()):
                        stage.RemovePrim(temp_prim.GetPath())
                    stage.RemovePrim(temp_prim.GetPath())

    def _on_selection_changed(self, event: object) -> None:
        """Stage selection changed event callback.

        Args:
            event: The stage event data.
        """
        self._on_kit_selection_changed()

    def _on_stage_closed(self, event: object) -> None:
        """Stage closed event callback.

        Args:
            event: The stage event data.
        """
        self.shutdown()

    def get_selection(self) -> Any:
        """Gets the currently selected conveyor track and its path.

        Analyzes the current USD stage selection to find conveyor tracks by examining
        references and payloads. Filters out temporary ConveyorBuilder paths and updates
        the previous selection state.

        Returns:
            A tuple containing the selected conveyor track object and its USD path.
            Returns (None, "/") if no valid conveyor selection is found.
        """
        stage = omni.usd.get_context().get_stage()
        if self._selection:
            for i, p in enumerate(self._selection.get_selected_prim_paths()):
                refs = []
                if "/ConveyorBuilder" in p:
                    self._selection.set_selected_prim_paths(self.previous_selection, False)
                    return None, "/"
                sel = stage.GetPrimAtPath(p)
                while sel != stage.GetPrimAtPath("/"):
                    if sel.GetReferences():
                        refs += omni.usd.get_composed_references_from_prim(sel, False)
                    if sel.GetPayloads():
                        refs += omni.usd.get_composed_payloads_from_prim(sel, False)
                    for a in refs:
                        if self._conveyor_selector.asset_path in a[0].assetPath:
                            asset = a[0].assetPath.replace(self._conveyor_selector.asset_path, "")
                            self._selection.set_selected_prim_paths([str(sel.GetPath())], False)
                            self.previous_selection = [str(sel.GetPath())]
                            return self._conveyor_selector.tracks[asset], p

                    sel = sel.GetParent()
        return None, "/"

    def _on_kit_selection_changed(self) -> None:
        """The selection in kit is changed."""
        # Kit's set_selected_prim_paths drops every selected prim back to group 0; if any
        # preview gprim is touched by selection, re-bind it to our highlight group.
        self._refresh_preview_highlight()
        selections = self.get_selection()
        if "/ConveyorBuilder" not in selections[1]:
            self.current_track.update_selection(selections[0], self.builder.get_available_connections(selections[1]))
            in_level = 0
            out_level = 0
            if self.current_track.selected_conveyor:
                out_level = self.current_track.selected_conveyor.end_level(direction=self.current_direction())
            for a in range(len(self.available_tiles)):
                if (
                    self._conveyor_selector.tracks[self.available_tiles[a]].start_level(
                        direction=self.get_ramp_and_curve_direction()[0]
                    )
                    == out_level
                ):
                    in_level = a
                    break
            scale = self.get_ramp_and_curve_direction()
            self.next_track.update_list(
                [self._conveyor_selector.tracks[a] for a in self.available_tiles], in_level, scale[0]
            )

            self.on_next_anchor_changed("")

    def get_ramp_and_curve_direction(self) -> Any:
        """Calculates the ramp and curve direction scaling factors for conveyor placement.

        Determines the direction multipliers based on the current slope and direction button
        selections, accounting for conveyor angles and flip placement settings.

        Returns:
            A Gf.Vec3d containing (ramp_scale, curve_scale, z_scale) direction multipliers.
        """
        ramp = -1 if bool([i for i in range(2) if self.slope_buttons[i].selected]) else 1
        curve = -1 if bool([i for i in range(2) if self.direction_buttons[i].selected]) else 1
        if self.next_track.selected_conveyor:
            if self.next_track.selected_conveyor.angle != Angle.NONE and self.next_track.get_next_anchor(ramp):
                curve = -curve
        if self.flip_placement:
            curve = -curve
        return Gf.Vec3d(ramp, curve, 1)

    def current_direction(self) -> int:
        """Gets the current direction of the selected conveyor track.

        Returns:
            The direction value of the currently selected conveyor, or 1 if no selection.
        """
        stage = omni.usd.get_context().get_stage()
        sel = self.get_selection()
        if sel[0]:
            return self.builder.get_direction(sel[1])
        return 1

    def get_next_base_pose(self) -> Any:
        """Calculates the base pose for the next conveyor track to be placed.

        Computes the transformation matrix for positioning the next conveyor track based on
        the selected track, anchor points, direction scaling, and current selection.

        Returns:
            The transformation matrix for the next conveyor track placement.
        """
        scale = self.get_ramp_and_curve_direction()
        return self.builder.get_next_pose(
            self.next_track.selected_conveyor,
            self.next_track.get_next_anchor(scale[0]),
            1,
            scale[1],
            self.get_selection()[1],
            self.current_track.get_current_anchor(),
        )

    def on_track_selection_changed(self, track: ConveyorTrack) -> None:
        """Handles changes to the selected conveyor track.

        Updates the temporary USD prim reference to display the newly selected track,
        refreshes the next anchor visualization, and re-applies the green selection-group
        tint to the new descendant gprims.

        Args:
            track: The newly selected conveyor track object.
        """
        stage = omni.usd.get_context().get_stage()
        temp_prim = stage.GetPrimAtPath("/ConveyorBuilder/conveyorBuilder_Temp")
        if temp_prim:
            with EditContext(stage, stage.GetSessionLayer()):
                temp_prim.GetReferences().ClearReferences()
                temp_prim.GetReferences().AddReference(track.base_usd)
                self.part_usd = track.base_usd
        self.on_next_anchor_changed("")
        self._refresh_preview_highlight()

    def _refresh_preview_highlight(self) -> None:
        """Re-collect Gprims under the preview prim and assign them to the green tint group.

        Called whenever the preview prim's reference target changes so newly composed
        descendants pick up the highlight. Safe to call repeatedly; the helper diffs
        against its previously assigned set.
        """
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        temp_prim = stage.GetPrimAtPath("/ConveyorBuilder/conveyorBuilder_Temp")
        self._preview_highlight.apply_to_subtree(stage, temp_prim)

    def on_next_anchor_changed(self, anchor: str | None) -> None:
        """Handles the change of the next conveyor anchor point.

        Updates the temporary preview prim's position and scale based on the new anchor and current conveyor settings.

        Args:
            anchor: The new anchor point for the next conveyor segment.
        """
        stage = omni.usd.get_context().get_stage()
        temp_prim = stage.GetPrimAtPath("/ConveyorBuilder/conveyorBuilder_Temp")
        with EditContext(stage, stage.GetSessionLayer()):
            scale = self.get_ramp_and_curve_direction()
            scale[0] = 1
            scale_unit = 1
            if self.part_usd:
                part_stage = Stage.Open(self.part_usd)
                a = UsdGeom.GetStageMetersPerUnit(stage)
                b = UsdGeom.GetStageMetersPerUnit(part_stage)
                scale_unit = b / a
            set_pose_from_transform(temp_prim, self.get_next_base_pose(), scale_unit * scale)

    def on_thumb_loaded(self, track: ConveyorTrack) -> None:
        """Handles the loading of a conveyor track thumbnail.

        Refreshes the UI for current and next track widgets when their thumbnails are loaded.

        Args:
            track: The conveyor track whose thumbnail was loaded.
        """
        if self.current_track:
            if self.current_track.selected_conveyor == track or self.current_track.selected_conveyor is None:
                self.current_track.build_ui()
        if self.next_track:
            if self.next_track.selected_conveyor == track:
                self.next_track.build_ui()

    def on_flip(self, current: bool) -> None:
        """Handles the flip placement action.

        Toggles the flip placement setting and updates the next anchor position accordingly.

        Args:
            current: Indicates if the flip is for the current track.
        """
        self.flip_placement = not self.flip_placement
        self.on_next_anchor_changed(None)

    def remove_track(self, track: ConveyorTrack) -> None:
        """Removes the currently selected conveyor track.

        Deletes the selected conveyor track from the stage and updates the selection to its parent.

        Args:
            track: The conveyor track to remove.
        """
        selection = self.get_selection()
        parent = None
        stage = omni.usd.get_context().get_stage()
        if selection[0]:
            parent = self.builder.remove_track(selection[1])
            stage.RemovePrim(selection[1])
        self._selection.set_selected_prim_paths([parent], False)

    def on_add_track(self, track: ConveyorTrack) -> None:
        """Handles adding a new conveyor track to the system.

        Creates a new conveyor track segment connected to the current selection and updates the selection to the new track.

        Args:
            track: The conveyor track to add to the system.
        """
        selection = self.get_selection()
        parent_anchor = ""
        if selection[0]:
            parent_anchor = self.current_track.get_current_anchor()
        scale = self.get_ramp_and_curve_direction()
        new_track = self.builder.add_track(
            self.next_track.selected_conveyor,
            self.next_track.get_next_anchor(scale[0]),
            1,
            scale[1],
            selection[1],
            parent_anchor,
        )
        self._selection.set_selected_prim_paths([new_track], False)

    def build_ui(self) -> None:
        """Constructs the main user interface for the conveyor builder.

        Creates the complete UI layout including style buttons, type buttons, direction buttons, slope buttons, and track selection widgets.
        """
        self.on_display()
        with self._frame:
            with ui.VStack(**self.style):
                with ui.HStack(height=ui.Pixel(60), alignment=ui.Alignment.CENTER):  # Conveyor Type
                    ui.Spacer(width=60)

                    self.style_buttons.append(
                        ui.Button(
                            name="conveyor_roller", height=60, width=60, tooltip="Roller-style conveyor", selected=False
                        )
                    )
                    self.style_buttons.append(
                        ui.Button(
                            name="conveyor_belt", height=60, width=60, tooltip="belt-style conveyor", selected=True
                        )
                    )
                    self.style_buttons.append(
                        ui.Button(name="conveyor_combine", height=60, width=60, tooltip="dual conveyor")
                    )

                for b in self.style_buttons:
                    b.set_clicked_fn(lambda a=b: self.on_style_selected(a))
                with ui.HStack(height=ui.Pixel(60), alignment=ui.Alignment.CENTER):  # Conveyor Type

                    self.type_buttons.append(
                        ui.Button(name="type_start", height=60, width=60, tooltip="conveyor start")
                    )
                    self.type_buttons.append(ui.Button(name="type_t", height=60, width=60, tooltip="T-style split"))
                    self.type_buttons.append(
                        ui.Button(
                            name="type_straight", height=60, width=60, tooltip="single-line conveyor", selected=True
                        )
                    )
                    self.type_buttons.append(ui.Button(name="type_y", height=60, width=60, tooltip="y-style split"))
                    self.type_buttons.append(
                        ui.Button(name="type_end", height=60, width=60, tooltip="conveyor endpoint")
                    )

                for b in self.type_buttons:
                    b.set_clicked_fn(lambda a=b: self.on_type_selected(a))
                with ui.HStack(height=ui.Pixel(60)):  # Conveyor Direction
                    self.direction_buttons.append(
                        ui.Button(name="arrow_180_left", height=60, width=60, tooltip="180 degrees turn left")
                    )
                    self.direction_buttons.append(
                        ui.Button(name="arrow_90_left", height=60, width=60, tooltip="90 degrees turn left")
                    )
                    self.direction_buttons.append(
                        ui.Button(name="arrow_up", height=60, width=60, tooltip="forward", selected=True)
                    )
                    self.direction_buttons.append(
                        ui.Button(name="arrow_90_right", height=60, width=60, tooltip="90 degrees turn right")
                    )
                    self.direction_buttons.append(
                        ui.Button(
                            name="arrow_180_right", height=60, width=60, tooltip="180 degrees turn right", enabled=False
                        )
                    )
                for b in self.direction_buttons:
                    b.set_clicked_fn(lambda a=b: self.on_direction_selected(a))
                with ui.HStack(height=ui.Pixel(60)):  # Conveyor ramp
                    self.slope_buttons.append(
                        ui.Button(name="ramp_2l", height=60, width=60, tooltip="2 level down ramp")
                    )
                    self.slope_buttons.append(
                        ui.Button(name="ramp_1l", height=60, width=60, tooltip="1 level down ramp")
                    )
                    self.slope_buttons.append(
                        ui.Button(name="ramp_flat", height=60, width=60, tooltip="flat piece", selected=True)
                    )
                    self.slope_buttons.append(ui.Button(name="ramp_1r", height=60, width=60, tooltip="1 level up ramp"))
                    self.slope_buttons.append(ui.Button(name="ramp_2r", height=60, width=60, tooltip="2 level up ramp"))
                for b in self.slope_buttons:
                    b.set_clicked_fn(lambda a=b: self.on_slope_selected(a))
                ui.Spacer(height=ui.Pixel(3))
                with ui.HStack(height=ui.Pixel(150)):
                    ui.Spacer(width=ui.Pixel(3))
                    self.current_track = SelectedConveyorWidget(
                        **self.style,
                        name="current",
                        anchor_changed_fn=self.on_next_anchor_changed,
                        flip_fn=lambda a: self.on_flip(True),
                        remove_track_fn=self.remove_track,
                    )
                    ui.Spacer(width=ui.Pixel(6))
                    self.next_track = SelectedConveyorWidget(
                        **self.style,
                        thumbnail="",
                        selection_changed_fn=self.on_track_selection_changed,
                        anchor_changed_fn=self.on_next_anchor_changed,
                        flip_fn=lambda a: self.on_flip(False),
                        add_track_fn=self.on_add_track,
                    )
                self.update_filter()

    def update_filter(self) -> None:
        """Updates the conveyor filter based on current UI selections.

        Refreshes the available conveyor tiles list and enables/disables UI buttons based on available conveyor types, angles, and ramps.
        """
        self.available_tiles = self._conveyor_selector.list_tracks(self._filter)
        filter = copy.copy(self._filter)
        filter.type = []
        type_list = self._conveyor_selector.list_tracks(filter)
        types = [
            bool([a for a in type_list if self._conveyor_selector.tracks[a].type == b])
            for b in [Type.START, Type.T_MERGE, Type.STRAIGHT, Type.Y_MERGE, Type.END]
        ]
        for a in range(len(types)):
            self.type_buttons[a].enabled = types[a]
        filter.type = self._filter.type
        if filter.type[0] not in [Type.Y_MERGE, Type.T_MERGE]:
            self.flip_placement = False

        filter.angle = []
        curvature_list = self._conveyor_selector.list_tracks(filter)
        curvature_1 = bool([a for a in curvature_list if self._conveyor_selector.tracks[a].angle == Angle.HALF])
        curvature_2 = bool([a for a in curvature_list if self._conveyor_selector.tracks[a].angle == Angle.FULL])
        self.direction_buttons[0].enabled = curvature_2
        self.direction_buttons[1].enabled = curvature_1
        self.direction_buttons[3].enabled = curvature_1
        self.direction_buttons[4].enabled = curvature_2
        if not bool([a for a in self.direction_buttons if a.selected]):
            self.direction_buttons[2].selected = True
            return self.on_direction_selected(self.direction_buttons[2])
        filter.angle = self._filter.angle

        filter.ramp = []
        slope_list = self._conveyor_selector.list_tracks(filter)
        for a in self.direction_buttons:
            if a.selected and not a.enabled:
                a.selected = False
        slope_1 = bool([a for a in slope_list if self._conveyor_selector.tracks[a].ramp == Ramp.ONE])
        slope_2 = bool([a for a in slope_list if self._conveyor_selector.tracks[a].ramp == Ramp.TWO])
        self.slope_buttons[0].enabled = slope_2
        self.slope_buttons[1].enabled = slope_1
        self.slope_buttons[3].enabled = slope_1
        self.slope_buttons[4].enabled = slope_2
        for a in self.slope_buttons:
            if a.selected and not a.enabled:
                a.selected = False

        if not bool([a for a in self.slope_buttons if a.selected]):
            self.slope_buttons[2].selected = True
            return self.on_slope_selected(self.slope_buttons[2])

        in_level = 0
        out_level = 0
        if self.current_track.selected_conveyor:
            out_level = self.current_track.selected_conveyor.end_level(direction=self.current_direction())
        for a in range(len(self.available_tiles)):
            if (
                self._conveyor_selector.tracks[self.available_tiles[a]].start_level(
                    direction=self.get_ramp_and_curve_direction()[0]
                )
                == out_level
            ):
                in_level = a
                break
        scale = self.get_ramp_and_curve_direction()
        self.next_track.update_list(
            [self._conveyor_selector.tracks[a] for a in self.available_tiles], in_level, scale[0]
        )

    def on_style_selected(self, selected: ui.Button) -> None:
        """Handles conveyor style selection.

        Updates the filter to match the selected conveyor style and refreshes the available options.

        Args:
            selected: The selected style button.
        """
        for a in self.style_buttons:
            a.selected = False
        selected.selected = True
        self._filter.style = self.button_to_style[selected.name]
        self.update_filter()

    def on_direction_selected(self, selected: ui.Button) -> None:
        """Handles conveyor direction selection.

        Updates the filter to match the selected direction and refreshes track selection if the direction changes.

        Args:
            selected: The selected direction button.
        """
        before = self.get_ramp_and_curve_direction()
        for a in self.direction_buttons:
            a.selected = False
        selected.selected = True
        self._filter.angle = self.button_to_angle[selected.name]
        self.update_filter()
        after = self.get_ramp_and_curve_direction()
        if before[1] != after[1]:
            self.on_track_selection_changed(self.next_track.selected_conveyor)

    def on_slope_selected(self, selected: ui.Button) -> None:
        """Handles conveyor slope selection.

        Updates the filter to match the selected slope and refreshes track selection if the slope changes.

        Args:
            selected: The selected slope button.
        """
        before = self.get_ramp_and_curve_direction()
        for a in self.slope_buttons:
            a.selected = False
        selected.selected = True
        self._filter.ramp = self.button_to_ramp[selected.name]
        self.update_filter()
        after = self.get_ramp_and_curve_direction()
        if before[0] != after[0]:
            self.on_track_selection_changed(self.next_track.selected_conveyor)

    def on_type_selected(self, selected: ui.Button) -> None:
        """Handles selection of a conveyor type button.

        Updates the filter based on the selected type and refreshes the available conveyor options.

        Args:
            selected: The type button that was selected.
        """
        for a in self.type_buttons:
            a.selected = False
        selected.selected = True
        self._filter.type = self.button_to_type[selected.name]
        self.update_filter()

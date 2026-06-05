# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Main UI builder class for the Gain Tuner extension interface that manages robot selection, gain parameter adjustment, testing functionality, and results visualization."""

import asyncio
from functools import partial

import carb
import numpy as np
import omni.kit.app
import omni.timeline
import omni.ui as ui
import pxr
from isaacsim.gui.components.element_wrappers import StateButton
from omni.kit.window.file import StageSaveDialog
from omni.physics.tensors import DofType
from pxr import Sdf, Usd
from usd.schema.isaac.robot_schema import Classes

from ..gains_tuner import GainsTestMode, GainTuner
from ..usd_layer_utils import (
    collect_gain_save_edits,
    find_layer_by_save_identifier,
    get_layer_save_identifier,
    is_layer_savable,
)
from .color_table_widget import ColorJointWidget
from .dropdown_widget import create_combo_list_model
from .frame_widget import CustomCollapsableFrame as CollapsableFrame
from .joint_table_widget import JointSettingMode, JointWidget
from .plot_widget import CustomXYPlot
from .style import get_style
from .test_table_widget import TestJointWidget


class UIBuilder:
    """Main UI builder class for the Gain Tuner extension interface.

    This class manages the complete user interface for robot joint gain tuning, including robot selection,
    gain parameter adjustment, testing functionality, and results visualization through charts. It handles
    UI lifecycle events such as timeline play/pause/stop, stage changes, physics steps, and render updates
    to maintain synchronization between the simulation state and the interface.

    The interface is organized into three main collapsible frames:
    - Tune Gains: Interactive table for adjusting stiffness/damping or natural frequency parameters
    - Test Gains Settings: Configuration and execution of gain validation tests
    - Charts: Visualization of test results with position and velocity plots

    The class integrates with the GainTuner backend to perform actual gain calculations and test execution,
    while managing UI state transitions and user interactions through various callback functions.
    """

    def __init__(self) -> None:
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self._gains_tuner = GainTuner()
        self._gains_tuner.add_inertia_updated_callback(self._on_joint_inertia_updated)

        self._test_mode = GainsTestMode.SNAP_TO_LIMITS
        self._test_running = False
        self._joint_setting_mode = JointSettingMode.STIFFNESS

        self._reset_ui_next_frame = False
        self._make_plot_on_next_frame = False

        self._gains_table_widget = None
        self._test_table_widget = None
        self._test_button = None
        self._color_joint_widget = None
        self._test_duration_frame = None
        self._snap_settings_frame = None
        self._hold_duration_field = None
        self._tolerance_field = None
        self._disable_self_collisions_cb = None
        self._disable_velocity_limits_cb = None
        self._stress_test_settings_frame = None
        self._stress_test_disable_self_collisions_cb = None
        self._stress_test_disable_velocity_limits_cb = None
        self._stress_test_submode_combo = None
        self._stress_test_duration_field = None
        self._stress_test_vel_threshold_field = None
        self._stress_test_sigma_field = None
        self._stress_test_sigma_frame = None
        self._stress_test_snap_interval_field = None
        self._stress_test_snap_interval_frame = None
        self._stress_test_seed_field = None
        self._radio_to_mode = [
            GainsTestMode.SNAP_TO_LIMITS,
            GainsTestMode.SINUSOIDAL,
            GainsTestMode.STEP,
            GainsTestMode.STRESS_TEST,
        ]
        self._self_collision_original: bool | None = None
        self._restarting_for_override = False
        self._position_frame = None
        self._velocity_frame = None
        self._plotting_indices = []
        self._plotting_colors = []
        self.force_query_mass = True
        self._save_stage_prompt = None
        self._initial_table_height = 150
        self._articulation_menu_model = None

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self) -> None:
        """Callback for when the UI is opened from the toolbar menu."""
        #     """Callback for when the UI is opened from the toolbar.
        #     This is called directly after build_ui().
        #     """
        #     # Reset internal state when UI window is closed and reopened
        #     # self._invalidate_articulation()

        #     # Handles the case where the user loads their Articulation and
        #     # presses play before opening this extension
        #     # self._articulation_menu.repopulate()
        #     # self._stop_text.visible = True

    def on_timeline_event(self, event: object) -> None:
        """Callback for Timeline events (Play, Pause, Stop).

        On play, collapses Tune Gains and expands Test Gains (so a collapsed Test Gains section opens for testing).
        On stop, expands Tune Gains again. Test Gains collapsed state is left unchanged on stop so the panel stays
        open for inspection and table layout stays stable.

        Args:
            event: Event Type
        """
        if self._restarting_for_override:
            return
        if not self._articulation_menu_model or not self._articulation_menu_model.has_item():
            return
        if event.event_name == omni.timeline.GLOBAL_EVENT_PLAY:
            self._gains_tuning_frame.collapsed = True
            self._test_gains_frame.collapsed = False
            if self._test_button:
                self._test_button.enabled = True
        if event.event_name == omni.timeline.GLOBAL_EVENT_STOP:
            self._gains_tuning_frame.collapsed = False
            if self._test_button:
                self._test_button.enabled = False

    def on_physics_step(self, step: float) -> None:
        """Callback for Physics Step.

        Physics steps only occur when the timeline is playing.

        Args:
            step: Size of physics step
        """

    def on_render_step(self, e: carb.events.IEvent) -> None:
        """Render event set up to cancel physics subscriptions that run the gains test.

        Args:
            e: Event object
        """
        if not self._articulation_menu_model or not self._articulation_menu_model.has_item():
            return
        if self._reset_ui_next_frame:
            if self._timeline.is_stopped():
                self._gains_tuning_frame.rebuild()
            if self._make_plot_on_next_frame:
                # TODO: rebuild plot here
                self._charts_frame.enabled = True
                self._charts_frame.visible = True
                self._charts_frame.rebuild()

            self._gains_tuning_frame.enabled = True
            self._test_gains_frame.enabled = True
            self._reset_ui_next_frame = False

    def on_stage_event(self, event: object) -> None:
        """Callback for Stage Events.

        Args:
            event: Event Type
        """
        if self._restarting_for_override:
            return
        if event.event_name == omni.usd.get_context().stage_event_name(
            omni.usd.StageEventType.ASSETS_LOADED
        ):  # Any asset added or removed
            items = self._populate_robot_menu()
            if self._articulation_menu_model:
                self._articulation_menu_model.refresh_list(items)
        elif event.event_name == omni.usd.get_context().stage_event_name(
            omni.usd.StageEventType.SIMULATION_START_PLAY
        ):  # Timeline played
            pass
        elif event.event_name == omni.usd.get_context().stage_event_name(
            omni.usd.StageEventType.SIMULATION_STOP_PLAY
        ):  # Timeline stopped
            self._reset_ui_next_frame = True

    def reset(self) -> None:
        """Called when the stage is closed or the extension is hot reloaded.

        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called.
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()
        self._gains_tuner.reset()

    def cleanup(self) -> None:
        """Called when the extension is closed.

        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called.
        """
        self.reset()
        self._gains_tuning_frame = None
        self._test_gains_frame = None
        self._charts_frame = None
        self._articulation_menu_model = None
        self._gains_table_widget = None
        self._test_table_widget = None

    def _on_help_click(self, b: int) -> None:
        """Opens an extension's documentation in a Web Browser.

        Args:
            b: Button event parameter
        """
        import webbrowser

        doc_link = (
            "https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html"
        )
        try:
            webbrowser.open(doc_link, new=2)
        except Exception as e:
            carb.log_warn(f"Could not open browswer with url: {doc_link}, {e}")

    def _populate_robot_menu(self) -> list:
        """Populates the robot selection menu with available robot prims from the stage.

        Returns:
            List of robot prim paths found on the stage.
        """
        items = []
        stage = omni.usd.get_context().get_stage()
        if stage:
            for prim in pxr.Usd.PrimRange(stage.GetPrimAtPath("/")):
                # Get prim type get_prim_object_type
                if prim.HasAPI(Classes.ROBOT_API.value):
                    path = str(prim.GetPath())
                    items.append(path)
        return items

    def build_ui(self) -> None:
        """Builds the complete user interface for the gains tuner extension.

        Creates the robot selection dropdown, gains tuning frame, test gains frame,
        and charts frame with all necessary UI components and event handlers.
        """
        with ui.VStack(style=get_style(), spacing=5, height=0):
            with ui.HStack(height=34):
                ui.Label("Robot Selection", name="robot_header")
                ui.Image(
                    name="help",
                    height=28,
                    width=28,
                    mouse_pressed_fn=lambda x, y, b, a: self._on_help_click(b),
                )

            def _update_articulation_selection(m: object, n: object) -> None:
                if m.has_item():
                    self._on_articulation_selection(m.get_current_string())
                else:
                    self._on_articulation_selection(None)

            self._articulation_menu_model = create_combo_list_model([], 0)
            ui.ComboBox(self._articulation_menu_model, name="articulation_menu")
            self._articulation_menu_model.add_item_changed_fn(_update_articulation_selection)

        self._gains_tuning_frame = CollapsableFrame(
            "Tune Gains",
            collapsed=False,
            enabled=True,
            build_fn=self._build_gains_tuning_frame,
            show_copy_button=False,
        )

        self._test_gains_frame = CollapsableFrame(
            "Test Gains Settings", collapsed=False, enabled=True, build_fn=self._build_test_gains_frame
        )

        self._charts_frame = CollapsableFrame("Charts", collapsed=True, enabled=True, build_fn=self._build_charts_frame)

        self._populate_robot_menu()

    def _build_gains_tuning_frame(self) -> None:
        """Builds the UI frame for tuning gains.

        Constructs the gains tuning interface including joint selection controls, stiffness/natural frequency mode
        switching, gains table widget, and save functionality. The frame displays joint entries from the gains tuner
        and provides controls for editing gain values.
        """
        with self._gains_tuning_frame:
            if self._gains_tuner._robot_prim_path is None:
                ui.Spacer(height=10)
                ui.Label("No robot selected/found", style={"color": 0xFF6666FF})

            else:
                with ui.VStack(style=get_style(), spacing=5, height=0, width=ui.Fraction(1)):
                    with ui.HStack():
                        ui.Spacer(width=10)
                        self._edit_mode_collection = ui.RadioCollection()
                        with ui.HStack(width=0):
                            with ui.HStack(width=0):
                                with ui.VStack(width=0):
                                    ui.Spacer()
                                    ui.RadioButton(width=20, height=20, radio_collection=self._edit_mode_collection)
                                    ui.Spacer()
                                ui.Spacer(width=4)
                                ui.Label(
                                    "Stiffness",
                                    width=0,
                                    mouse_pressed_fn=lambda x, y, m, w: self._edit_mode_collection.model.set_value(0),
                                )
                            ui.Spacer(width=10)
                            with ui.HStack(width=0):
                                with ui.VStack(width=0):
                                    ui.Spacer()
                                    ui.RadioButton(width=20, height=20, radio_collection=self._edit_mode_collection)
                                    ui.Spacer()
                                ui.Spacer(width=4)
                                ui.Label(
                                    "Natural Frequency",
                                    mouse_pressed_fn=lambda x, y, m, w: self._edit_mode_collection.model.set_value(1),
                                )
                                ui.Spacer(width=20)

                        self._edit_mode_collection.model.set_value(0)
                        self._edit_mode_collection.model.add_value_changed_fn(lambda m: self._switch_tuning_mode(m))

                    with ui.ZStack(height=self._initial_table_height, width=ui.Fraction(1)):
                        with ui.VStack(width=ui.Fraction(1)):
                            joint_entries = self._gains_tuner.get_joint_entries()
                            self._gains_table_widget = JointWidget(
                                joint_entries,
                                lambda joint: self._gains_tuner._joint_accumulated_inertia.get(joint, 0.0),
                            )
                            if self._gains_tuner._joint_accumulated_inertia:
                                self._gains_table_widget.refresh_inertia_derived_columns()

                        self._gains_splitter = ui.Placer(
                            offset_y=self._initial_table_height,
                            drag_axis=ui.Axis.Y,
                            draggable=True,
                        )
                        with self._gains_splitter:
                            ui.Rectangle(height=4, style_type_name_override="Splitter")
                        self._gains_splitter.set_offset_y_changed_fn(self._on_gains_splitter_dragged)

                    ui.Spacer(height=5)
                    with ui.HStack(width=ui.Fraction(1)):
                        ui.Spacer(width=ui.Fraction(1))
                        ui.Button(
                            "Save Gains to Physics Layer".upper(),
                            height=30,
                            width=210,
                            clicked_fn=self._on_save_gains_to_physics_layer,
                        )
                        ui.Spacer(width=5)
                        ui.Spacer(width=10)
                        with ui.Frame(width=0):
                            with ui.VStack():
                                ui.Spacer()
                                with ui.Placer(offset_x=0, offset_y=5):
                                    ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
                                ui.Spacer()
                        ui.Spacer(width=5)
                    ui.Spacer(height=5)
                    self._no_permision_frame = ui.Frame(width=ui.Fraction(1))
                    with self._no_permision_frame:
                        with ui.HStack(width=ui.Fraction(1)):
                            ui.Spacer(width=ui.Fraction(1))
                            ui.Label(
                                "Physics Layer not found or No edit permission.\n Ctrl-S (File > Save) to save the changes to new file.",
                                width=0,
                                alignment=ui.Alignment.RIGHT_CENTER,
                                style={"color": 0xFF6666FF},
                            )
                            ui.Spacer(width=5)
                            ui.Spacer(width=10)
                            with ui.Frame(width=0):
                                with ui.VStack():
                                    ui.Spacer()
                                    with ui.Placer(offset_x=0, offset_y=5):
                                        ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
                                    ui.Spacer()
                            ui.Spacer(width=5)
                    self._no_permision_frame.visible = False
                    ui.Spacer(height=5)

    def _on_gains_splitter_dragged(self, position_y: int) -> None:
        """Handles dragging of the gains frame splitter.

        Ensures the splitter position does not go below the initial table height.

        Args:
            position_y: The new Y position of the splitter.
        """
        if self._gains_splitter.offset_y.value < self._initial_table_height:
            self._gains_splitter.offset_y = ui.Pixel(self._initial_table_height)

    def _on_save_gains_to_physics_layer(self, *args: object) -> None:
        """Saves gain values to the physics layer.

        Collects joint gain data from the table widget and attempts to save the values to the appropriate USD
        layers. Shows the stage save dialog for layers with edit permissions or displays a warning for layers
        without permissions.

        Args:
            *args: Variable arguments passed from the UI callback.
        """
        joint_gains = self._gains_table_widget.model.get_item_children()
        self._no_permision_frame.visible = False
        stage = omni.usd.get_context().get_stage()
        edits, physics_layer = collect_gain_save_edits(joint_gains, stage)

        def on_save(edits: dict[str, list[tuple[Sdf.Path, object]]], selected_layers: list, comment: str = "") -> None:
            for layer_path in selected_layers:
                layer_edits = edits.get(layer_path)
                if not layer_edits:
                    continue
                layer = find_layer_by_save_identifier(layer_path)
                open_path = get_layer_save_identifier(layer) or layer_path
                edit_stage = Usd.Stage.Open(open_path)
                for path, value in layer_edits:
                    attr = edit_stage.GetAttributeAtPath(path)
                    if attr:
                        attr.Set(value)
                edit_stage.Save()

        savable_layers = []
        for layer_id in edits:
            layer = find_layer_by_save_identifier(layer_id)
            if is_layer_savable(layer):
                savable_layers.append(layer_id)
            else:
                self._no_permision_frame.visible = True

        if physics_layer is not None and is_layer_savable(physics_layer):
            physics_id = get_layer_save_identifier(physics_layer)
            if physics_id is None:
                pass
            elif physics_id in savable_layers:
                savable_layers = [physics_id] + [layer_id for layer_id in savable_layers if layer_id != physics_id]
            elif physics_id in edits:
                savable_layers = [physics_id]

        if self._save_stage_prompt:
            self._save_stage_prompt.destroy()
        self._save_stage_prompt = StageSaveDialog(
            enable_dont_save=True,
            on_save_fn=partial(on_save, edits),
        )
        if savable_layers:
            self._save_stage_prompt.show(savable_layers)
        else:
            self._no_permision_frame.visible = True

    def _on_save_file(self, *args: object) -> None:
        """Handles saving the USD stage file.

        Identifies writable layers in the stage and shows the stage save dialog. Displays a warning if no
        layers have edit permissions.

        Args:
            *args: Variable arguments passed from the UI callback.
        """
        writable_layers = []
        stage = omni.usd.get_context().get_stage()
        layers = stage.GetLayerStack()
        for layer in layers:
            if layer.permissionToEdit and layer.permissionToSave:
                writable_layers.append(layer.identifier)
            else:
                self._no_permision_frame.visible = True

        if self._save_stage_prompt:
            self._save_stage_prompt.destroy()
        self._save_stage_prompt = StageSaveDialog(
            enable_dont_save=True,
        )

        self._save_stage_prompt.show(writable_layers)

    def _switch_tuning_mode(self, switch: object) -> None:
        """Switches the tuning mode between stiffness and natural frequency.

        Updates the gains table widget to display the appropriate tuning parameters based on the selected mode.

        Args:
            switch: The radio button model containing the selected mode value.
        """
        if self._gains_table_widget:
            self._gains_table_widget.switch_mode(JointSettingMode(switch.get_value_as_int()))

    def _on_test_duration_changed(self, model: object) -> None:
        """Handles changes to the test duration field.

        Updates the gains tuner's test duration when the user modifies the duration field value.

        Args:
            model: The float field model containing the new duration value.
        """
        self._gains_tuner.test_duration = model.get_value_as_float()

    def _build_test_gains_frame(self) -> None:
        """Builds the UI frame for testing gains settings.

        Constructs the test gains interface including test duration controls, test mode selection (sinewave or step
        function), test table widget, and test execution buttons. Requires the gains tuner to be initialized.
        """
        if not self._gains_tuner.initialized:
            if self._articulation_menu_model.has_item():
                self._gains_tuner.initialize()
            else:
                self._test_table_widget = None
                ui.Label("Start Simulation to run Tests", name="gains_tuner_not_initialized")
                return
        with ui.VStack(style=get_style(), spacing=5, height=0, width=ui.Fraction(1)):
            with ui.HStack():
                self._test_duration_frame = ui.Frame(width=0, visible=False)
                with self._test_duration_frame:
                    with ui.HStack(width=0):
                        ui.Spacer(width=10)
                        ui.Label("Test Duration (s)", width=0)
                        ui.Spacer(width=10)
                        self._test_duration_field = ui.FloatField(
                            value=5.0,
                            height=20,
                            width=100,
                            step=0.1,
                            min_value=0.0,
                            max_value=10.0,
                        )
                        self._test_duration_field.model.set_value(5)
                        self._gains_tuner.set_test_duration(5)
                        self._test_duration_field.model.add_value_changed_fn(
                            lambda m: self._on_test_duration_changed(m)
                        )

                # Test Mode
                ui.Spacer(width=10)
                self._test_mode_collection = ui.RadioCollection()
                with ui.HStack(width=0):
                    with ui.HStack(width=0):
                        with ui.VStack(width=0):
                            ui.Spacer()
                            ui.RadioButton(width=20, height=20, radio_collection=self._test_mode_collection)
                            ui.Spacer()
                        ui.Spacer(width=4)
                        ui.Label(
                            "Snap to Limits",
                            width=0,
                            mouse_pressed_fn=lambda x, y, m, w: self._test_mode_collection.model.set_value(0),
                        )
                    ui.Spacer(width=10)
                    with ui.HStack(width=0):
                        with ui.VStack(width=0):
                            ui.Spacer()
                            ui.RadioButton(width=20, height=20, radio_collection=self._test_mode_collection)
                            ui.Spacer()
                        ui.Spacer(width=4)
                        ui.Label(
                            "Sinewave",
                            mouse_pressed_fn=lambda x, y, m, w: self._test_mode_collection.model.set_value(1),
                        )
                    ui.Spacer(width=10)
                    with ui.HStack(width=0):
                        with ui.VStack(width=0):
                            ui.Spacer()
                            ui.RadioButton(width=20, height=20, radio_collection=self._test_mode_collection)
                            ui.Spacer()
                        ui.Spacer(width=4)
                        ui.Label(
                            "Step Function",
                            mouse_pressed_fn=lambda x, y, m, w: self._test_mode_collection.model.set_value(2),
                        )
                    ui.Spacer(width=10)
                    with ui.HStack(width=0):
                        with ui.VStack(width=0):
                            ui.Spacer()
                            ui.RadioButton(width=20, height=20, radio_collection=self._test_mode_collection)
                            ui.Spacer()
                        ui.Spacer(width=4)
                        ui.Label(
                            "Stress Test",
                            mouse_pressed_fn=lambda x, y, m, w: self._test_mode_collection.model.set_value(3),
                        )
                    ui.Spacer(width=10)
                self._radio_to_mode = [
                    GainsTestMode.SNAP_TO_LIMITS,
                    GainsTestMode.SINUSOIDAL,
                    GainsTestMode.STEP,
                    GainsTestMode.STRESS_TEST,
                ]
                self._test_mode_collection.model.add_value_changed_fn(lambda m: self._switch_test_mode(m))
                self._test_mode_collection.model.set_value(0)
                self._test_mode = GainsTestMode.SNAP_TO_LIMITS

                # ui.Spacer(width=30)
                # self._radian_degree_collection = ui.RadioCollection()
                # with ui.HStack(width=0):
                #     with ui.HStack(width=0):
                #         with ui.VStack(width=0):
                #             ui.Spacer()
                #             ui.RadioButton(width=20, height=20, radio_collection=self._radian_degree_collection)
                #             ui.Spacer()
                #         ui.Spacer(width=4)
                #         ui.Label(
                #             "Radians",
                #             width=0,
                #             mouse_pressed_fn=lambda x, y, m, w: self._radian_degree_collection.model.set_value(0),
                #         )
                #     ui.Spacer(width=10)
                #     with ui.HStack(width=0):
                #         with ui.VStack(width=0):
                #             ui.Spacer()
                #             ui.RadioButton(width=20, height=20, radio_collection=self._radian_degree_collection)
                #             ui.Spacer()
                #         ui.Spacer(width=4)
                #         ui.Label(
                #             "Degrees",
                #             mouse_pressed_fn=lambda x, y, m, w: self._radian_degree_collection.model.set_value(1),
                #         )
                #     ui.Spacer(width=10)
                # self._radian_degree_collection.model.set_value(0)
                # self._radian_degree_collection.model.add_value_changed_fn(lambda m: self._switch_radian_degree(m))

            self._snap_settings_frame = ui.Frame(visible=True)
            with self._snap_settings_frame:
                with ui.VStack(height=0, spacing=4):
                    with ui.HStack(height=0):
                        ui.Spacer(width=10)
                        ui.Label("Hold Duration (s)", width=0)
                        ui.Spacer(width=10)
                        self._hold_duration_field = ui.FloatField(
                            height=20,
                            width=100,
                            step=0.1,
                            min_value=0.1,
                            max_value=5.0,
                        )
                        self._hold_duration_field.model.set_value(1.0)
                        ui.Spacer(width=20)
                        ui.Label("Tolerance (rad/m)", width=0)
                        ui.Spacer(width=10)
                        self._tolerance_field = ui.FloatField(
                            height=20,
                            width=100,
                            step=0.001,
                            min_value=0.001,
                            max_value=1.0,
                        )
                        self._tolerance_field.model.set_value(0.01)
                        ui.Spacer(width=ui.Fraction(1))
                    with ui.HStack(height=0):
                        ui.Spacer(width=10)
                        self._disable_self_collisions_cb = ui.CheckBox(width=20, height=20)
                        self._disable_self_collisions_cb.model.set_value(False)
                        ui.Spacer(width=4)
                        ui.Label("Disable Self-Collisions During Test", width=0)
                        ui.Spacer(width=ui.Fraction(1))
                    with ui.HStack(height=0):
                        ui.Spacer(width=10)
                        self._disable_velocity_limits_cb = ui.CheckBox(width=20, height=20)
                        self._disable_velocity_limits_cb.model.set_value(False)
                        ui.Spacer(width=4)
                        ui.Label("Disable Velocity Limits During Test", width=0)
                        ui.Spacer(width=ui.Fraction(1))

            self._stress_test_settings_frame = ui.Frame(visible=False)
            with self._stress_test_settings_frame:
                with ui.VStack(height=0, spacing=4):
                    with ui.HStack(height=0):
                        ui.Spacer(width=10)
                        ui.Label("Sub-mode", width=0)
                        ui.Spacer(width=10)
                        self._stress_test_submode_combo = ui.ComboBox(
                            0, "Random Walk", "Adversarial", width=140, height=20
                        )
                        self._stress_test_submode_combo.model.add_item_changed_fn(
                            lambda m, i: self._on_stress_test_submode_changed(m)
                        )
                        ui.Spacer(width=20)
                        ui.Label("Duration (s)", width=0)
                        ui.Spacer(width=10)
                        self._stress_test_duration_field = ui.FloatField(
                            height=20, width=80, step=1.0, min_value=0.1, max_value=600.0
                        )
                        self._stress_test_duration_field.model.set_value(10.0)
                        ui.Spacer(width=20)
                        ui.Label("Seed", width=0)
                        ui.Spacer(width=10)
                        self._stress_test_seed_field = ui.IntField(height=20, width=80)
                        self._stress_test_seed_field.model.set_value(42)
                        ui.Spacer(width=ui.Fraction(1))
                    with ui.HStack(height=0):
                        ui.Spacer(width=10)
                        ui.Label("Vel Threshold (rad/s)", width=0)
                        ui.Spacer(width=10)
                        self._stress_test_vel_threshold_field = ui.FloatField(
                            height=20, width=80, step=1.0, min_value=0.1
                        )
                        self._stress_test_vel_threshold_field.model.set_value(100.0)
                        ui.Spacer(width=20)
                        self._stress_test_sigma_frame = ui.Frame(visible=True, width=0)
                        with self._stress_test_sigma_frame:
                            with ui.HStack(width=0):
                                ui.Label("Sigma (% range)", width=0)
                                ui.Spacer(width=10)
                                self._stress_test_sigma_field = ui.FloatField(
                                    height=20, width=80, step=0.1, min_value=0.01, max_value=100.0
                                )
                                self._stress_test_sigma_field.model.set_value(1.0)
                        self._stress_test_snap_interval_frame = ui.Frame(visible=False, width=0)
                        with self._stress_test_snap_interval_frame:
                            with ui.HStack(width=0):
                                ui.Label("Snap Interval (steps)", width=0)
                                ui.Spacer(width=10)
                                self._stress_test_snap_interval_field = ui.IntField(height=20, width=80)
                                self._stress_test_snap_interval_field.model.set_value(10)
                        ui.Spacer(width=ui.Fraction(1))
                    with ui.HStack(height=0):
                        ui.Spacer(width=10)
                        self._stress_test_disable_self_collisions_cb = ui.CheckBox(width=20, height=20)
                        self._stress_test_disable_self_collisions_cb.model.set_value(False)
                        ui.Spacer(width=4)
                        ui.Label("Disable Self-Collisions During Test", width=0)
                        ui.Spacer(width=ui.Fraction(1))
                    with ui.HStack(height=0):
                        ui.Spacer(width=10)
                        self._stress_test_disable_velocity_limits_cb = ui.CheckBox(width=20, height=20)
                        self._stress_test_disable_velocity_limits_cb.model.set_value(False)
                        ui.Spacer(width=4)
                        ui.Label("Disable Velocity Limits During Test", width=0)
                        ui.Spacer(width=ui.Fraction(1))

            with ui.ZStack(height=self._initial_table_height, width=ui.Fraction(1)):
                with ui.VStack(width=ui.Fraction(1)):
                    self._test_table_widget = TestJointWidget(self._gains_tuner)
                    self._test_table_widget.switch_mode(int(self._test_mode))

                self._test_splitter = ui.Placer(
                    offset_y=self._initial_table_height,
                    drag_axis=ui.Axis.Y,
                    draggable=True,
                )
                with self._test_splitter:
                    ui.Rectangle(height=4, style_type_name_override="Splitter")
                self._test_splitter.set_offset_y_changed_fn(self._on_test_splitter_dragged)

            ui.Spacer(height=10)
            with ui.HStack(height=40):

                ui.Spacer(width=ui.Fraction(1))
                with ui.Frame(width=0):
                    self._test_button = StateButton(
                        "",
                        "Run Test",
                        "Cancel Test",
                        on_a_click_fn=partial(self._on_run_gains_test, self._test_mode),
                        on_b_click_fn=self._on_cancel_gains_test,
                        physics_callback_fn=self._update_gains_test,
                    )
                # ui.Spacer(width=10)
                self._test_button.enabled = self._timeline.is_playing()

    def _on_test_splitter_dragged(self, position_y: int) -> None:
        """Handles dragging of the test frame splitter.

        Ensures the splitter position does not go below the initial table height.

        Args:
            position_y: The new Y position of the splitter.
        """
        if self._test_splitter.offset_y.value < self._initial_table_height:
            self._test_splitter.offset_y = ui.Pixel(self._initial_table_height)

    def _on_select_all(self) -> None:
        """Selects all joints in the test table widget.

        Enables testing for all available joints in the test gains interface.
        """
        self._test_table_widget.select_all()

    def _on_clear_all(self) -> None:
        """Clears all joint selections in the test table widget.

        Disables testing for all joints in the test gains interface.
        """
        self._test_table_widget.clear_all()

    def _on_cancel_gains_test(self) -> None:
        """Cancels the running gains test and resets the test UI state."""
        self._test_running = False
        if self._test_button:
            self._test_button.reset()
        self._gains_tuner.stop_test()
        self._restore_self_collision_override()

    def _switch_test_mode(self, switch: object) -> None:
        """Switches the test mode between different gain testing options.

        Args:
            switch: The switch object containing the selected test mode value.
        """
        radio_idx = switch.get_value_as_int()
        mode = self._radio_to_mode[radio_idx] if radio_idx < len(self._radio_to_mode) else GainsTestMode.SINUSOIDAL
        self._test_mode = mode
        self._gains_tuner.test_mode = int(mode)
        if self._test_table_widget:
            self._test_table_widget.switch_mode(int(mode))
        is_snap = mode == GainsTestMode.SNAP_TO_LIMITS
        is_stress_test = mode == GainsTestMode.STRESS_TEST
        if self._snap_settings_frame is not None:
            self._snap_settings_frame.visible = is_snap
        if self._stress_test_settings_frame is not None:
            self._stress_test_settings_frame.visible = is_stress_test
        if self._test_duration_frame is not None:
            self._test_duration_frame.visible = not is_snap and not is_stress_test

    def _on_stress_test_submode_changed(self, model: object) -> None:
        """Toggle sigma / snap-interval fields based on stress test sub-mode."""
        is_random_walk = model.get_item_value_model().get_value_as_int() == 0
        if self._stress_test_sigma_frame is not None:
            self._stress_test_sigma_frame.visible = is_random_walk
        if self._stress_test_snap_interval_frame is not None:
            self._stress_test_snap_interval_frame.visible = not is_random_walk

    # def _switch_radian_degree(self, switch):
    #     radian_degree_mode = switch.get_value_as_int()
    #     if self._test_table_widget:
    #         self._test_table_widget.switch_radian_degree(radian_degree_mode)

    def _build_charts_frame(self) -> None:
        """Builds the charts frame UI with joint color selection and position/velocity plots."""
        if not self._gains_tuner.initialized:
            return
        with ui.HStack(style=get_style(), spacing=5, height=0):
            self._color_joint_widget = ColorJointWidget(
                self._gains_tuner, selected_changed_fn=self._on_color_joint_selection
            )
            if not self._gains_tuner.is_data_ready():
                return
            self._charts_frame.collapsed = False
            with ui.VStack(style=get_style(), spacing=5, height=0):

                metrics = self._gains_tuner.get_test_result_metrics()
                if metrics:
                    first = next(iter(metrics.values()), {})
                    is_stress_test = first.get("test_mode") in ("random_walk", "adversarial")
                    if is_stress_test:
                        self._results_frame = CollapsableFrame(
                            "Stress Test Results", collapsed=False, show_copy_button=False
                        )
                        self._results_frame.set_build_fn(self._build_stress_test_results)
                    else:
                        self._results_frame = CollapsableFrame(
                            "Snap to Limits Results", collapsed=False, show_copy_button=False
                        )
                        self._results_frame.set_build_fn(self._build_snap_results)

                self._position_frame = CollapsableFrame("Position charts", collapsed=False, show_copy_button=False)
                self._position_frame.set_build_fn(self._build_position_plot)

                self._velocity_frame = CollapsableFrame("Velocity charts", collapsed=False, show_copy_button=False)
                self._velocity_frame.set_build_fn(self._build_velocity_plot)

    def _build_snap_results(self) -> None:
        """Builds a per-joint results table for the snap-to-limits test."""
        metrics = self._gains_tuner.get_test_result_metrics()
        if not metrics:
            return

        entries = self._gains_tuner.get_joint_entries()
        dof_names = {e.dof_index: e.display_name for e in entries}

        from omni.physics.tensors import DofType

        _STATUS_COLORS = {
            "pass": 0xFF00FF00,
            "blocked": 0xFF00CCFF,
            "fail": 0xFF4444FF,
        }

        with ui.VStack(style=get_style(), spacing=2, height=0):
            with ui.HStack(height=20):
                ui.Label("Joint", width=ui.Fraction(3), name="header", style_type_name_override="TreeView")
                ui.Label("Lower Mean", width=ui.Fraction(2), name="header", style_type_name_override="TreeView")
                ui.Label("Lower Max", width=ui.Fraction(2), name="header", style_type_name_override="TreeView")
                ui.Label("Lower Settle", width=ui.Fraction(2), name="header", style_type_name_override="TreeView")
                ui.Label("Upper Mean", width=ui.Fraction(2), name="header", style_type_name_override="TreeView")
                ui.Label("Upper Max", width=ui.Fraction(2), name="header", style_type_name_override="TreeView")
                ui.Label("Upper Settle", width=ui.Fraction(2), name="header", style_type_name_override="TreeView")
                ui.Label("Result", width=ui.Fraction(1), name="header", style_type_name_override="TreeView")

            any_failed = False
            any_blocked = False
            for dof_idx in sorted(metrics.keys()):
                m = metrics[dof_idx]
                name = dof_names.get(dof_idx, f"DOF {dof_idx}")
                status = m.get("status", "fail")
                if status == "fail":
                    any_failed = True
                elif status == "blocked":
                    any_blocked = True

                scale = 1.0
                unit = "m"
                if self._gains_tuner._articulation.dof_types[dof_idx] == DofType.Rotation:
                    scale = 180.0 / np.pi
                    unit = "deg"

                lower_mean = m.get("lower_position_error", float("inf")) * scale
                lower_max = m.get("lower_max_error", float("inf")) * scale
                upper_mean = m.get("upper_position_error", float("inf")) * scale
                upper_max = m.get("upper_max_error", float("inf")) * scale
                lower_settle = m.get("lower_settling_time", float("nan"))
                upper_settle = m.get("upper_settling_time", float("nan"))
                lower_settle_str = f"{lower_settle:.3f} s" if lower_settle == lower_settle else "N/A"
                upper_settle_str = f"{upper_settle:.3f} s" if upper_settle == upper_settle else "N/A"
                result_text = status.upper()
                result_color = _STATUS_COLORS.get(status, 0xFF4444FF)

                with ui.HStack(height=20):
                    ui.Label(name, width=ui.Fraction(3))
                    ui.Label(f"{lower_mean:.4f} {unit}", width=ui.Fraction(2))
                    ui.Label(f"{lower_max:.4f} {unit}", width=ui.Fraction(2))
                    ui.Label(lower_settle_str, width=ui.Fraction(2))
                    ui.Label(f"{upper_mean:.4f} {unit}", width=ui.Fraction(2))
                    ui.Label(f"{upper_max:.4f} {unit}", width=ui.Fraction(2))
                    ui.Label(upper_settle_str, width=ui.Fraction(2))
                    ui.Label(result_text, width=ui.Fraction(1), style={"color": result_color})

            ui.Spacer(height=5)
            if any_failed:
                summary_text = "SOME JOINTS FAILED"
                summary_color = _STATUS_COLORS["fail"]
            elif any_blocked:
                summary_text = "ALL GAINS OK — SOME JOINTS BLOCKED"
                summary_color = _STATUS_COLORS["blocked"]
            else:
                summary_text = "ALL JOINTS PASSED"
                summary_color = _STATUS_COLORS["pass"]
            ui.Label(summary_text, height=20, style={"color": summary_color, "font_size": 14})
            ui.Spacer(height=5)

    def _on_color_joint_selection(self, selection: list) -> None:
        """Handles joint selection changes for plotting and updates the chart colors.

        Args:
            selection: List of selected joint items with associated colors and indices.
        """
        self._plotting_indices = [item.joint_index for item in selection]
        group_colors = [item.colors for item in selection]
        self._plotting_colors = []
        for i in range(2):
            for colors in group_colors:
                self._plotting_colors.append(colors[i])
        if self._position_frame:
            self._position_frame.rebuild()
        if self._velocity_frame:
            self._velocity_frame.rebuild()

    def _build_position_plot(self) -> None:
        """Builds the position plot chart with command and observed position data from the gains test."""
        if len(self._plotting_indices) == 0:
            return
        joint_indices = self._plotting_indices
        cmd_pos_list = []
        obs_pos_list = []
        cmd_vel_list = []
        obs_vel_list = []
        cmd_times_list = []
        scale = [1.0] * len(joint_indices)
        for i, joint_index in enumerate(joint_indices):
            if self._gains_tuner._articulation.dof_types[joint_index] == DofType.Rotation:
                scale[i] = 180.0 / np.pi
        for i, joint_index in enumerate(joint_indices):
            cmd_pos, cmd_vel, obs_pos, obs_vel, cmd_times = self._gains_tuner.get_joint_states_from_gains_test(
                joint_index
            )
            if cmd_pos is None:
                continue
            cmd_pos_list.append(cmd_pos * scale[i])
            obs_pos_list.append(obs_pos * scale[i])
            cmd_vel_list.append(cmd_vel * scale[i])
            obs_vel_list.append(obs_vel * scale[i])
            cmd_times_list.append(cmd_times)

        plot = CustomXYPlot(
            show_legend=True,
            x_data=cmd_times_list + cmd_times_list,
            # TODO: not sure how to get user  data here, use obs_pos_list for now
            y_data=cmd_pos_list + obs_pos_list,
            data_colors=self._plotting_colors,
            header_count=2,
        )
        plot.set_data_colors(self._plotting_colors)

    def _build_velocity_plot(self) -> None:
        """Builds the velocity plot chart with command and observed velocity data from the gains test."""
        if len(self._plotting_indices) == 0:
            return
        joint_indices = self._plotting_indices
        cmd_pos_list = []
        cmd_vel_list = []
        obs_pos_list = []
        obs_vel_list = []
        cmd_times_list = []
        for joint_index in joint_indices:
            cmd_pos, cmd_vel, obs_pos, obs_vel, cmd_times = self._gains_tuner.get_joint_states_from_gains_test(
                joint_index
            )
            if cmd_pos is None:
                continue
            cmd_pos_list.append(cmd_pos)
            cmd_vel_list.append(cmd_vel)
            obs_pos_list.append(obs_pos)
            obs_vel_list.append(obs_vel)
            cmd_times_list.append(cmd_times)
        plot = CustomXYPlot(
            show_legend=True,
            x_data=cmd_times_list + cmd_times_list,
            # TODO: not sure how to get user  data here, use obs_vel_list for now
            y_data=cmd_vel_list + obs_vel_list,
            data_colors=self._plotting_colors,
            header_count=2,
        )
        plot.set_data_colors(self._plotting_colors)

    def _invalidate_articulation(self) -> None:
        """This function handles the event that the existing articulation becomes invalid and there is.

        not a new articulation to select.  It is called explicitly in the code when the timeline is
        stopped and when the DropDown menu finds no articulations on the stage.
        """
        self._gains_tuner.reset()
        self._gains_tuning_frame.rebuild()

    def _on_joint_inertia_updated(self) -> None:
        """Refresh NF/DR table columns after deferred inertia computation completes."""
        if self._gains_table_widget is not None:
            self._gains_table_widget.refresh_inertia_derived_columns()

    def _on_articulation_selection(self, articulation_path: str) -> None:
        """Handles articulation selection changes and updates the gains tuner with the new robot.

        Args:
            articulation_path: USD path to the selected articulation prim.
        """
        if articulation_path is None:
            self._invalidate_articulation()
            return

        if self._gains_tuner._robot_prim_path != articulation_path:
            self._gains_tuner.reset()
            self._gains_tuner.setup(articulation_path)
            self._gains_tuning_frame.rebuild()
            self._test_running = False
            self._test_mode = GainsTestMode.SNAP_TO_LIMITS
            if self._test_table_widget:
                self._test_table_widget.switch_mode(self._test_mode)
            self._test_gains_frame.rebuild()

            self._reset_ui_next_frame = True

    def _update_gains_test(self, step: float, context: object) -> None:
        """Updates the running gains test and handles test completion.

        Args:
            step: Physics step size.
            context: Physics step context.
        """
        if not self._test_running:
            return
        done = self._gains_tuner.update_gains_test(step)
        if done:
            self._test_running = False
            self._reset_ui_next_frame = True
            self._restore_self_collision_override()
            if self._gains_tuner.is_data_ready():
                self._make_plot_on_next_frame = True
                self._test_button.reset()
                metrics = self._gains_tuner.get_test_result_metrics()
                first = next(iter(metrics.values()), {}) if metrics else {}
                if first.get("test_mode") in ("random_walk", "adversarial"):
                    self._log_stress_test_results()
                else:
                    self._log_snap_results()

    def _log_snap_results(self) -> None:
        """Log snap-to-limits per-joint results to the console."""
        metrics = self._gains_tuner.get_test_result_metrics()
        if not metrics:
            return
        entries = self._gains_tuner.get_joint_entries()
        dof_names = {e.dof_index: e.display_name for e in entries}
        from omni.physics.tensors import DofType

        lines = ["Snap to Limits results:"]
        any_failed = False
        any_blocked = False
        for dof_idx in sorted(metrics.keys()):
            m = metrics[dof_idx]
            name = dof_names.get(dof_idx, f"DOF {dof_idx}")
            status = m.get("status", "fail")
            if status == "fail":
                any_failed = True
            elif status == "blocked":
                any_blocked = True
            scale = 1.0
            unit = "m"
            if self._gains_tuner._articulation.dof_types[dof_idx] == DofType.Rotation:
                scale = 180.0 / np.pi
                unit = "deg"
            lower_mean = m.get("lower_position_error", float("inf")) * scale
            lower_max = m.get("lower_max_error", float("inf")) * scale
            upper_mean = m.get("upper_position_error", float("inf")) * scale
            upper_max = m.get("upper_max_error", float("inf")) * scale
            lower_settle = m.get("lower_settling_time", float("nan"))
            upper_settle = m.get("upper_settling_time", float("nan"))
            lower_settle_str = f"{lower_settle:.3f}s" if lower_settle == lower_settle else "N/A"
            upper_settle_str = f"{upper_settle:.3f}s" if upper_settle == upper_settle else "N/A"
            lines.append(
                f"  {name}: lower(mean={lower_mean:.4f}, max={lower_max:.4f}){unit} settle={lower_settle_str}"
                f"  upper(mean={upper_mean:.4f}, max={upper_max:.4f}){unit} settle={upper_settle_str}"
                f"  [{status.upper()}]"
            )
        if any_failed:
            lines.append("  Overall: SOME JOINTS FAILED")
        elif any_blocked:
            lines.append("  Overall: ALL GAINS OK — SOME JOINTS BLOCKED")
        else:
            lines.append("  Overall: ALL PASSED")
        carb.log_info("\n".join(lines))

    def _build_stress_test_results(self) -> None:
        """Builds a per-joint results table for the stress test."""
        metrics = self._gains_tuner.get_test_result_metrics()
        if not metrics:
            return

        entries = self._gains_tuner.get_joint_entries()
        dof_names = {e.dof_index: e.display_name for e in entries}

        _STATUS_COLORS = {
            "stable": 0xFF00FF00,
            "unstable": 0xFF4444FF,
        }

        first = next(iter(metrics.values()), {})
        mode_str = first.get("test_mode", "random_walk")
        seed_val = first.get("seed", "?")

        with ui.VStack(style=get_style(), spacing=2, height=0):
            ui.Label(
                f"Mode: {mode_str.replace('_', ' ').title()}  |  Seed: {seed_val}",
                height=20,
                style={"font_size": 12},
            )
            ui.Spacer(height=4)
            with ui.HStack(height=20):
                ui.Label("Joint", width=ui.Fraction(3), name="header", style_type_name_override="TreeView")
                ui.Label("Max Vel", width=ui.Fraction(2), name="header", style_type_name_override="TreeView")
                ui.Label("Trigger Time", width=ui.Fraction(2), name="header", style_type_name_override="TreeView")
                ui.Label("Trigger Vel", width=ui.Fraction(2), name="header", style_type_name_override="TreeView")
                ui.Label("Result", width=ui.Fraction(1), name="header", style_type_name_override="TreeView")

            any_unstable = False
            for dof_idx in sorted(metrics.keys()):
                m = metrics[dof_idx]
                name = dof_names.get(dof_idx, f"DOF {dof_idx}")
                status = m.get("status", "stable")
                if status == "unstable":
                    any_unstable = True

                max_vel = m.get("max_velocity", 0.0)
                trigger_time = m.get("trigger_time", float("nan"))
                trigger_vel = m.get("trigger_velocity", float("nan"))
                trigger_time_str = f"{trigger_time:.3f} s" if trigger_time == trigger_time else "N/A"
                trigger_vel_str = f"{trigger_vel:.2f}" if trigger_vel == trigger_vel else "N/A"
                result_text = status.upper()
                result_color = _STATUS_COLORS.get(status, 0xFF4444FF)

                with ui.HStack(height=20):
                    ui.Label(name, width=ui.Fraction(3))
                    ui.Label(f"{max_vel:.2f}", width=ui.Fraction(2))
                    ui.Label(trigger_time_str, width=ui.Fraction(2))
                    ui.Label(trigger_vel_str, width=ui.Fraction(2))
                    ui.Label(result_text, width=ui.Fraction(1), style={"color": result_color})

            ui.Spacer(height=5)
            if any_unstable:
                summary_text = "INSTABILITY DETECTED"
                summary_color = _STATUS_COLORS["unstable"]
            else:
                summary_text = "ALL JOINTS STABLE"
                summary_color = _STATUS_COLORS["stable"]
            ui.Label(summary_text, height=20, style={"color": summary_color, "font_size": 14})
            ui.Spacer(height=5)

    def _log_stress_test_results(self) -> None:
        """Log stress test per-joint results to the console."""
        metrics = self._gains_tuner.get_test_result_metrics()
        if not metrics:
            return
        entries = self._gains_tuner.get_joint_entries()
        dof_names = {e.dof_index: e.display_name for e in entries}

        first = next(iter(metrics.values()), {})
        mode_str = first.get("test_mode", "random_walk")
        seed_val = first.get("seed", "?")

        lines = [f"Stress test results (mode={mode_str}, seed={seed_val}):"]
        any_unstable = False
        for dof_idx in sorted(metrics.keys()):
            m = metrics[dof_idx]
            name = dof_names.get(dof_idx, f"DOF {dof_idx}")
            status = m.get("status", "stable")
            if status == "unstable":
                any_unstable = True
            max_vel = m.get("max_velocity", 0.0)
            trigger_time = m.get("trigger_time", float("nan"))
            trigger_vel = m.get("trigger_velocity", float("nan"))
            trigger_time_str = f"{trigger_time:.3f}s" if trigger_time == trigger_time else "N/A"
            trigger_vel_str = f"{trigger_vel:.2f}" if trigger_vel == trigger_vel else "N/A"
            lines.append(
                f"  {name}: max_vel={max_vel:.2f}  trigger_time={trigger_time_str}"
                f"  trigger_vel={trigger_vel_str}  [{status.upper()}]"
            )
        if any_unstable:
            lines.append("  Overall: INSTABILITY DETECTED")
        else:
            lines.append("  Overall: ALL JOINTS STABLE")
        carb.log_info("\n".join(lines))

    def _get_self_collision_attr(self) -> Usd.Attribute | None:
        """Return the ``physxArticulation:enabledSelfCollisions`` USD attribute, or *None*."""
        articulation = self._gains_tuner._articulation
        if articulation is None:
            return None
        prim = articulation.prims[0]
        return prim.GetAttribute("physxArticulation:enabledSelfCollisions") or None

    def _restore_self_collision_override(self) -> None:
        """Restore the saved self-collision value on the USD prim.

        Safe to call even when no override was applied (no-op).
        Because ``enabledSelfCollisions`` is a cook-time property,
        a timeline stop/play cycle is required for PhysX to pick up
        the restored value.
        """
        if self._self_collision_original is None:
            return
        attr = self._get_self_collision_attr()
        if attr:
            attr.Set(self._self_collision_original)
        self._self_collision_original = None
        asyncio.ensure_future(self._restart_timeline_for_cook())

    async def _restart_timeline_for_cook(self) -> None:
        """Stop and restart the timeline so PhysX re-cooks cook-time properties."""
        app = omni.kit.app.get_app()
        self._restarting_for_override = True
        try:
            self._timeline.stop()
            for _ in range(3):
                await app.next_update_async()
            self._timeline.play()
            for _ in range(3):
                await app.next_update_async()
        finally:
            self._restarting_for_override = False
        self._gains_tuner.initialize()
        await app.next_update_async()

    async def _apply_override_and_run(self, test_params: dict) -> None:
        """Set the self-collision USD attribute, restart the timeline, and launch the test.

        ``physxArticulation:enabledSelfCollisions`` is a cook-time
        property — it only takes effect when physics loads the scene.
        A timeline stop/play cycle forces that reload.
        """
        app = omni.kit.app.get_app()

        attr = self._get_self_collision_attr()
        if attr:
            self._self_collision_original = attr.Get()
            attr.Set(False)

        self._restarting_for_override = True
        cancelled = False
        try:
            self._timeline.stop()
            for _ in range(3):
                await app.next_update_async()

            self._timeline.play()
            for _ in range(3):
                await app.next_update_async()

            if not self._test_button:
                cancelled = True
        finally:
            self._restarting_for_override = False

        if cancelled:
            self._restore_self_collision_override()
            return

        self._gains_tuner.initialize()
        await app.next_update_async()

        self._gains_tuner.initialize_gains_test(test_params)
        self._test_running = True

    def _on_run_gains_test(self, gains_test_mode: object) -> None:
        """Starts the gains test with the configured parameters from the test table.

        Args:
            gains_test_mode: The test mode to run for the gains test.
        """
        if not self._gains_tuner.initialized:
            self._gains_tuner.initialize()
        self._gains_tuning_frame.collapsed = True

        test_mode = self._test_table_widget.mode

        joint_params = [p for p in self._test_table_widget.model.get_item_children() if p.test]

        test_params = {
            "test_mode": test_mode,
            "joint_indices": [param.joint_index for param in joint_params],
        }

        if test_mode == GainsTestMode.SNAP_TO_LIMITS:
            test_params["hold_duration"] = self._hold_duration_field.model.get_value_as_float()
            test_params["tolerance"] = self._tolerance_field.model.get_value_as_float()
            test_params["disable_self_collisions"] = self._disable_self_collisions_cb.model.get_value_as_bool()
            test_params["disable_velocity_limits"] = self._disable_velocity_limits_cb.model.get_value_as_bool()
            test_params["sequence"] = [
                {
                    "joint_indices": np.array(
                        [param.joint_index for param in joint_params if param.sequence == i], dtype=np.int32
                    ),
                }
                for i in {p.sequence for p in joint_params}
            ]
        elif test_mode == GainsTestMode.STRESS_TEST:
            test_params["stress_test_mode"] = (
                self._stress_test_submode_combo.model.get_item_value_model().get_value_as_int()
            )
            test_params["duration"] = self._stress_test_duration_field.model.get_value_as_float()
            test_params["velocity_threshold"] = self._stress_test_vel_threshold_field.model.get_value_as_float()
            test_params["sigma"] = self._stress_test_sigma_field.model.get_value_as_float() / 100.0
            test_params["snap_interval"] = self._stress_test_snap_interval_field.model.get_value_as_int()
            test_params["seed"] = self._stress_test_seed_field.model.get_value_as_int()
            test_params["disable_self_collisions"] = (
                self._stress_test_disable_self_collisions_cb.model.get_value_as_bool()
            )
            test_params["disable_velocity_limits"] = (
                self._stress_test_disable_velocity_limits_cb.model.get_value_as_bool()
            )
            test_params["sequence"] = [
                {
                    "joint_indices": np.array(
                        [param.joint_index for param in joint_params if param.sequence == i], dtype=np.int32
                    ),
                }
                for i in {p.sequence for p in joint_params}
            ]
        else:
            test_params["test_duration"] = self._test_duration_field.model.get_value_as_float()
            test_params["sequence"] = [
                {
                    "joint_indices": np.array(
                        [param.joint_index for param in joint_params if param.sequence == i], dtype=np.int32
                    ),
                    "joint_amplitudes": np.array(
                        [param.amplitude * 0.005 for param in joint_params if param.sequence == i], dtype=np.float32
                    ),
                    "joint_offsets": np.array(
                        [param.offset / param.values_scale for param in joint_params if param.sequence == i],
                        dtype=np.float32,
                    ),
                    "joint_periods": np.array(
                        [param.period for param in joint_params if param.sequence == i], dtype=np.float32
                    ),
                    "joint_phases": np.array(
                        [param.phase for param in joint_params if param.sequence == i], dtype=np.float32
                    ),
                    "joint_step_max": np.array(
                        [param.step_max / param.values_scale for param in joint_params if param.sequence == i],
                        dtype=np.float32,
                    ),
                    "joint_step_min": np.array(
                        [param.step_min / param.values_scale for param in joint_params if param.sequence == i],
                        dtype=np.float32,
                    ),
                    "joint_user_provided": [param.user_provided for param in joint_params if param.sequence == i],
                }
                for i in {p.sequence for p in joint_params}
            ]

        if test_params.get("disable_self_collisions") and test_mode in (
            GainsTestMode.SNAP_TO_LIMITS,
            GainsTestMode.STRESS_TEST,
        ):
            asyncio.ensure_future(self._apply_override_and_run(test_params))
            return

        self._gains_tuner.initialize_gains_test(test_params)
        self._test_running = True

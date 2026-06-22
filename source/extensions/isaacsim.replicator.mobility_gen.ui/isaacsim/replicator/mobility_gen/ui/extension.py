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

"""Extension for generating mobility data using teleoperated robots in Omniverse."""

import asyncio
import glob
import os
import shutil
import tempfile
from typing import Any

import carb
import isaacsim.core.experimental.utils.app as app_utils

# import examples to register example robots / scenarios
import isaacsim.replicator.mobility_gen.examples  # noqa: F401
import omni.ext
import omni.kit
import omni.ui as ui
import omni.usd
from isaacsim.core.experimental.utils.stage import get_current_stage, open_stage_async
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.replicator.experimental.mobility_gen import (
    ROBOTS,
    SCENARIOS,
    GamepadDriver,
    KeyboardDriver,
    OccupancyMap,
    RecordingSession,
    collect_input,
    route_chase_through_ppisp,
)
from isaacsim.replicator.mobility_gen.examples import GamepadTeleoperationScenario, KeyboardTeleoperationScenario
from isaacsim.replicator.nurec_utils.rendering_setup import setup_for_rendering
from omni.kit.notification_manager import NotificationStatus, post_notification
from omni.kit.widget.filebrowser import FileBrowserItem
from omni.kit.window.filepicker import FilePickerDialog

if "MOBILITY_GEN_DATA" in os.environ:
    DATA_DIR = os.environ["MOBILITY_GEN_DATA"]
else:
    DATA_DIR = os.path.expanduser("~/MobilityGenData")

RECORDINGS_DIR = os.path.join(DATA_DIR, "recordings")
SCENARIOS_DIR = os.path.join(DATA_DIR, "scenarios")

# Maps each scenario class to the set of input drivers it requires.
# Scenarios not listed here need no input devices.
_SCENARIO_DRIVERS: dict[type, set[type]] = {
    KeyboardTeleoperationScenario: {KeyboardDriver},
    GamepadTeleoperationScenario: {GamepadDriver},
}


class MobilityGenExtension(omni.ext.IExt):
    """Extension for generating mobility data using teleoperated robots in Omniverse.

    This extension provides a user interface for configuring and running mobility generation scenarios.
    Users can select robot types, scenario types, and occupancy maps to create teleoperated data collection
    sessions. The extension handles stage loading, robot spawning, camera setup, and data recording for
    machine learning training datasets.

    The extension creates two windows:
    - MobilityGen: Main control panel for scenario configuration and recording management
    - MobilityGen - Occupancy Map: Visualization window showing the occupancy map with robot position

    Key features include:
    - Support for multiple robot types and scenario configurations
    - Real-time occupancy map visualization with robot tracking
    - Keyboard and gamepad input support for teleoperation
    - Automatic data recording with timestamped output files
    - Stage caching and management for consistent scenario replay
    - Physics-based simulation with configurable time steps
    """

    def on_startup(self, _ext_id: str) -> None:
        """Initialize the MobilityGen extension.

        Args:
            _ext_id: Extension identifier provided by Kit.
        """
        self._init_state()
        self._build_visualization_window()
        self._build_control_window()
        self._init_dialogs()

    def _init_state(self) -> None:
        """Declare all instance state before anything that can raise.

        Keeping defaults here ensures on_shutdown() is always safe even if
        startup fails partway through.
        """
        # Build / recording orchestration (user-interface-free)
        self.session = RecordingSession()

        # Visualization state
        self._omap_update_counter: int = 0

        # Infrastructure — used by on_shutdown() to know what to clean up
        self._physics_callback_id: int | None = None
        self._keyboard_connected: bool = False
        self._gamepad_connected: bool = False

    def _build_visualization_window(self) -> None:
        """Build the occupancy map visualization window.

        Creates a shared image provider that is updated each time the scenario
        advances, and a Frame whose build function renders the provider into
        an image widget.
        """
        self._omap_image_provider = omni.ui.ByteImageProvider()

        self._visualize_window = omni.ui.Window("MobilityGen - Occupancy Map", width=300, height=300)
        with self._visualize_window.frame:
            self._omap_frame = ui.Frame()
            self._omap_frame.set_build_fn(self.build_omap_frame)

    def _build_control_window(self) -> None:
        """Build the main MobilityGen control window.

        Left panel: scenario configuration inputs and Build button.
        Right panel: recording status labels and playback controls.
        Reset / Start / Stop Recording are disabled until a scenario is built.
        """
        _btn_style = {
            "Button:disabled": {"background_color": 0xFF3A3A3A, "color": 0xFF666666},
        }
        self._teleop_window = omni.ui.Window("MobilityGen", width=300, height=300)
        with self._teleop_window.frame:
            with ui.VStack():
                with ui.VStack():
                    with ui.HStack():
                        ui.Label("Stage")
                        self.scene_usd_field_string_model = ui.SimpleStringModel()
                        ui.StringField(model=self.scene_usd_field_string_model, height=25)
                        ui.Button("...", width=30, height=25, clicked_fn=self._browse_stage)

                    with ui.HStack():
                        ui.Label("Occupancy Map")
                        self.omap_field_string_model = ui.SimpleStringModel()
                        ui.StringField(model=self.omap_field_string_model, height=25)
                        ui.Button("...", width=30, height=25, clicked_fn=self._browse_omap)

                    with ui.HStack():
                        ui.Label("Robot Type")
                        self.robot_combo_box = ui.ComboBox(0, *ROBOTS.names())

                    with ui.HStack():
                        ui.Label("Scenario Type")
                        self.scenario_combo_box = ui.ComboBox(0, *SCENARIOS.names())

                    self._build_button = ui.Button("Build", clicked_fn=self.build_scenario, style=_btn_style)

                with ui.VStack():
                    self.recording_count_label = ui.Label("")
                    self.recording_dir_label = ui.Label(f"Output directory: {RECORDINGS_DIR}")
                    self.recording_name_label = ui.Label("Current recording name: ")
                    self.recording_step_label = ui.Label("Current recording duration: ")

                    self._reset_button = ui.Button("Reset", clicked_fn=self.reset, enabled=False, style=_btn_style)
                    with ui.HStack():
                        self._start_recording_button = ui.Button(
                            "Start Recording",
                            clicked_fn=self.enable_recording,
                            enabled=False,
                            style=_btn_style,
                        )
                        self._stop_recording_button = ui.Button(
                            "Stop Recording",
                            clicked_fn=self.disable_recording,
                            enabled=False,
                            style=_btn_style,
                        )

        self.update_recording_count()

        def _on_settings_changed(_model: Any = None) -> None:
            self._build_button.enabled = True

        self.scene_usd_field_string_model.add_value_changed_fn(_on_settings_changed)
        self.omap_field_string_model.add_value_changed_fn(_on_settings_changed)
        self.robot_combo_box.model.get_item_value_model().add_value_changed_fn(_on_settings_changed)
        self.scenario_combo_box.model.get_item_value_model().add_value_changed_fn(_on_settings_changed)

    def _reconnect_input_drivers(self, scenario_type: type) -> None:
        """Disconnect all input drivers, then connect only what this scenario needs.

        Args:
            scenario_type: The scenario class about to be built.
        """
        if self._keyboard_connected:
            self._keyboard_connected = False
            KeyboardDriver.disconnect()
        if self._gamepad_connected:
            self._gamepad_connected = False
            GamepadDriver.disconnect()

        for driver_cls in _SCENARIO_DRIVERS.get(scenario_type, set()):
            driver_cls.connect()
            if driver_cls is KeyboardDriver:
                self._keyboard_connected = True
            elif driver_cls is GamepadDriver:
                self._gamepad_connected = True

    def _init_dialogs(self) -> None:
        """Create reusable error dialogs for occupancy map path validation."""
        self._omap_not_found_dialog = omni.kit.window.popup_dialog.MessageDialog(
            message="Occupancy map does not exist.  Please enter a file path that exists.",
            disable_cancel_button=True,
            title="Invalid occupancy map.",
            ok_handler=lambda dialog: dialog.hide(),
        )

    def _browse_stage(self) -> None:
        """Open a file picker dialog to select the stage USD or USDZ file."""

        def _filter_usd(item: FileBrowserItem) -> bool:
            if not item or item.is_folder:
                return True
            return os.path.splitext(item.path)[1].lower() in (".usd", ".usda", ".usdc", ".usdz")

        self._stage_file_picker = None
        self._stage_file_picker = FilePickerDialog(
            "Select Stage",
            allow_multi_selection=False,
            apply_button_label="Select",
            click_apply_handler=lambda filename, dirname: self._on_stage_file_selected(filename, dirname),
            click_cancel_handler=lambda _filename, _dirname: self._stage_file_picker.hide(),
            item_filter_fn=_filter_usd,
            item_filter_options=[".usd Files (*.usd, *.usda, *.usdc, *.usdz)"],
        )

    def _on_stage_file_selected(self, filename: str, dirname: str) -> None:
        """Set the stage field to the selected file path and close the picker.

        Args:
            filename: Name of the selected stage file.
            dirname: Directory containing the selected stage file.
        """
        self.scene_usd_field_string_model.set_value(os.path.join(dirname, filename))
        self._stage_file_picker.hide()

    def _browse_omap(self) -> None:
        """Open a file picker dialog to select the occupancy map YAML file."""

        def _filter_yaml(item: FileBrowserItem) -> bool:
            if not item or item.is_folder:
                return True
            return os.path.splitext(item.path)[1].lower() in (".yaml", ".yml")

        self._omap_file_picker = None
        self._omap_file_picker = FilePickerDialog(
            "Select Occupancy Map",
            allow_multi_selection=False,
            apply_button_label="Select",
            click_apply_handler=lambda filename, dirname: self._on_omap_file_selected(filename, dirname),
            click_cancel_handler=lambda _filename, _dirname: self._omap_file_picker.hide(),
            item_filter_fn=_filter_yaml,
            item_filter_options=[".yaml Files (*.yaml, *.yml)"],
        )

    def _on_omap_file_selected(self, filename: str, dirname: str) -> None:
        """Set the occupancy map field to the selected file path and close the picker.

        Args:
            filename: Name of the selected occupancy map file.
            dirname: Directory containing the selected occupancy map file.
        """
        self.omap_field_string_model.set_value(os.path.join(dirname, filename))
        self._omap_file_picker.hide()

    def build_omap_frame(self) -> None:
        """Build the occupancy map visualization frame.

        Creates an image widget to display the occupancy map visualization if a scenario is active.
        """
        if self.session.scenario is not None:
            with ui.VStack():
                ui.ImageWithProvider(self._omap_image_provider)

    def draw_visualization_image(self) -> None:
        """Update the occupancy map visualization image.

        Retrieves the current visualization image from the scenario and updates the image provider for display
        in the UI.
        """
        if self.session.scenario is not None:
            image = self.session.scenario.get_visualization_image().copy().convert("RGBA")
            data = list(image.tobytes())
            self._omap_image_provider.set_bytes_data(data, [image.width, image.height])
            self._omap_frame.rebuild()

    def update_recording_count(self) -> None:
        """Update the recording count display.

        Counts the number of existing recordings in the recordings directory and updates the UI label.
        """
        num_recordings = len(glob.glob(os.path.join(RECORDINGS_DIR, "*")))
        self.recording_count_label.text = f"Number of recordings: {num_recordings}"

    def on_shutdown(self) -> None:
        """Clean up resources when the extension shuts down.

        Disconnects input drivers, removes physics callbacks, and cleans up
        the cached stage temp directory. Safe to call even if on_startup()
        failed partway through.
        """
        if self._keyboard_connected:
            KeyboardDriver.disconnect()
        if self._gamepad_connected:
            GamepadDriver.disconnect()
        self._deregister_physics_callback()
        self.clear_scenario()

    def _deregister_physics_callback(self) -> None:
        """Deregister the physics step callback if one is currently registered."""
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

    def _on_recording_started(self) -> None:
        """Refresh the recording name/duration labels and count for a new recording."""
        recording_name = os.path.basename(self.session.recording_path) if self.session.recording_path else ""
        self.recording_name_label.text = f"Current recording name: {recording_name}"
        self.recording_step_label.text = f"Current recording duration: {self.session.recording_time:.2f}s"
        self.update_recording_count()

    def clear_recording(self) -> None:
        """Clear the current recording session.

        Closes the writer and clears recording display labels.
        """
        self.session.clear_recording()
        self.recording_name_label.text = "Current recording name: "
        self.recording_step_label.text = "Current recording duration: "

    def _set_scenario_controls_enabled(self, enabled: bool) -> None:
        """Enable or disable the buttons that require an active scenario.

        Args:
            enabled: Whether scenario controls should be enabled.
        """
        self._reset_button.enabled = enabled
        self._start_recording_button.enabled = enabled
        self._stop_recording_button.enabled = False

    def clear_scenario(self) -> None:
        """Clear the current scenario.

        Drops the scenario instance and cleans up the cached stage temp directory.
        """
        self.session.clear()
        self._omap_update_counter = 0
        self._set_scenario_controls_enabled(False)

    def enable_recording(self) -> None:
        """Enable data recording for the current scenario.

        Starts a new recording session if a scenario is active and recording is not already enabled.
        """
        if not self.session.recording_enabled:
            self.session.enable_recording()
            self._on_recording_started()
            self._reset_button.enabled = False
            self._start_recording_button.enabled = False
            self._stop_recording_button.enabled = True

    def disable_recording(self) -> None:
        """Disable data recording and clear the current recording session."""
        self.session.disable_recording()
        self.recording_name_label.text = "Current recording name: "
        self.recording_step_label.text = "Current recording duration: "
        if self.session.scenario is not None:
            self._reset_button.enabled = True
            self._start_recording_button.enabled = True
            self._stop_recording_button.enabled = False

    def reset(self) -> None:
        """Reset the scenario to its initial state.

        Resets the scenario and starts a new recording if recording is enabled.
        """
        self.session.reset()
        if self.session.recording_enabled:
            self._on_recording_started()
        self.draw_visualization_image()

    def on_physics(self, step_size: int, _context: Any = None) -> None:
        """Physics step callback that advances the scenario and handles recording.

        Args:
            step_size: The physics step size in simulation time units.
            _context: Optional simulation context passed by the simulation manager.
        """
        if self.session.scenario is None:
            return

        is_alive = self.session.step(step_size)

        if not is_alive:
            # The session restarted the recording on reset; refresh the labels.
            if self.session.recording_enabled:
                self._on_recording_started()
            self.draw_visualization_image()

        if self.session.writer is not None and self.session.step_count % 15 == 0:
            self.recording_step_label.text = f"Current recording duration: {self.session.recording_time:.2f}s"

        self._omap_update_counter += 1
        if self._omap_update_counter >= 15:
            self._omap_update_counter = 0
            self.draw_visualization_image()

    def build_scenario(self) -> None:
        """Build and initialize a new mobility generation scenario based on UI parameters."""
        self._build_button.enabled = False
        asyncio.ensure_future(self._build_scenario_async())

    async def _build_scenario_async(self) -> None:
        """Orchestrate the full scenario build lifecycle asynchronously.

        Sequence: teardown any active scenario → wait for stage to be closeable →
        clear recording/scenario state → resolve UI selections → connect input drivers →
        load and cache the stage → set up simulation and spawn the robot → start physics →
        register the physics step callback → enable scenario controls.

        Re-enables the Build button and returns early on any failure.
        """
        # Teardown previous scenario before touching the stage
        self._deregister_physics_callback()
        if app_utils.is_playing():
            app_utils.stop()
            await app_utils.update_app_async()

        # Wait until Kit considers the stage safe to close.
        # initialize_physics() bumps the stage ref count to 2; open_stage_async
        # expects 1.  can_close_stage() returns True once the count drops back.
        while not omni.usd.get_context().can_close_stage():
            await app_utils.update_app_async()

        self.clear_recording()
        self.clear_scenario()
        self.disable_recording()

        # Resolve types from UI
        scenario_type = SCENARIOS.get_index(self.scenario_combo_box.model.get_item_value_model().get_value_as_int())
        robot_type = ROBOTS.get_index(self.robot_combo_box.model.get_item_value_model().get_value_as_int())
        scene_usd_str = self.scene_usd_field_string_model.as_string

        try:
            self._reconnect_input_drivers(scenario_type)
        except Exception as e:
            carb.log_error(f"MobilityGen: failed to connect input drivers — {e}")
            self._build_button.enabled = True
            return

        omap_path = os.path.expanduser(self.omap_field_string_model.as_string)
        try:
            occupancy_map = OccupancyMap.from_ros_yaml(omap_path)
        except FileNotFoundError:
            carb.log_error(f"MobilityGen: occupancy map not found: {omap_path}")
            self._omap_not_found_dialog.show()
            self._build_button.enabled = True
            return
        except Exception as e:
            carb.log_error(f"MobilityGen: failed to load occupancy map — {e}")
            self._build_button.enabled = True
            return

        try:
            cached_stage_path = await self._cache_stage(scene_usd_str)
        except Exception as e:
            carb.log_error(f"MobilityGen: failed to cache stage — {e}")
            self._build_button.enabled = True
            return

        # NuRec: apply the render carb overrides for the loaded stage. Abort the build (rather than
        # render a black scene) if a launch prerequisite is unmet (e.g. omni.rtx.spg not enabled).
        stage = get_current_stage()
        success, nurec, spg, problems = setup_for_rendering(stage)
        carb.log_warn(f"[nurec] teleop render setup: nurec={nurec} spg={spg} success={success} (stage={scene_usd_str})")
        if not success:
            post_notification("NuRec scene cannot render. " + " ".join(problems), status=NotificationStatus.WARNING)
            self._build_button.enabled = True
            return

        try:
            self.session.build(
                robot_type,
                scenario_type,
                occupancy_map,
                scene_usd=scene_usd_str,
                cached_stage_path=cached_stage_path,
                recordings_dir=RECORDINGS_DIR,
                add_ground_plane=True,
                build_chase_camera=True,
            )
            if ViewportManager.get_viewport_api() is not None and self.session.chase_camera_path is not None:
                # For an SPG scene, render the chase view through the export's authored PPISP graph;
                # otherwise point the viewport straight at the chase camera.
                if not (spg and route_chase_through_ppisp(stage, self.session.chase_camera_path)):
                    ViewportManager.set_camera(self.session.chase_camera_path)
        except Exception as e:
            carb.log_error(f"MobilityGen: failed to set up scenario — {e}")
            self._build_button.enabled = True
            return

        self.draw_visualization_image()

        try:
            app_utils.play()
            await app_utils.update_app_async()
            self.session.initialize()
        except Exception as e:
            carb.log_error(f"MobilityGen: failed to start simulation — {e}")
            app_utils.stop()
            self._build_button.enabled = True
            return

        self._deregister_physics_callback()
        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics, event=SimulationEvent.PHYSICS_POST_STEP
        )

        self._set_scenario_controls_enabled(True)
        self.reset()

    async def _cache_stage(self, scene_usd_str: str) -> str:
        """Copy the input scene into a temporary folder and open it from there.

        The scene and the files it needs are copied first (see
        :func:`collect_input`), then the stage is opened from the copy.

        Args:
            scene_usd_str: Path to the input scene USD file.

        Returns:
            Path to the copied stage.
        """
        tmp_dir = tempfile.mkdtemp()
        try:
            cached_path = await collect_input(scene_usd_str, tmp_dir)
            await open_stage_async(cached_path)
        except Exception:
            shutil.rmtree(tmp_dir, ignore_errors=True)
            raise

        return cached_path

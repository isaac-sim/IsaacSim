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

"""UI window for configuring and controlling synthetic data recording in Isaac Sim."""

import asyncio
import json
import os

import carb
import carb.eventdispatcher
import omni.kit.app
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.ui_utils import get_style
from omni.kit.menu.utils import MenuHelperWindow
from omni.kit.viewport.utility import get_active_viewport
from omni.kit.window.extensions.utils import open_file_using_os_default

from .synthetic_recorder import RecorderState, SyntheticRecorder

GLYPHS = {
    "delete": ui.get_custom_glyph_code("${glyphs}/menu_delete.svg"),
    "open_folder": ui.get_custom_glyph_code("${glyphs}/folder_open.svg"),
    "reset": ui.get_custom_glyph_code("${glyphs}/menu_refresh.svg"),
}

PARAM_TOOLTIPS = {
    "rgb": (
        "Produces an array of type np.uint8 with shape (width, height, 4), "
        "where the four channels correspond to R,G,B,A."
    ),
    "bounding_box_2d_tight": (
        "Outputs tight 2d bounding box of each entity with semantics in the camera's viewport.\n"
        "Tight bounding boxes bound only the visible pixels of entities.\n"
        "Completely occluded entities are omitted.\n"
        "Bounds only visible pixels."
    ),
    "bounding_box_2d_loose": (
        "Outputs loose 2d bounding box of each entity with semantics in the camera's field of view.\n"
        "Loose bounding boxes bound the entire entity regardless of occlusions.\n"
        "Will produce the loose 2d bounding box of any prim in the viewport, no matter if it is partially occluded or fully occluded."
    ),
    "semantic_segmentation": (
        "Outputs semantic segmentation of each entity in the camera's viewport that has semantic labels.\n"
        "If colorize is set to True (mapping from color to semantic labels), the image will be a 2d array of types np.uint8 with 4 channels.\n"
        "If colorize is set to False (mapping from semantic id to semantic labels), the image will be a 2d array of types np.uint32 with 1 channel, which is the semantic id of the entities."
    ),
    "colorize_semantic_segmentation": (
        "If True, semantic segmentation is converted to an image where semantic ids are mapped to colors and saved as a uint8 4 channel PNG image.\n"
        "If False, the output is saved as a uint32 PNG image."
    ),
    "instance_id_segmentation": (
        "Outputs instance id segmentation of each entity in the camera's viewport.\n"
        "The instance id is unique for each prim in the scene with different paths.\n"
        "If colorize is set to True (mapping from color to usd prim path of that entity), the image will be a 2d array of types np.uint8 with 4 channels.\n"
        "If colorize is set to False (mapping from instance id to usd prim path of that entity), the image will be a 2d array of types np.uint32 with 1 channel, which is the instance id of the entities."
    ),
    "colorize_instance_id_segmentation": (
        "If True, instance id segmentation is converted to an image where instance ids are mapped to colors and saved as a uint8 4 channel PNG image.\n"
        "If False, the output is saved as a uint32 PNG image."
    ),
    "instance_segmentation": (
        "Outputs instance segmentation of each entity in the camera's viewport.\n"
        "The main difference between instance id segmentation and instance segmentation is that instance segmentation annotator goes down the hierarchy to the lowest level prim which has semantic labels,\n"
        "whereas instance id segmentation always goes down to the leaf prim."
    ),
    "colorize_instance_segmentation": (
        "If True, instance segmentation is converted to an image where instances are mapped to colors and saved as a uint8 4 channel PNG image.\n"
        "If False, the output is saved as a uint32 PNG image."
    ),
    "distance_to_camera": (
        "Outputs a depth map from objects to camera positions.\n"
        "Produces a 2d array of type np.float32 with 1 channel."
    ),
    "distance_to_image_plane": (
        "Outputs a depth map from objects to the image plane of the camera.\n"
        "Produces a 2d array of type np.float32 with 1 channel."
    ),
    "colorize_depth": (
        "If True, will output an additional grayscale PNG image (0-255) for depth visualization using logarithmic scaling."
    ),
    "bounding_box_3d": (
        "Outputs 3D bounding box of each entity with semantics in the camera's viewport, generated regardless of occlusion."
    ),
    "occlusion": (
        "Outputs the occlusion of each entity in the camera's viewport.\n"
        "Contains the instanceId, semanticId, and the occlusionRatio."
    ),
    "normals": (
        "Produces an array of type np.float32 with shape (height, width, 4).\n"
        "The first three channels correspond to (x, y, z).\n"
        "The fourth channel is unused."
    ),
    "motion_vectors": (
        "Outputs a 2D array of motion vectors representing the relative motion of a pixel in the camera's viewport between frames.\n"
        "Produces a 2d array of type np.float32 with 4 channels.\n"
        "Each value is a normalized direction in 3D space.\n"
        "The values represent motion relative to camera space."
    ),
    "camera_params": (
        "Outputs the camera model (pinhole or fisheye models), view matrix, projection matrix, fisheye nominal width/height, fisheye optical centre, fisheye maximum field of view, fisheye polynomial, near/far clipping range."
    ),
    "pointcloud": (
        "Outputs a 2D array of shape (N, 3) representing the points sampled on the surface of the prims in the viewport, where N is the number of points.\n"
        "Point positions are in the world space.\n"
        "Sample resolution is determined by the resolution of the render product.\n"
        "To get the mapping from semantic id to semantic labels, pointcloud annotator is better used with semantic segmentation annotator, and users can extract the idToLabels data from the semantic segmentation annotator."
    ),
    "pointcloud_include_unlabelled": (
        "If True, pointcloud annotator will capture any prim in the camera's perspective, no matter if it has semantics or not.\n"
        "If False, only prims with semantics will be captured."
    ),
    "skeleton_data": ("Retrieves skeleton data given skeleton prims and camera parameters"),
    "s3_bucket": (
        "The S3 Bucket name to write to. If not provided, disk backend will be used instead.\n"
        "This backend requires that AWS credentials are set up in ~/.aws/credentials."
    ),
    "s3_region": ("If provided, this is the region the S3 bucket will be set to. Default: us-east-1"),
    "s3_endpoint": ("Gateway endpoint for Amazon S3"),
}


class SyntheticRecorderWindow(MenuHelperWindow):
    """Synthetic Recorder UI window.

    Provides a graphical interface for configuring and controlling synthetic data recording in Isaac Sim.
    Allows users to set up render products, configure writer parameters, manage output settings, and control
    the recording process through an intuitive UI.

    Args:
        title: The window title.
        ext_id: The extension ID.
    """

    def __init__(self, title: str, ext_id: str) -> None:
        self._ext_id = ext_id
        # Create the window
        super().__init__(title, verbose=False)
        self.deferred_dock_in("Property")

        # Recorder
        self._recorder = None

        # UI frame collapsed states
        self._collapsed_states = {}

        # UI buttons
        self._start_stop_button = None
        self._pause_resume_button = None

        # UI frames
        self._writer_frame = None
        self._control_frame = None
        self._rp_frame = None
        self._params_frame = None
        self._output_frame = None
        self._s3_frame = None
        self._config_frame = None
        self._control_params_frame = None

        # Create the recorder with the default values
        self._recorder = SyntheticRecorder()
        self._recorder.num_frames = 0
        self._recorder.rt_subframes = 0
        self._recorder.control_timeline = False
        self._recorder.verbose = False
        self._recorder.backend_type = "DiskBackend"
        self._recorder.backend_params = {}
        self._recorder.writer_name = "BasicWriter"
        self._recorder.writer_params = {}
        self._recorder.rp_data = [["/OmniverseKit_Persp", 1280, 720, ""]]

        # UI-only fields
        self._out_working_dir = ""  # Empty = use current working directory
        self._out_dir = "_out_sdrec"
        self._s3_params = {"s3_bucket": "", "s3_region": "", "s3_endpoint": ""}
        self._custom_writer_params = {}
        self._basic_writer_params = {
            "rgb": True,
            "bounding_box_2d_tight": False,
            "bounding_box_2d_loose": False,
            "semantic_segmentation": False,
            "colorize_semantic_segmentation": False,
            "instance_id_segmentation": False,
            "colorize_instance_id_segmentation": False,
            "instance_segmentation": False,
            "colorize_instance_segmentation": False,
            "distance_to_camera": False,
            "distance_to_image_plane": False,
            "colorize_depth": False,
            "bounding_box_3d": False,
            "occlusion": False,
            "normals": False,
            "motion_vectors": False,
            "camera_params": False,
            "pointcloud": False,
            "pointcloud_include_unlabelled": False,
            "skeleton_data": False,
        }

        # Overwrite parmaters with custom config files if available
        self._config_dir = os.path.abspath(
            os.path.join(
                omni.kit.app.get_app().get_extension_manager().get_extension_path(self._ext_id),
                "data",
                "",
            )
        )
        self._last_config_path = os.path.join(self._config_dir, "last_config.json")
        self._custom_params_path = ""
        self._custom_writer_name = "MyCustomWriter"
        self._config_file = "custom_config.json"

        # Load the last config file if it exists, otherwise load the default config file
        if os.path.isfile(self._last_config_path):
            self._load_config(self._last_config_path)
        else:
            self._load_config(os.path.join(self._config_dir, "default_config.json"))

        # Listen to stage closing event to stop the recorder
        self._usd_context = omni.usd.get_context()
        self._sub_stage_event = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSING),
            on_event=self._on_stage_closing_event,
            observer_name="isaacsim.replicator.synthetic_recorder._on_stage_closing_event",
        )

        # Listen to editor quit event to stop the recorder, and save the last config file
        self._sub_shutdown = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_POST_QUIT,
            on_event=self._on_editor_quit_event,
            observer_name="isaacsim.replicator.synthetic_recorder._on_editor_quit_event",
            order=0,
        )

        # Build the window UI
        self._build_window_ui()

        # Subscribe to the recorder state change event to update the buttons state accordingly
        self._recorder.subscribe_state_changed(self._on_state_changed)

    def destroy(self) -> None:
        """Overwriting the destroy method to clean up the window."""
        self._recorder.clear_recorder()
        self._save_config(self._last_config_path)
        self._sub_stage_event = None
        self._sub_shutdown = None
        self._start_stop_button = None
        self._pause_resume_button = None
        self._recorder = None

        super().destroy()

    def _on_collapsed_changed(self, key: str, collapsed: bool) -> None:
        """Keep track in a dict of the collapsed state of the frames.

        Args:
            key: The frame key identifier.
            collapsed: Whether the frame is collapsed.
        """
        self._collapsed_states[key] = collapsed

    def _on_stage_closing_event(self, e: carb.events.IEvent) -> None:
        """Callback function for stage closing event.

        Args:
            e: The stage closing event.
        """
        self._recorder.clear_recorder()

    def _on_editor_quit_event(self, e: carb.events.IEvent) -> None:
        """Callback function for editor quit event.

        Args:
            e: The editor quit event.
        """
        self._recorder.clear_recorder()
        self._save_config(self._last_config_path)

    def _open_dir(self, path: str) -> None:
        """Open the directory through the editor.

        Args:
            path: The directory path to open.
        """
        if not os.path.isdir(path):
            carb.log_warn(f"Could not open directory {path}.")
            return
        open_file_using_os_default(path)

    def _configure_disk_backend(self) -> None:
        """Configure DiskBackend parameters from UI fields.

        Replaces any previously configured backend params with a fresh DiskBackend-only set so that
        stale keys (e.g. ``bucket`` from a previous S3 session) do not leak into ``initialize()``.
        """
        if self._out_working_dir:
            output_dir = os.path.join(self._out_working_dir, self._out_dir)
        else:
            output_dir = self._out_dir
        self._recorder.backend_params = {"output_dir": output_dir}

    def _configure_s3_backend(self) -> None:
        """Configure S3Backend parameters from UI fields.

        Replaces any previously configured backend params with a fresh S3Backend-only set so that
        stale keys (e.g. ``output_dir`` from a previous Disk session) do not leak into ``initialize()``.
        """
        backend_params = {}
        if self._out_dir:
            backend_params["key_prefix"] = self._out_dir
        bucket = self._s3_params.get("s3_bucket")
        if bucket:
            backend_params["bucket"] = bucket
        region = self._s3_params.get("s3_region")
        if region:
            backend_params["region"] = region
        endpoint = self._s3_params.get("s3_endpoint")
        if endpoint:
            backend_params["endpoint_url"] = endpoint
        self._recorder.backend_params = backend_params

    def _load_config(self, path: str) -> None:
        """Load the json config file and set the recorder parameters.

        Args:
            path: The path to the config file.
        """
        if not os.path.isfile(path):
            carb.log_warn(f"Could not find config file: '{path}'.")
            return

        if os.path.getsize(path) == 0:
            carb.log_warn(f"Config file '{path}' is empty.")
            return

        try:
            with open(path) as f:
                config = json.load(f)
        except json.JSONDecodeError as e:
            carb.log_warn(f"Could not parse JSON in config file: '{path}', exception: {e}")
            return
        except Exception as e:
            carb.log_warn(f"An error occurred while reading '{path}': {e}")
            return

        # Recorder params
        if self._recorder is None:
            carb.log_warn("Recorder is not initialized.")
            return
        self._recorder.writer_name = config.get("writer_name", self._recorder.writer_name)
        self._recorder.num_frames = config.get("num_frames", self._recorder.num_frames)
        self._recorder.rt_subframes = config.get("rt_subframes", self._recorder.rt_subframes)
        self._recorder.control_timeline = config.get("control_timeline", self._recorder.control_timeline)

        # Load UI-only fields
        self._out_working_dir = config.get("out_working_dir", self._out_working_dir)
        self._out_dir = config.get("out_dir", self._out_dir)
        s3_params = config.get("s3_params")
        if isinstance(s3_params, dict):
            self._s3_params.update(s3_params)

        # Resolve backend type from config. ``backend_type`` is the source of truth; the legacy
        # ``use_s3`` flag is honored only when no ``backend_type`` was saved.
        backend_type = config.get("backend_type")
        if backend_type not in ("DiskBackend", "S3Backend"):
            backend_type = "S3Backend" if config.get("use_s3") else "DiskBackend"
        self._recorder.backend_type = backend_type

        # Hydrate S3 UI fields from saved backend_params for backwards compatibility with configs
        # that stored bucket/region/endpoint only under ``backend_params``.
        backend_params = config.get("backend_params")
        if isinstance(backend_params, dict) and backend_type == "S3Backend":
            self._s3_params["s3_bucket"] = backend_params.get("bucket", self._s3_params["s3_bucket"])
            self._s3_params["s3_region"] = backend_params.get("region", self._s3_params["s3_region"])
            self._s3_params["s3_endpoint"] = backend_params.get("endpoint_url", self._s3_params["s3_endpoint"])

        # Always (re)derive backend_params from UI fields so disk/S3 toggles never leak stale keys.
        if backend_type == "S3Backend":
            self._configure_s3_backend()
        else:
            self._configure_disk_backend()

        # Load writer params (UI-only fields)
        basic_writer_params = config.get("basic_writer_params")
        if isinstance(basic_writer_params, dict):
            for key, value in basic_writer_params.items():
                if key in self._basic_writer_params:
                    self._basic_writer_params[key] = value

        rp_data = config.get("rp_data")
        if isinstance(rp_data, list):
            self._recorder.rp_data = rp_data
            for rp in self._recorder.rp_data:
                if len(rp) == 3:
                    rp.append("")  # If no custom name provided, set it to an empty string

        # UI params
        self._config_file = config.get("config_file", self._config_file)
        self._custom_params_path = config.get("custom_params_path", self._custom_params_path)
        self._custom_writer_name = config.get("custom_writer_name", self._custom_writer_name)

    def _load_config_and_refresh_ui(self, directory: str, filename: str) -> None:
        """Load the config file and refresh the UI to reflect the changes.

        Args:
            directory: The directory containing the config file.
            filename: The config filename to load.
        """
        self._load_config(os.path.join(directory, filename))
        # Rebuild all child frames directly to ensure they refresh with new config data
        if self._rp_frame:
            self._rp_frame.rebuild()
        if self._params_frame:
            self._params_frame.rebuild()
        if self._output_frame:
            self._output_frame.rebuild()
        if self._config_frame:
            self._config_frame.rebuild()
        if self._control_params_frame:
            self._control_params_frame.rebuild()

    def _save_config(self, path: str) -> None:
        """Save the current recorder parameters to a json config file.

        Args:
            path: The path where to save the config file.
        """
        dir_name = os.path.dirname(path)
        if not dir_name:
            carb.log_warn(f"Could not save config file to '{path}', missing directory path.")
            return
        os.makedirs(dir_name, exist_ok=True)
        if os.path.isfile(path):
            carb.log_info(f"Overwriting config file '{path}'.")
        try:
            with open(path, "w") as json_file:
                # Recorder
                config = {
                    "num_frames": self._recorder.num_frames,
                    "rt_subframes": self._recorder.rt_subframes,
                    "control_timeline": self._recorder.control_timeline,
                    "rp_data": self._recorder.rp_data,
                }
                if self._recorder.writer_name:
                    config["writer_name"] = self._recorder.writer_name

                # Save UI-only fields
                if self._out_working_dir is not None:
                    config["out_working_dir"] = self._out_working_dir
                if self._out_dir:
                    config["out_dir"] = self._out_dir
                if self._s3_params:
                    config["s3_params"] = self._s3_params
                if self._basic_writer_params:
                    config["basic_writer_params"] = self._basic_writer_params

                # Save backend configuration if specified
                if self._recorder.backend_type:
                    config["backend_type"] = self._recorder.backend_type
                if self._recorder.backend_params:
                    config["backend_params"] = self._recorder.backend_params

                # UI
                if self._custom_writer_name:
                    config["custom_writer_name"] = self._custom_writer_name
                if self._config_file:
                    config["config_file"] = self._config_file
                if self._custom_params_path:
                    config["custom_params_path"] = self._custom_params_path

                json.dump(config, json_file, indent=4)

        except Exception as e:
            carb.log_warn(f"An error occurred while saving the config file '{path}': {e}")

    def _get_custom_params(self, path: str) -> dict:
        """Load the custom writer parameters as a dict from the given json file.

        Args:
            path: Path to the JSON file containing custom writer parameters.

        Returns:
            Dictionary containing the loaded custom writer parameters. Returns empty dict if file doesn't exist
            or contains invalid JSON.
        """
        if not os.path.isfile(path):
            carb.log_warn(f"Could not find params file '{path}'.")
            return {}
        try:
            with open(path) as f:
                params = json.load(f)
        except (json.JSONDecodeError, IOError) as e:
            carb.log_warn(f"Error reading '{path}': {e}")
            return {}
        if not isinstance(params, dict):
            carb.log_warn(f"The file '{path}' does not contain a valid dictionary.")
            return {}

        return params

    def _reset_config_dir(self) -> None:
        """Reset the configuration directory to the default extension data path.

        Resets the config directory to the extension's data folder and rebuilds the config UI frame
        to reflect the changes.
        """
        self._config_dir = os.path.abspath(
            os.path.join(
                omni.kit.app.get_app().get_extension_manager().get_extension_path(self._ext_id),
                "data",
                "",
            )
        )
        if self._config_frame:
            self._config_frame.rebuild()

    def _reset_out_working_dir(self) -> None:
        """Reset the working directory to the default value.

        Clears the output working directory field and rebuilds the output UI frame to reflect the changes.
        """
        self._out_working_dir = ""
        if self._output_frame:
            self._output_frame.rebuild()

    def _set_buttons_state(self) -> None:
        """Set the state of the start/stop and pause/resume buttons based on the recorder state.

        Updates button text and enabled state according to the current recorder state (STOPPED, RUNNING,
        or PAUSED).
        """
        if self._recorder.get_state() == RecorderState.STOPPED:
            self._start_stop_button.text = "Start"
            self._pause_resume_button.text = "Pause"
            self._start_stop_button.enabled = True
            self._pause_resume_button.enabled = False
        elif self._recorder.get_state() == RecorderState.RUNNING:
            self._start_stop_button.text = "Stop"
            self._pause_resume_button.text = "Pause"
            self._start_stop_button.enabled = True
            self._pause_resume_button.enabled = True
        elif self._recorder.get_state() == RecorderState.PAUSED:
            self._start_stop_button.text = "Stop"
            self._pause_resume_button.text = "Resume"
            self._start_stop_button.enabled = True
            self._pause_resume_button.enabled = True

    def _start_stop_recorder(self) -> None:
        """Start/stop the recorder, construct writer params from UI fields before starting.

        Configures writer parameters based on the selected writer type (BasicWriter or custom writer)
        and backend parameters, then starts or stops the recording process asynchronously.
        """
        # Construct writer_params from UI fields before recording
        if self._recorder.writer_name == "BasicWriter":
            self._recorder.writer_params = self._basic_writer_params.copy()
        else:
            # If custom writer, load parameters from the given json config file
            self._recorder.writer_params = self._get_custom_params(self._custom_params_path)

        # Configure backend params from UI fields before recording (always rebuilt from scratch
        # so Disk/S3 toggles never leak stale keys into backend.initialize()).
        if self._recorder.backend_type == "S3Backend":
            self._configure_s3_backend()
        else:
            self._configure_disk_backend()

        asyncio.ensure_future(self._recorder.start_stop_async())

    def _pause_resume_recorder(self) -> None:
        """Pause/resume the recorder.

        Toggles the recorder between paused and running states asynchronously.
        """
        asyncio.ensure_future(self._recorder.pause_resume_async())

    def _on_state_changed(self) -> None:
        """Callback function when the recorder state changes (finished/running/paused) to update the buttons state.

        Updates the UI button states to reflect the current recorder state.
        """
        self._set_buttons_state()

    def _build_config_ui(self) -> None:
        """Build the config part of the UI.

        Creates UI elements for managing configuration files, including directory path input, file name
        field, and load/save buttons for configuration management.
        """
        with ui.VStack(spacing=5):
            with ui.HStack():
                ui.Spacer(width=10)
                ui.Label("Config Directory", tooltip="Config files directory path")
            with ui.HStack():
                ui.Spacer(width=10)
                config_dir_model = ui.StringField(read_only=False).model
                config_dir_model.set_value(self._config_dir)

                def config_dir_changed(model: object) -> None:
                    self._config_dir = model.as_string

                config_dir_model.add_value_changed_fn(config_dir_changed)

                ui.Spacer(width=5)
                ui.Button(
                    f"{GLYPHS['open_folder']}",
                    width=20,
                    clicked_fn=lambda: self._open_dir(self._config_dir),
                    tooltip="Open config directory",
                )

                ui.Button(
                    f"{GLYPHS['reset']}",
                    width=20,
                    clicked_fn=lambda: self._reset_config_dir(),
                    tooltip="Reset config directory to default",
                )

            with ui.HStack(spacing=5):
                ui.Spacer(width=5)
                config_file_model = ui.StringField(tooltip="Config file name").model
                config_file_model.set_value(self._config_file)

                def config_file_changed(model: object) -> None:
                    self._config_file = model.as_string

                config_file_model.add_value_changed_fn(config_file_changed)

                ui.Button(
                    "Load",
                    clicked_fn=lambda: self._load_config_and_refresh_ui(self._config_dir, self._config_file),
                    tooltip="Load and apply selected config file",
                )
                ui.Button(
                    "Save",
                    clicked_fn=lambda: self._save_config(os.path.join(self._config_dir, self._config_file)),
                    tooltip="Save current config to selected file",
                )

    def _build_s3_ui(self) -> None:
        """Build the S3 part of the UI.

        Creates UI elements for configuring S3 backend parameters including the S3 enable checkbox
        and input fields for bucket, region, and endpoint settings. The "Use S3" checkbox is bound
        directly to ``self._recorder.backend_type`` to keep a single source of truth.
        """
        with ui.VStack(spacing=5):
            with ui.HStack():
                ui.Spacer(width=10)
                ui.Label("Use S3", width=250, alignment=ui.Alignment.LEFT, tooltip="Write data to S3 buckets")
                s3_model = ui.CheckBox(width=10, height=0).model
                s3_model.set_value(self._recorder.backend_type == "S3Backend")

                def value_changed(m: object) -> None:
                    self._recorder.backend_type = "S3Backend" if m.as_bool else "DiskBackend"

                s3_model.add_value_changed_fn(value_changed)

            for key, val in self._s3_params.items():
                with ui.HStack():
                    ui.Spacer(width=10)
                    ui.Label(key, alignment=ui.Alignment.LEFT, tooltip=PARAM_TOOLTIPS[key])
                    model = ui.StringField().model
                    if val:
                        model.set_value(val)
                    else:
                        model.set_value("")

                    def value_changed(m: object, k: object = key) -> None:
                        self._s3_params[k] = m.as_string

                    model.add_value_changed_fn(value_changed)

    def _build_output_ui(self) -> None:
        """Build the output part of the UI.

        Creates UI elements for configuring output settings including working directory path, output
        directory name, and the collapsible S3 configuration frame.
        """
        with ui.VStack(spacing=5):
            with ui.HStack():
                ui.Spacer(width=10)
                ui.Label("Working Directory")
            with ui.HStack():
                ui.Spacer(width=10)
                out_working_dir_model = ui.StringField().model
                out_working_dir_model.set_value(self._out_working_dir)

                def out_working_dir_changed(model: object) -> None:
                    self._out_working_dir = model.as_string

                out_working_dir_model.add_value_changed_fn(out_working_dir_changed)

                ui.Spacer(width=5)
                ui.Button(
                    f"{GLYPHS['open_folder']}",
                    width=20,
                    clicked_fn=lambda: self._open_dir(self._out_working_dir if self._out_working_dir else os.getcwd()),
                    tooltip="Open working directory",
                )

                ui.Button(
                    f"{GLYPHS['reset']}",
                    width=20,
                    clicked_fn=lambda: self._reset_out_working_dir(),
                    tooltip="Reset directory to default",
                )

            with ui.HStack(spacing=5):
                ui.Spacer(width=5)
                out_dir_model = ui.StringField(
                    tooltip="Disk: leaf directory under Working Directory.  S3: key_prefix inside the bucket."
                ).model
                out_dir_model.set_value(self._out_dir)

                def out_dir_changed(model: object) -> None:
                    self._out_dir = model.as_string

                out_dir_model.add_value_changed_fn(out_dir_changed)

            # Create S3 Bucket frame
            frame_name = "S3 Bucket"
            frame_collapsed = self._collapsed_states.get(frame_name, True)
            self._s3_frame = ui.CollapsableFrame(
                frame_name, height=0, collapsed=frame_collapsed, style=get_style(), build_fn=self._build_s3_ui
            )
            self._s3_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))

    def _update_rp_entry(self, idx: int, field: int, value: object) -> None:
        """Callback function to update the render product entry.

        Args:
            idx: Index of the render product entry to update.
            field: Field index within the entry to modify.
            value: New value to set for the specified field.
        """
        self._recorder.rp_data[idx][field] = value

    def _remove_rp_entry(self, idx: int) -> None:
        """Callback function to remove the render product entry.

        Args:
            idx: Index of the render product entry to remove.
        """
        del self._recorder.rp_data[idx]
        if self._rp_frame:
            self._rp_frame.rebuild()

    def _add_new_rp_field(self) -> None:
        """Add a new UI render product entry."""
        context = omni.usd.get_context()
        stage = context.get_stage()
        selected_prims = context.get_selection().get_selected_prim_paths()
        selected_cameras = [path for path in selected_prims if stage.GetPrimAtPath(path).GetTypeName() == "Camera"]

        if selected_cameras:
            for path in selected_cameras:
                self._recorder.rp_data.append([path, 1280, 720, ""])
        else:
            active_vp = get_active_viewport()
            active_cam = active_vp.get_active_camera()
            self._recorder.rp_data.append([str(active_cam), 1280, 720, ""])
        if self._rp_frame:
            self._rp_frame.rebuild()

    def _build_rp_ui(self) -> None:
        """Build the render product part of the UI."""
        with ui.VStack(spacing=5):
            with ui.HStack(spacing=5):
                ui.Spacer(width=15)
                ui.Label("Camera Path", width=200, tooltip="Camera prim to be used as a render product")
                ui.Spacer(width=10)
                ui.Label("X", tooltip="X resolution of the render product")
                ui.Spacer(width=0)
                ui.Label("Y", tooltip="Y resolution of the render product")
                ui.Spacer(width=0)
                ui.Label("Name", tooltip="Render product name (optional)")
            for i, entry in enumerate(self._recorder.rp_data):
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    path_field_model = ui.StringField(width=200).model
                    path_field_model.set_value(entry[0])
                    path_field_model.add_value_changed_fn(lambda m, idx=i: self._update_rp_entry(idx, 0, m.as_string))
                    ui.Spacer(width=10)
                    x_field = ui.IntField()
                    x_field.model.set_value(entry[1])
                    x_field.model.add_value_changed_fn(lambda m, idx=i: self._update_rp_entry(idx, 1, m.as_int))
                    ui.Spacer(width=10)
                    y_field = ui.IntField()
                    y_field.model.set_value(entry[2])
                    y_field.model.add_value_changed_fn(lambda m, idx=i: self._update_rp_entry(idx, 2, m.as_int))
                    ui.Spacer(width=10)
                    name_field = ui.StringField()
                    name_field.model.set_value(entry[3])
                    name_field.model.add_value_changed_fn(lambda m, idx=i: self._update_rp_entry(idx, 3, m.as_string))
                    ui.Button(
                        f"{GLYPHS['delete']}",
                        width=30,
                        clicked_fn=lambda idx=i: self._remove_rp_entry(idx),
                        tooltip="Remove entry",
                    )
            with ui.HStack(spacing=5):
                ui.Spacer(width=5)
                ui.Button(
                    "Add New Render Product Entry", clicked_fn=self._add_new_rp_field, tooltip="Create a new entry"
                )

    def _build_params_ui(self) -> None:
        """Build the writer parameters part of the UI."""
        with ui.VStack(spacing=5):
            with ui.HStack():
                ui.Spacer(width=10)
                ui.Label("Writer")

                writer_type_collection = ui.RadioCollection()
                if self._recorder.writer_name == "BasicWriter":
                    writer_type_collection.model.set_value(0)
                else:
                    writer_type_collection.model.set_value(1)

                def writer_type_collection_changed(model: object) -> None:
                    if model.as_int == 0:
                        self._recorder.writer_name = "BasicWriter"
                    else:
                        self._recorder.writer_name = self._custom_writer_name
                    if self._params_frame:
                        self._params_frame.rebuild()

                writer_type_collection.model.add_value_changed_fn(writer_type_collection_changed)

                ui.RadioButton(
                    text="Default", radio_collection=writer_type_collection, tooltip="Uses the default BasicWriter"
                )
                ui.RadioButton(
                    text="Custom", radio_collection=writer_type_collection, tooltip="Loads a custom writer by name"
                )

            if self._recorder.writer_name == "BasicWriter":
                self._build_basic_writer_ui()
            else:
                self._build_custom_writer_ui()

    def _build_basic_writer_ui(self) -> None:
        """Build the basic writer part of the UI."""
        for key, val in self._basic_writer_params.items():
            with ui.HStack(spacing=5):
                ui.Spacer(width=10)
                ui.Label(key, width=300, alignment=ui.Alignment.LEFT, tooltip=PARAM_TOOLTIPS[key])
                model = ui.CheckBox(width=10, height=0).model
                model.set_value(val)

                def value_changed(m: object, k: object = key) -> None:
                    self._basic_writer_params[k] = m.as_bool

                model.add_value_changed_fn(value_changed)

        with ui.HStack(spacing=5):
            ui.Spacer(width=10)

            def select_all() -> None:
                for k in self._basic_writer_params:
                    self._basic_writer_params[k] = True
                if self._params_frame:
                    self._params_frame.rebuild()

            def toggle_all() -> None:
                for k in self._basic_writer_params:
                    self._basic_writer_params[k] = not self._basic_writer_params[k]
                if self._params_frame:
                    self._params_frame.rebuild()

            ui.Button(text="Select All", clicked_fn=select_all, tooltip="Select all parameters")
            ui.Button(text="Toggle All", clicked_fn=toggle_all, tooltip="Toggle all parameters")

    def _build_custom_writer_ui(self) -> None:
        """Build the custom writer part of the UI."""
        with ui.HStack(spacing=5):
            ui.Spacer(width=10)
            ui.Label("Name", tooltip="The name of the custom writer from the registry")
            writer_name_model = ui.StringField().model
            writer_name_model.set_value(self._recorder.writer_name)

            def writer_name_changed(m: object) -> None:
                self._recorder.writer_name = m.as_string

            writer_name_model.add_value_changed_fn(writer_name_changed)

        with ui.HStack(spacing=5):
            ui.Spacer(width=10)
            ui.Label("Parameters Path", tooltip="Path to the json file storing the custom writer parameters")
            path_model = ui.StringField().model
            path_model.set_value(self._custom_params_path)

            def path_changed(m: object) -> None:
                self._custom_params_path = m.as_string

            path_model.add_value_changed_fn(path_changed)

    def _create_writer_child_frames(self) -> None:
        """Create all child frames within the writer frame container."""
        # Render Products frame
        frame_name = "Render Products"
        frame_collapsed = self._collapsed_states.get(frame_name, False)
        self._rp_frame = ui.CollapsableFrame(
            frame_name, height=0, collapsed=frame_collapsed, style=get_style(), build_fn=self._build_rp_ui
        )
        self._rp_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))

        # Parameters frame
        frame_name = "Parameters"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        self._params_frame = ui.CollapsableFrame(
            frame_name, height=0, collapsed=frame_collapsed, style=get_style(), build_fn=self._build_params_ui
        )
        self._params_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))

        # Output frame
        frame_name = "Output"
        frame_collapsed = self._collapsed_states.get(frame_name, False)
        self._output_frame = ui.CollapsableFrame(
            frame_name, height=0, collapsed=frame_collapsed, style=get_style(), build_fn=self._build_output_ui
        )
        self._output_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))

        # Config frame
        frame_name = "Config"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        self._config_frame = ui.CollapsableFrame(
            frame_name, height=0, collapsed=frame_collapsed, style=get_style(), build_fn=self._build_config_ui
        )
        self._config_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))

    def _build_writer_ui(self) -> None:
        """Build the writer UI frame container."""
        with ui.VStack(spacing=5):
            self._create_writer_child_frames()

    def _build_control_params_ui(self) -> None:
        """Build the control parameters part of the UI."""
        with ui.VStack(spacing=10):
            with ui.HStack(spacing=5):
                ui.Spacer(width=10)
                ui.Label("Number of frames", tooltip="If set to 0, data acquisition will run indefinitely")
                num_frames_model = ui.IntField().model
                num_frames_model.set_value(self._recorder.num_frames)

                def num_frames_changed(m: object) -> None:
                    self._recorder.num_frames = m.as_int

                num_frames_model.add_value_changed_fn(num_frames_changed)

                ui.Label("RTSubframes", tooltip="Render extra frames between captures to avoid rendering artifacts")
                rt_subframes_model = ui.IntField().model
                rt_subframes_model.set_value(self._recorder.rt_subframes)

                def num_rt_subframes_changed(m: object) -> None:
                    self._recorder.rt_subframes = m.as_int

                rt_subframes_model.add_value_changed_fn(num_rt_subframes_changed)

            with ui.HStack(spacing=5):
                ui.Spacer(width=10)
                ui.Label("Control Timeline", width=130, tooltip="Start/Stop/Pause timeline as well with the recorder")
                control_timeline_model = ui.CheckBox(width=10, height=0).model
                control_timeline_model.set_value(self._recorder.control_timeline)

                def control_timeline_value_changed(m: object) -> None:
                    self._recorder.control_timeline = m.as_bool

                control_timeline_model.add_value_changed_fn(control_timeline_value_changed)

                ui.Spacer(width=180)
                ui.Label("Verbose", width=80, tooltip="Print recorder status to the terminal (e.g. current frame)")
                verbose_model = ui.CheckBox(width=10, height=0).model
                verbose_model.set_value(self._recorder.verbose)

                def verbose_value_changed(m: object) -> None:
                    self._recorder.verbose = m.as_bool

                verbose_model.add_value_changed_fn(verbose_value_changed)

    def _create_control_child_frames(self) -> None:
        """Create child frame within the control frame container."""
        # Control Parameters frame
        frame_name = "Control Parameters"
        frame_collapsed = self._collapsed_states.get(frame_name, False)
        self._control_params_frame = ui.CollapsableFrame(
            frame_name,
            height=0,
            collapsed=frame_collapsed,
            style=get_style(),
            build_fn=self._build_control_params_ui,
        )
        self._control_params_frame.set_collapsed_changed_fn(
            lambda collapsed: self._on_collapsed_changed(frame_name, collapsed)
        )

    def _build_control_ui(self) -> None:
        """Build the control UI frame container."""
        with ui.VStack(spacing=5):
            self._create_control_child_frames()

            with ui.HStack(spacing=5):
                ui.Spacer(width=5)
                self._start_stop_button = ui.Button(
                    "Start",
                    clicked_fn=self._start_stop_recorder,
                    enabled=True,
                    tooltip="Start/stop the recording",
                )
                self._pause_resume_button = ui.Button(
                    "Pause",
                    clicked_fn=self._pause_resume_recorder,
                    enabled=False,
                    tooltip="Pause/resume recording",
                )
            self._set_buttons_state()

    def _build_window_ui(self) -> None:
        """Build the window UI."""
        with self.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5):
                    # Create stored CollapsableFrames with build_fn
                    frame_name = "Writer"
                    frame_collapsed = self._collapsed_states.get(frame_name, False)
                    self._writer_frame = ui.CollapsableFrame(
                        frame_name,
                        height=0,
                        collapsed=frame_collapsed,
                        style=get_style(),
                        build_fn=self._build_writer_ui,
                    )
                    self._writer_frame.set_collapsed_changed_fn(
                        lambda collapsed: self._on_collapsed_changed(frame_name, collapsed)
                    )

                    frame_name = "Control"
                    frame_collapsed = self._collapsed_states.get(frame_name, False)
                    self._control_frame = ui.CollapsableFrame(
                        frame_name,
                        height=0,
                        collapsed=frame_collapsed,
                        style=get_style(),
                        build_fn=self._build_control_ui,
                    )
                    self._control_frame.set_collapsed_changed_fn(
                        lambda collapsed: self._on_collapsed_changed(frame_name, collapsed)
                    )

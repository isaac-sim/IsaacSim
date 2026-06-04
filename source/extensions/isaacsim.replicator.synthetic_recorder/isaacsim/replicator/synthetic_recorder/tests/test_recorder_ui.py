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

"""Tests for the Synthetic Data Recorder UI interactions."""

import os
import shutil

import omni.kit.app
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.replicator.core as rep
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.replicator.synthetic_recorder.synthetic_recorder import RecorderState
from isaacsim.replicator.synthetic_recorder.synthetic_recorder_extension import SyntheticRecorderExtension
from isaacsim.test.utils.file_validation import validate_folder_contents
from isaacsim.test.utils.menu_utils import menu_click_with_retry

WINDOW_TITLE = SyntheticRecorderExtension.WINDOW_NAME
MENU_PATH = f"{SyntheticRecorderExtension.MENU_GROUP}/{SyntheticRecorderExtension.WINDOW_NAME}"

OUT_DIR_RGB_NO_CONTROL_TIMELINE = "_out_sdrec_rgb_no_control_timeline"
OUT_DIR_DEPTH_CONTROL_TIMELINE = "_out_sdrec_depth_control_timeline"
NUM_FRAMES = 5

# UI widget paths
_CF = "Frame[0]/ZStack[0]/VStack[0]/Frame[0]/VStack[0]"
_BASE = f"{WINDOW_TITLE}//Frame/ScrollingFrame[0]/VStack[0]"
_WRITER = f"{_BASE}/CollapsableFrame[0]/{_CF}"
_CONTROL = f"{_BASE}/CollapsableFrame[1]/{_CF}"
_CTRL_PARAMS = f"{_CONTROL}/CollapsableFrame[0]/{_CF}"
START_BUTTON = f"{_CONTROL}/HStack[0]/Button[0]"
NUM_FRAMES_FIELD = f"{_CTRL_PARAMS}/HStack[0]/IntField[0]"
RP_CAMERA_PATH_FIELD = f"{_WRITER}/CollapsableFrame[0]/{_CF}/HStack[1]/StringField[0]"
OUT_WORKING_DIR_FIELD = f"{_WRITER}/CollapsableFrame[2]/{_CF}/HStack[1]/StringField[0]"
OUT_DIR_FIELD = f"{_WRITER}/CollapsableFrame[2]/{_CF}/HStack[2]/StringField[0]"
WRITER_DEFAULT_RADIO = f"{_WRITER}/CollapsableFrame[1]/{_CF}/HStack[0]/RadioButton[0]"
RGB_CHECKBOX = f"{_WRITER}/CollapsableFrame[1]/{_CF}/HStack[1]/CheckBox[0]"
DEPTH_CHECKBOX = f"{_WRITER}/CollapsableFrame[1]/{_CF}/HStack[10]/CheckBox[0]"
CONTROL_TIMELINE_CHECKBOX = f"{_CTRL_PARAMS}/HStack[1]/CheckBox[0]"
VERBOSE_CHECKBOX = f"{_CTRL_PARAMS}/HStack[1]/CheckBox[1]"


class TestRecorderUI(omni.kit.test.AsyncTestCase):
    """Test the Synthetic Data Recorder through UI interactions."""

    async def setUp(self) -> None:
        """Set up a new stage before each test."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Stop timeline, close stage, and wait for assets after each test."""
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing():
            timeline.stop()
            timeline.commit()
            await omni.kit.app.get_app().next_update_async()

        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_ui_record_rgb_no_control_timeline(self) -> None:
        """Test recording RGB output through UI without timeline control."""
        out_dir = os.path.join(os.getcwd(), OUT_DIR_RGB_NO_CONTROL_TIMELINE)
        if os.path.exists(out_dir):
            shutil.rmtree(out_dir)

        # Scene setup.
        await omni.usd.get_context().new_stage_async()
        rep.functional.create.cube(semantics=[("class", "cube")])
        cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), name="my_rep_camera")
        cam_path = str(cam.GetPath())

        # Open and stabilize the recorder window.
        await menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
        window = ui.Workspace.get_window(WINDOW_TITLE)
        self.assertIsNotNone(window, "Synthetic Data Recorder window not found")
        window.visible = True
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        start_btn = ui_test.find(START_BUTTON)
        self.assertIsNotNone(start_btn, f"Start button not found at: {START_BUTTON}")
        self.assertEqual(start_btn.widget.text, "Start")

        # Configure common recorder settings via UI fields.
        num_frames_field = ui_test.find(NUM_FRAMES_FIELD)
        self.assertIsNotNone(num_frames_field, f"Number of frames field not found at: {NUM_FRAMES_FIELD}")
        num_frames_field.model.set_value(NUM_FRAMES)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._recorder.num_frames, NUM_FRAMES, "Recorder num_frames was not updated by UI")

        rp_camera_path_field = ui_test.find(RP_CAMERA_PATH_FIELD)
        self.assertIsNotNone(rp_camera_path_field, f"RP camera path field not found at: {RP_CAMERA_PATH_FIELD}")
        rp_camera_path_field.model.set_value(cam_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._recorder.rp_data[0][0], cam_path, "Render product camera path was not updated by UI")

        out_working_dir_field = ui_test.find(OUT_WORKING_DIR_FIELD)
        self.assertIsNotNone(
            out_working_dir_field, f"Output working directory field not found at: {OUT_WORKING_DIR_FIELD}"
        )
        out_working_dir_field.model.set_value(os.getcwd())
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._out_working_dir, os.getcwd(), "Output working directory was not updated by UI")

        out_dir_field = ui_test.find(OUT_DIR_FIELD)
        self.assertIsNotNone(out_dir_field, f"Output directory field not found at: {OUT_DIR_FIELD}")
        out_dir_field.model.set_value(OUT_DIR_RGB_NO_CONTROL_TIMELINE)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._out_dir, OUT_DIR_RGB_NO_CONTROL_TIMELINE, "Output directory was not updated by UI")

        writer_default_radio = ui_test.find(WRITER_DEFAULT_RADIO)
        self.assertIsNotNone(writer_default_radio, f"Default writer radio not found at: {WRITER_DEFAULT_RADIO}")
        await writer_default_radio.click()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._recorder.writer_name, "BasicWriter", "Writer was not set to BasicWriter via UI")

        self.assertEqual(window._recorder.backend_type, "DiskBackend", "Expected default backend to be DiskBackend")

        # Test-specific options: RGB on, control timeline off.
        for key in window._basic_writer_params:
            window._basic_writer_params[key] = False

        rgb_checkbox = ui_test.find(RGB_CHECKBOX)
        self.assertIsNotNone(rgb_checkbox, f"RGB checkbox not found at: {RGB_CHECKBOX}")
        rgb_checkbox.model.set_value(False)
        await omni.kit.app.get_app().next_update_async()
        rgb_checkbox.model.set_value(True)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(window._basic_writer_params["rgb"], "RGB checkbox did not set basic writer param")

        control_timeline_checkbox = ui_test.find(CONTROL_TIMELINE_CHECKBOX)
        self.assertIsNotNone(
            control_timeline_checkbox, f"Control Timeline checkbox not found at: {CONTROL_TIMELINE_CHECKBOX}"
        )
        control_timeline_checkbox.model.set_value(False)
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(window._recorder.control_timeline, "Control Timeline checkbox did not update recorder state")

        verbose_checkbox = ui_test.find(VERBOSE_CHECKBOX)
        self.assertIsNotNone(verbose_checkbox, f"Verbose checkbox not found at: {VERBOSE_CHECKBOX}")
        verbose_checkbox.model.set_value(True)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(window._recorder.verbose, "Verbose checkbox did not update recorder state")

        # Start through UI and wait for completion.
        await start_btn.click()

        saw_stop_state = False
        for _ in range(30):
            await omni.kit.app.get_app().next_update_async()
            if start_btn.widget.text == "Stop":
                saw_stop_state = True
                break
        self.assertTrue(saw_stop_state, "Button did not change to 'Stop' after starting recording")

        recorder_stopped = False
        for _ in range(NUM_FRAMES + 120):
            await omni.kit.app.get_app().next_update_async()
            if window._recorder.get_state() == RecorderState.STOPPED:
                recorder_stopped = True
                break
        self.assertTrue(recorder_stopped, "Recorder did not return to STOPPED state after starting")
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(start_btn.widget.text, "Start", "Button did not return to 'Start' after recording completed")
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        # Validate expected RGB output.
        self.assertTrue(os.path.exists(out_dir), f"Output not created: {out_dir}")
        self.assertTrue(
            validate_folder_contents(out_dir, {"png": NUM_FRAMES}),
            f"Expected {NUM_FRAMES} png files in {out_dir}",
        )

    async def test_ui_record_depth_control_timeline(self) -> None:
        """Test recording depth output through UI with timeline control enabled."""
        # This test mirrors the RGB test with only two intentional differences:
        # - control timeline enabled
        # - depth annotator enabled (no RGB)
        out_dir_name = OUT_DIR_DEPTH_CONTROL_TIMELINE
        out_dir = os.path.join(os.getcwd(), out_dir_name)
        if os.path.exists(out_dir):
            shutil.rmtree(out_dir)

        # Scene setup.
        await omni.usd.get_context().new_stage_async()
        rep.functional.create.cube(semantics=[("class", "cube")])
        cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), name="my_rep_camera_depth")
        cam_path = str(cam.GetPath())

        # Open and stabilize the recorder window.
        await menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
        window = ui.Workspace.get_window(WINDOW_TITLE)
        self.assertIsNotNone(window, "Synthetic Data Recorder window not found")
        window.visible = True
        window.focus()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        # Guard against cross-test state leakage.
        self.assertFalse(window._recorder.verbose, "Recorder verbose leaked from previous test")

        start_btn = ui_test.find(START_BUTTON)
        self.assertIsNotNone(start_btn, f"Start button not found at: {START_BUTTON}")
        self.assertEqual(start_btn.widget.text, "Start")

        # Configure common recorder settings via UI fields.
        num_frames_field = ui_test.find(NUM_FRAMES_FIELD)
        self.assertIsNotNone(num_frames_field, f"Number of frames field not found at: {NUM_FRAMES_FIELD}")
        num_frames_field.model.set_value(NUM_FRAMES)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._recorder.num_frames, NUM_FRAMES, "Recorder num_frames was not updated by UI")

        rp_camera_path_field = ui_test.find(RP_CAMERA_PATH_FIELD)
        self.assertIsNotNone(rp_camera_path_field, f"RP camera path field not found at: {RP_CAMERA_PATH_FIELD}")
        rp_camera_path_field.model.set_value(cam_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._recorder.rp_data[0][0], cam_path, "Render product camera path was not updated by UI")

        out_working_dir_field = ui_test.find(OUT_WORKING_DIR_FIELD)
        self.assertIsNotNone(
            out_working_dir_field, f"Output working directory field not found at: {OUT_WORKING_DIR_FIELD}"
        )
        out_working_dir_field.model.set_value(os.getcwd())
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._out_working_dir, os.getcwd(), "Output working directory was not updated by UI")

        out_dir_field = ui_test.find(OUT_DIR_FIELD)
        self.assertIsNotNone(out_dir_field, f"Output directory field not found at: {OUT_DIR_FIELD}")
        out_dir_field.model.set_value(out_dir_name)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._out_dir, out_dir_name, "Output directory was not updated by UI")

        writer_default_radio = ui_test.find(WRITER_DEFAULT_RADIO)
        self.assertIsNotNone(writer_default_radio, f"Default writer radio not found at: {WRITER_DEFAULT_RADIO}")
        await writer_default_radio.click()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(window._recorder.writer_name, "BasicWriter", "Writer was not set to BasicWriter via UI")

        # Test-specific options: depth on, RGB off, control timeline on.
        for key in window._basic_writer_params:
            window._basic_writer_params[key] = False

        rgb_checkbox = ui_test.find(RGB_CHECKBOX)
        self.assertIsNotNone(rgb_checkbox, f"RGB checkbox not found at: {RGB_CHECKBOX}")
        rgb_checkbox.model.set_value(False)
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(window._basic_writer_params["rgb"], "RGB checkbox did not clear basic writer param")

        depth_checkbox = ui_test.find(DEPTH_CHECKBOX)
        self.assertIsNotNone(depth_checkbox, f"Depth checkbox not found at: {DEPTH_CHECKBOX}")
        depth_checkbox.model.set_value(False)
        await omni.kit.app.get_app().next_update_async()
        depth_checkbox.model.set_value(True)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(window._basic_writer_params["distance_to_camera"], "Depth checkbox did not set depth annotator")

        control_timeline_checkbox = ui_test.find(CONTROL_TIMELINE_CHECKBOX)
        self.assertIsNotNone(
            control_timeline_checkbox, f"Control Timeline checkbox not found at: {CONTROL_TIMELINE_CHECKBOX}"
        )
        control_timeline_checkbox.model.set_value(True)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(window._recorder.control_timeline, "Control Timeline checkbox did not update recorder state")

        # Start through UI and wait for completion.
        await start_btn.click()

        saw_stop_state = False
        for _ in range(30):
            await omni.kit.app.get_app().next_update_async()
            if start_btn.widget.text == "Stop":
                saw_stop_state = True
                break
        self.assertTrue(saw_stop_state, "Button did not change to 'Stop' after starting recording")

        recorder_stopped = False
        for _ in range(NUM_FRAMES + 120):
            await omni.kit.app.get_app().next_update_async()
            if window._recorder.get_state() == RecorderState.STOPPED:
                recorder_stopped = True
                break
        self.assertTrue(recorder_stopped, "Recorder did not return to STOPPED state after starting")

        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(start_btn.widget.text, "Start", "Button did not return to 'Start' after recording completed")
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        # Validate expected depth-only output.
        self.assertTrue(os.path.exists(out_dir), f"Output not created: {out_dir}")
        npy_count = 0
        png_count = 0
        for root, _, files in os.walk(out_dir):
            npy_count += sum(1 for file_name in files if file_name.lower().endswith(".npy"))
            png_count += sum(1 for file_name in files if file_name.lower().endswith(".png"))
        self.assertEqual(npy_count, NUM_FRAMES, f"Expected {NUM_FRAMES} npy files in {out_dir}, found {npy_count}")
        self.assertEqual(png_count, 0, f"Expected 0 png files in {out_dir}, found {png_count}")

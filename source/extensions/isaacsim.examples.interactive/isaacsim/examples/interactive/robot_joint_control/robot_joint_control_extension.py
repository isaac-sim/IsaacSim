# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio
import os

import omni.ext
import omni.ui as ui
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from isaacsim.examples.interactive.robot_joint_control import RobotJointControl
from isaacsim.gui.components.ui_utils import btn_builder, get_style, setup_ui_headers, state_btn_builder, float_builder


class RobotJointControlExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.example_name = "Robot Joint Control"
        self.category = "Manipulation"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Robot Joint Control",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/examples.html",
            "overview": "This example demonstrates direct robot joint position control.\n\n"
                        "Click 'Load' to load the scene with a Franka robot.\n"
                        "Use the sliders to control individual joint positions.\n"
                        "Click 'Start' to begin applying joint positions.\n\n"
                        "Features:\n"
                        "- Direct joint position control via sliders\n"
                        "- Home position preset\n"
                        "- Gripper open/close control",
            "sample": RobotJointControl(),
        }

        ui_handle = RobotJointControlUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )
        return

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        return


class RobotJointControlUI(BaseSampleUITemplate):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._joint_sliders = []

    def build_extra_frames(self):
        extra_stacks = self.get_extra_frames_handle()
        self.task_ui_elements = {}

        with extra_stacks:
            with ui.CollapsableFrame(
                title="Joint Control",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_joint_control_ui()

            with ui.CollapsableFrame(
                title="Quick Actions",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_quick_actions_ui()
        return

    def build_joint_control_ui(self):
        # Joint limits for Franka robot (approximate)
        joint_limits = [
            (-2.8973, 2.8973),   # Joint 1
            (-1.7628, 1.7628),   # Joint 2
            (-2.8973, 2.8973),   # Joint 3
            (-3.0718, -0.0698),  # Joint 4
            (-2.8973, 2.8973),   # Joint 5
            (-0.0175, 3.7525),   # Joint 6
            (-2.8973, 2.8973),   # Joint 7
            (0.0, 0.04),         # Finger 1
            (0.0, 0.04),         # Finger 2
        ]
        joint_names = [
            "Joint 1 (Shoulder)",
            "Joint 2 (Shoulder)",
            "Joint 3 (Elbow)",
            "Joint 4 (Elbow)",
            "Joint 5 (Wrist)",
            "Joint 6 (Wrist)",
            "Joint 7 (Wrist)",
            "Finger Left",
            "Finger Right",
        ]

        with ui.VStack(spacing=5):
            # Start/Stop control button
            dict = {
                "label": "Joint Control",
                "type": "button",
                "a_text": "START",
                "b_text": "STOP",
                "tooltip": "Start/Stop joint control",
                "on_clicked_fn": self._on_start_control_button_event,
            }
            self.task_ui_elements["Start Control"] = state_btn_builder(**dict)
            self.task_ui_elements["Start Control"].enabled = False

            ui.Spacer(height=10)
            ui.Label("Joint Positions (radians):", height=20)
            ui.Spacer(height=5)

            # Create sliders for each joint
            self._joint_sliders = []
            for i, (name, limits) in enumerate(zip(joint_names, joint_limits)):
                with ui.HStack(height=25):
                    ui.Label(name, width=120)
                    slider = ui.FloatSlider(min=limits[0], max=limits[1], step=0.01)
                    slider.model.set_value(0.0)
                    slider.model.add_value_changed_fn(lambda m, idx=i: self._on_joint_slider_changed(idx, m.get_value_as_float()))
                    self._joint_sliders.append(slider)
                    slider.enabled = False

    def build_quick_actions_ui(self):
        with ui.VStack(spacing=5):
            dict = {
                "label": "Home Position",
                "type": "button",
                "text": "GO HOME",
                "tooltip": "Move robot to home position",
                "on_clicked_fn": self._on_home_button_event,
            }
            self.task_ui_elements["Home Position"] = btn_builder(**dict)
            self.task_ui_elements["Home Position"].enabled = False

            dict = {
                "label": "Open Gripper",
                "type": "button",
                "text": "OPEN",
                "tooltip": "Open the gripper",
                "on_clicked_fn": self._on_open_gripper_button_event,
            }
            self.task_ui_elements["Open Gripper"] = btn_builder(**dict)
            self.task_ui_elements["Open Gripper"].enabled = False

            dict = {
                "label": "Close Gripper",
                "type": "button",
                "text": "CLOSE",
                "tooltip": "Close the gripper",
                "on_clicked_fn": self._on_close_gripper_button_event,
            }
            self.task_ui_elements["Close Gripper"] = btn_builder(**dict)
            self.task_ui_elements["Close Gripper"].enabled = False

    def _on_start_control_button_event(self, val):
        asyncio.ensure_future(self.sample._on_start_control_event_async(val))
        return

    def _on_joint_slider_changed(self, joint_index: int, value: float):
        if self.sample is not None:
            self.sample.set_joint_position(joint_index, value)
        return

    def _on_home_button_event(self):
        if self.sample is not None:
            self.sample.move_to_home_position()
            # Update sliders to reflect home position
            home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04]
            for i, pos in enumerate(home_positions):
                if i < len(self._joint_sliders):
                    self._joint_sliders[i].model.set_value(pos)
        return

    def _on_open_gripper_button_event(self):
        if self.sample is not None:
            self.sample.open_gripper()
            # Update gripper sliders
            if len(self._joint_sliders) >= 9:
                self._joint_sliders[7].model.set_value(0.04)
                self._joint_sliders[8].model.set_value(0.04)
        return

    def _on_close_gripper_button_event(self):
        if self.sample is not None:
            self.sample.close_gripper()
            # Update gripper sliders
            if len(self._joint_sliders) >= 9:
                self._joint_sliders[7].model.set_value(0.0)
                self._joint_sliders[8].model.set_value(0.0)
        return

    def post_reset_button_event(self):
        self.task_ui_elements["Start Control"].enabled = True
        self.task_ui_elements["Home Position"].enabled = True
        self.task_ui_elements["Open Gripper"].enabled = True
        self.task_ui_elements["Close Gripper"].enabled = True
        for slider in self._joint_sliders:
            slider.enabled = True
        if self.task_ui_elements["Start Control"].text == "STOP":
            self.task_ui_elements["Start Control"].text = "START"
        # Reset sliders to zero
        for slider in self._joint_sliders:
            slider.model.set_value(0.0)
        return

    def post_load_button_event(self):
        self.task_ui_elements["Start Control"].enabled = True
        self.task_ui_elements["Home Position"].enabled = True
        self.task_ui_elements["Open Gripper"].enabled = True
        self.task_ui_elements["Close Gripper"].enabled = True
        for slider in self._joint_sliders:
            slider.enabled = True
        # Initialize sliders with current joint positions
        if self.sample is not None:
            positions = self.sample.get_joint_positions()
            for i, pos in enumerate(positions):
                if i < len(self._joint_sliders):
                    self._joint_sliders[i].model.set_value(float(pos))
        return

    def post_clear_button_event(self):
        self.task_ui_elements["Start Control"].enabled = False
        self.task_ui_elements["Home Position"].enabled = False
        self.task_ui_elements["Open Gripper"].enabled = False
        self.task_ui_elements["Close Gripper"].enabled = False
        for slider in self._joint_sliders:
            slider.enabled = False
        if self.task_ui_elements["Start Control"].text == "STOP":
            self.task_ui_elements["Start Control"].text = "START"
        return

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import carb
import omni.ui as ui
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.gui.components.ui_utils import btn_builder, get_style


class ExampleUI(BaseSampleUITemplate):
    """Custom UI for {{title}}.

    Extends BaseSampleUITemplate which provides Load World and Reset buttons.
    Add your custom controls in build_extra_frames().
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

    def build_extra_frames(self) -> None:
        """Build custom UI controls below the World Controls frame.

        This is where you add buttons, sliders, and other UI elements
        specific to your extension. The Load World and Reset buttons
        are already provided by the base class.
        """
        extra_stacks = self.get_extra_frames_handle()
        self._task_buttons = {}

        with extra_stacks:
            with ui.CollapsableFrame(
                title="Actions",
                width=ui.Fraction(1),
                height=0,
                collapsed=False,
                style=get_style(),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    self._task_buttons["Run"] = btn_builder(
                        label="Run Action",
                        type="button",
                        text="Run",
                        tooltip="Execute the example action",
                        on_clicked_fn=self._on_run_action,
                    )
                    self._task_buttons["Run"].enabled = False

    def _on_run_action(self) -> None:
        """Handle the Run button click. Add your custom logic here."""
        carb.log_info("[{{extension_name}}] Run action triggered")

    def post_load_button_event(self) -> None:
        """Called after Load World completes. Enable custom controls here."""
        for btn in self._task_buttons.values():
            btn.enabled = True

    def post_reset_button_event(self) -> None:
        """Called after Reset completes. Re-enable custom controls here."""
        for btn in self._task_buttons.values():
            btn.enabled = True

    def post_clear_button_event(self) -> None:
        """Called when the timeline stops. Disable custom controls here."""
        for btn in self._task_buttons.values():
            btn.enabled = False

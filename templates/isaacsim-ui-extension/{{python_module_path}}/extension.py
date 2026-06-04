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

import os

import omni.ext
from isaacsim.examples.browser import get_instance as get_browser_instance

from .scenario import ExampleScenario
from .ui import ExampleUI


class Extension(omni.ext.IExt):
    """{{title}} extension.

    Registers the example with the Examples Browser and provides
    a UI with Load/Reset world controls and custom action buttons.
    """

    def on_startup(self, ext_id: str) -> None:
        self.example_name = "{{title}}"
        self.category = "{{category}}"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "{{title}}",
            "doc_link": "",
            "overview": "{{description}}",
            "sample": ExampleScenario(),
        }

        self._ui_handle = ExampleUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=self._ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        self._ui_handle.on_shutdown()
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)

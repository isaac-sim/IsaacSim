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

"""Viewport scene management for TF frame visualization in ROS 2."""

import omni.ui as ui
from omni.ui import scene as sc

from .view_manipulator import ViewManipulator


class ViewportScene:
    """Manage a viewport scene overlay for TF frame visualization."""

    def __init__(self, viewport_window: ui.Window, ext_id: str) -> None:
        self._scene_view = None
        self._viewport_window = viewport_window

        # scene view frame
        with self._viewport_window.get_frame(ext_id):
            # scene view (default camera-model)
            self._scene_view = sc.SceneView()

            # add handlers into the scene view's scene
            with self._scene_view.scene:
                self.manipulator = ViewManipulator()
                self.manipulator.update_transforms({}, [])

            # register the scene view to get projection and view updates
            self._viewport_window.viewport_api.add_scene_view(self._scene_view)

    def __del__(self) -> None:
        """Clean up the viewport scene."""
        self.destroy()

    def destroy(self) -> None:
        """Destroy the viewport scene and release resources."""
        if self._scene_view:
            # empty the scene view
            self._scene_view.scene.clear()
            # un-register the scene view
            if self._viewport_window:
                self._viewport_window.viewport_api.remove_scene_view(self._scene_view)
        # remove references
        self._viewport_window = None
        self._scene_view = None

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

"""A manipulator that displays clickable icons in 3D viewport space for USD prims."""

__all__ = ["IconManipulator", "PreventOthers"]

import asyncio
import functools

import carb.settings
import omni.kit.app
import omni.kit.viewport.utility as vpUtil
from omni.ui import scene as sc
from pxr import Gf, Sdf

SHOW_TITLE_PATH = "exts/omni.kit.prim.icon/showTitle"


class PreventOthers(sc.GestureManager):
    """Prevent other gestures from hiding the icon click gesture."""

    def __init__(self) -> None:
        super().__init__()

    def can_be_prevented(self, gesture: object) -> bool:
        """Determines if this gesture can be prevented by other gestures.

        Args:
            gesture: The gesture to check for prevention capability.

        Returns:
            bool: Always returns False to prevent this gesture from being hidden by others.
        """
        return False

    def should_prevent(self, gesture: object, preventer: object) -> bool:
        """Determines if a preventer gesture should prevent the given gesture.

        Args:
            gesture: The gesture that might be prevented.
            preventer: The gesture that might do the preventing.

        Returns:
            bool: True if the preventer is in BEGAN or CHANGED state, otherwise delegates to parent implementation.
        """
        if preventer.state == sc.GestureState.BEGAN or preventer.state == sc.GestureState.CHANGED:
            return True
        return super().should_prevent(gesture, preventer)


class IconManipulator(sc.Manipulator):
    """A scene manipulator that displays clickable icons in 3D viewport space for USD prims.

    This manipulator renders icons as billboard images that always face the camera, positioned at specific
    3D world coordinates. Each icon can display an optional text label and responds to click interactions.
    The manipulator automatically updates icon visibility and positions when the underlying model changes.

    The icons are built from a model that provides prim paths, positions, icon URLs, and click handlers.
    When an icon is clicked, the manipulator delegates to the model's registered click handler for that prim.
    Icons can be dynamically added, removed, or updated based on model notifications.

    Args:
        icon_scale: Scaling factor for icon size in the viewport.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, icon_scale: float = 1.0, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._icons = {}
        self._icons_images = {}
        self._icon_panel = None
        self._icon_scale = icon_scale

    def on_build(self) -> None:
        """Builds the icon panel and populates it with icons for all prims in the model."""
        if not self.model:
            return
        self._icon_panel = sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 0, 0))
        self.rebuild_icons()

    def rebuild_icons(self, need_check: bool = False) -> None:
        """Rebuilds all icons by clearing existing ones and creating new ones for all prims in the model.

        Args:
            need_check: Whether to check if the icon position is within the viewport before building.
        """
        for prim_path in list(self._icons.keys()):
            if self._icons[prim_path]:
                self._icons[prim_path].clear()
            self._icons.pop(prim_path, None)
            self._icons_images.pop(prim_path, None)

        if not self._icon_panel:
            return

        self._icon_panel.clear()
        with self._icon_panel:
            for prim_path in self.model.get_prim_paths():
                self.build_icon_by_path(prim_path, need_check)

    def check_viewport_pos(self, position: object) -> bool:
        """Check if the world position is within the viewport screen space.

        Args:
            position: The world position to check.

        Returns:
            bool: True if the position is within the viewport screen space.
        """
        viewport_api = vpUtil.get_active_viewport()
        if not viewport_api:
            return False
        if not isinstance(position, Gf.Vec3d):
            try:
                position = Gf.Vec3d(position)
            except Exception:
                return False  # Cannot convert

        world_to_ndc = viewport_api.world_to_ndc
        ndc_pos = world_to_ndc.Transform(position)

        # map_ndc_to_texture returns None for viewport if position is outside
        pos, viewport = viewport_api.map_ndc_to_texture([ndc_pos[0], ndc_pos[1]])
        return viewport is not None

    def build_icon_by_path(self, prim_path: object, need_check: bool) -> None:
        """Build the UI elements for a single icon at the given path.

        Args:
            prim_path: The path of the prim to build an icon for.
            need_check: Whether to check if the icon position is within the viewport before building.
        """
        icon_pos = self.model.get_position(prim_path)
        if not icon_pos:
            return
        if need_check:
            if not self.check_viewport_pos(icon_pos):
                return

        icon_trans = sc.Transform(
            look_at=sc.Transform.LookAt.CAMERA,
            transform=sc.Matrix44.get_translation_matrix(*icon_pos),
        )
        icon_url = self.model.get_icon_url(prim_path)
        prevent_others = PreventOthers()

        self._icons[prim_path] = icon_trans
        with icon_trans:
            with sc.Transform():
                # Click gesture prevents other gestures from hiding it
                icons_image = sc.Image(
                    icon_url,
                    0.09 * self._icon_scale,
                    0.09 * self._icon_scale,
                    gesture=sc.ClickGesture(functools.partial(self._icon_clicked, prim_path), manager=prevent_others),
                )
                self._icons_images[prim_path] = icons_image

            # Optionally show prim name as label
            show_title = carb.settings.get_settings().get(SHOW_TITLE_PATH)
            if show_title:
                with sc.Transform(scale_to=sc.Space.NDC, transform=sc.Matrix44.get_translation_matrix(-0.03, -0.04, 0)):
                    name = prim_path.name
                    if len(name) > 12:
                        name = name[0:4] + "..." + name[-4:]
                    sc.Label(name)

        # Ensure the UI element respects the model's visibility state
        item = self.model.get_item(prim_path) if self.model else None
        if item is not None:
            icon_trans.visible = item.visible

    def update_icon_position(self, prim_path: object) -> None:
        """Update the transform of an existing icon UI element.

        Args:
            prim_path: The path of the prim whose icon position should be updated.
        """
        icon_pos = self.model.get_position(prim_path)
        if not icon_pos:
            return
        if prim_path in self._icons:
            self._icons[prim_path].transform = sc.Matrix44.get_translation_matrix(*icon_pos)

    def on_model_updated(self, item: object) -> None:
        """Callback when the model signals an item has changed.

        Args:
            item: The item that changed in the model.
        """
        if not item:
            # Model cleared or major change, rebuild everything
            self.invalidate()
            return

        prim_path = item.prim_path
        if prim_path in self._icons:
            # Item exists in UI, update or remove it
            if item.removed:
                if self._icons[prim_path]:
                    self._icons[prim_path].clear()
                self._icons.pop(prim_path, None)
                self._icons_images.pop(prim_path, None)
            else:
                # Update visibility and position
                if self._icons[prim_path]:
                    self._icons[prim_path].visible = item.visible
                self.update_icon_position(prim_path)
        elif not item.removed:
            # Item is new and not marked for removal, build it
            if not self._icon_panel:
                self.on_build()
            if self._icon_panel:
                with self._icon_panel:
                    self.build_icon_by_path(prim_path, False)

    def _icon_clicked(self, prim_path: Sdf.Path, shape: sc.AbstractShape) -> None:
        """Handle click gestures on the icon image.

        Args:
            prim_path: The path of the prim whose icon was clicked.
            shape: The scene shape that was clicked.
        """

        async def delay_click() -> None:
            await omni.kit.app.get_app().next_update_async()
            # Re-fetch the handler inside async func to ensure it's still valid
            # and check it before calling
            actual_click_handler = self.model.get_on_click(prim_path)
            if actual_click_handler:
                actual_click_handler(prim_path)
            else:
                pass  # No handler registered, do nothing silently

        asyncio.ensure_future(delay_click())

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

"""Extension entry point for the robot schema UI."""

__all__ = ["SchemaUIExtension"]

from functools import partial
from pathlib import Path
from typing import Any

import carb.eventdispatcher
import carb.settings
import omni.ext
import omni.kit.app
import omni.ui as ui
import omni.usd
from omni.kit.menu.utils import MenuHelperExtension
from omni.kit.viewport.registry import RegisterScene
from omni.kit.widget.stage import StageColumnDelegateRegistry

from .anchor_column import AnchorColumnDelegate
from .bypass_column import BypassColumnDelegate
from .deactivate_column import DeactivateColumnDelegate
from .masking_ops import MaskingOperations
from .masking_state import MaskingState, is_joint_type, is_maskable_type
from .robot_inspector_window import RobotInspectorWindow
from .scene import ConnectionScene


class SchemaUIExtension(omni.ext.IExt, MenuHelperExtension):
    """Extension providing the Robot Inspector window and viewport visualization.

    Registers a dockable window showing the robot structure with component
    inspection and masking controls, and a viewport scene for visualizing
    joint connections.
    """

    WINDOW_NAME = "Robot Inspector"
    """The name of the Robot Inspector window."""
    MENU_GROUP = "Window"
    """The menu group where the Robot Hierarchy window option is located."""

    def on_startup(self, extension_id: str) -> None:
        """Initialize the extension when loaded.

        Registers the window, menu entry, viewport scene, deactivate column,
        and disabled prim type icons.

        Args:
            extension_id: Extension identifier provided by the extension manager.
        """
        self._window: RobotInspectorWindow | None = None
        self._viewport_scene: ConnectionScene | None = None
        self._column_delegate_sub: Any | None = None
        self._bypass_column_delegate_sub: Any | None = None
        self._anchor_column_delegate_sub: Any | None = None
        self._icon_override_sub: Any | None = None
        self._masking_ops: MaskingOperations | None = None
        self._stage_event_subs: list[Any] = []
        self._icons_dir = self._get_icons_dir(extension_id)

        # Wire masking operations into the state singleton
        self._masking_ops = MaskingOperations()
        MaskingState.get_instance().operations = self._masking_ops

        # Clear masking layer on stage open or close
        self._stage_event_subs = self._create_stage_event_subs()

        self._joint_disabled_icon = str(Path(self._icons_dir) / "icoJointDisabled.svg") if self._icons_dir else ""
        self._xform_disabled_icon = str(Path(self._icons_dir) / "icoXformDisabled.svg") if self._icons_dir else ""

        self._register_icon_override()
        self._register_deactivate_column()

        ui.Workspace.set_show_window_fn(SchemaUIExtension.WINDOW_NAME, self.show_window)
        ui.Workspace.show_window(SchemaUIExtension.WINDOW_NAME)
        self.menu_startup(
            SchemaUIExtension.WINDOW_NAME,
            SchemaUIExtension.WINDOW_NAME,
            SchemaUIExtension.MENU_GROUP,
        )
        original_joints_visual = carb.settings.get_settings().get("/persistent/physics/visualizationDisplayJoints")
        carb.settings.get_settings().set("/persistent/physics/visualizationDisplayJoints", True)
        self._viewport_scene = RegisterScene(ConnectionScene, extension_id)
        carb.settings.get_settings().set("/persistent/physics/visualizationDisplayJoints", original_joints_visual)

    def _create_stage_event_subs(self) -> list[Any]:
        """Subscribe to stage open and close events to clear the masking layer.

        Returns:
            List of subscription handles that must be kept alive.
        """
        usd_context = omni.usd.get_context()
        return [
            carb.eventdispatcher.get_eventdispatcher().observe_event(
                observer_name="isaacsim.robot.schema.ui.masking",
                event_name=usd_context.stage_event_name(event),
                on_event=lambda _: self._on_stage_cleared(),
            )
            for event in (omni.usd.StageEventType.OPENED, omni.usd.StageEventType.CLOSING)
        ]

    def _on_stage_cleared(self) -> None:
        """Clear the masking layer when a new stage is opened or the current one closes."""
        if self._masking_ops:
            self._masking_ops.clear_all()
        MaskingState.get_instance().clear()

    def _get_icons_dir(self, extension_id: str) -> str:
        """Get the path to the extension's icons directory.

        Uses carb tokens for reliable path resolution during startup.

        Args:
            extension_id: The extension identifier.

        Returns:
            The icons directory path.
        """
        import carb.tokens

        ext_path = carb.tokens.get_tokens_interface().resolve("${isaacsim.robot.schema.ui}")
        if not ext_path:
            ext_manager = omni.ext.get_ext_manager()
            ext_path = ext_manager.get_extension_path(extension_id)
        if not ext_path:
            carb.log_warn("Could not resolve extension path for isaacsim.robot.schema.ui")
            return ""
        return str(Path(ext_path) / "data" / "icons")

    def _register_icon_override(self) -> None:
        """Register a per-prim icon override callback with StageIcons.

        The callback checks the ``isaacsim:deactivated`` customData key on
        each prim and returns the appropriate disabled icon when set.
        The subscription is held alive in ``self._icon_override_sub``.

        Returns:
            None.
        """
        return
        # icons = StageIcons()
        # self._icon_override_sub = icons.register_icon_override(self._icon_override_fn)

    def _icon_override_fn(self, prim_type: str, prim_path: Any, stage: Any) -> str | None:
        """Icon override callback for deactivated prims.

        Args:
            prim_type: The USD prim type name.
            prim_path: The Sdf.Path of the prim in the stage.
            stage: The Usd.Stage the prim belongs to.

        Returns:
            Path to the disabled icon SVG, or None to use the default icon.
        """
        if not stage or not prim_path:
            return None
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return None
        if not prim.GetCustomDataByKey("isaacsim:deactivated"):
            return None
        if is_joint_type(prim):
            return self._joint_disabled_icon
        if is_maskable_type(prim):
            return self._xform_disabled_icon
        return None

    def _register_deactivate_column(self) -> None:
        """Register the Deactivate and Bypass column delegates.

        The subscription objects must be held alive; dropping them unregisters
        the columns.
        """
        registry = StageColumnDelegateRegistry()
        icons_dir = self._icons_dir
        self._column_delegate_sub = registry.register_column_delegate(
            "Deactivate", partial(DeactivateColumnDelegate, icons_dir=icons_dir)
        )
        self._bypass_column_delegate_sub = registry.register_column_delegate(
            "Bypass", partial(BypassColumnDelegate, icons_dir=icons_dir)
        )
        self._anchor_column_delegate_sub = registry.register_column_delegate(
            "Anchor", partial(AnchorColumnDelegate, icons_dir=icons_dir)
        )

    def on_shutdown(self) -> None:
        """Clean up when the extension is unloaded.

        Cleans up the window, menu entry, viewport scene, connection singleton
        (and its USD listener), column delegate, and deactivated icons.
        """
        self.menu_shutdown()
        if self._window:
            self._window.destroy()
            self._window = None

        ui.Workspace.set_show_window_fn(SchemaUIExtension.WINDOW_NAME, None)
        if self._viewport_scene:
            self._viewport_scene.destroy()
            self._viewport_scene = None

        from .scene import ConnectionInstance

        try:
            ConnectionInstance.get_instance().destroy()
        except Exception:
            pass

        # Remove masking layer and detach from state singleton
        if self._masking_ops:
            self._masking_ops.clear_all()
        masking_state = MaskingState.get_instance()
        masking_state.operations = None
        masking_state.clear()
        self._masking_ops = None

        self._stage_event_subs = []
        self._icon_override_sub = None
        self._column_delegate_sub = None
        self._bypass_column_delegate_sub = None
        self._anchor_column_delegate_sub = None

    def _on_visibility_changed(self, visible: bool) -> None:
        """Handle window visibility changes.

        Keeps the window instance when hidden so it can be reused with cached
        state when shown again, avoiding full rebuild on tab switch.

        Args:
            visible: True if visible, False if hidden.
        """
        self.menu_refresh()

    def show_window(self, value: bool) -> None:
        """Show or hide the Robot Inspector window.

        Reuses the existing window when showing again so cached hierarchy
        and path map are preserved instead of rebuilding.

        Args:
            value: True to show, False to hide.

        Returns:
            None.

        Example:

        .. code-block:: python

            extension.show_window(True)
        """
        if value:
            if self._window is not None:
                self._window.visible = True
                return
            window = RobotInspectorWindow()
            window.set_visibility_changed_listener(self._on_visibility_changed)
            self._window = window
        elif self._window:
            self._window.visible = False

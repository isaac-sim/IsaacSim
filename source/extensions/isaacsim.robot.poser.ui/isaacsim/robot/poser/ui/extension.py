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

"""Robot Poser UI extension: window, property panel, and stage icon for IsaacNamedPose."""

from __future__ import annotations

import asyncio
import gc
from pathlib import Path
from typing import Any

import carb
import carb.eventdispatcher
import omni
import omni.kit.app
import omni.kit.widget.stage
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.menu import MenuItemDescription
from isaacsim.robot.poser.robot_poser import invalidate_articulation_cache
from omni.kit.menu.utils import add_menu_items, remove_menu_items

from .ui.ui_builder import UIBuilder

EXTENSION_TITLE = "Robot Poser"


def _promote_to_front(property_window: Any, scheme: str, widget_name: str) -> None:
    """Move widget_name to position 0 in the top-stack widget dict.

    Args:
        property_window: Kit property window instance.
        scheme: Scheme name (e.g. 'prim').
        widget_name: Name of the widget to promote.
    """
    top = getattr(property_window, "_widgets_top", {}).get(scheme)
    if not top or widget_name not in top:
        return
    ref = top.pop(widget_name)
    reordered = {widget_name: ref}
    reordered.update(top)
    top.clear()
    top.update(reordered)


def _is_named_pose_payload(payload: Any) -> bool:
    """Return True when payload targets a single IsaacNamedPose prim.

    Args:
        payload: Property window payload.

    Returns:
        True if the payload has exactly one path and that prim is IsaacNamedPose.
    """
    try:
        paths = payload.get_paths()
        if len(paths) != 1:
            return False
        stage = payload.get_stage()
        if not stage:
            return False
        prim = stage.GetPrimAtPath(paths[0])
        return prim and prim.IsValid() and prim.GetTypeName() == "IsaacNamedPose"
    except Exception:
        return False


_KEEP_WIDGETS = frozenset(
    {
        "isaac_named_pose",
        "transform",  # Transform panel
        "raw_usd_properties",  # Raw USD Properties
        "raw",  # alternate name
        "raw_data",
        "path",  # alternate name
    }
)


def _suppress_other_widgets(property_window: Any, scheme: str) -> dict[str, Any]:
    """Wrap every other widget's on_new_payload to return False for IsaacNamedPose prims.

    Widgets in _KEEP_WIDGETS or whose name contains 'transform' or 'raw' are untouched.

    Args:
        property_window: Kit property window instance.
        scheme: Scheme name (e.g. 'prim').

    Returns:
        Dict mapping widget name to (widget, original_on_new_payload) for restore.
    """
    originals = {}
    for stack in (
        getattr(property_window, "_widgets_top", {}).get(scheme, {}),
        getattr(property_window, "_widgets_bottom", {}).get(scheme, {}),
    ):
        for name, widget in stack.items():
            lower = name.lower()
            if name in _KEEP_WIDGETS or "transform" in lower or "raw" in lower:
                continue
            orig = widget.on_new_payload

            def _make_wrapper(orig_fn: Any) -> Any:
                """Return a wrapper that rejects IsaacNamedPose payloads.

                Args:
                    orig_fn: Original on_new_payload to call for non-named-pose payloads.

                Returns:
                    A callable that wraps orig_fn.
                """

                def wrapper(payload: Any) -> bool:
                    """Forward payload to orig_fn unless it is a named pose payload.

                    Args:
                        payload: Property window payload.

                    Returns:
                        False for named pose payloads, else orig_fn(payload).
                    """
                    if payload and _is_named_pose_payload(payload):
                        return False
                    return orig_fn(payload)

                return wrapper

            widget.on_new_payload = _make_wrapper(orig)
            originals[name] = (widget, orig)
    return originals


def _restore_other_widgets(originals: dict[str, Any]) -> None:
    """Undo the wrapping applied by _suppress_other_widgets.

    Args:
        originals: Dict returned by _suppress_other_widgets.
    """
    for _name, (widget, orig) in originals.items():
        try:
            widget.on_new_payload = orig
        except Exception:
            pass


class Extension(omni.ext.IExt):
    """Standard UI extension boilerplate for the Robot Poser."""

    def on_startup(self, ext_id: str) -> None:
        """Run when the extension is loaded.

        Args:
            ext_id: Extension identifier from the application.
        """
        self._ext_id = ext_id
        self._ext_name = omni.ext.get_extension_name(ext_id)
        self._usd_context = omni.usd.get_context()
        self._task: asyncio.Task[None] | None = None

        self._window = ui.Window(
            title=EXTENSION_TITLE,
            width=1200,
            height=500,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
            identifier="poser_main_window",
        )
        self._window.set_visibility_changed_fn(self._on_window)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            self._ext_name,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        menu_entry = [
            MenuItemDescription(
                name=EXTENSION_TITLE, onclick_action=(self._ext_name, f"CreateUIExtension:{EXTENSION_TITLE}")
            )
        ]
        self._menu_items = [MenuItemDescription(name="Robotics", sub_menu=menu_entry)]
        add_menu_items(self._menu_items, "Tools")

        self.ui_builder = UIBuilder()
        # Wire up the callback so the UIBuilder can request/release the update subscription
        self.ui_builder._request_update_subscription = self._ensure_update_subscription
        self.ui_builder._release_update_subscription = self._release_update_subscription

        self._timeline = omni.timeline.get_timeline_interface()
        self._update_subscription = None

        # Register stage-panel icon for IsaacNamedPose prims
        self._register_stage_icon()

        # Register the Named Pose property panel in the Kit Property Window
        self._named_pose_widget: Any = None  # NamedPosePropertiesWidget | None, avoid circular import
        self._suppressed_originals: dict[str, Any] = {}
        self._register_property_widget()

    def on_shutdown(self) -> None:
        """Run when the extension is unloaded."""
        invalidate_articulation_cache()
        self._unregister_stage_icon()
        self._unregister_property_widget()
        remove_menu_items(self._menu_items, "Tools")

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self._ext_name, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window.set_visibility_changed_fn(None)
            self._window = None
        self._update_subscription = None

        # Break bound-method references stored on UIBuilder back to this Extension.
        self.ui_builder._request_update_subscription = lambda: None
        self.ui_builder._release_update_subscription = lambda: None
        self.ui_builder._on_tracking_state_changed_fn = None
        self.ui_builder.cleanup()

        # Cancel the async dock task if it hasn't completed.
        if hasattr(self, "_task") and self._task is not None and not self._task.done():
            self._task.cancel()
        self._task = None

        gc.collect()

    # ###################################################################
    # Stage Icon Registration
    # ###################################################################

    def _register_stage_icon(self) -> None:
        """Register a custom icon for IsaacNamedPose prims in the stage panel."""
        ext_dir = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module("isaacsim.robot.poser.ui")
        icon_path = str(Path(ext_dir).joinpath("icons/named_pose_stage.svg"))
        self._stage_icons = omni.kit.widget.stage.StageIcons()
        self._stage_icons.set("IsaacNamedPose", icon_path)

    def _unregister_stage_icon(self) -> None:
        """Remove the custom stage-panel icon."""
        if hasattr(self, "_stage_icons") and self._stage_icons:
            self._stage_icons = None

    # ###################################################################
    # Property Panel Registration
    # ###################################################################

    def _on_window_tracking_state_changed(self, prim_path: str, enabled: bool) -> None:
        """Forward Robot Poser window tracking changes to the property panel.

        Args:
            prim_path: Named pose prim path whose tracking changed.
            enabled: True if tracking started, False if stopped.
        """
        if self._named_pose_widget is not None:
            self._named_pose_widget.set_window_tracking(prim_path, enabled)

    def _on_prop_panel_tracking_toggled(self, prim_path: str, enabled: bool) -> bool:
        """Forward property panel tracking requests to the Robot Poser window.

        Args:
            prim_path: Named pose prim path to track.
            enabled: True to start tracking, False to stop.

        Returns:
            True if the window handled the request, False otherwise.
        """
        return self.ui_builder.toggle_tracking_for_path(prim_path, enabled)

    def _register_property_widget(self) -> None:
        """Register the Named Pose property panel in the Kit Property Window."""
        try:
            import omni.kit.window.property as p
        except ImportError:
            carb.log_warn("Robot Poser: omni.kit.window.property not available, property panel disabled")
            return

        from .properties import NamedPosePropertiesWidget

        self._named_pose_widget = NamedPosePropertiesWidget(title="Named Pose", collapsed=False)

        # Bidirectional tracking sync:
        # Window → panel: when Robot Poser window toggles a named pose's tracking.
        self.ui_builder._on_tracking_state_changed_fn = self._on_window_tracking_state_changed
        # Panel → window: when Track Target button in the property panel is clicked.
        self._named_pose_widget._notify_window_tracking_fn = self._on_prop_panel_tracking_toggled
        # Panel init: so on_new_payload can read the window's current tracking state.
        self._named_pose_widget._query_window_tracking_fn = lambda p: p in self.ui_builder._tracked_paths

        w = p.get_window()
        if w:
            w.register_widget("prim", "isaac_named_pose", self._named_pose_widget, True)
            _promote_to_front(w, "prim", "isaac_named_pose")
            self._suppressed_originals = _suppress_other_widgets(w, "prim")
            w.request_rebuild()

    def _unregister_property_widget(self) -> None:
        """Unregister the Named Pose property panel and restore wrapped widgets."""
        _restore_other_widgets(self._suppressed_originals)
        self._suppressed_originals = {}
        try:
            import omni.kit.window.property as p

            w = p.get_window()
            if w:
                w.unregister_widget("prim", "isaac_named_pose")
        except (ImportError, Exception):
            pass
        if self._named_pose_widget:
            self._named_pose_widget.destroy()
            self._named_pose_widget = None

    def _on_window(self, visible: bool) -> None:
        """Handle window visibility change; subscribe to stage/timeline when shown.

        Args:
            visible: Whether the window became visible.
        """
        if self._window.visible:
            self._usd_context = omni.usd.get_context()

            self._stage_event_sub_opened = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._on_stage_opened,
                observer_name="isaacsim.robot.poser.ui.Extension._on_stage_opened",
            )
            self._stage_event_sub_closed = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self._on_stage_closed,
                observer_name="isaacsim.robot.poser.ui.Extension._on_stage_closed",
            )
            self._stage_event_sub_assets_loaded = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.ASSETS_LOADED),
                on_event=self._on_assets_loaded,
                observer_name="isaacsim.robot.poser.ui.Extension._on_assets_loaded",
            )
            self._stage_event_sub_sim_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SIMULATION_STOP_PLAY),
                on_event=self._on_simulation_stop_play,
                observer_name="isaacsim.robot.poser.ui.Extension._on_simulation_stop_play",
            )

            self._timeline_event_sub_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_PLAY,
                on_event=self._on_timeline_play,
                observer_name="isaacsim.robot.poser.ui.Extension._on_timeline_play",
            )
            self._timeline_event_sub_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_STOP,
                on_event=self._on_timeline_stop,
                observer_name="isaacsim.robot.poser.ui.Extension._on_timeline_stop",
            )

            # No per-frame subscription by default; created on demand by _ensure_update_subscription
            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub_opened = None
            self._stage_event_sub_closed = None
            self._stage_event_sub_assets_loaded = None
            self._stage_event_sub_sim_stop = None
            self._timeline_event_sub_play = None
            self._timeline_event_sub_stop = None
            self._update_subscription = None
            self.ui_builder.cleanup()

    def _ensure_update_subscription(self) -> None:
        """Create the per-frame update subscription if not already active."""
        if self._update_subscription is None:
            self._update_subscription = (
                omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update)
            )

    def _release_update_subscription(self) -> None:
        """Destroy the per-frame update subscription."""
        self._update_subscription = None

    def _build_ui(self) -> None:
        """Build the main Robot Poser UI and schedule docking."""
        with self._window.frame:
            self.ui_builder.build_ui()

        async def dock_window() -> None:
            """Wait one frame then dock the Robot Poser window next to the Viewport."""
            await omni.kit.app.get_app().next_update_async()

            def dock(space: Any, name: str, location: Any, pos: float = 0.5) -> Any:
                """Dock the named window into the given space.

                Args:
                    space: Workspace space to dock into.
                    name: Window name.
                    location: Dock position.
                    pos: Fractional position (default 0.5).

                Returns:
                    The window if found, else None.
                """
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.45)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    def _menu_callback(self) -> None:
        """Toggle window visibility and notify UIBuilder."""
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_play(self, event: Any) -> None:
        """Forward timeline play event to UIBuilder.

        Args:
            event: Timeline play event payload.
        """
        self.ui_builder.on_timeline_event(event)

    def _on_timeline_stop(self, event: Any) -> None:
        """Forward timeline stop event to UIBuilder.

        Args:
            event: Timeline stop event payload.
        """
        invalidate_articulation_cache()
        self.ui_builder.on_timeline_event(event)

    def _on_update(self, event: Any) -> None:
        """Per-frame callback; delegate to UIBuilder.

        Args:
            event: Update event payload.
        """
        self.ui_builder.on_update(0.0)

    def _on_stage_opened(self, event: Any) -> None:
        """Clear update subscription and cleanup UI when stage is opened.

        Args:
            event: Stage opened event payload.
        """
        self._update_subscription = None
        self.ui_builder.cleanup()

    def _on_stage_closed(self, event: Any) -> None:
        """Clear update subscription and cleanup UI when stage is closed.

        Args:
            event: Stage closed event payload.
        """
        invalidate_articulation_cache()
        self._update_subscription = None
        self.ui_builder.cleanup()

    def _on_assets_loaded(self, event: Any) -> None:
        """Notify UIBuilder when stage assets are loaded.

        Args:
            event: Assets loaded event payload.
        """
        self.ui_builder.on_assets_loaded()

    def _on_simulation_stop_play(self, event: Any) -> None:
        """Notify UIBuilder when simulation stops.

        Args:
            event: Simulation stop event payload.
        """
        self.ui_builder.on_simulation_stop_play()

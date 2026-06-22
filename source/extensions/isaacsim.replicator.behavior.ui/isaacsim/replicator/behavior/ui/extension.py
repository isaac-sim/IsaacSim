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

"""UI extension for Isaac Sim Replicator behavior management with property widgets for exposed variables."""

from typing import Any

import carb.eventdispatcher
import omni.ext
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS, EXPOSED_VARS_CHANGED_EVENT

from .exposed_variables_widget import ExposedVariablesPropertyWidget
from .global_variables import WIDGET_NAME, WIDGET_TITLE

# ``omni.kit.window.property`` is only available when a UI is loaded. Guard the import so the UI
# extension can still be imported in headless contexts (e.g. test discovery), where the event
# subscription simply becomes a no-op.
try:
    import omni.kit.window.property as _prop_window
except ImportError:
    _prop_window = None


def rebuild_property_window() -> None:
    """Rebuild the property window if available (no-op in headless mode)."""
    if _prop_window is not None and _prop_window.get_window() is not None:
        _prop_window.get_window().request_rebuild()


class Extension(omni.ext.IExt):
    """Extension for Isaac Sim Replicator Behavior UI integration.

    This extension provides user interface components for managing behavior-related functionality within Isaac Sim's Replicator framework. It registers a custom property widget that allows users to interact with exposed variables in the Property window, enhancing the workflow for behavior configuration and monitoring.

    The extension integrates with the Property window by adding an ExposedVariablesPropertyWidget that filters and displays attributes based on the exposed attribute namespace, providing a streamlined interface for behavior parameter management.

    It also subscribes to the ``EXPOSED_VARS_CHANGED_EVENT`` dispatched by the core
    ``isaacsim.replicator.behavior`` extension when exposed USD variables are created or removed,
    and refreshes the property window in response. This decoupling allows the core extension to
    run headless without any UI dependency.
    """

    def __init__(self) -> None:
        super().__init__()
        self._registered = False
        self._exposed_vars_changed_sub = None

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is starting up.

        Args:
            ext_id: The extension identifier.
        """
        self._register_widget()
        self._exposed_vars_changed_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=EXPOSED_VARS_CHANGED_EVENT,
            on_event=self._on_exposed_vars_changed,
            observer_name="isaacsim.replicator.behavior.ui._exposed_vars_changed",
        )

    def on_shutdown(self) -> None:
        """Called when the extension is shutting down."""
        if self._exposed_vars_changed_sub is not None:
            self._exposed_vars_changed_sub.reset()
            self._exposed_vars_changed_sub = None
        if self._registered:
            self._unregister_widget()

    def _on_exposed_vars_changed(self, event: Any) -> None:
        """Refresh the property window when exposed variables are created or removed.

        Args:
            event: Exposed-variable change event.
        """
        rebuild_property_window()

    def _register_widget(self) -> None:
        """Register the exposed variables property widget with the property window."""
        if _prop_window is None:
            return
        property_window = _prop_window.get_window()
        if property_window:
            property_window.register_widget(
                "prim",
                WIDGET_NAME,
                ExposedVariablesPropertyWidget(title=WIDGET_TITLE, attribute_namespace_filter=[EXPOSED_ATTR_NS]),
            )
            self._registered = True

    def _unregister_widget(self) -> None:
        """Unregister the exposed variables property widget from the property window."""
        if _prop_window is None:
            return
        property_window = _prop_window.get_window()
        if property_window:
            property_window.unregister_widget("prim", WIDGET_NAME)
            self._registered = False

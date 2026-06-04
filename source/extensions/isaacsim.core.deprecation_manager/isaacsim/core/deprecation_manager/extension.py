# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension for managing deprecated settings and OmniGraph nodes in Isaac Sim."""

from __future__ import annotations

from typing import Any

import carb
import omni.ext
import omni.usd


class Extension(omni.ext.IExt):
    """Extension for managing deprecated settings and OmniGraph nodes in Isaac Sim.

    This extension automatically handles the migration of deprecated configuration settings and OmniGraph node types
    to their updated equivalents. It monitors for deprecated settings during startup and updates OmniGraph nodes
    when a stage is opened.

    The extension performs two main functions:

    1. **Settings Migration**: On startup, it checks for deprecated settings entries and automatically migrates
       them to their new counterparts. Deprecation warnings are logged to inform users of the changes.

    2. **OmniGraph Node Updates**: When a USD stage is opened, it traverses all prims to identify deprecated
       OmniGraph node types and updates them to their current equivalents. The extension provides detailed
       logging and notifications about the changes, including information about referenced assets that may
       need manual handling.

    The extension uses configuration entries from the extension settings to determine which deprecated items
    to migrate. For OmniGraph nodes, it also attempts to reload all graphs after making changes to ensure
    proper functionality.

    Deprecation warnings and notifications help users understand what changes have been made and provide
    guidance on preserving changes in referenced USD assets.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the deprecation manager extension.

        Loads deprecation settings, updates deprecated settings, and subscribes to stage events for OmniGraph node updates.

        Args:
            ext_id: The extension identifier.
        """
        # get extension settings
        settings = carb.settings.get_settings()
        self._settings_entries = settings.get("/exts/isaacsim.core.deprecation_manager/settings")
        self._omnigraph_entries = settings.get("/exts/isaacsim.core.deprecation_manager/omnigraph")

        # update deprecated settings
        self._update_deprecated_settings()

        # subscribe to stage event to update OmniGraph nodes
        self._stage_event_subscription = None
        if self._omnigraph_entries:
            self._stage_event_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._on_stage_event,
                observer_name="isaacsim.core.deprecation_manager._stage_event_subscription",
            )

    def on_shutdown(self) -> None:
        """Cleans up the extension resources.

        Removes stage event subscriptions and releases associated resources.
        """
        # delete stage event subscription
        self._stage_event_subscription = None

    def _update_deprecated_settings(self) -> None:
        """Update deprecated settings with their new equivalents.

        Iterates through configured deprecated settings, transfers values to new settings, and logs deprecation warnings.
        """
        settings = carb.settings.get_settings()
        for entry in self._settings_entries:
            deprecated_setting = entry.get("deprecated", "")
            new_setting = entry.get("new", "")
            value = settings.get(deprecated_setting)
            if value is not None:
                deprecation_message = f"'{deprecated_setting}' has been deprecated in favor of '{new_setting}'."
                # update setting
                if entry.get("update", True):
                    settings.set(new_setting, value)
                    deprecation_message += f" Setting '{new_setting}' to {value}."
                # show deprecation message
                carb.log_warn(deprecation_message)

    def _on_stage_event(self, event: Any) -> None:
        """Handle stage opening events to update deprecated OmniGraph nodes.

        Traverses the stage to find deprecated OmniGraph node types, updates them to new types, tracks referenced assets, and displays deprecation warnings with notifications.

        Args:
            event: The stage event that triggered this handler.
        """
        usd_reference_paths = set()
        deprecation_changes = []
        for prim in omni.usd.get_context().get_stage().Traverse():
            # OmniGraph node
            if prim.HasAttribute("node:type"):
                attr = prim.GetAttribute("node:type")
                value = attr.Get()
                # iterate through entries
                for entry in self._omnigraph_entries:
                    if entry["deprecated"] in value:
                        # rename OmniGraph type
                        new_value = value.replace(entry["deprecated"], entry["new"])
                        attr.Set(new_value)
                        deprecation_changes.append((entry["deprecated"], entry["new"], value, new_value))
                        # find USD references
                        while (prim := prim.GetParent()).IsValid():
                            references = omni.usd.get_composed_references_from_prim(prim, False)
                            payloads = omni.usd.get_composed_payloads_from_prim(prim, False)
                            if references or payloads:
                                for index, reference in enumerate(references):
                                    usd_reference_paths.add(
                                        (
                                            prim.GetPath().pathString,
                                            index,
                                            reference[1].ComputeAbsolutePath(reference[0].assetPath),
                                        )
                                    )
                                for index, payload in enumerate(payloads):
                                    usd_reference_paths.add(
                                        (
                                            prim.GetPath().pathString,
                                            index,
                                            payload[1].ComputeAbsolutePath(payload[0].assetPath),
                                        )
                                    )
                                break
        if not deprecation_changes:
            return
        usd_reference_paths = sorted(usd_reference_paths, key=lambda item: (item[0], item[1]), reverse=True)
        usd_reference_paths = list(dict.fromkeys([item[2] for item in usd_reference_paths]))
        deprecation_changes = sorted(set(deprecation_changes), key=lambda item: item[2])
        # show deprecation message
        save_message = "" if usd_reference_paths else "Save it to preserve the changes."
        carb.log_warn(f"The stage contains the following deprecated nodes that have been updated. {save_message}")
        if usd_reference_paths:
            carb.log_warn(
                f"  |-- Referenced assets (open listed assets in order before the current one to preserve changes on each reference):"
            )
            for item in usd_reference_paths:
                carb.log_warn(f"  |     |-- {item}")
        carb.log_warn(f"  |-- Deprecated nodes:")
        for item in deprecation_changes:
            carb.log_warn(f"  |     |-- {item[2]} -> {item[3]}")
        # show notification in Kit window
        try:
            import omni.kit.notification_manager as notification_manager
        except ImportError:
            pass
        else:
            text = f"The stage contains the following deprecated nodes that have been updated. {save_message}\n"
            if usd_reference_paths:
                text += "\nReferenced assets (open listed assets in order before the current one to preserve changes on each reference):\n"
                text += "\n".join([f" - {item}" for item in usd_reference_paths]) + "\n"
            text += "\nDeprecated nodes:\n"
            text += "\n".join([f" - {item[2]}" for item in deprecation_changes])
            notification_manager.post_notification(
                text,
                duration=0,
                hide_after_timeout=False,
                status=notification_manager.NotificationStatus.WARNING,
                button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
            )

        try:
            # reload all graphs if we made any changes
            # in some cases the omni.graph.core extension may not be enabled so this is optional
            import omni.graph.core as og

            all_graphs = og.get_all_graphs()
            for graph in all_graphs:
                graph.reload_from_stage()
        except Exception as e:
            carb.log_warn(f"Could not reload graphs after renaming nodes: {e}")

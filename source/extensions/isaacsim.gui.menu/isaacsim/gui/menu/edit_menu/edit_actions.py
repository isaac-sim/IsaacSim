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

"""Edit menu action registration helpers."""

__all__ = ["register_actions", "deregister_actions"]

# pylint: disable=protected-access
import asyncio
from collections.abc import Callable
from typing import Any

import omni.kit.actions.core


def register_actions(extension_id: str, cls: type[Any], get_self_fn: Callable[[], Any]) -> None:
    """Register edit-related actions for an extension.

    Args:
        extension_id: Identifier of the extension registering actions.
        cls: Class that implements the action handlers.
        get_self_fn: Callable that returns the class instance.

    Example:
        .. code-block:: python

            register_actions("isaacsim.gui.menu.edit_menu", EditMenuExtension, lambda: instance)
    """

    def select_prim(description: str, select_name: str, show_error: bool = True) -> None:
        """Select prims using a selection action and track the result.

        Args:
            description: Label for the selection operation.
            select_name: Action name to execute.
            show_error: Whether to post errors when no prims are selected.

        Returns:
            None.
        """
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not paths:
            if show_error:
                cls.post_notification(f'Cannot select "{description}" as no prims are selected')
            return None
        return do_select(description, select_name, paths)

    def do_select(description: str, select_name: str, paths: list[str]) -> None:
        """Execute a selection action and update the recent list.

        Args:
            description: Label for the selection operation.
            select_name: Action name to execute.
            paths: Current selection paths.
        """
        old_selection = omni.usd.get_context().get_selection().get_selected_prim_paths()

        async def select_func() -> None:
            await omni.kit.app.get_app().next_update_async()
            omni.kit.actions.core.execute_action("omni.kit.selection", select_name)
            last_description = f"{description} {old_selection}"
            new_selection = omni.usd.get_context().get_selection().get_selected_prim_paths()
            if len(last_description) > 45:
                last_description = last_description[:42] + "..."
            get_self_fn()._add_to_recent(last_description, new_selection)

        asyncio.ensure_future(select_func())

    action_registry = omni.kit.actions.core.get_action_registry()
    actions_tag = "Edit Menu Actions"

    # actions
    action_registry.register_action(
        extension_id,
        "select_recent",
        get_self_fn()._on_select_recent,
        display_name="Edit->Recent",
        description="Recent",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "select_selection_set",
        get_self_fn()._on_select_selection_set,
        display_name="Edit->Selection Set",
        description="Selection Set",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "select_by_kind",
        get_self_fn()._on_select_by_kind,
        display_name="Edit->Select By Kind",
        description="Select By Kind",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "selection_invert",
        lambda: select_prim(description="Inverse of", select_name="invert"),
        display_name="Edit->Select Invert",
        description="Select Invert",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "selection_parent",
        lambda: select_prim(description="Parent of", select_name="parent", show_error=False),
        display_name="Edit->Select Parent",
        description="Select Parent",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "selection_leaf",
        lambda: select_prim(description="Leaves of", select_name="leaf"),
        display_name="Edit->Select Leaf",
        description="Select Leaf",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "selection_hierarchy",
        lambda: select_prim(description="Hierarchy of", select_name="hierarchy"),
        display_name="Edit->Select Hierarchy",
        description="Select Hierarchy",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "selection_similar",
        lambda: select_prim(description="Similar of", select_name="similar"),
        display_name="Edit->Select Similar",
        description="Select Similar",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "create_selection_set",
        get_self_fn()._create_selection_set,
        display_name="Edit->Create Selection Set",
        description="Create Selection Set",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "instance_prim",
        cls.instance_prim,
        display_name="Edit->Instance Prim",
        description="Instance Prim",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "duplicate_prim",
        lambda: cls.duplicate_prim(duplicate_layers=False, combine_layers=False),
        display_name="Edit->Duplicate Prim",
        description="Duplicate Prim",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "duplicate_prim_and_layers",
        lambda: cls.duplicate_prim(duplicate_layers=True, combine_layers=False),
        display_name="Edit->Duplicate Prim & Duplicate Layers",
        description="Duplicate Prim & Duplicate Layers",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "duplicate_prim_and_combine_layers",
        lambda: cls.duplicate_prim(duplicate_layers=True, combine_layers=True),
        display_name="Edit->Duplicate Prim & Combine Layers",
        description="Duplicate Prim & Combine Layers",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "parent_prims",
        cls.parent_prims,
        display_name="Edit->Parent Prims",
        description="Parent Prims",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "unparent_prims",
        cls.unparent_prims,
        display_name="Edit->Unparent Prims",
        description="Unparent Prims",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "create_xform_to_group",
        cls.create_xform_to_group,
        display_name="Edit->Create XForm To Group",
        description="Create XForm To Group",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "ungroup_prims",
        cls.ungroup_prims,
        display_name="Edit->Create XForm To Group",
        description="Create XForm To Group",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "toggle_visibillity",
        cls.toggle_visibillity,
        display_name="Edit->Toggle Visibility",
        description="Toggle Visibility",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "deactivate_prims",
        cls.deactivate_prims,
        display_name="Edit->Deactivate Prims",
        description="Deactivate Prims",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "delete_prim",
        lambda: cls.delete_prim(False),
        display_name="Edit->Delete Prim",
        description="Delete Prim",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "delete_prim_all_layers",
        lambda: cls.delete_prim(True),
        display_name="Edit->Delete Prim - All Layers",
        description="Delete Prim - All Layers",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "menu_rename_prim_dialog",
        get_self_fn().menu_rename_prim_dialog,
        display_name="Edit->Rename Prim",
        description="Rename Prim",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "focus_prim",
        cls.focus_prim,
        display_name="Edit->Focus Prim",
        description="Focus Prim",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "toggle_global_visibility",
        cls.toggle_global_visibility,
        display_name="Edit->Toggle Global Visibility",
        description="Toggle Global Visibility",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "capture_screenshot",
        cls.capture_screenshot,
        display_name="Edit->Capture Screenshot",
        description="Capture Screenshot",
        tag=actions_tag,
    )


def deregister_actions(extension_id: str) -> None:
    """Deregister all actions for an extension.

    Args:
        extension_id: The unique identifier of the extension whose actions are to be deregistered.

    Example:
        .. code-block:: python

            deregister_actions("isaacsim.gui.menu.edit_menu")
    """
    action_registry = omni.kit.actions.core.get_action_registry()
    action_registry.deregister_all_actions_for_extension(extension_id)

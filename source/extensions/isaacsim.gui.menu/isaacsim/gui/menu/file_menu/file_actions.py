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

"""File menu action handlers for Isaac Sim."""

__all__ = ["register_actions", "deregister_actions"]

import carb
import omni.kit.actions.core
import omni.kit.window.file
import omni.usd


def post_notification(message: str, info: bool = False, duration: int = 3) -> None:
    """Post a notification with a message.

    Args:
        message: The message to display.
        info: Whether to post an info notification.
        duration: Duration in seconds for the notification.

    Example:
        .. code-block:: python

            post_notification("Scene saved.")
    """
    try:
        import omni.kit.notification_manager as nm

        _type = nm.NotificationStatus.WARNING
        if info:
            _type = nm.NotificationStatus.INFO

        nm.post_notification(message, status=_type, duration=duration)
    except ModuleNotFoundError:
        carb.log_warn(message)


def quit_kit(fast: bool = False) -> None:
    """Request the application to quit.

    Args:
        fast: Whether to enable fast shutdown.

    Example:
        .. code-block:: python

            quit_kit(fast=True)
    """
    if fast:
        carb.settings.get_settings().set("/app/fastShutdown", True)
    omni.kit.app.get_app().post_quit()


def open_stage_with_new_edit_layer() -> None:
    """Open the current USD stage with a new edit layer.

    This posts a notification if no valid stage is available.

    Example:
        .. code-block:: python

            open_stage_with_new_edit_layer()
    """
    stage = omni.usd.get_context().get_stage()
    if not stage:
        post_notification("Cannot Re-open with New Edit Layer. No valid stage")
        return

    omni.kit.window.file.open_with_new_edit_layer(stage.GetRootLayer().identifier)


def register_actions(extension_id: str) -> None:
    """Register file-related actions within an extension.

    Args:
        extension_id: The unique identifier for the extension that is registering the actions.

    Example:
        .. code-block:: python

            register_actions("isaacsim.gui.menu.file_menu")
    """
    action_registry = omni.kit.actions.core.get_action_registry()
    actions_tag = "File Actions"

    action_registry.register_action(
        extension_id,
        "quit",
        lambda: quit_kit(fast=False),
        display_name="File->Exit",
        description="Exit",
        tag=actions_tag,
    )
    action_registry.register_action(
        extension_id,
        "quit_fast",
        lambda: quit_kit(fast=True),
        display_name="File->Exit Fast",
        description="Exit Fast",
        tag=actions_tag,
    )

    action_registry.register_action(
        extension_id,
        "open_stage_with_new_edit_layer",
        open_stage_with_new_edit_layer,
        display_name="File->Open Current Stage With New Edit Layer",
        description="Open Stage With New Edit Layer",
        tag=actions_tag,
    )


def deregister_actions(extension_id: str) -> None:
    """Remove all registered actions for an extension.

    Args:
        extension_id: The unique identifier of the extension whose actions are to be deregistered.

    Example:
        .. code-block:: python

            deregister_actions("isaacsim.gui.menu.file_menu")
    """
    action_registry = omni.kit.actions.core.get_action_registry()
    action_registry.deregister_all_actions_for_extension(extension_id)

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

"""Utility functions for managing exposed behavior variables on USD prims."""

from __future__ import annotations

from typing import Any

import carb
import carb.eventdispatcher
import carb.events
import omni.kit.app
import omni.kit.commands
from pxr import Sdf, Usd

from ..global_variables import EXPOSED_VARS_CHANGED_EVENT


def _notify_exposed_variables_changed() -> None:
    """Dispatch an event signaling that exposed USD variables were created or removed.

    The UI extension (``isaacsim.replicator.behavior.ui``) subscribes to this event to
    refresh the property window. Dispatched unconditionally; if no subscriber is
    registered (e.g. headless / no-UI mode) the event is a no-op.
    """
    carb.eventdispatcher.get_eventdispatcher().dispatch_event(event_name=EXPOSED_VARS_CHANGED_EVENT, payload={})


def create_exposed_variable(
    prim: Usd.Prim, full_attr_name: str, attr_type: Sdf.ValueTypeName, default_value: object, doc: str = None
) -> Usd.Attribute:
    """Create a USD attribute on the prim to expose the variable to the UI, if the variable exits it returns it.

    Args:
        prim: The USD prim to create the attribute on.
        full_attr_name: The full namespaced attribute name.
        attr_type: The Sdf value type for the attribute.
        default_value: The default value for the attribute.
        doc: Optional documentation string for the attribute.

    Returns:
        The created or existing USD attribute.
    """
    attr = prim.GetAttribute(full_attr_name)
    if attr:
        return attr
    attr = prim.CreateAttribute(full_attr_name, attr_type)
    attr.Set(default_value)
    if doc:
        attr.SetDocumentation(doc)
    return attr


def create_exposed_variables(prim: Usd.Prim, exposed_attr_ns: str, behavior_ns: str, variables_to_expose: dict) -> None:
    """Create exposed variables based on the provided namespaces and data dictionary.

    Args:
        prim: The USD prim to create attributes on.
        exposed_attr_ns: The namespace for exposed attributes.
        behavior_ns: The behavior namespace.
        variables_to_expose: Dictionary of variables to expose.
    """
    # Check if there are any attributes to lock (e.g. constant placeholders -- cannot be edited in the UI)
    attr_to_lock = []
    for var in variables_to_expose:
        attr_name = var["attr_name"]
        full_attr_name = f"{exposed_attr_ns}:{behavior_ns}:{attr_name}"
        attr = create_exposed_variable(
            prim=prim,
            full_attr_name=full_attr_name,
            attr_type=var["attr_type"],
            default_value=var["default_value"],
            doc=var.get("doc"),
        )
        if var.get("lock"):
            attr_to_lock.append(attr.GetPath())
    import asyncio

    asyncio.ensure_future(lock_exposed_variables(attr_to_lock))

    _notify_exposed_variables_changed()


async def lock_exposed_variables(attr_paths: object) -> None:
    """Lock exposed variables to prevent editing in the UI.

    Args:
        attr_paths: List of attribute paths to lock.
    """
    await omni.kit.app.get_app().next_update_async()
    omni.kit.commands.execute("LockSpecsCommand", spec_paths=attr_paths)


def check_if_exposed_variables_should_be_removed(prim: Usd.Prim, script_file_path: str) -> bool:
    """Remove exposed variables if the script is no longer assigned to the prim.

    Args:
        prim: The USD prim to check.
        script_file_path: The file path of the script.

    Returns:
        True if exposed variables should be removed, False otherwise.
    """
    if prim is None or not prim.IsValid():
        # Invalid prim, cannot remove variables
        return False

    scripts_attr = prim.GetAttribute("omni:scripting:scripts")
    if not scripts_attr:
        # No scripts attribute, remove variables
        return True

    scripts_paths: Sdf.AssetPathArray = scripts_attr.Get()
    if not scripts_paths:
        # Empty scripts attribute, remove variables
        return True

    # Remove variables if script is not in the attached scripts list (e.g. script was removed)
    return not any(script_file_path == asset.path for asset in scripts_paths)


def remove_exposed_variable(prim: Usd.Prim, full_attr_name: str, remove_from_fabric: bool = True) -> None:
    """Remove the exposed variable from the prim.

    Args:
        prim: The USD prim to remove the attribute from.
        full_attr_name: The full namespaced attribute name.
        remove_from_fabric: Whether to also remove the attribute from fabric.
    """
    if prim is None or not prim.IsValid():
        carb.log_warn(f"Prim {prim.GetPath()} is not valid, cannot remove exposed variable {full_attr_name}")
        return
    attr = prim.GetAttribute(full_attr_name)
    if attr:
        prim.RemoveProperty(attr.GetName())
        # Remove the attribute from fabric as well
        if remove_from_fabric:
            import usdrt
            from pxr import UsdUtils

            stage = prim.GetStage()
            stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
            stage_rt = usdrt.Usd.Stage.Attach(stage_id)
            prim_rt = stage_rt.GetPrimAtPath(usdrt.Sdf.Path(prim.GetPath().pathString))
            attr_rt = prim_rt.GetAttribute(full_attr_name)
            if attr_rt:
                prim_rt.RemoveProperty(full_attr_name)
    else:
        carb.log_warn(f"Attribute {full_attr_name} not found on {prim.GetPath()}")


def remove_exposed_variables(prim: Usd.Prim, exposed_attr_ns: str, behavior_ns: str, variables_to_expose: dict) -> None:
    """Remove exposed variables based on the provided namespaces and data dictionary.

    Args:
        prim: The USD prim to remove attributes from.
        exposed_attr_ns: The namespace for exposed attributes.
        behavior_ns: The behavior namespace.
        variables_to_expose: Dictionary of variables to remove.
    """
    for var in variables_to_expose:
        attr_name = var["attr_name"]
        full_attr_name = f"{exposed_attr_ns}:{behavior_ns}:{attr_name}"
        remove_exposed_variable(prim, full_attr_name)

    _notify_exposed_variables_changed()


def get_exposed_variable(prim: Usd.Prim, full_attr_name: str) -> Any:
    """Get the value of an exposed attribute.

    Args:
        prim: The USD prim to get the attribute from.
        full_attr_name: The full namespaced attribute name.

    Returns:
        The value of the exposed attribute, or None if not found.
    """
    if prim is None or not prim.IsValid():
        carb.log_warn(f"Prim is not valid, cannot receive exposed variable {full_attr_name}")
        return None
    attr = prim.GetAttribute(full_attr_name)
    if attr:
        return attr.Get()
    else:
        return None


def set_exposed_variables(prim: Usd.Prim, exposed_variables: dict) -> None:
    """Set exposed variables based on the provided data dictionary.

    Args:
        prim: The USD prim to set attributes on.
        exposed_variables: Dictionary mapping attribute names to values.
    """
    if prim is None or not prim.IsValid():
        carb.log_warn(f"Prim is not valid, cannot receive exposed variable")
        return
    for attr_name, value in exposed_variables.items():
        attr = prim.GetAttribute(attr_name)
        if attr:
            attr.Set(value)
        else:
            print(f"Attribute {attr_name} not found on {prim.GetPath()}")


def remove_empty_scopes(prim: Usd.Prim, stage: Usd.Stage) -> None:
    """Recursively (post-order) remove Scope or GenericPrim prims with no valid children from stage.

    Args:
        prim: The root prim to start removing empty scopes from.
        stage: The USD stage containing the prims.
    """
    if prim is None or not prim.IsValid():
        return

    for child in prim.GetChildren():
        remove_empty_scopes(child, stage)

    prim_type = prim.GetTypeName() or "GenericPrim"

    if prim_type in ("GenericPrim", "Scope"):
        if not any(child.IsValid() for child in prim.GetChildren()):
            stage.RemovePrim(prim.GetPath())


def add_behavior_script(prim: Usd.Prim, script_path: str, allow_duplicates: bool = False) -> None:
    """Add a behavior script to the prim avoiding duplicates by default.

    Args:
        prim: The USD prim to add the script to.
        script_path: The path to the behavior script.
        allow_duplicates: Whether to allow duplicate scripts.
    """
    if prim is None or not prim.IsValid():
        carb.log_warn(f"Prim is not valid, cannot add behavior script.")
        return

    # Ensure the scripting API is applied to the prim
    scripts_attr = prim.GetAttribute("omni:scripting:scripts")
    if not scripts_attr:
        print(f"Applying scripting API to prim: {prim.GetPath()}")
        omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[prim.GetPath()])
        scripts_attr = prim.GetAttribute("omni:scripting:scripts")
        if not scripts_attr:
            print(f"Failed to create scripting attribute on prim: {prim.GetPath()}, cannot add behavior script.")
            return

    # Convert paths from immutable AssetPathArray to list of strings
    current_scripts = [asset.path for asset in scripts_attr.Get() or []]
    if allow_duplicates or script_path not in current_scripts:
        current_scripts.append(script_path)
        scripts_attr.Set(Sdf.AssetPathArray(current_scripts))


async def add_behavior_script_with_parameters_async(
    prim: Usd.Prim, script_path: str, exposed_variables: dict | None = None, allow_duplicates: bool = False
) -> None:
    """Add a behavior script to a prim and set its parameters.

    Args:
        prim: The USD prim to add the script to.
        script_path: The path to the behavior script.
        exposed_variables: Dictionary of exposed variables to set.
        allow_duplicates: Whether to allow duplicate scripts.
    """
    if prim is None or not prim.IsValid():
        carb.log_warn(f"Prim is not valid, cannot add behavior script.")
        return

    # Add the behavior script to the prim
    add_behavior_script(prim, script_path, allow_duplicates=allow_duplicates)

    # Wait a few frames for the exposed variables to be available
    for _ in range(3):
        await omni.kit.app.get_app().next_update_async()

    # Set the behavior script parameters
    set_exposed_variables(prim, exposed_variables or {})


async def publish_event_and_wait_for_completion_async(
    publish_payload: dict,
    expected_payload: dict,
    publish_event_name: str,
    subscribe_event_name: str,
    max_wait_updates: int,
    verbose: bool = True,
) -> bool:
    """Publish an event and waits for a response that matches the expected payload.

    Args:
        publish_payload: The payload to publish.
        expected_payload: The expected payload to match in the response.
        publish_event_name: The name of the event to publish.
        subscribe_event_name: The name of the event to subscribe to.
        max_wait_updates: Maximum number of update frames to wait.
        verbose: Whether to print verbose output.

    Returns:
        True if the expected payload was received, False on timeout.
    """
    # Keep track of whether the action is complete
    is_action_complete = False

    # Callback to listen to incoming events
    def on_event_received(event: carb.events.IEvent) -> None:
        """Check if the event payload matches the expected key-value pairs.

        Args:
            event: The incoming event to check.
        """
        if all(item in event.payload.items() for item in expected_payload.items()):
            if verbose:
                print(f"\tAction completed, received payload: {event.payload}")
            nonlocal is_action_complete
            is_action_complete = True
        else:
            if verbose:
                print(f"\tAction not complete, received payload: {event.payload}, continuing..")

    # Start listening to incoming events
    message_bus = carb.eventdispatcher.get_eventdispatcher()
    if verbose:
        print(f"Subscribing to {subscribe_event_name}")
    sub_pop = message_bus.observe_event(
        event_name=subscribe_event_name, on_event=on_event_received, observer_name="BehaviorUtils._subscribe_event"
    )

    try:
        # Publish the payload
        if verbose:
            print(f"Publishing payload: {publish_payload}")
            print(f"Waiting for response: {expected_payload}")
        message_bus.dispatch_event(event_name=publish_event_name, payload=publish_payload)

        # Wait for action completion
        for i in range(max_wait_updates):
            if is_action_complete:
                if verbose:
                    print(f"\t\tExpected payload received after {i} updates.")
                break
            await omni.kit.app.get_app().next_update_async()
        else:
            if verbose:
                print(f"\t\tTimeout after {max_wait_updates} updates.")
    finally:
        # Unsubscribe from events
        if verbose:
            print(f"Unsubscribing from {subscribe_event_name}")
        sub_pop.reset()
        sub_pop = None

    return is_action_complete

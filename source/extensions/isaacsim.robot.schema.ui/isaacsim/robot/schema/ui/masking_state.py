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

"""State management for robot masking (deactivation of joints and links)."""

from __future__ import annotations

import traceback
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

import carb
import usd.schema.isaac.robot_schema as robot_schema
from pxr import UsdPhysics

from .utils import PathMap

if TYPE_CHECKING:
    from pxr import Usd


def is_maskable_type(prim: Usd.Prim) -> bool:
    """Check if a prim is maskable (can be deactivated).

    Args:
        prim: The prim to check.

    Returns:
        True if the prim is a link or joint that can be masked.
    """
    schemas = prim.GetAppliedSchemas()
    return robot_schema.Classes.JOINT_API.value in schemas or robot_schema.Classes.LINK_API.value in schemas


def get_selected_maskable_paths() -> list[str]:
    """Return the original stage prim paths of currently selected maskable prims.

    Returns:
        List of path strings for selected prims that are joints or links.
    """
    import omni.usd

    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    if not stage:
        return []
    selection = usd_context.get_selection()
    if not selection:
        return []
    return [
        path_str for path_str in selection.get_selected_prim_paths() if is_maskable_type(stage.GetPrimAtPath(path_str))
    ]


def is_joint_type(prim: Usd.Prim) -> bool:
    """Check if a prim is a joint type.

    Args:
        prim: The prim to check.

    Returns:
        True if the prim is a physics joint.
    """
    schemas = prim.GetAppliedSchemas()
    return robot_schema.Classes.JOINT_API.value in schemas


def is_link_type(prim: Usd.Prim) -> bool:
    """Check if a prim is a robot link (not a joint).

    Args:
        prim: The prim to check.

    Returns:
        True if the prim has the link API applied.
    """
    schemas = prim.GetAppliedSchemas()
    return robot_schema.Classes.LINK_API.value in schemas


def is_anchorable_link(prim: Usd.Prim) -> bool:
    """Check if a prim can be anchored to the world (link with RigidBodyAPI).

    Args:
        prim: The prim to check.

    Returns:
        True if the prim has both the link API and RigidBodyAPI applied.
    """
    return is_link_type(prim) and prim.HasAPI(UsdPhysics.RigidBodyAPI)


def get_selected_link_paths() -> list[str]:
    """Return the original stage prim paths of currently selected link prims.

    Returns:
        List of path strings for selected prims that are links.
    """
    import omni.usd

    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    if not stage:
        return []
    selection = usd_context.get_selection()
    if not selection:
        return []
    return [path_str for path_str in selection.get_selected_prim_paths() if is_link_type(stage.GetPrimAtPath(path_str))]


def get_selected_anchorable_link_paths() -> list[str]:
    """Return the original stage prim paths of currently selected anchorable link prims.

    Returns:
        List of path strings for selected prims that are links with RigidBodyAPI.
    """
    import omni.usd

    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    if not stage:
        return []
    selection = usd_context.get_selection()
    if not selection:
        return []
    return [
        path_str
        for path_str in selection.get_selected_prim_paths()
        if is_anchorable_link(stage.GetPrimAtPath(path_str))
    ]


class MaskingState:
    """Singleton tracking which prims are deactivated (masked/bypassed).

    Tracks two distinct states:

    * **masked** -- the element is disabled for simulation.
    * **bypassed** -- the element is disabled AND the kinematic chain is
      reconnected around it.

    Notifies subscribers when any state changes.
    """

    _instance: MaskingState | None = None

    @classmethod
    def get_instance(cls) -> MaskingState:
        """Return the singleton `MaskingState` instance, creating it if needed.

        Returns:
            The shared ``MaskingState`` instance.
        """
        if cls._instance is None:
            cls._instance = MaskingState()
        return cls._instance

    def __init__(self) -> None:
        self._deactivated_paths: set[str] = set()
        self._bypassed_paths: set[str] = set()
        self._anchored_paths: set[str] = set()
        # Maps bypassed-link path → (backward_joint_path, prev_state).
        # prev_state is "enabled", "masked", or "bypassed".
        # Used to lock the joint while the forward link is bypassed and to
        # restore its UI representation correctly on unbypass.
        self._bypass_deactivated_joints: dict[str, tuple[str, str]] = {}
        self._path_map: PathMap | None = None
        self._operations: Any = None
        self._on_changed_callbacks: list[Callable[[], None]] = []

    # -- properties -------------------------------------------------------

    @property
    def path_map(self) -> PathMap | None:
        """Path mapping between hierarchy and original stage; used for lookups."""
        return self._path_map

    @path_map.setter
    def path_map(self, value: PathMap | None) -> None:
        """Set the path mapping between hierarchy and original stage.

        Args:
            value: New ``PathMap`` instance, or ``None`` to clear.
        """
        self._path_map = value

    @property
    def operations(self) -> Any:
        """Masking operations backend (e.g. `MaskingOperations`); None if not set."""
        return self._operations

    @operations.setter
    def operations(self, value: Any) -> None:
        """Set the masking operations backend.

        Args:
            value: ``MaskingOperations`` instance, or ``None`` to clear.
        """
        self._operations = value

    # -- queries ----------------------------------------------------------

    def is_deactivated(self, original_path: str) -> bool:
        """Return True if the prim is masked or bypassed.

        Args:
            original_path: Original stage prim path string.

        Returns:
            ``True`` if the path is in the deactivated set.
        """
        return original_path in self._deactivated_paths

    def is_bypassed(self, original_path: str) -> bool:
        """Return True only if the prim is in the bypassed state.

        Args:
            original_path: Original stage prim path string.

        Returns:
            ``True`` if the path is in the bypassed set.
        """
        return original_path in self._bypassed_paths

    def is_bypass_controlled(self, original_path: str) -> bool:
        """Return True if ``original_path`` is a backward joint locked by an active link bypass.

        While locked, the joint's bypass and deactivation state must not be
        changed through the UI; the owning link bypass owns its state.

        Args:
            original_path: Original stage prim path string.

        Returns:
            ``True`` if the joint is currently locked by a link bypass.
        """
        return any(joint_path == original_path for joint_path, _ in self._bypass_deactivated_joints.values())

    def get_masking_layer_id(self) -> str | None:
        """Return the masking sublayer identifier, or None if absent.

        Returns:
            Layer identifier string, or ``None`` if no masking layer exists.
        """
        if self._operations:
            return self._operations.get_masking_layer_id()
        return None

    def get_deactivated_paths(self) -> set[str]:
        """Return a copy of the set of deactivated (masked or bypassed) prim paths.

        Returns:
            Snapshot of the deactivated path set.
        """
        return set(self._deactivated_paths)

    def get_bypassed_paths(self) -> set[str]:
        """Return a copy of the set of bypassed prim paths.

        Returns:
            Snapshot of the bypassed path set.
        """
        return set(self._bypassed_paths)

    # -- mask toggle ------------------------------------------------------

    def toggle_deactivated(self, original_path: str) -> bool:
        """Toggle plain mask; unmasking a bypassed prim unbypasses it first.

        Args:
            original_path: Original stage prim path string.

        Returns:
            False if the prim is a backward joint locked by an active link
            bypass (no change); True if now deactivated, False if now active
            (either because the prim was already masked and was un-masked, or
            because the underlying USD mask operation was rejected by the
            backend — in the latter case the in-memory set is NOT mutated).
        """
        if self.is_bypass_controlled(original_path):
            return False
        if original_path in self._deactivated_paths:
            if original_path in self._bypassed_paths:
                self._do_unbypass(original_path)
                self._discard_bypass_joint(original_path)
            else:
                self._do_unmask(original_path)
            self._deactivated_paths.discard(original_path)
            self._bypassed_paths.discard(original_path)
            result = False
        else:
            if self._do_mask(original_path):
                self._deactivated_paths.add(original_path)
                result = True
            else:
                result = False
        self._notify_changed()
        return result

    def _set_deactivated_impl(self, original_path: str, deactivated: bool) -> None:
        """Apply deactivation state without notifying subscribers.

        Skips bypass-controlled joints (those locked by an active link bypass).
        The in-memory deactivated set is only mutated to record activation when
        the backend mask operation reports success.

        Args:
            original_path: Original stage prim path string.
            deactivated: True to deactivate, False to reactivate.
        """
        if self.is_bypass_controlled(original_path):
            return
        if deactivated:
            if original_path not in self._deactivated_paths:
                if self._do_mask(original_path):
                    self._deactivated_paths.add(original_path)
        else:
            if original_path in self._deactivated_paths:
                if original_path in self._bypassed_paths:
                    self._do_unbypass(original_path)
                    self._discard_bypass_joint(original_path)
                else:
                    self._do_unmask(original_path)
                self._deactivated_paths.discard(original_path)
                self._bypassed_paths.discard(original_path)

    def set_deactivated(self, original_path: str, deactivated: bool) -> None:
        """Set deactivation state for a single prim and notify subscribers.

        Args:
            original_path: Original stage prim path string.
            deactivated: True to deactivate, False to reactivate.
        """
        self._set_deactivated_impl(original_path, deactivated)
        self._notify_changed()

    def set_deactivated_batch(self, paths: list[str], deactivated: bool) -> None:
        """Set deactivation state for multiple prims with a single change notification.

        Args:
            paths: Original stage prim path strings to update.
            deactivated: True to deactivate, False to reactivate.
        """
        for path in paths:
            self._set_deactivated_impl(path, deactivated)
        self._notify_changed()

    # -- bypass toggle ----------------------------------------------------

    def toggle_bypassed(self, original_path: str) -> bool:
        """Toggle bypass state.

        Args:
            original_path: Original stage prim path string.

        Returns:
            False if the prim is a backward joint locked by an active link
            bypass (no change); True if now bypassed, False if now unbypassed.
            The returned value tracks the actual masking state, never an
            aspirational one:

            * If the backend rejects ``bypass_prim``, the in-memory sets are
              left untouched and the call returns False (still unbypassed).
            * If the backend rejects ``unbypass_prim``, the in-memory sets
              are left untouched and the call returns True (still bypassed).
        """
        if self.is_bypass_controlled(original_path):
            return False
        if original_path in self._bypassed_paths:
            success, _ = self._do_unbypass(original_path)
            if success:
                self._discard_bypass_joint(original_path)
                self._deactivated_paths.discard(original_path)
                self._bypassed_paths.discard(original_path)
                result = False
            else:
                # Unbypass rejected by backend; USD opinions are unchanged so
                # leave the in-memory sets unchanged too -- otherwise we would
                # re-introduce the same in-memory/USD divergence on the
                # opposite direction that this fix set out to remove.
                result = True
        else:
            # If already masked, unmask first so bypass starts clean
            was_masked = original_path in self._deactivated_paths
            if was_masked:
                self._do_unmask(original_path)
                self._deactivated_paths.discard(original_path)
            success, joint_info = self._do_bypass(original_path)
            if success:
                self._deactivated_paths.add(original_path)
                self._bypassed_paths.add(original_path)
                if joint_info:
                    joint_path, prev_state = joint_info
                    self._bypass_deactivated_joints[original_path] = (joint_path, prev_state)
                    if prev_state == "enabled":
                        # Only newly deactivated joints need to be added
                        self._deactivated_paths.add(joint_path)
                    # "masked" and "bypassed": joint already in deactivated/bypassed paths
                result = True
            else:
                # Bypass rejected by backend; do NOT pollute the in-memory sets.
                # If we pre-emptively unmasked above, restore the prior mask so
                # the user-visible state matches the pre-toggle situation.
                if was_masked and self._do_mask(original_path):
                    self._deactivated_paths.add(original_path)
                result = False
        self._notify_changed()
        return result

    def _set_bypassed_impl(self, original_path: str, bypassed: bool) -> None:
        """Apply bypass state without notifying subscribers.

        Skips bypass-controlled joints (those locked by an active link bypass).
        The in-memory bypass set is only mutated to record activation when the
        backend bypass operation reports success.

        Args:
            original_path: Original stage prim path string.
            bypassed: True to bypass, False to unbypass.
        """
        if self.is_bypass_controlled(original_path):
            return
        if bypassed:
            if original_path not in self._bypassed_paths:
                was_masked = original_path in self._deactivated_paths
                if was_masked:
                    self._do_unmask(original_path)
                    self._deactivated_paths.discard(original_path)
                success, joint_info = self._do_bypass(original_path)
                if success:
                    self._deactivated_paths.add(original_path)
                    self._bypassed_paths.add(original_path)
                    if joint_info:
                        joint_path, prev_state = joint_info
                        self._bypass_deactivated_joints[original_path] = (joint_path, prev_state)
                        if prev_state == "enabled":
                            self._deactivated_paths.add(joint_path)
                else:
                    # Restore prior mask on failure so the pre-call state is
                    # preserved (matches toggle_bypassed semantics).
                    if was_masked and self._do_mask(original_path):
                        self._deactivated_paths.add(original_path)
        else:
            if original_path in self._bypassed_paths:
                success, _ = self._do_unbypass(original_path)
                if success:
                    self._discard_bypass_joint(original_path)
                    self._deactivated_paths.discard(original_path)
                    self._bypassed_paths.discard(original_path)
                # On failure leave the in-memory sets unchanged so they keep
                # tracking the actual USD state (mirrors ``toggle_bypassed``).

    def set_bypassed(self, original_path: str, bypassed: bool) -> None:
        """Set bypass state explicitly for a single prim.

        Args:
            original_path: Original stage prim path string.
            bypassed: True to bypass, False to unbypass.
        """
        self._set_bypassed_impl(original_path, bypassed)
        self._notify_changed()

    def set_bypassed_batch(self, paths: list[str], bypassed: bool) -> None:
        """Set bypass state for multiple prims with a single change notification.

        Args:
            paths: Original stage prim path strings to update.
            bypassed: True to bypass, False to unbypass.
        """
        for path in paths:
            self._set_bypassed_impl(path, bypassed)
        self._notify_changed()

    # -- anchor toggle ----------------------------------------------------

    def is_anchored(self, original_path: str) -> bool:
        """Return True if the link is currently anchored to the world.

        Args:
            original_path: Original stage prim path string.

        Returns:
            ``True`` if the path is in the anchored set.
        """
        return original_path in self._anchored_paths

    def get_anchored_paths(self) -> set[str]:
        """Return a copy of the set of anchored link paths.

        Returns:
            Snapshot of the anchored path set.
        """
        return set(self._anchored_paths)

    def toggle_anchored(self, original_path: str) -> bool:
        """Toggle anchor state on a link.

        Args:
            original_path: Original stage prim path string.

        Returns:
            True if now anchored, False if now unanchored. Also returns False
            — leaving in-memory state untouched — when the backend rejects the
            anchor operation (e.g. the prim is not a link or has no
            ``RigidBodyAPI``). The returned value tracks the actual anchor
            state, never an aspirational one.
        """
        if original_path in self._anchored_paths:
            self._do_unanchor(original_path)
            self._anchored_paths.discard(original_path)
            result = False
        else:
            if self._do_anchor(original_path):
                self._anchored_paths.add(original_path)
                result = True
            else:
                result = False
        self._notify_changed()
        return result

    def _set_anchored_impl(self, original_path: str, anchored: bool) -> None:
        if anchored:
            if original_path not in self._anchored_paths:
                if self._do_anchor(original_path):
                    self._anchored_paths.add(original_path)
        else:
            if original_path in self._anchored_paths:
                self._do_unanchor(original_path)
                self._anchored_paths.discard(original_path)

    def set_anchored(self, original_path: str, anchored: bool) -> None:
        """Set anchor state for a single link and notify subscribers.

        Args:
            original_path: Original stage prim path string.
            anchored: True to anchor, False to unanchor.
        """
        self._set_anchored_impl(original_path, anchored)
        self._notify_changed()

    def set_anchored_batch(self, paths: list[str], anchored: bool) -> None:
        """Set anchor state for multiple links with a single change notification.

        Args:
            paths: Original stage prim path strings to update.
            anchored: True to anchor, False to unanchor.
        """
        for path in paths:
            self._set_anchored_impl(path, anchored)
        self._notify_changed()

    # -- clear ------------------------------------------------------------

    def clear(self) -> None:
        """Clear all masking, bypass, and anchor state and notify subscribers."""
        self._deactivated_paths.clear()
        self._bypassed_paths.clear()
        self._anchored_paths.clear()
        self._bypass_deactivated_joints.clear()
        self._notify_changed()

    # -- subscriptions ----------------------------------------------------

    def subscribe_changed(self, callback: Callable[[], None]) -> None:
        """Register a callback to be invoked when any masking state changes.

        Args:
            callback: No-argument callable invoked on change.
        """
        self._on_changed_callbacks.append(callback)

    def unsubscribe_changed(self, callback: Callable[[], None]) -> None:
        """Unregister a previously registered change callback.

        Args:
            callback: Callable to remove.
        """
        if callback in self._on_changed_callbacks:
            self._on_changed_callbacks.remove(callback)

    # -- internal ---------------------------------------------------------

    def _do_mask(self, path: str) -> bool:
        """Call mask operation; returns True iff the backend reported success.

        When no operations backend is configured, returns ``True`` so the
        state machine can be exercised standalone (e.g. in unit tests).
        """
        if self._operations:
            try:
                return bool(self._operations.mask_prim(path))
            except Exception as exc:
                carb.log_error(f"Failed to mask prim at {path}: {exc}\n{traceback.format_exc()}")
                return False
        return True

    def _do_unmask(self, path: str) -> bool:
        """Call unmask operation; returns True iff the backend reported success.

        When no operations backend is configured, returns ``True``.
        """
        if self._operations:
            try:
                return bool(self._operations.unmask_prim(path))
            except Exception as exc:
                carb.log_error(f"Failed to unmask prim at {path}: {exc}\n{traceback.format_exc()}")
                return False
        return True

    def _do_bypass(self, path: str) -> tuple[bool, tuple[str, str] | None]:
        """Call bypass operation.

        Args:
            path: Original stage prim path string.

        Returns:
            ``(success, joint_info)`` where ``success`` is True iff USD masking
            opinions were written, and ``joint_info`` is
            ``(backward_joint_path, prev_state)`` for a link bypass that
            disabled a backward joint or ``None`` otherwise. When no operations
            backend is configured, returns ``(True, None)``.
        """
        if self._operations:
            try:
                result = self._operations.bypass_prim(path)
            except Exception as exc:
                carb.log_error(f"Failed to bypass prim at {path}: {exc}\n{traceback.format_exc()}")
                return (False, None)
            return self._normalize_bypass_result(result)
        return (True, None)

    def _do_unbypass(self, path: str) -> tuple[bool, tuple[str, str] | None]:
        """Call unbypass operation.

        Args:
            path: Original stage prim path string.

        Returns:
            ``(success, joint_info)`` where ``joint_info`` is
            ``(backward_joint_path, prev_state)`` for a link unbypass or
            ``None``. When no operations backend is configured, returns
            ``(True, None)``.
        """
        if self._operations:
            try:
                result = self._operations.unbypass_prim(path)
            except Exception as exc:
                carb.log_error(f"Failed to unbypass prim at {path}: {exc}\n{traceback.format_exc()}")
                return (False, None)
            return self._normalize_bypass_result(result)
        return (True, None)

    def _do_anchor(self, path: str) -> bool:
        """Call anchor operation; returns True iff the backend reported success.

        When no operations backend is configured, returns ``True``.
        """
        if self._operations:
            try:
                return bool(self._operations.anchor_link(path))
            except Exception as exc:
                carb.log_error(f"Failed to anchor link at {path}: {exc}\n{traceback.format_exc()}")
                return False
        return True

    def _do_unanchor(self, path: str) -> bool:
        """Call unanchor operation; returns True iff the backend reported success.

        When no operations backend is configured, returns ``True``.
        """
        if self._operations:
            try:
                return bool(self._operations.unanchor_link(path))
            except Exception as exc:
                carb.log_error(f"Failed to unanchor link at {path}: {exc}\n{traceback.format_exc()}")
                return False
        return True

    @staticmethod
    def _normalize_bypass_result(result: Any) -> tuple[bool, tuple[str, str] | None]:
        """Normalize a backend bypass/unbypass return value.

        Backends MUST return ``(success: bool, joint_info: tuple[str, str] | None)``.
        Any other shape (including a bare ``None`` or a bare joint tuple) is
        treated as failure and produces a warning; the previous "bare ``None``
        implies success" contract was the source of the silent-success defect.

        Args:
            result: Raw value from ``bypass_prim``/``unbypass_prim``.

        Returns:
            ``(success, joint_info)`` with ``success`` being a plain bool and
            ``joint_info`` being either ``None`` or a 2-tuple of strings.
        """
        if isinstance(result, tuple) and len(result) == 2 and isinstance(result[0], bool):
            success, joint_info = result
            if joint_info is None:
                return (bool(success), None)
            if isinstance(joint_info, tuple) and len(joint_info) == 2:
                return (bool(success), (str(joint_info[0]), str(joint_info[1])))
            return (bool(success), None)
        carb.log_warn(
            f"masking_ops backend returned unsupported value {result!r}; treating as failure. "
            "Backends must return (success: bool, joint_info: tuple[str, str] | None)."
        )
        return (False, None)

    def _discard_bypass_joint(self, link_path: str) -> None:
        """Restore backward joint UI state when a link is unbypassed.

        Args:
            link_path: Original stage path string of the link being unbypassed.
        """
        entry = self._bypass_deactivated_joints.pop(link_path, None)
        if entry:
            joint_path, prev_state = entry
            if prev_state == "enabled":
                # We added this joint; remove it from deactivated
                self._deactivated_paths.discard(joint_path)
            # "masked" and "bypassed": joint stays in its pre-bypass tracking sets

    def _notify_changed(self) -> None:
        for callback in self._on_changed_callbacks:
            try:
                callback()
            except Exception:
                pass

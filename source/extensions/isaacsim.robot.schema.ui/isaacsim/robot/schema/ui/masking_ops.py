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

"""USD operations for robot masking and bypassing.

All opinions are written to a dedicated anonymous in-memory sublayer
inserted at the top of the session layer's sublayer stack.

**Mask** disables an element for simulation without reconnecting the chain.
**Bypass** disables an element AND reconnects the chain around it.

Joint bypass:  create fixed joints between the nearest backward non-masked
link and each forward non-masked link.

Link bypass:   deactivate the backward joint, reparent forward joints to
the nearest backward non-masked link with recalculated offsets.

Created prims (fixed joints) live under ``/__masked__/`` with
``hide_in_stage_window`` metadata.
"""

from __future__ import annotations

import carb
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

from .masking_state import is_joint_type, is_link_type, is_maskable_type

# Custom-data keys
MASKED_KEY = "isaacsim:masked"
BYPASSED_KEY = "isaacsim:bypassed"
ANCHORED_KEY = "isaacsim:anchored"
ANCHOR_FIXED_JOINT_KEY = "isaacsim:anchor_fixed_joint"
NON_INSTANCEABLE_KEY = "isaacsim:made_non_instanceable"
BYPASS_FIXED_JOINTS_KEY = "isaacsim:bypass_fixed_joints"
BYPASS_DEACTIVATED_JOINT_KEY = "isaacsim:bypass_deactivated_joint"
BYPASS_REPARENTED_JOINTS_KEY = "isaacsim:bypass_reparented_joints"
BYPASS_BACKWARD_LINK_KEY = "isaacsim:bypass_backward_link"
# State of the backward joint *before* a link bypass was applied.
# Values: "enabled" | "masked" | "bypassed"
BYPASS_JOINT_PREV_STATE_KEY = "isaacsim:bypass_joint_prev_state"
# Fixed joints (from a prior joint bypass) that were disabled during link bypass.
BYPASS_AUX_FIXED_JOINTS_KEY = "isaacsim:bypass_aux_fixed_joints"

# Scope for prims created by bypass operations
MASKED_SCOPE = "__masked__"

_MASKING_LAYER_DISPLAY_NAME = "Robot Masking"


# ======================================================================
# Masking Layer
# ======================================================================


class MaskingLayer:
    """Anonymous sublayer inserted into the session layer stack.

    Args:
        usd_context_name: USD context name; empty string uses the default context.
    """

    def __init__(self, usd_context_name: str = "") -> None:
        self._usd_context_name = usd_context_name
        self._layer: Sdf.Layer | None = None

    def acquire(self) -> Sdf.Layer | None:
        """Create or return the masking layer; insert into session sublayers if new.

        Returns:
            The masking layer, or None if the session layer is unavailable.
        """
        if self._layer and not self._layer.expired:
            return self._layer
        session_layer = self._get_session_layer()
        if not session_layer:
            return None
        layer = Sdf.Layer.CreateAnonymous(_MASKING_LAYER_DISPLAY_NAME + ".usda")
        layer.documentation = _MASKING_LAYER_DISPLAY_NAME
        session_layer.subLayerPaths.insert(0, layer.identifier)
        self._layer = layer
        return layer

    def release(self) -> None:
        """Remove the masking layer from the session and clear the reference."""
        self._remove_layer()

    def destroy(self) -> None:
        """Drop the masking layer from the session; same as `release()`."""
        self.release()

    def get(self) -> Sdf.Layer | None:
        """Return the current masking layer if it exists and is valid.

        Returns:
            The masking layer, or None if not acquired or expired.
        """
        if self._layer and not self._layer.expired:
            return self._layer
        return None

    def is_empty(self) -> bool:
        """Return True if no masking layer exists or it has no prim opinions.

        Returns:
            True if the layer is absent, expired, or has no root children.
        """
        layer = self.get()
        if not layer:
            return True
        root = layer.GetPrimAtPath(Sdf.Path.absoluteRootPath)
        return not (root and root.nameChildren)

    def _get_session_layer(self) -> Sdf.Layer | None:
        """Return the session layer of the current USD stage.

        Returns:
            The session layer, or None if the context or stage is unavailable.
        """
        ctx = omni.usd.get_context(self._usd_context_name)
        if not ctx:
            return None
        stage = ctx.get_stage()
        return stage.GetSessionLayer() if stage else None

    def _remove_layer(self) -> None:
        """Remove the masking sublayer from the session layer's sublayer stack."""
        layer = self._layer
        self._layer = None
        if not layer or layer.expired:
            return
        session_layer = self._get_session_layer()
        if session_layer:
            paths = list(session_layer.subLayerPaths)
            if layer.identifier in paths:
                paths.remove(layer.identifier)
                session_layer.subLayerPaths[:] = paths


# ======================================================================
# Masking Operations
# ======================================================================


class MaskingOperations:
    """Performs mask and bypass USD operations on the masking sublayer.

    Args:
        usd_context_name: USD context name (empty string = default context).
    """

    def __init__(self, usd_context_name: str = "") -> None:
        self._usd_context_name = usd_context_name
        self._masking_layer = MaskingLayer(usd_context_name)

    # ------------------------------------------------------------------
    # Public API -- mask / unmask
    # ------------------------------------------------------------------

    def mask_prim(self, original_path: str) -> bool:
        """Mask (disable) a joint or link for simulation.

        Args:
            original_path: Original stage prim path string.

        Returns:
            True if masking was applied, False if stage/prim/layer unavailable.
        """
        stage = self._get_stage()
        if not stage:
            return False
        prim = stage.GetPrimAtPath(original_path)
        if not prim or not prim.IsValid():
            carb.log_warn(f"Cannot mask: prim not found at {original_path}")
            return False
        masking_layer = self._masking_layer.acquire()
        if not masking_layer:
            return False
        with Usd.EditContext(stage, masking_layer):
            if is_joint_type(prim):
                self._mask_joint(stage, prim)
            elif is_maskable_type(prim):
                self._mask_link(stage, prim)
            else:
                return False
        return True

    def unmask_prim(self, original_path: str) -> bool:
        """Remove mask opinions for the prim, restoring base-layer values.

        Args:
            original_path: Original stage prim path string.

        Returns:
            True on success or if prim/layer already absent.
        """
        stage = self._get_stage()
        if not stage:
            return False
        prim = stage.GetPrimAtPath(original_path)
        if not prim or not prim.IsValid():
            return True
        masking_layer = self._masking_layer.get()
        if not masking_layer:
            return True
        with Usd.EditContext(stage, masking_layer):
            if is_joint_type(prim):
                self._unmask_joint(stage, prim, masking_layer)
            elif is_maskable_type(prim):
                self._unmask_link(stage, prim, masking_layer)
        self._release_if_empty()
        return True

    # ------------------------------------------------------------------
    # Public API -- bypass / unbypass
    # ------------------------------------------------------------------

    def bypass_prim(self, original_path: str) -> tuple[bool, tuple[str, str] | None]:
        """Bypass a joint or link (mask it and reconnect the chain around it).

        Args:
            original_path: Original stage prim path string.

        Returns:
            A tuple ``(success, joint_info)``:

            * ``success`` is True iff USD masking opinions were written; False
              when the stage/prim/layer is unavailable or the prim is not a
              joint or maskable link.
            * ``joint_info`` is ``(backward_joint_path, prev_state)`` for a
              successful link bypass that disabled a backward joint (where
              ``prev_state`` is ``"enabled"``, ``"masked"``, or ``"bypassed"``),
              or ``None`` for joint bypass, link bypass with no backward joint,
              or any failure.

            ``success`` MUST be honored by the caller before recording
            in-memory state — silent "success without USD effect" is a bug.
        """
        stage = self._get_stage()
        if not stage:
            return (False, None)
        prim = stage.GetPrimAtPath(original_path)
        if not prim or not prim.IsValid():
            carb.log_warn(f"Cannot bypass: prim not found at {original_path}")
            return (False, None)
        masking_layer = self._masking_layer.acquire()
        if not masking_layer:
            return (False, None)
        with Usd.EditContext(stage, masking_layer):
            if is_joint_type(prim):
                self._bypass_joint(stage, prim)
                return (True, None)
            elif is_maskable_type(prim):
                return (True, self._bypass_link(stage, prim))
            else:
                carb.log_warn(f"Cannot bypass: unsupported type at {original_path}")
                return (False, None)

    def unbypass_prim(self, original_path: str) -> tuple[bool, tuple[str, str] | None]:
        """Remove bypass (unmask the element and undo chain reconnection).

        Args:
            original_path: Original stage prim path string.

        Returns:
            A tuple ``(success, joint_info)``:

            * ``success`` is True when the unbypass either ran USD cleanup or
              had nothing to clean (no masking layer, no prim spec); False
              only when the stage or live prim is unavailable.
            * ``joint_info`` is ``(backward_joint_path, prev_state)`` for link
              unbypass that restored a backward joint, otherwise ``None``.

            A True ``success`` with ``joint_info is None`` is the normal case
            for joint unbypass; it is distinguishable from failure by
            ``success`` being True.

        Note on asymmetry with :meth:`bypass_prim`:
            ``bypass_prim`` uses ``MaskingLayer.acquire()`` (a write attempt
            that creates the sublayer on demand); if acquisition fails, the
            requested USD write is impossible and the result is ``(False, None)``.
            ``unbypass_prim`` uses ``MaskingLayer.get()`` (a read-only lookup);
            a missing layer means there is no bypass on disk to undo, which
            is the desired post-condition and is reported as
            ``(True, None)``. The two ``None`` returns therefore have
            deliberately different semantics.
        """
        stage = self._get_stage()
        if not stage:
            return (False, None)
        prim = stage.GetPrimAtPath(original_path)
        if not prim or not prim.IsValid():
            return (False, None)
        masking_layer = self._masking_layer.get()
        if not masking_layer:
            # Nothing to undo at the USD layer; treat as success so the caller
            # can still drop its in-memory tracking.
            return (True, None)
        result: tuple[str, str] | None = None
        with Usd.EditContext(stage, masking_layer):
            if is_joint_type(prim):
                self._unbypass_joint(stage, prim, masking_layer)
            elif is_maskable_type(prim):
                result = self._unbypass_link(stage, prim, masking_layer)
        self._release_if_empty()
        return (True, result)

    def get_masking_layer_id(self) -> str | None:
        """Return the identifier of the masking sublayer, or None if it does not exist.

        Returns:
            Layer identifier string, or ``None`` if no masking layer exists.
        """
        layer = self._masking_layer.get()
        return layer.identifier if layer else None

    def clear_all(self) -> None:
        """Drop the masking layer entirely, reverting everything at once."""
        self._masking_layer.destroy()

    # ------------------------------------------------------------------
    # Public API -- anchor / unanchor
    # ------------------------------------------------------------------

    def anchor_link(self, original_path: str) -> bool:
        """Anchor a link to the world by creating a fixed joint with no body0.

        Args:
            original_path: Original stage prim path of the link.

        Returns:
            True if anchoring was applied, False if stage/prim/layer unavailable.
        """
        stage = self._get_stage()
        if not stage:
            return False
        prim = stage.GetPrimAtPath(original_path)
        if not prim or not prim.IsValid():
            carb.log_warn(f"Cannot anchor: prim not found at {original_path}")
            return False
        if not is_link_type(prim):
            carb.log_warn(f"Cannot anchor: prim at {original_path} is not a link")
            return False
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            carb.log_warn(f"Cannot anchor: prim at {original_path} has no RigidBodyAPI")
            return False
        masking_layer = self._masking_layer.acquire()
        if not masking_layer:
            return False
        with Usd.EditContext(stage, masking_layer):
            self._anchor_link(stage, prim)
        return True

    def unanchor_link(self, original_path: str) -> bool:
        """Remove the world-anchor fixed joint from a link.

        Args:
            original_path: Original stage prim path of the link.

        Returns:
            True on success or if prim/layer already absent.
        """
        stage = self._get_stage()
        if not stage:
            return False
        prim = stage.GetPrimAtPath(original_path)
        if not prim or not prim.IsValid():
            return True
        masking_layer = self._masking_layer.get()
        if not masking_layer:
            return True
        with Usd.EditContext(stage, masking_layer):
            self._unanchor_link(stage, prim, masking_layer)
        self._release_if_empty()
        return True

    # ==================================================================
    # Link anchor / unanchor
    # ==================================================================

    def _anchor_link(self, stage: Usd.Stage, link_prim: Usd.Prim) -> None:
        """Create a fixed joint pinning *link_prim* to the world at its current pose.

        Args:
            stage: USD stage containing the prim.
            link_prim: The link prim to anchor.
        """
        link_path = link_prim.GetPath()

        scope_path = Sdf.Path(f"/{MASKED_SCOPE}")
        if not stage.GetPrimAtPath(scope_path):
            scope_prim = stage.DefinePrim(scope_path, "Scope")
            scope_prim.SetMetadata("hide_in_stage_window", True)

        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        world_xform = xc.GetLocalToWorldTransform(link_prim)
        world_pos, world_rot = self._decompose_local_frame(world_xform)

        artifact_name = "anchor__" + str(link_path).lstrip("/").replace("/", "__")
        fixed_path = scope_path.AppendChild(artifact_name)

        fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_path)
        fixed_joint.CreateBody1Rel().SetTargets([link_path])
        fixed_joint.CreateLocalPos0Attr().Set(world_pos)
        fixed_joint.CreateLocalRot0Attr().Set(world_rot)
        fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        link_prim.SetCustomDataByKey(ANCHORED_KEY, True)
        link_prim.SetCustomDataByKey(ANCHOR_FIXED_JOINT_KEY, str(fixed_path))

    def _unanchor_link(self, stage: Usd.Stage, link_prim: Usd.Prim, layer: Sdf.Layer) -> None:
        """Remove the anchor fixed joint and clear metadata.

        Args:
            stage: USD stage containing the prim.
            link_prim: The link prim to unanchor.
            layer: Masking sublayer holding the anchor opinions.
        """
        fixed_path_str = link_prim.GetCustomDataByKey(ANCHOR_FIXED_JOINT_KEY)
        if fixed_path_str:
            self._remove_prim_spec(layer, Sdf.Path(fixed_path_str))

        spec = layer.GetPrimAtPath(link_prim.GetPath())
        if spec:
            cd = dict(spec.customData)
            cd.pop(ANCHORED_KEY, None)
            cd.pop(ANCHOR_FIXED_JOINT_KEY, None)
            spec.customData = cd
            self._remove_inert_spec(layer, link_prim.GetPath())

        self._cleanup_masked_scope(layer)

    # ==================================================================
    # Joint mask / unmask
    # ==================================================================

    def _mask_joint(self, stage: Usd.Stage, joint_prim: Usd.Prim) -> None:
        """Disable the joint and mark it as masked via custom data.

        Args:
            stage: USD stage containing the prim.
            joint_prim: The joint prim to mask.
        """
        joint = UsdPhysics.Joint(joint_prim)
        if not joint:
            return
        joint.GetJointEnabledAttr().Set(False)
        joint_prim.SetCustomDataByKey(MASKED_KEY, True)

    def _unmask_joint(self, stage: Usd.Stage, joint_prim: Usd.Prim, layer: Sdf.Layer) -> None:
        """Remove all masking opinions for a joint, restoring base-layer values.

        Args:
            stage: USD stage containing the prim.
            joint_prim: The joint prim to unmask.
            layer: Masking sublayer holding the joint opinions.
        """
        self._clear_prim_spec_recursive(layer, joint_prim.GetPath())

    # ==================================================================
    # Link mask / unmask
    # ==================================================================

    def _mask_link(self, stage: Usd.Stage, link_prim: Usd.Prim) -> None:
        """Disable the link's rigid body, collisions, and visibility, then mark as masked.

        Args:
            stage: USD stage containing the prim.
            link_prim: The link prim to mask.
        """
        if link_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(link_prim).CreateRigidBodyEnabledAttr().Set(False)
        self._disable_collisions_under(stage, link_prim)
        self._hide_non_rigid_children(link_prim)
        link_prim.SetCustomDataByKey(MASKED_KEY, True)

    def _unmask_link(self, stage: Usd.Stage, link_prim: Usd.Prim, layer: Sdf.Layer) -> None:
        """Remove all masking opinions for a link, restoring base-layer values.

        Args:
            stage: USD stage containing the prim.
            link_prim: The link prim to unmask.
            layer: Masking sublayer holding the link opinions.
        """
        self._clear_prim_spec_recursive(layer, link_prim.GetPath())

    # ==================================================================
    # Joint bypass / unbypass
    # ==================================================================

    def _bypass_joint(self, stage: Usd.Stage, joint_prim: Usd.Prim) -> None:
        """Mask the joint and create fixed joints bridging the gap.

        Args:
            stage: USD stage containing the prim.
            joint_prim: The joint prim to bypass.
        """
        joint_api = UsdPhysics.Joint(joint_prim)
        if not joint_api:
            return

        b0_targets = joint_api.GetBody0Rel().GetTargets() if joint_api.GetBody0Rel() else []
        b1_targets = joint_api.GetBody1Rel().GetTargets() if joint_api.GetBody1Rel() else []
        if not b0_targets or not b1_targets:
            self._mask_joint(stage, joint_prim)
            joint_prim.SetCustomDataByKey(BYPASSED_KEY, True)
            return

        body0_path = b0_targets[0]
        body1_path = b1_targets[0]

        robot_root = self._find_robot_root(stage, joint_prim)
        backward_link = self._find_backward_non_masked(stage, body0_path)
        forward_links = self._find_forward_non_masked(stage, body1_path, robot_root)

        # Mask the joint itself
        joint_api.GetJointEnabledAttr().Set(False)
        joint_prim.SetCustomDataByKey(MASKED_KEY, True)
        joint_prim.SetCustomDataByKey(BYPASSED_KEY, True)

        # Create fixed joints connecting backward_link → each forward_link
        created_paths: list[str] = []
        if backward_link and forward_links:
            for fwd_path in forward_links:
                fixed_path = self._create_fixed_joint(stage, joint_prim, backward_link, fwd_path)
                if fixed_path:
                    created_paths.append(str(fixed_path))

        if created_paths:
            joint_prim.SetCustomDataByKey(BYPASS_FIXED_JOINTS_KEY, Vt.StringArray(created_paths))

    def _unbypass_joint(self, stage: Usd.Stage, joint_prim: Usd.Prim, layer: Sdf.Layer) -> None:
        """Remove fixed joints and re-enable the original joint.

        Args:
            stage: USD stage containing the prim.
            joint_prim: The joint prim to unbypass.
            layer: Masking sublayer holding the bypass opinions.
        """
        fixed_paths = joint_prim.GetCustomDataByKey(BYPASS_FIXED_JOINTS_KEY)
        if fixed_paths:
            for fp_str in fixed_paths:
                self._remove_prim_spec(layer, Sdf.Path(fp_str))
        self._clear_prim_spec_recursive(layer, joint_prim.GetPath())
        self._cleanup_masked_scope(layer)

    # ==================================================================
    # Link bypass / unbypass
    # ==================================================================

    def _bypass_link(self, stage: Usd.Stage, link_prim: Usd.Prim) -> tuple[str, str] | None:
        """Mask the link, handle its backward joint, and reparent forward joints.

        Saves the backward joint's previous state and acts accordingly:

        * **"enabled"** -- disables the joint (standard behaviour).
        * **"masked"**  -- joint is already disabled; leave it alone and skip
          reparenting (no active backward connection).
        * **"bypassed"** -- joint is already disabled; disables the auxiliary
          fixed joints created by that bypass (they now connect to a masked
          link and are redundant).

        Args:
            stage: USD stage containing the prim.
            link_prim: The link prim to bypass.

        Returns:
            ``(backward_joint_path, prev_state)`` or ``None``.
        """
        link_path = link_prim.GetPath()
        robot_root = self._find_robot_root(stage, link_prim)

        # 1. Mask the link (rigid body, collisions, visibility)
        self._mask_link(stage, link_prim)
        link_prim.SetCustomDataByKey(BYPASSED_KEY, True)

        # 2. Find the *original* backward joint regardless of enabled state
        backward_joint = self._find_original_joint_to_link(stage, link_path, robot_root, body_index=1)
        backward_link_path: Sdf.Path | None = None
        result: tuple[str, str] | None = None

        if backward_joint:
            bj_api = UsdPhysics.Joint(backward_joint)
            b0 = bj_api.GetBody0Rel().GetTargets() if bj_api.GetBody0Rel() else []
            parent_path = b0[0] if b0 else None
            joint_path_str = str(backward_joint.GetPath())

            is_joint_bypassed = bool(backward_joint.GetCustomDataByKey(BYPASSED_KEY))
            is_joint_masked = bool(backward_joint.GetCustomDataByKey(MASKED_KEY))

            if is_joint_bypassed:
                prev_state = "bypassed"
                # Backward_link_path from the original joint's body0
                backward_link_path = self._find_backward_non_masked(stage, parent_path) if parent_path else None
                # Disable the auxiliary fixed joints – they now point at a masked link
                fixed_paths = backward_joint.GetCustomDataByKey(BYPASS_FIXED_JOINTS_KEY)
                if fixed_paths:
                    aux_disabled: list[str] = []
                    for fp_str in fixed_paths:
                        fp = stage.GetPrimAtPath(fp_str)
                        if fp and fp.IsValid():
                            UsdPhysics.Joint(fp).GetJointEnabledAttr().Set(False)
                            aux_disabled.append(fp_str)
                    if aux_disabled:
                        link_prim.SetCustomDataByKey(BYPASS_AUX_FIXED_JOINTS_KEY, Vt.StringArray(aux_disabled))

            elif is_joint_masked:
                prev_state = "masked"
                # Joint already disabled independently; no reparenting (no active link)

            else:
                prev_state = "enabled"
                backward_link_path = self._find_backward_non_masked(stage, parent_path) if parent_path else None
                bj_api.GetJointEnabledAttr().Set(False)

            link_prim.SetCustomDataByKey(BYPASS_JOINT_PREV_STATE_KEY, prev_state)
            link_prim.SetCustomDataByKey(BYPASS_DEACTIVATED_JOINT_KEY, joint_path_str)
            result = (joint_path_str, prev_state)

        if not backward_link_path:
            return result

        link_prim.SetCustomDataByKey(BYPASS_BACKWARD_LINK_KEY, str(backward_link_path))

        # 3. Find forward joints (body0 = this link) and reparent them
        forward_joints = self._find_forward_joints_from_link(stage, link_path, robot_root)
        reparented: list[str] = []
        for fj_prim in forward_joints:
            self._reparent_joint_body0(stage, fj_prim, link_path, backward_link_path)
            reparented.append(str(fj_prim.GetPath()))

        if reparented:
            link_prim.SetCustomDataByKey(BYPASS_REPARENTED_JOINTS_KEY, Vt.StringArray(reparented))

        return result

    def _unbypass_link(self, stage: Usd.Stage, link_prim: Usd.Prim, layer: Sdf.Layer) -> tuple[str, str] | None:
        """Restore reparented joints, restore backward joint state, then unmask.

        Backward joint restoration depends on the saved prev_state:

        * **"enabled"**  -- removes the ``jointEnabled = False`` opinion so the
          joint returns to its default enabled state.
        * **"masked"**   -- leaves the joint disabled (it was independently masked).
        * **"bypassed"** -- re-enables the auxiliary fixed joints (removes the
          ``jointEnabled = False`` opinions added during bypass); the original
          joint remains bypassed.

        Args:
            stage: USD stage containing the prim.
            link_prim: The link prim to unbypass.
            layer: Masking sublayer holding the bypass opinions.

        Returns:
            ``(backward_joint_path, prev_state)`` or ``None``.
        """
        # 1. Restore reparented joints (clear body0, localPos0, localRot0 opinions)
        reparented = link_prim.GetCustomDataByKey(BYPASS_REPARENTED_JOINTS_KEY)
        if reparented:
            for jp_str in reparented:
                jp = Sdf.Path(jp_str)
                spec = layer.GetPrimAtPath(jp)
                if spec:
                    for prop_name in list(spec.properties.keys()):
                        spec.RemoveProperty(spec.properties[prop_name])
                    self._remove_inert_spec(layer, jp)

        # 2. Restore backward joint — read metadata BEFORE clearing the link spec
        deact_str = link_prim.GetCustomDataByKey(BYPASS_DEACTIVATED_JOINT_KEY)
        prev_state: str = link_prim.GetCustomDataByKey(BYPASS_JOINT_PREV_STATE_KEY) or "enabled"

        if deact_str:
            if prev_state == "enabled":
                # Remove the Set(False) opinion; joint returns to enabled default
                self._remove_joint_enabled_opinion(layer, Sdf.Path(deact_str))
            elif prev_state == "masked":
                pass  # Joint stays independently disabled; nothing to undo
            elif prev_state == "bypassed":
                # Re-enable the auxiliary fixed joints (removes Set(False) opinions)
                aux_paths = link_prim.GetCustomDataByKey(BYPASS_AUX_FIXED_JOINTS_KEY)
                if aux_paths:
                    for fp_str in aux_paths:
                        self._remove_joint_enabled_opinion(layer, Sdf.Path(fp_str))

        # 3. Unmask the link itself
        self._clear_prim_spec_recursive(layer, link_prim.GetPath())
        self._cleanup_masked_scope(layer)

        return (deact_str, prev_state) if deact_str else None

    # ==================================================================
    # Kinematic chain traversal
    # ==================================================================

    def _find_original_joint_to_link(
        self,
        stage: Usd.Stage,
        link_path: Sdf.Path,
        robot_root: Usd.Prim | None,
        body_index: int,
    ) -> Usd.Prim | None:
        """Return the first joint in the robot hierarchy targeting the given link path.

        Unlike ``_find_active_joint_to_link``, this only searches the robot
        hierarchy (not the ``__masked__`` scope) and does not skip disabled
        joints, making it suitable for reading the *original* backward joint's
        previous state before a link bypass.

        Args:
            stage: USD stage to search.
            link_path: Path of the link to find a joint for.
            robot_root: Root prim of the robot articulation.
            body_index: 0 to match ``body0``, 1 to match ``body1``.

        Returns:
            The matching joint prim, or ``None`` if not found.
        """
        if not robot_root:
            return None
        for prim in Usd.PrimRange(robot_root):
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            rel = joint.GetBody0Rel() if body_index == 0 else joint.GetBody1Rel()
            targets = rel.GetTargets() if rel else []
            if targets and targets[0] == link_path:
                return prim
        return None

    @staticmethod
    def _remove_joint_enabled_opinion(layer: Sdf.Layer, prim_path: Sdf.Path) -> None:
        """Remove only the ``physics:jointEnabled`` opinion from *prim_path* in *layer*.

        This re-enables the joint (by falling back to the default ``True``)
        without disturbing other properties on the spec (e.g. body0/body1 of
        fixed joints created by a joint bypass).

        Args:
            layer: Masking sublayer containing the opinion.
            prim_path: Path of the joint prim spec to modify.
        """
        spec = layer.GetPrimAtPath(prim_path)
        if not spec:
            return
        prop = spec.properties.get("physics:jointEnabled")
        if prop:
            spec.RemoveProperty(prop)

    def _find_backward_non_masked(self, stage: Usd.Stage, start_path: Sdf.Path | None) -> Sdf.Path | None:
        """Walk backward through the kinematic chain to the nearest non-masked link.

        Args:
            stage: USD stage to traverse.
            start_path: Starting link path, or ``None`` to return immediately.

        Returns:
            Path of the nearest non-masked ancestor link, or ``None``.
        """
        if not start_path:
            return None
        current = start_path
        visited: set[str] = set()
        while current:
            cs = str(current)
            if cs in visited:
                break
            visited.add(cs)
            prim = stage.GetPrimAtPath(current)
            if not prim or not prim.IsValid():
                return current
            if not prim.GetCustomDataByKey(MASKED_KEY):
                return current
            # Masked -- walk to parent via its parent joint's body0
            robot_root = self._find_robot_root(stage, prim)
            parent_joint = self._find_active_joint_to_link(stage, current, robot_root, body_index=1)
            if not parent_joint:
                return current
            b0 = UsdPhysics.Joint(parent_joint).GetBody0Rel().GetTargets()
            current = b0[0] if b0 else None
        return current

    def _find_forward_non_masked(
        self, stage: Usd.Stage, start_path: Sdf.Path, robot_root: Usd.Prim | None
    ) -> list[Sdf.Path]:
        """From *start_path* find the nearest non-masked link(s) forward.

        If the link is not masked, returns ``[start_path]``.
        If masked, recurses into child joints collecting non-masked
        descendants.  Halts at masked (non-bypassed) joints.

        Args:
            stage: USD stage to traverse.
            start_path: Path of the link to start from.
            robot_root: Root prim of the robot articulation.

        Returns:
            List of non-masked forward link paths.
        """
        prim = stage.GetPrimAtPath(start_path)
        if not prim or not prim.IsValid():
            return []
        if not prim.GetCustomDataByKey(MASKED_KEY):
            return [start_path]
        # Link is masked -- collect via child joints
        result: list[Sdf.Path] = []
        child_joints = self._find_forward_joints_from_link(stage, start_path, robot_root)
        for cj_prim in child_joints:
            cj_api = UsdPhysics.Joint(cj_prim)
            b1 = cj_api.GetBody1Rel().GetTargets() if cj_api.GetBody1Rel() else []
            if b1:
                result.extend(self._find_forward_non_masked(stage, b1[0], robot_root))
        return result

    def _find_active_joint_to_link(
        self,
        stage: Usd.Stage,
        link_path: Sdf.Path,
        robot_root: Usd.Prim | None,
        body_index: int,
    ) -> Usd.Prim | None:
        """Find an active joint where ``body{body_index} == link_path``.

        Searches both the robot hierarchy and the ``__masked__`` scope
        (for fixed joints from prior bypasses).  Returns the first
        non-disabled match.

        Args:
            stage: USD stage to search.
            link_path: Path of the link to match.
            robot_root: Root prim of the robot articulation.
            body_index: 0 to match ``body0``, 1 to match ``body1``.

        Returns:
            The first enabled matching joint prim, or ``None``.
        """
        candidates: list[Usd.Prim] = []
        search_roots: list[Usd.Prim] = []
        if robot_root:
            search_roots.append(robot_root)
        scope_prim = stage.GetPrimAtPath(Sdf.Path(f"/{MASKED_SCOPE}"))
        if scope_prim and scope_prim.IsValid():
            search_roots.append(scope_prim)

        for root in search_roots:
            for prim in Usd.PrimRange(root):
                if not prim.IsA(UsdPhysics.Joint):
                    continue
                joint = UsdPhysics.Joint(prim)
                rel = joint.GetBody0Rel() if body_index == 0 else joint.GetBody1Rel()
                targets = rel.GetTargets() if rel else []
                if targets and targets[0] == link_path:
                    # Skip disabled joints
                    enabled = joint.GetJointEnabledAttr()
                    if enabled and enabled.Get() is False:
                        continue
                    candidates.append(prim)

        return candidates[0] if candidates else None

    def _find_forward_joints_from_link(
        self,
        stage: Usd.Stage,
        link_path: Sdf.Path,
        robot_root: Usd.Prim | None,
    ) -> list[Usd.Prim]:
        """Return all active joints whose ``body0 == link_path``.

        Includes fixed joints from ``__masked__`` scope.  Skips joints that
        are masked (non-bypassed) -- those branches are considered halted.

        Args:
            stage: USD stage to search.
            link_path: Path of the parent link.
            robot_root: Root prim of the robot articulation.

        Returns:
            List of active forward joint prims.
        """
        results: list[Usd.Prim] = []
        search_roots: list[Usd.Prim] = []
        if robot_root:
            search_roots.append(robot_root)
        scope_prim = stage.GetPrimAtPath(Sdf.Path(f"/{MASKED_SCOPE}"))
        if scope_prim and scope_prim.IsValid():
            search_roots.append(scope_prim)

        for root in search_roots:
            for prim in Usd.PrimRange(root):
                if not prim.IsA(UsdPhysics.Joint):
                    continue
                joint = UsdPhysics.Joint(prim)
                b0 = joint.GetBody0Rel().GetTargets() if joint.GetBody0Rel() else []
                if not b0 or b0[0] != link_path:
                    continue
                # Skip disabled non-bypassed joints (masked = halted)
                enabled = joint.GetJointEnabledAttr()
                if enabled and enabled.Get() is False:
                    if not prim.GetCustomDataByKey(BYPASSED_KEY):
                        continue
                    # Bypassed: use its fixed joint substitutes instead
                    fixed_paths = prim.GetCustomDataByKey(BYPASS_FIXED_JOINTS_KEY)
                    if fixed_paths:
                        for fp_str in fixed_paths:
                            fp = stage.GetPrimAtPath(fp_str)
                            if fp and fp.IsValid():
                                results.append(fp)
                    continue
                results.append(prim)
        return results

    # ==================================================================
    # Joint reparenting with offset recalculation
    # ==================================================================

    def _reparent_joint_body0(
        self,
        stage: Usd.Stage,
        joint_prim: Usd.Prim,
        old_body0_path: Sdf.Path,
        new_body0_path: Sdf.Path,
    ) -> None:
        """Redirect body0 to *new_body0_path*, recalculating localPos0/localRot0.

        Args:
            stage: USD stage containing the joint and body prims.
            joint_prim: The joint prim whose body0 is being redirected.
            old_body0_path: Current body0 link path (used to compute world transform).
            new_body0_path: New body0 link path to redirect to.
        """
        joint = UsdPhysics.Joint(joint_prim)
        old_local = self._build_local_frame(
            joint.GetLocalPos0Attr().Get() if joint.GetLocalPos0Attr() else None,
            joint.GetLocalRot0Attr().Get() if joint.GetLocalRot0Attr() else None,
        )
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        old_prim = stage.GetPrimAtPath(old_body0_path)
        new_prim = stage.GetPrimAtPath(new_body0_path)
        if not old_prim or not new_prim:
            carb.log_warn(f"Cannot reparent {joint_prim.GetPath()}: missing body prim")
            return
        old_w = xc.GetLocalToWorldTransform(old_prim)
        new_w = xc.GetLocalToWorldTransform(new_prim)
        new_local = old_local * old_w * new_w.GetInverse()
        new_pos, new_rot = self._decompose_local_frame(new_local)

        joint.GetBody0Rel().SetTargets([new_body0_path])
        joint.GetLocalPos0Attr().Set(new_pos)
        joint.GetLocalRot0Attr().Set(new_rot)

    # ==================================================================
    # Fixed-joint creation for joint bypass
    # ==================================================================

    def _create_fixed_joint(
        self,
        stage: Usd.Stage,
        source_joint_prim: Usd.Prim,
        backward_link_path: Sdf.Path,
        forward_link_path: Sdf.Path,
    ) -> Sdf.Path | None:
        """Create a bypass ``PhysicsFixedJoint`` in the ``/__masked__/`` scope.

        Bridges *backward_link_path* to *forward_link_path*. The body1 local
        frame is copied from *source_joint_prim* so the world-space pose is
        preserved. The body0 local frame is recalculated for the new backward
        link.

        Args:
            stage: USD stage in which to create the fixed joint.
            source_joint_prim: Original joint prim whose local frames are copied.
            backward_link_path: Path of the body0 (backward/parent) link.
            forward_link_path: Path of the body1 (forward/child) link.

        Returns:
            Path of the newly created fixed joint prim, or ``None`` on failure.
        """
        scope_path = Sdf.Path(f"/{MASKED_SCOPE}")
        if not stage.GetPrimAtPath(scope_path):
            scope_prim = stage.DefinePrim(scope_path, "Scope")
            scope_prim.SetMetadata("hide_in_stage_window", True)

        source_api = UsdPhysics.Joint(source_joint_prim)
        local_pos0 = source_api.GetLocalPos0Attr().Get() if source_api.GetLocalPos0Attr() else None
        local_rot0 = source_api.GetLocalRot0Attr().Get() if source_api.GetLocalRot0Attr() else None
        local_pos1 = source_api.GetLocalPos1Attr().Get() if source_api.GetLocalPos1Attr() else None
        local_rot1 = source_api.GetLocalRot1Attr().Get() if source_api.GetLocalRot1Attr() else None

        # Recalculate body0 offset for backward_link
        orig_b0 = source_api.GetBody0Rel().GetTargets()
        orig_b0_path = orig_b0[0] if orig_b0 else None
        if orig_b0_path and orig_b0_path != backward_link_path:
            frame0 = self._build_local_frame(local_pos0, local_rot0)
            xc = UsdGeom.XformCache(Usd.TimeCode.Default())
            old_prim = stage.GetPrimAtPath(orig_b0_path)
            new_prim = stage.GetPrimAtPath(backward_link_path)
            if old_prim and new_prim:
                old_w = xc.GetLocalToWorldTransform(old_prim)
                new_w = xc.GetLocalToWorldTransform(new_prim)
                new_frame0 = frame0 * old_w * new_w.GetInverse()
                local_pos0, local_rot0 = self._decompose_local_frame(new_frame0)

        # Recalculate body1 offset for forward_link
        orig_b1 = source_api.GetBody1Rel().GetTargets()
        orig_b1_path = orig_b1[0] if orig_b1 else None
        if orig_b1_path and orig_b1_path != forward_link_path:
            frame1 = self._build_local_frame(local_pos1, local_rot1)
            xc = UsdGeom.XformCache(Usd.TimeCode.Default())
            old_prim = stage.GetPrimAtPath(orig_b1_path)
            new_prim = stage.GetPrimAtPath(forward_link_path)
            if old_prim and new_prim:
                old_w = xc.GetLocalToWorldTransform(old_prim)
                new_w = xc.GetLocalToWorldTransform(new_prim)
                new_frame1 = frame1 * old_w * new_w.GetInverse()
                local_pos1, local_rot1 = self._decompose_local_frame(new_frame1)

        # Build a unique artifact name
        src_name = str(source_joint_prim.GetPath()).lstrip("/").replace("/", "__")
        fwd_name = str(forward_link_path).lstrip("/").replace("/", "__")
        artifact_name = f"{src_name}__to__{fwd_name}"
        fixed_path = scope_path.AppendChild(artifact_name)

        fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_path)
        fixed_joint.CreateBody0Rel().SetTargets([backward_link_path])
        fixed_joint.CreateBody1Rel().SetTargets([forward_link_path])
        if local_pos0 is not None:
            fixed_joint.CreateLocalPos0Attr().Set(local_pos0)
        if local_rot0 is not None:
            fixed_joint.CreateLocalRot0Attr().Set(local_rot0)
        if local_pos1 is not None:
            fixed_joint.CreateLocalPos1Attr().Set(local_pos1)
        if local_rot1 is not None:
            fixed_joint.CreateLocalRot1Attr().Set(local_rot1)

        return fixed_path

    # ==================================================================
    # Internal helpers
    # ==================================================================

    def _get_stage(self) -> Usd.Stage | None:
        """Return the USD stage from the configured context.

        Returns:
            The current USD stage, or None if the context is unavailable.
        """
        ctx = omni.usd.get_context(self._usd_context_name)
        return ctx.get_stage() if ctx else None

    def _release_if_empty(self) -> None:
        """Release the masking layer if it contains no prim opinions."""
        if self._masking_layer.is_empty():
            self._masking_layer.release()

    def _find_robot_root(self, stage: Usd.Stage, prim: Usd.Prim) -> Usd.Prim | None:
        """Walk up the hierarchy to find the nearest ancestor with ``RobotAPI``.

        Args:
            stage: USD stage containing the prim.
            prim: Starting prim from which to search upward.

        Returns:
            The ancestor prim carrying ``RobotAPI``, or None if not found.
        """
        from usd.schema.isaac import robot_schema

        current = prim
        while current and current.IsValid() and current.GetPath() != Sdf.Path("/"):
            if current.HasAPI(robot_schema.Classes.ROBOT_API.value):
                return current
            current = current.GetParent()
        return None

    # -- local frame math -------------------------------------------------

    @staticmethod
    def _build_local_frame(local_pos: Gf.Vec3f | None, local_rot: Gf.Quatf | None) -> Gf.Matrix4d:
        """Compose a 4x4 transform matrix from position and rotation components.

        Args:
            local_pos: Translation vector, or None to leave at origin.
            local_rot: Rotation quaternion, or None to leave as identity.

        Returns:
            The composed 4x4 transformation matrix.
        """
        m = Gf.Matrix4d(1.0)
        if local_rot is not None:
            m.SetRotateOnly(Gf.Quatd(float(local_rot.GetReal()), Gf.Vec3d(local_rot.GetImaginary())))
        if local_pos is not None:
            m.SetTranslateOnly(Gf.Vec3d(local_pos))
        return m

    @staticmethod
    def _decompose_local_frame(m: Gf.Matrix4d) -> tuple[Gf.Vec3f, Gf.Quatf]:
        """Extract position and rotation from a 4x4 transform matrix.

        Returns single-precision types (``Vec3f``, ``Quatf``) to match the
        USD joint local-frame attribute types (``localPos*``, ``localRot*``).

        Args:
            m: The 4x4 transformation matrix to decompose.

        Returns:
            Tuple of (position, rotation quaternion) in single precision.
        """
        pos = Gf.Vec3f(m.ExtractTranslation())
        rot_d = m.ExtractRotation().GetQuat()
        rot_f = Gf.Quatf(float(rot_d.GetReal()), Gf.Vec3f(rot_d.GetImaginary()))
        return pos, rot_f

    # -- spec cleanup -----------------------------------------------------

    @staticmethod
    def _remove_inert_spec(layer: Sdf.Layer, prim_path: Sdf.Path) -> None:
        """Remove a prim spec from the layer if it has no properties or children.

        Args:
            layer: Layer containing the spec.
            prim_path: Path of the prim spec to check and potentially remove.
        """
        spec = layer.GetPrimAtPath(prim_path)
        if spec and not spec.properties and not spec.nameChildren:
            edit = Sdf.BatchNamespaceEdit()
            edit.Add(prim_path, Sdf.Path.emptyPath)
            layer.Apply(edit)

    @staticmethod
    def _remove_prim_spec(layer: Sdf.Layer, prim_path: Sdf.Path) -> None:
        """Remove a prim spec and all its contents from the layer.

        Args:
            layer: Layer containing the spec.
            prim_path: Path of the prim spec to remove.
        """
        if not layer.GetPrimAtPath(prim_path):
            return
        edit = Sdf.BatchNamespaceEdit()
        edit.Add(prim_path, Sdf.Path.emptyPath)
        layer.Apply(edit)

    @staticmethod
    def _clear_prim_spec_recursive(layer: Sdf.Layer, prim_path: Sdf.Path) -> None:
        """Recursively remove all properties, custom data, and children from a prim spec.

        Args:
            layer: Layer containing the spec.
            prim_path: Path of the prim spec to clear.
        """
        spec = layer.GetPrimAtPath(prim_path)
        if not spec:
            return
        for child_name in list(spec.nameChildren.keys()):
            MaskingOperations._clear_prim_spec_recursive(layer, prim_path.AppendChild(child_name))
        for prop_name in list(spec.properties.keys()):
            spec.RemoveProperty(spec.properties[prop_name])
        spec.customData.clear()
        MaskingOperations._remove_inert_spec(layer, prim_path)

    def _cleanup_masked_scope(self, layer: Sdf.Layer) -> None:
        """Remove the ``/__masked__/`` scope if it has no remaining children.

        Args:
            layer: Masking sublayer to clean up.
        """
        scope_path = Sdf.Path(f"/{MASKED_SCOPE}")
        spec = layer.GetPrimAtPath(scope_path)
        if spec and not spec.nameChildren:
            self._remove_prim_spec(layer, scope_path)

    # -- visibility -------------------------------------------------------

    @staticmethod
    def _hide_non_rigid_children(link_prim: Usd.Prim) -> None:
        """Set visibility to invisible on children that lack ``RigidBodyAPI``.

        Args:
            link_prim: Parent link prim whose non-rigid children are hidden.
        """
        for child in link_prim.GetChildren():
            if not child.HasAPI(UsdPhysics.RigidBodyAPI):
                imageable = UsdGeom.Imageable(child)
                if imageable:
                    imageable.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    # -- collisions -------------------------------------------------------

    @staticmethod
    def _collect_owned_prims(link_prim: Usd.Prim) -> list[Usd.Prim]:
        """Collect all descendant prims owned by the link's rigid body.

        Traversal stops at children that have their own ``RigidBodyAPI``
        (they belong to a different rigid body).

        Args:
            link_prim: Root link prim to traverse.

        Returns:
            List of prims within the link's rigid-body scope.
        """
        result: list[Usd.Prim] = []
        prim_range = Usd.PrimRange(link_prim, Usd.TraverseInstanceProxies())
        for prim in prim_range:
            if prim != link_prim and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim_range.PruneChildren()
                continue
            result.append(prim)
        return result

    def _disable_collisions_under(self, stage: Usd.Stage, link_prim: Usd.Prim) -> None:
        """Disable all collision APIs under the link, breaking instanceability as needed.

        Args:
            stage: USD stage containing the prim.
            link_prim: The link prim whose descendant collisions are disabled.
        """
        owned = self._collect_owned_prims(link_prim)

        instanceable_paths: set[Sdf.Path] = set()
        for prim in owned:
            if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsInstanceProxy():
                ancestor = self._find_instanceable_ancestor(prim)
                if ancestor:
                    instanceable_paths.add(ancestor.GetPath())

        made_non_instanceable: list[str] = []
        for path in instanceable_paths:
            ancestor_prim = stage.GetPrimAtPath(path)
            if ancestor_prim and ancestor_prim.IsValid():
                ancestor_prim.SetInstanceable(False)
                made_non_instanceable.append(str(path))

        if made_non_instanceable:
            link_prim.SetCustomDataByKey(NON_INSTANCEABLE_KEY, Vt.StringArray(made_non_instanceable))

        owned = self._collect_owned_prims(link_prim)
        for prim in owned:
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)

    @staticmethod
    def _find_instanceable_ancestor(prim: Usd.Prim) -> Usd.Prim | None:
        """Walk up the hierarchy to find the nearest instanceable ancestor.

        Args:
            prim: Starting prim from which to search upward.

        Returns:
            The nearest ancestor that is an instance, or None if not found.
        """
        current = prim.GetParent()
        while current and current.IsValid() and current.GetPath() != Sdf.Path("/"):
            if current.IsInstance():
                return current
            current = current.GetParent()
        return None

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

"""CTRL_DIRECT synchronization helpers for the C++ Newton tensor backend.

These functions mirror the logic in ``NewtonArticulationView`` (Python)
for syncing PD gains and position targets to the MuJoCo solver. The C++
backend calls them via pybind11 after set_dof_stiffnesses / set_dof_dampings
/ set_dof_position_targets.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import warp as wp

vec10 = wp.types.vector(length=10, dtype=float)

_dof_map_cache: wp.array | None = None
_dof_to_target_q_cache: wp.array | None = None
_dof_map_model_id: int | None = None


# ---------------------------------------------------------------------------
# Warp kernels (self-contained to avoid importing the heavy kernels module
# from isaacsim.physics.newton.impl.tensors)
# ---------------------------------------------------------------------------


@wp.kernel(enable_backward=False)
def _sync_ctrl_direct_targets(
    dof_to_act: wp.array(dtype=wp.int32),  # type: ignore[valid-type]
    dof_to_target_q: wp.array(dtype=wp.int32),  # type: ignore[valid-type]
    joint_target_pos: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    targets_per_world: wp.int32,
    ctrls_per_world: wp.int32,
    # output
    mujoco_ctrl: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Sync joint_target_pos to control.mujoco.ctrl for CTRL_DIRECT joint actuators.

    Args:
        dof_to_act: Mapping from DOF indices to CTRL_DIRECT actuator indices.
        dof_to_target_q: Template DOF -> Newton position-target coordinate mapping.
        joint_target_pos: Flattened coordinate-layout joint position targets.
        targets_per_world: Number of position-target coordinates in each world.
        ctrls_per_world: Number of MuJoCo controls in each world.
        mujoco_ctrl: Flattened MuJoCo control buffer to update.
    """
    world, dof = wp.tid()  # type: ignore[misc]
    act_idx = dof_to_act[dof]
    if act_idx < 0:
        return
    target_idx = dof_to_target_q[dof]
    if target_idx < 0:
        return
    src = world * targets_per_world + target_idx
    dst = world * ctrls_per_world + act_idx
    mujoco_ctrl[dst] = joint_target_pos[src]


@wp.kernel(enable_backward=False)
def _sync_ctrl_direct_gains(
    dof_to_act: wp.array(dtype=wp.int32),  # type: ignore[valid-type]
    joint_target_ke: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    joint_target_kd: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    # output
    actuator_gainprm: wp.array(dtype=vec10),  # type: ignore[valid-type]
    actuator_biasprm: wp.array(dtype=vec10),  # type: ignore[valid-type]
) -> None:
    """Sync joint_target_ke/kd to actuator gainprm/biasprm for CTRL_DIRECT joint actuators.

    Args:
        dof_to_act: Mapping from DOF indices to CTRL_DIRECT actuator indices.
        joint_target_ke: Per-DOF stiffness targets.
        joint_target_kd: Per-DOF damping targets.
        actuator_gainprm: MuJoCo actuator gain parameters to update.
        actuator_biasprm: MuJoCo actuator bias parameters to update.
    """
    dof = wp.tid()
    act_idx = dof_to_act[dof]
    if act_idx < 0:
        return
    kp = joint_target_ke[dof]
    kd = joint_target_kd[dof]
    actuator_gainprm[act_idx][0] = kp
    actuator_biasprm[act_idx][1] = -kp
    actuator_biasprm[act_idx][2] = -kd


def _build_ctrl_direct_dof_mapping(model: Any) -> wp.array | None:
    """Build a DOF-to-CTRL_DIRECT-actuator mapping from model custom attributes.

    For each template DOF that has a CTRL_DIRECT joint actuator, stores the
    mujoco:actuator index. Returns None if no CTRL_DIRECT joint actuators exist.

    Args:
        model: Newton model with mujoco custom attributes.

    Returns:
        wp.array of shape (dofs_per_world,) with dtype int32, or None.
    """
    mujoco_attrs = getattr(model, "mujoco", None)
    if mujoco_attrs is None:
        return None

    actuator_count = model.custom_frequency_counts.get("mujoco:actuator", 0)
    if actuator_count == 0:
        return None

    has_trnid = hasattr(mujoco_attrs, "actuator_trnid")
    has_ctrl_source = hasattr(mujoco_attrs, "ctrl_source")
    has_trntype = hasattr(mujoco_attrs, "actuator_trntype")

    if not has_trnid:
        return None

    trnid = mujoco_attrs.actuator_trnid.numpy()
    ctrl_source = mujoco_attrs.ctrl_source.numpy() if has_ctrl_source else None
    trntype = mujoco_attrs.actuator_trntype.numpy() if has_trntype else None

    target_labels = getattr(mujoco_attrs, "actuator_target_label", None)
    joint_dof_labels = getattr(mujoco_attrs, "joint_dof_label", None)
    dof_label_to_idx: dict[str, int] = {}
    if isinstance(joint_dof_labels, list):
        for i, label in enumerate(joint_dof_labels):
            if label:
                dof_label_to_idx[label] = i

    nworlds = model.world_count if hasattr(model, "world_count") else 1
    dofs_per_world = model.joint_dof_count // nworlds if nworlds > 0 else model.joint_dof_count

    if dofs_per_world == 0:
        return None

    dof_to_act = np.full(dofs_per_world, -1, dtype=np.int32)
    found_any = False

    for act_idx in range(actuator_count):
        is_ctrl_direct = ctrl_source is None or int(ctrl_source[act_idx]) == 1
        is_joint = trntype is None or int(trntype[act_idx]) == 0
        if not (is_ctrl_direct and is_joint):
            continue

        dof_idx = int(trnid[act_idx, 0]) if trnid.ndim > 1 else int(trnid[act_idx])

        if dof_idx < 0 and isinstance(target_labels, list) and act_idx < len(target_labels):
            label = target_labels[act_idx]
            if label in dof_label_to_idx:
                dof_idx = dof_label_to_idx[label]

        if 0 <= dof_idx < dofs_per_world:
            dof_to_act[dof_idx] = act_idx
            found_any = True

    if not found_any:
        return None

    return wp.array(dof_to_act, dtype=wp.int32, device=model.device)


def _build_dof_target_q_mapping(model: Any) -> wp.array | None:
    """Build a template DOF-to-position-target-coordinate mapping.

    Args:
        model: Newton model containing joint start arrays.

    Returns:
        Per-template-DOF position-target indices, or None when the model has no degrees of freedom.
    """
    nworlds = max(int(getattr(model, "world_count", 1)), 1)
    dofs_per_world = model.joint_dof_count // nworlds
    targets_per_world = model.joint_target_q.shape[0] // nworlds
    if dofs_per_world == 0 or targets_per_world == 0:
        return None

    dof_starts = model.joint_qd_start.numpy()
    target_starts = model.joint_target_q_start.numpy()
    dof_to_target_q = np.full(dofs_per_world, -1, dtype=np.int32)

    joint_count = min(len(dof_starts), len(target_starts)) - 1
    for joint_index in range(joint_count):
        dof_start = int(dof_starts[joint_index])
        target_start = int(target_starts[joint_index])
        if dof_start >= dofs_per_world or target_start >= targets_per_world:
            break

        dof_end = min(int(dof_starts[joint_index + 1]), dofs_per_world)
        target_end = min(int(target_starts[joint_index + 1]), targets_per_world)
        mapped_count = min(dof_end - dof_start, target_end - target_start)
        if mapped_count > 0:
            dof_to_target_q[dof_start : dof_start + mapped_count] = np.arange(
                target_start, target_start + mapped_count, dtype=np.int32
            )

    return wp.array(dof_to_target_q, dtype=wp.int32, device=model.device)


# ---------------------------------------------------------------------------
# Module-level cache and public API
# ---------------------------------------------------------------------------


def reset() -> None:
    """Reset cached state (call when simulation is destroyed)."""
    global _dof_map_cache, _dof_map_model_id, _dof_to_target_q_cache
    _dof_map_cache = None
    _dof_to_target_q_cache = None
    _dof_map_model_id = None


def _get_dof_map(model: Any) -> wp.array | None:
    """Return the cached CTRL_DIRECT DOF mapping array, rebuilding when the model changes.

    Args:
        model: The Newton model object.

    Returns:
        Warp array mapping DOF indices to actuator indices, or None if unavailable.
    """
    global _dof_map_cache, _dof_map_model_id, _dof_to_target_q_cache
    model_id = id(model)
    if _dof_map_model_id != model_id:
        _dof_map_model_id = model_id
        try:
            _dof_map_cache = _build_ctrl_direct_dof_mapping(model)
            _dof_to_target_q_cache = _build_dof_target_q_mapping(model)
        except Exception:
            _dof_map_cache = None
            _dof_to_target_q_cache = None
    return _dof_map_cache


def sync_actuator_gains(newton_stage: Any, model: Any) -> None:
    """Sync joint_target_ke/kd to MuJoCo actuator gainprm/biasprm.

    Args:
        newton_stage: The Newton stage object.
        model: The Newton model object.
    """
    dof_map = _get_dof_map(model)
    if dof_map is None:
        return
    mujoco_attrs = getattr(model, "mujoco", None)
    if mujoco_attrs is None:
        return
    gainprm = getattr(mujoco_attrs, "actuator_gainprm", None)
    biasprm = getattr(mujoco_attrs, "actuator_biasprm", None)
    if gainprm is None or biasprm is None:
        return
    nworlds = model.world_count if hasattr(model, "world_count") else 1
    dofs_per_world = model.joint_dof_count // max(nworlds, 1)

    _set_ctrl_direct_biastype_affine(newton_stage, model, dof_map)

    wp.launch(
        _sync_ctrl_direct_gains,
        dim=(dofs_per_world,),
        inputs=[dof_map, model.joint_target_ke, model.joint_target_kd],
        outputs=[gainprm, biasprm],
        device=model.device,
    )

    try:
        import newton

        solver = getattr(newton_stage, "solver", None)
        if solver is not None:
            solver.notify_model_changed(newton.ModelFlags.ACTUATOR_PROPERTIES)
    except (AttributeError, ImportError):
        pass


def sync_position_targets(newton_stage: Any, model: Any) -> None:
    """Sync joint_target_pos to control.mujoco.ctrl for CTRL_DIRECT actuators.

    Args:
        newton_stage: The Newton stage object.
        model: The Newton model object.
    """
    dof_map = _get_dof_map(model)
    if dof_map is None:
        return
    control = getattr(newton_stage, "control", None)
    if control is None:
        return
    mujoco_ctrl = getattr(getattr(control, "mujoco", None), "ctrl", None)
    if mujoco_ctrl is None:
        return
    dof_to_target_q = _dof_to_target_q_cache
    if dof_to_target_q is None:
        return
    nworlds = model.world_count if hasattr(model, "world_count") else 1
    dofs_per_world = model.joint_dof_count // max(nworlds, 1)
    targets_per_world = control.joint_target_q.shape[0] // max(nworlds, 1)
    ctrls_per_world = mujoco_ctrl.shape[0] // max(nworlds, 1)

    _set_ctrl_direct_biastype_affine(newton_stage, model, dof_map)

    wp.launch(
        _sync_ctrl_direct_targets,
        dim=(nworlds, dofs_per_world),
        inputs=[dof_map, dof_to_target_q, control.joint_target_q, targets_per_world, ctrls_per_world],
        outputs=[mujoco_ctrl],
        device=model.device,
    )


def _set_ctrl_direct_biastype_affine(newton_stage: Any, model: Any, dof_map: wp.array) -> None:
    """Set biastype=AFFINE for CTRL_DIRECT actuators.

    Args:
        newton_stage: The Newton stage object.
        model: The Newton model object.
        dof_map: Warp array mapping DOF indices to actuator indices.
    """
    BIAS_AFFINE = 1
    dof_map_np = dof_map.numpy()
    newton_act_indices = sorted({int(a) for a in dof_map_np if a >= 0})
    mujoco_attrs = getattr(model, "mujoco", None)
    if mujoco_attrs is not None:
        bt = getattr(mujoco_attrs, "actuator_biastype", None)
        if bt is not None:
            bt_np = bt.numpy()
            for act_idx in newton_act_indices:
                if act_idx < len(bt_np):
                    bt_np[act_idx] = BIAS_AFFINE
            wp.copy(bt, wp.array(bt_np, dtype=bt.dtype, device=bt.device))

    solver = getattr(newton_stage, "solver", None)
    if solver is None:
        return
    ctrl_source = getattr(solver, "mjc_actuator_ctrl_source", None)
    if ctrl_source is None:
        return
    ctrl_np = ctrl_source.numpy()
    mjc_act_indices = [i for i in range(len(ctrl_np)) if int(ctrl_np[i]) == 1]
    if not mjc_act_indices:
        return

    mj_model = getattr(solver, "mj_model", None)
    if mj_model is not None and hasattr(mj_model, "actuator_biastype"):
        for act_idx in mjc_act_indices:
            if act_idx < len(mj_model.actuator_biastype):
                mj_model.actuator_biastype[act_idx] = BIAS_AFFINE

    mjw_model = getattr(solver, "mjw_model", None)
    if mjw_model is not None:
        bt_warp = getattr(mjw_model, "actuator_biastype", None)
        if bt_warp is not None:
            bt_np = bt_warp.numpy()
            if bt_np.ndim == 2:
                for act_idx in mjc_act_indices:
                    if act_idx < bt_np.shape[1]:
                        bt_np[:, act_idx] = BIAS_AFFINE
            elif bt_np.ndim == 1:
                for act_idx in mjc_act_indices:
                    if act_idx < len(bt_np):
                        bt_np[act_idx] = BIAS_AFFINE
            wp.copy(bt_warp, wp.array(bt_np, dtype=bt_warp.dtype, device=bt_warp.device))

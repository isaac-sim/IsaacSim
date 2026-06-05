# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""USD layer helpers for persisting gain-tuner drive attributes."""

from __future__ import annotations

import os
import re
from collections.abc import Iterator

import pxr
from pxr import Sdf, Usd

_PHYSICS_LAYER_RE = re.compile(r"_?physics\.usda?$")
_STANDARD_PHYSICS_REL_PATHS = (
    os.path.join("payloads", "Physics", "physics.usda"),
    os.path.join("payloads", "Physics", "_physics.usd"),
)


def is_physics_layer(layer_identifier: str) -> bool:
    """Return True if ``layer_identifier`` names a robot physics payload layer."""
    return _PHYSICS_LAYER_RE.search(layer_identifier) is not None


def _layer_files_match(layer_a: Sdf.Layer, layer_b: Sdf.Layer) -> bool:
    if layer_a == layer_b:
        return True
    path_a = layer_a.realPath or layer_a.identifier
    path_b = layer_b.realPath or layer_b.identifier
    if not path_a or not path_b:
        return layer_a.identifier == layer_b.identifier
    return os.path.normpath(path_a) == os.path.normpath(path_b)


def get_layer_save_identifier(layer: Sdf.Layer | None) -> str | None:
    """Return the identifier used for save dialogs and ``Sdf.Layer.Find``."""
    if layer is None:
        return None
    if layer.realPath:
        return layer.realPath
    return layer.identifier


def find_layer_by_save_identifier(layer_id: str) -> Sdf.Layer | None:
    """Find a layer from a save identifier (``realPath`` or ``identifier``)."""
    layer = Sdf.Layer.Find(layer_id)
    if layer is not None:
        return layer
    if os.path.isfile(layer_id):
        return Sdf.Layer.FindOrOpen(layer_id)
    return None


def is_layer_savable(layer: Sdf.Layer | None) -> bool:
    """Return True when gain edits can be written and saved on ``layer``."""
    if layer is None:
        return False
    if layer.permissionToEdit and layer.permissionToSave:
        return True
    real_path = layer.realPath
    if real_path and os.path.isfile(real_path):
        return os.access(real_path, os.W_OK)
    return False


def iter_stage_layers(stage: Usd.Stage, *, include_session_layers: bool = False) -> Iterator[Sdf.Layer]:
    """Yield every layer in the stage composition, including nested subLayers.

    ``Usd.Stage.GetLayerStack()`` lists payload and reference layers, but physics
    layers are often nested as subLayers (for example ``physx.usda`` sublayering
    ``physics.usda``). This walks those subLayer trees as well.
    """
    if stage is None:
        return

    seen: set[str] = set()
    queue: list[Sdf.Layer] = list(stage.GetLayerStack(includeSessionLayers=include_session_layers))
    while queue:
        layer = queue.pop(0)
        if layer is None or layer.identifier in seen:
            continue
        seen.add(layer.identifier)
        yield layer
        for sub_path in layer.subLayerPaths:
            sub_layer = Sdf.Layer.FindOrOpenRelativeToLayer(layer, sub_path)
            if sub_layer is not None and sub_layer.identifier not in seen:
                queue.append(sub_layer)


def _physics_layer_paths_near_root(root_layer: Sdf.Layer) -> list[str]:
    """Return absolute paths to conventional robot physics layers next to the root asset."""
    root_dir = root_layer.realPath or root_layer.identifier
    if root_dir.startswith("file:"):
        root_dir = root_dir[5:]
    root_dir = os.path.dirname(os.path.abspath(root_dir))
    return [os.path.join(root_dir, rel_path) for rel_path in _STANDARD_PHYSICS_REL_PATHS]


def find_physics_layer_from_prim(prim: pxr.Usd.Prim) -> Sdf.Layer | None:
    """Find a physics layer that authors ``prim`` (walks the prim stack)."""
    if not prim or not prim.IsValid():
        return None
    for prim_spec in prim.GetPrimStack():
        if is_physics_layer(prim_spec.layer.identifier):
            return prim_spec.layer
    return None


def find_physics_layer_on_disk(stage: Usd.Stage) -> Sdf.Layer | None:
    """Open the standard ``payloads/Physics/physics.usda`` next to the stage root if present."""
    if stage is None:
        return None
    root_layer = stage.GetRootLayer()
    for path in _physics_layer_paths_near_root(root_layer):
        if not os.path.isfile(path):
            continue
        layer = Sdf.Layer.FindOrOpen(path)
        if layer is not None and is_physics_layer(layer.identifier):
            return layer
    return None


def find_physics_layer(stage: Usd.Stage, anchor_prim: pxr.Usd.Prim | None = None) -> Sdf.Layer | None:
    """Find the physics.usda (or _physics.usd) layer for a robot stage.

    Resolution order:
    1. Prim stack of ``anchor_prim`` (joint or robot) when provided.
    2. All stage layers and nested subLayers.
    3. Conventional ``payloads/Physics/physics.usda`` beside the root asset file.
    """
    if stage is None:
        return None

    if anchor_prim is not None and anchor_prim.IsValid():
        layer = find_physics_layer_from_prim(anchor_prim)
        if layer is not None:
            return layer
        parent = anchor_prim.GetParent()
        while parent and parent.IsValid():
            layer = find_physics_layer_from_prim(parent)
            if layer is not None:
                return layer
            parent = parent.GetParent()

    for layer in iter_stage_layers(stage):
        if is_physics_layer(layer.identifier):
            return layer

    return find_physics_layer_on_disk(stage)


def get_property_path_for_layer(attr: pxr.Usd.Attribute, target_layer: Sdf.Layer) -> Sdf.Path:
    """Return the property path to author on ``target_layer`` for ``attr``."""
    if not attr or target_layer is None:
        return Sdf.Path()

    for spec in attr.GetPropertyStack():
        if _layer_files_match(spec.layer, target_layer):
            return spec.path

    prim = attr.GetPrim()
    prop_name = attr.GetName()
    for prim_spec in prim.GetPrimStack():
        if _layer_files_match(prim_spec.layer, target_layer):
            return prim_spec.path.AppendProperty(prop_name)

    return attr.GetPath()


def resolve_gain_save_target(
    attr: pxr.Usd.Attribute, *, anchor_prim: pxr.Usd.Prim | None = None
) -> tuple[Sdf.Layer | None, Sdf.Path | None]:
    """Resolve the layer and property path for persisting a drive attribute.

    When a physics layer exists in the stage, gains are written there (matching the
    "Save Gains to Physics Layer" workflow) even if live UI edits created a stronger
    opinion on the root or session layer.

    Args:
        attr: USD attribute whose composed value should be saved.
        anchor_prim: Optional joint or robot prim used to locate nested physics layers.

    Returns:
        ``(layer, property_path)`` to pass to ``Stage.Open`` / ``GetAttributeAtPath``,
        or ``(None, None)`` when no suitable target exists.
    """
    if not attr:
        return None, None

    stage = attr.GetPrim().GetStage()
    property_stack = attr.GetPropertyStack()
    prim = anchor_prim if anchor_prim is not None else attr.GetPrim()

    for spec in property_stack:
        if is_physics_layer(spec.layer.identifier):
            return spec.layer, spec.path

    physics_layer = find_physics_layer(stage, anchor_prim=prim)
    if physics_layer is not None:
        return physics_layer, get_property_path_for_layer(attr, physics_layer)

    if property_stack:
        authored_spec = property_stack[0]
        return authored_spec.layer, authored_spec.path

    return None, None


def collect_gain_save_edits(
    joint_gains: list,
    stage: Usd.Stage | None,
) -> tuple[dict[str, list[tuple[Sdf.Path, object]]], Sdf.Layer | None]:
    """Collect drive attribute values keyed by save layer identifier.

    Args:
        joint_gains: Joint table rows (objects with ``joint`` and optional ``drive_axis``).
        stage: USD stage hosting the robot.

    Returns:
        ``(edits, physics_layer)`` where ``edits`` maps layer save ids to
        ``(property_path, value)`` pairs, remapped to the physics layer when found.
    """
    from isaacsim.robot_setup.gain_tuner.ui.joint_table_widget import (
        get_damping_attr,
        get_joint_drive_type_attr,
        get_mimic_damping_ratio_attr,
        get_mimic_natural_frequency_attr,
        get_stiffness_attr,
        is_joint_mimic,
    )

    edits: dict[str, list[tuple[Sdf.Path, object]]] = {}
    anchor_prim = joint_gains[0].joint if joint_gains else None
    physics_layer = find_physics_layer(stage, anchor_prim=anchor_prim) if stage else None

    for joint_gain in joint_gains:
        joint = joint_gain.joint
        if is_joint_mimic(joint):
            attrs = [get_mimic_natural_frequency_attr(joint), get_mimic_damping_ratio_attr(joint)]
        else:
            drive_axis = getattr(joint_gain, "drive_axis", None)
            attrs = [
                get_stiffness_attr(joint, drive_axis),
                get_damping_attr(joint, drive_axis),
                get_joint_drive_type_attr(joint, drive_axis),
            ]
        for attr in attrs:
            if attr is None or not attr.IsValid():
                continue
            current_value = attr.Get()
            target_layer, target_path = resolve_gain_save_target(attr, anchor_prim=joint)
            if target_layer is None or target_path is None:
                continue
            layer_id = get_layer_save_identifier(target_layer)
            if layer_id is None:
                continue
            pending = edits.setdefault(layer_id, [])
            if not any(path == target_path for path, _ in pending):
                pending.append((target_path, current_value))

    if physics_layer is not None:
        edits = remap_edits_to_physics_layer(edits, physics_layer)

    return edits, physics_layer


def remap_edits_to_physics_layer(
    edits: dict[str, list[tuple[Sdf.Path, object]]],
    physics_layer: Sdf.Layer,
) -> dict[str, list[tuple[Sdf.Path, object]]]:
    """Merge pending edits so they are all authored on ``physics_layer``."""
    if physics_layer is None:
        return edits

    merged: list[tuple[Sdf.Path, object]] = []
    seen_paths: set[Sdf.Path] = set()
    for entries in edits.values():
        for path, value in entries:
            if path in seen_paths:
                continue
            seen_paths.add(path)
            merged.append((path, value))

    if not merged:
        return edits

    save_id = get_layer_save_identifier(physics_layer)
    if save_id is None:
        return edits
    return {save_id: merged}

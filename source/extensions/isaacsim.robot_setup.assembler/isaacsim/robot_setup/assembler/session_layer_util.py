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

"""Utilities for managing USD session sublayers during robot assembly operations."""

import os
import uuid

import omni.client
from pxr import Sdf, Usd

# Keep edits in a sibling layer so relative asset paths retain their anchor.
# Only these owned working files may be removed on cancellation.
_sessions: dict[tuple[str, str], tuple[Sdf.Layer, Usd.EditTarget]] = {}


def remove_session_sublayer(stage: Usd.Stage, layer_filename: str, save: bool = False) -> None:
    """Remove a sublayer from the session layer.

    Args:
        stage: The USD stage to modify.
        layer_filename: The filename of the layer to remove.
        save: Whether to save the layer before removing.
    """
    session_layer = stage.GetSessionLayer()
    key = (session_layer.identifier, layer_filename)
    session = _sessions.get(key)
    if session is None:
        return
    layer, previous_target = session
    if save:
        save_assembly_session_sublayer(stage, layer_filename)
    if stage.GetEditTarget().GetLayer() == layer:
        stage.SetEditTarget(previous_target)
    if layer.identifier in session_layer.subLayerPaths:
        session_layer.subLayerPaths.remove(layer.identifier)
    del _sessions[key]
    omni.client.delete(layer.identifier)


def save_assembly_session_sublayer(stage: Usd.Stage, layer_filename: str) -> None:
    """Export an active payload while retaining its session for finalization.

    Args:
        stage: Stage owning the assembly session.
        layer_filename: Destination payload filename.

    Raises:
        RuntimeError: If the session is missing or the export fails.
    """
    session = _sessions.get((stage.GetSessionLayer().identifier, layer_filename))
    if session is None:
        raise RuntimeError(f"No assembly session for: {layer_filename}")
    if not session[0].Export(layer_filename):
        raise RuntimeError(f"Could not save assembly payload: {layer_filename}")
    committed_layer = Sdf.Layer.Find(layer_filename)
    if committed_layer:
        committed_layer.Reload(force=True)


def start_assembly_session_sublayer(stage: Usd.Stage, sublayer_filename: str) -> Sdf.Layer:
    """Start a new assembly session sublayer and add it to the stage.

    Args:
        stage: The USD stage to modify.
        sublayer_filename: The filename for the new sublayer.

    Returns:
        The root layer of the new sublayer.
    """
    session_layer = stage.GetSessionLayer()
    key = (session_layer.identifier, sublayer_filename)
    if key in _sessions:
        raise RuntimeError(f"Assembly session already active: {sublayer_filename}")
    stem, extension = os.path.splitext(sublayer_filename)
    working_filename = f"{stem}.assembly-{uuid.uuid4().hex}{extension or '.usd'}"
    layer = Sdf.Layer.CreateNew(working_filename)
    try:
        existing = Sdf.Layer.FindOrOpen(sublayer_filename)
        if existing:
            layer.TransferContent(existing)
        _sessions[key] = (layer, stage.GetEditTarget())
        session_layer.subLayerPaths.append(layer.identifier)
    except Exception:
        _sessions.pop(key, None)
        omni.client.delete(layer.identifier)
        raise
    return layer


def stop_assembly_session_sublayer(stage: Usd.Stage, sublayer_filename: str, save: bool = False) -> None:
    """Stop and remove an assembly session sublayer.

    Args:
        stage: The USD stage to modify.
        sublayer_filename: The filename of the sublayer to stop.
        save: Whether to save the layer before stopping.
    """
    remove_session_sublayer(stage, sublayer_filename, save)

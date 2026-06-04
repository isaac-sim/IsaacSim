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

import asyncio
import os

import omni.client
import omni.usd
from pxr import Sdf, Usd, UsdUtils

ASSEMBLY_SUBLAYER_NAME = "Assembly"


def get_sublayer_filename(base_name: str) -> str:
    """Generate a USD file name from a base name.

    Args:
        base_name: The base name for the sublayer.

    Returns:
        The complete filename with .usd extension.
    """
    return f"{base_name}.usd"


def remove_session_sublayer(stage: Usd.Stage, layer_filename: str, save: bool = False) -> None:
    """Remove a sublayer from the session layer.

    Args:
        stage: The USD stage to modify.
        layer_filename: The filename of the layer to remove.
        save: Whether to save the layer before removing.
    """
    session_layer = stage.GetSessionLayer()
    if session_layer:
        layers = session_layer.subLayerPaths
        working_layer_index = layers.index(layer_filename)
        if working_layer_index != -1:
            if save:
                layer = Sdf.Layer.Find(session_layer.subLayerPaths[working_layer_index])
                layer.Save()
            session_layer.subLayerPaths.remove(layer_filename)
            if not save:
                asyncio.ensure_future(omni.client.delete_async(layer_filename))


def get_or_create_layer_stage(filename: str) -> Usd.Stage:
    """Get an existing USD stage or create a new one if it doesn't exist.

    Creates a new stage with correct up-axis settings by using the active stage as a template.

    Args:
        filename: The filename for the USD stage.

    Returns:
        The new or existing USD stage.
    """
    try:
        if not os.path.exists(filename):
            new_stage = Usd.Stage.CreateNew(filename)
            new_stage.Save()
            return new_stage
    except Exception as e:
        # kit probably has the layer cached
        pass

    # use the active stage layer as a template, so things like up-axis are correct
    active_stage = omni.usd.get_context().get_stage()
    active_layer = active_stage.GetRootLayer()
    active_layer.Export(filename)

    existing_stage = Usd.Stage.Open(filename)
    existing_stage.GetRootLayer().Clear()

    return existing_stage


def start_assembly_session_sublayer(stage: Usd.Stage, sublayer_filename: str) -> Sdf.Layer:
    """Start a new assembly session sublayer and add it to the stage.

    Args:
        stage: The USD stage to modify.
        sublayer_filename: The filename for the new sublayer.

    Returns:
        The root layer of the new sublayer.
    """
    session_layer = stage.GetSessionLayer()

    remove_session_sublayer(stage, sublayer_filename)
    stage = get_or_create_layer_stage(sublayer_filename)
    session_layer.subLayerPaths.insert(len(session_layer.subLayerPaths), sublayer_filename)
    return stage.GetRootLayer()


def stop_assembly_session_sublayer(stage: Usd.Stage, sublayer_filename: str, save: bool = False) -> None:
    """Stop and remove an assembly session sublayer.

    Args:
        stage: The USD stage to modify.
        sublayer_filename: The filename of the sublayer to stop.
        save: Whether to save the layer before stopping.
    """
    remove_session_sublayer(stage, sublayer_filename, save)


def merge_assembly_session_sublayer(stage: Usd.Stage) -> None:
    """Merge the assembly session sublayer into the main stage and cleanup.

    Stitches the contents of the assembly layer into the main stage's root layer,
    removes the session sublayer, and deletes the temporary file.

    Args:
        stage: The USD stage to merge into.
    """
    default_prim_path = stage.GetDefaultPrim().GetPath()

    assembly_layer_filename = get_sublayer_filename(ASSEMBLY_SUBLAYER_NAME)
    assembly_stage = Usd.Stage.Open(assembly_layer_filename)
    assembly_stage.Save()
    remove_session_sublayer(stage, assembly_layer_filename)

    UsdUtils.StitchLayers(assembly_stage.GetRootLayer(), stage.GetRootLayer())

    remove_session_sublayer(stage, assembly_layer_filename)
    assembly_stage = None

    os.remove(assembly_layer_filename)

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

"""NuRec stage detection and replay-flag overrides.

NuRec (Neural Reconstruction) scenes are USD stages that embed
3D-Gaussian-splat geometry as ``UsdVolParticleField`` prims. Only RGB is a
meaningful modality on splats; semantic / depth / instance / normals
annotators have no source data to read. This module:

1. Detects a NuRec stage by traversing for ``ParticleField`` prims.
2. When detected, forces the replay-flag namespace into the supported state
   (RGB only) and disables the gaussian-pass tonemap bypass so rendered RGB
   matches the viewport.

Log severity is split by signal: actions that changed state (``FLIPPED``)
and the "NuRec detected" banner go to ``carb.log_warn``; ``unchanged`` /
``skipped`` audit detail and the non-NuRec path go to ``carb.log_info``.
"""

from __future__ import annotations

import argparse

import carb
import carb.settings
from pxr import Usd

# Prim type-name prefix authored by NuRec exporters for Gaussian-splat
# geometry. The base OpenUSD schema is ``UsdVolParticleField`` (type name
# ``ParticleField``), but real NuRec stages use specialised subclasses
# (observed: ``ParticleField3DGaussianSplat``). We match on the prefix so
# every known splat-bearing schema is detected, including future siblings.
#
# We use a string check rather than ``prim.IsA(UsdVol.ParticleField)``
# because some Kit builds ship the C++ schema without the Python binding,
# causing ``AttributeError: module 'pxr.UsdVol' has no attribute
# 'ParticleField'``.
_PARTICLE_FIELD_TYPE_PREFIX = "ParticleField"

# argparse Namespace attributes forced to a fixed value for NuRec replay.
# Only RGB is supported on splats.
_NUREC_REPLAY_FLAG_STATE: dict[str, bool] = {
    "rgb_enabled": True,
    "segmentation_enabled": False,
    "depth_enabled": False,
    "instance_id_segmentation_enabled": False,
    "normals_enabled": False,
}

# Runtime-settable carb settings forced when NuRec is detected.
#
# The gaussian-splat pass can opt out of the post-tonemap stage by default,
# which yields linear-space RGB that disagrees with the viewport (and with
# perception models trained on tonemapped sRGB). Force the pass back into the
# tonemap chain. Re-applied on every NuRec detection because opening a USD
# stage can revert process-wide carb settings via the stage's customLayerData.
_NUREC_CARB_SETTINGS: dict[str, bool] = {
    "/rtx/rtpt/gaussian/skipTonemapping/enabled": False,
}


def is_nurec_stage(stage: Usd.Stage | None) -> bool:
    """Return True if ``stage`` contains any ``UsdVolParticleField`` prims.

    NuRec assets embed Gaussian-splat geometry as ParticleField prims; any
    other stage (synthetic warehouse, robot rig, empty stage) returns False.
    Traversal short-circuits on the first hit.
    """
    if stage is None:
        return False
    for prim in stage.Traverse():
        type_name = prim.GetTypeName()
        if type_name and str(type_name).startswith(_PARTICLE_FIELD_TYPE_PREFIX):
            return True
    return False


def apply_nurec_replay_overrides(args: argparse.Namespace, stage: Usd.Stage | None) -> bool:
    """Force the replay-flag namespace into the NuRec-supported state.

    No-op when ``stage`` is not a NuRec stage. When it is, mutates ``args``
    in place so only ``rgb_enabled`` is True, and forces the gaussian-pass
    tonemap setting (``/rtx/rtpt/gaussian/skipTonemapping/enabled``) to
    ``False``.

    Args:
        args: ``argparse.Namespace`` carrying the replay rendering flag
            attributes. Attributes that are not present are skipped
            silently (logged as ``<not in args namespace, skipped>``).
        stage: Currently-loaded USD stage. ``None`` is treated as "no
            stage" and the function returns False without side effects.

    Returns:
        ``True`` if the stage was detected as NuRec and overrides were
        applied; ``False`` otherwise.
    """
    if not is_nurec_stage(stage):
        carb.log_info("[nurec-overrides] Stage has no ParticleField prims; leaving replay flags unchanged.")
        return False

    carb.log_warn(
        "[nurec-overrides] NuRec stage detected (UsdVolParticleField prims present); forcing RGB-only replay flags."
    )

    for name, target in _NUREC_REPLAY_FLAG_STATE.items():
        if not hasattr(args, name):
            carb.log_info(f"[nurec-overrides]    {name}: <not in args namespace, skipped>")
            continue
        current = getattr(args, name)
        if current == target:
            carb.log_info(f"[nurec-overrides]    {name} = {target}  (unchanged)")
            continue
        setattr(args, name, target)
        carb.log_warn(f"[nurec-overrides]  * {name}: {current} -> {target}  (FLIPPED)")

    settings = carb.settings.get_settings()
    for path, target in _NUREC_CARB_SETTINGS.items():
        current = settings.get_as_bool(path)
        if current == target:
            carb.log_info(f"[nurec-overrides]    {path} = {target}  (unchanged)")
            continue
        settings.set_bool(path, target)
        carb.log_warn(f"[nurec-overrides]  * {path}: {current} -> {target}  (FLIPPED)")

    return True

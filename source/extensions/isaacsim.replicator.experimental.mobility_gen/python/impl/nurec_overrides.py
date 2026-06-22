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

"""Replay setup for NuRec stages.

NuRec (Neural Reconstruction) scenes embed reconstructed geometry as Gaussian-splat
particle fields or NuRec volumes. Only RGB is a meaningful modality on them; semantic /
depth / instance / normals annotators have no source data to read.

``setup_for_replay`` is the replay entry point: it runs the ``nurec_utils`` render setup
(detection + case-aware carb overrides + launch-prereq gating) and, when the stage is
NuRec, restricts the replay modalities to RGB via ``ensure_nurec_replay_flags``.
"""

from __future__ import annotations

import argparse

import carb
from isaacsim.replicator.nurec_utils.rendering_setup import setup_for_rendering
from pxr import Usd

# argparse Namespace attributes forced to a fixed value for NuRec replay.
# Only RGB is supported on splats.
_NUREC_REPLAY_FLAG_STATE: dict[str, bool] = {
    "rgb_enabled": True,
    "segmentation_enabled": False,
    "depth_enabled": False,
    "instance_id_segmentation_enabled": False,
    "normals_enabled": False,
}

# NuRec splats render noisy with few path-trace samples. Recommended (not forced)
# per-frame subframe count for usable RGB; below this we warn. (Per-frame quality
# only — not the temporal staleness fix.)
_NUREC_RECOMMENDED_RT_SUBFRAMES = 36


def ensure_nurec_replay_flags(args: argparse.Namespace) -> None:
    """Restrict the replay-flag namespace to the NuRec-supported subset (RGB only).

    Mutates ``args`` in place so only ``rgb_enabled`` is True, and warns when
    ``render_rt_subframes`` is below the recommended value for splats.

    Args:
        args: ``argparse.Namespace`` carrying the replay rendering flag attributes.
            Attributes that are not present are skipped silently.
    """
    carb.log_warn("[nurec-overrides] NuRec stage; forcing RGB-only replay flags.")
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

    rt_subframes = getattr(args, "render_rt_subframes", None)
    if rt_subframes is not None and rt_subframes < _NUREC_RECOMMENDED_RT_SUBFRAMES:
        carb.log_warn(
            f"[nurec-overrides] render_rt_subframes={rt_subframes} is low for NuRec splats; "
            f"recommend >= {_NUREC_RECOMMENDED_RT_SUBFRAMES} for usable RGB."
        )


def setup_for_replay(args: argparse.Namespace, stage: Usd.Stage | None) -> tuple[bool, bool, bool, list[str]]:
    """Prepare a stage for MobilityGen replay.

    Runs the ``nurec_utils`` render setup (NuRec detection + case-aware carb overrides for
    particle / volume, ISP / no-ISP + launch-prereq gating); when the stage is NuRec,
    restricts the replay modalities to RGB. On a non-NuRec stage (or ``None``) it is a
    no-op: no carb overrides are applied and the replay flags are left untouched.

    Args:
        args: ``argparse.Namespace`` carrying the replay rendering flag attributes.
        stage: The loaded USD stage, or ``None`` (treated as a non-NuRec no-op).

    Returns:
        ``setup_for_rendering``'s ``(success, nurec, spg, problems)``.
    """
    if stage is None:
        return True, False, False, []
    success, nurec, spg, problems = setup_for_rendering(stage)
    for problem in problems:
        carb.log_warn(f"[nurec-overrides] {problem}")
    if nurec:
        ensure_nurec_replay_flags(args)
    return success, nurec, spg, problems

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

"""Experimental NuRec replay camera for MobilityGen.

Develops replay sensor RGB through PPISP by cloning an authored SPG RenderProduct for each sensor camera,
without modifying the MobilityGen extension. Used by replay_directory_for_nurec.py.

Experimental — interface subject to change in future versions. Import only after the SimulationApp exists
(it pulls in omni/Kit modules at import time).
"""

from __future__ import annotations

import omni.replicator.core as rep
from isaacsim.replicator.experimental.mobility_gen import MobilityGenCamera


def _spg_rgb_source(stage) -> str | None:
    """Return an authored RenderProduct path to clone for ISP-developed RGB.

    Returns None for a non-SPG stage, or when isaacsim.replicator.nurec_utils is not enabled, so the
    caller falls back to a plain RenderProduct.
    """
    if stage is None:
        return None
    try:
        from isaacsim.replicator.nurec_utils.render import discover_render_products
        from isaacsim.replicator.nurec_utils.rendering_setup import has_spg
    except ImportError:
        return None
    if not has_spg(stage):
        return None
    return next(iter(discover_render_products(stage).values()), None)


class ExperimentalNuRecCamera(MobilityGenCamera):
    """A MobilityGenCamera that captures RGB through a cloned PPISP RenderProduct on SPG stages.

    On an SPG/PPISP NuRec stage it clones an authored RenderProduct, rebases it onto this camera, and
    reads its ISP-developed LdrColor; on any other stage it uses the plain MobilityGenCamera RGB path.
    The rgb_image Buffer and update_state are inherited unchanged, so the writer and scenario cascade
    work as-is.

    Experimental — interface subject to change in future versions.
    """

    def __init__(self, prim_path: str, resolution: tuple[int, int]) -> None:
        super().__init__(prim_path, resolution)
        self._spg_target = None

    def enable_rgb_rendering(self) -> None:
        """Enable RGB through a PPISP clone on SPG stages, else the plain RenderProduct."""
        if self._rgb_annotator is not None:
            return
        stage = self._prim.GetStage() if self._prim and self._prim.IsValid() else None
        spg_source = _spg_rgb_source(stage)
        if spg_source is None:
            super().enable_rgb_rendering()
            return

        from isaacsim.replicator.nurec_utils.render import RenderTargetFactory, ensure_identity_exposure

        ensure_identity_exposure(stage, self._prim_path)
        name = self._prim_path.strip("/").replace("/", "_")
        self._spg_target = RenderTargetFactory(has_spg=True).clone(
            stage, name, src_rp_path=spg_source, camera_path=self._prim_path
        )
        self._spg_target.tex.set_updates_enabled(False)
        self._rgb_annotator = rep.AnnotatorRegistry.get_annotator("LdrColor")
        self._rgb_annotator.attach(self._spg_target.attach_target)

    def finalize_rendering(self) -> None:
        """Re-enable hydra texture updates, including the PPISP clone."""
        super().finalize_rendering()
        if self._spg_target is not None:
            self._spg_target.tex.set_updates_enabled(True)

    def disable_rendering(self) -> None:
        """Release the PPISP clone (if any) and the base render resources."""
        if self._spg_target is not None:
            if self._rgb_annotator is not None:
                self._rgb_annotator.detach()
                self._rgb_annotator = None
            self._spg_target.tex.set_updates_enabled(False)
            self._spg_target = None
        super().disable_rendering()


def substitute_cameras_with_nurec(module) -> None:
    """Replace every MobilityGenCamera under ``module`` with an ExperimentalNuRecCamera in place.

    Keeps each camera's prim path, resolution, and attribute name, so the writer's state keys are
    unchanged. Call after load_scenario and before enable_rgb_rendering. Needed because the sensor rig
    constructs MobilityGenCamera directly.
    """
    for name, child in list(module.children().items()):
        if isinstance(child, ExperimentalNuRecCamera):
            continue
        if isinstance(child, MobilityGenCamera):
            setattr(module, name, ExperimentalNuRecCamera(child._prim_path, child._resolution))
        else:
            substitute_cameras_with_nurec(child)

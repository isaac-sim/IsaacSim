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

"""NuRec detection, launch-prerequisite gating, and exposure-identity checks.

Uses synthetic in-memory stages — no rendering or downloaded assets. The launch-prerequisite
checks set the multi-GPU flag (a violated prerequisite) and restore it afterwards.
"""

from __future__ import annotations

import carb.settings
import omni.kit.test
from isaacsim.replicator.nurec_utils import rendering_setup as setup
from pxr import Sdf, Usd

_MULTI_GPU = "/renderer/multiGpu/enabled"


def _spg_stage() -> Usd.Stage:
    """Build an in-memory stage with a prim authoring `info:spg:sourceAsset` (an SPG NuRec USD).

    Returns:
        The synthetic SPG stage.
    """
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/Render/front_stereo_camera_left/PPISPAuto", "Shader")
    prim.CreateAttribute("info:spg:sourceAsset", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("ppisp.cu"))
    return stage


def _particle_stage() -> Usd.Stage:
    """Build an in-memory stage with a gaussian-splat `ParticleField*` prim (a particle NuRec USD).

    Returns:
        The synthetic particle stage.
    """
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World/gaussians/NuRec/background_gaussians", "ParticleField3DGaussianSplat")
    return stage


def _volume_stage() -> Usd.Stage:
    """Build an in-memory stage with a NuRec `Volume` + `OmniNuRecFieldAsset` field (a volume NuRec USD).

    Returns:
        The synthetic volume stage.
    """
    stage = Usd.Stage.CreateInMemory()
    vol = stage.DefinePrim("/World/volume", "Volume")
    vol.CreateAttribute("omni:nurec:isNuRecVolume", Sdf.ValueTypeNames.Bool).Set(True)
    stage.DefinePrim("/World/volume/density_field", "OmniNuRecFieldAsset")
    return stage


class TestDetection(omni.kit.test.AsyncTestCase):
    """Stage-detection predicates on synthetic stages."""

    async def test_has_spg(self) -> None:
        """has_spg is True only when a prim authors info:spg:sourceAsset."""
        self.assertFalse(setup.has_spg(Usd.Stage.CreateInMemory()))
        self.assertTrue(setup.has_spg(_spg_stage()))

    async def test_particle_vs_volume(self) -> None:
        """is_particle_field and is_volume identify their geometry and reject the other."""
        self.assertTrue(setup.is_particle_field(_particle_stage()))
        self.assertFalse(setup.is_particle_field(_volume_stage()))
        self.assertTrue(setup.is_volume(_volume_stage()))
        self.assertFalse(setup.is_volume(_particle_stage()))

    async def test_is_nurec(self) -> None:
        """is_nurec holds for particle/volume/SPG stages and not for an empty stage."""
        self.assertFalse(setup.is_nurec(Usd.Stage.CreateInMemory()))
        self.assertTrue(setup.is_nurec(_particle_stage()))
        self.assertTrue(setup.is_nurec(_volume_stage()))
        self.assertTrue(setup.is_nurec(_spg_stage()))

    async def test_volume_flag_false_or_absent_is_not_nurec(self) -> None:
        """A plain Volume counts as NuRec only when omni:nurec:isNuRecVolume is authored True."""
        false_stage = Usd.Stage.CreateInMemory()
        vol = false_stage.DefinePrim("/World/volume", "Volume")
        vol.CreateAttribute("omni:nurec:isNuRecVolume", Sdf.ValueTypeNames.Bool).Set(False)
        self.assertFalse(setup.is_volume(false_stage))
        self.assertFalse(setup.is_nurec(false_stage))

        absent_stage = Usd.Stage.CreateInMemory()
        absent_stage.DefinePrim("/World/volume", "Volume")
        self.assertFalse(setup.is_volume(absent_stage))


class TestLaunchPrereqGate(omni.kit.test.AsyncTestCase):
    """Launch-prerequisite gating with multi-GPU on (a violated prerequisite)."""

    async def setUp(self) -> None:
        """Turn multi-GPU on so it registers as an unmet prerequisite; remember the prior value."""
        self._settings = carb.settings.get_settings()
        self._prev_multi_gpu = bool(self._settings.get(_MULTI_GPU))
        self._settings.set(_MULTI_GPU, True)

    async def tearDown(self) -> None:
        """Restore the multi-GPU flag to its prior value."""
        self._settings.set(_MULTI_GPU, self._prev_multi_gpu)

    async def test_multi_gpu_reported_for_any_nurec(self) -> None:
        """Multi-GPU is reported as an unmet prerequisite for both SPG and non-SPG stages."""
        self.assertTrue(any("multiGpu" in e for e in setup._nurec_launch_prereq_errors(spg=True)))
        self.assertTrue(any("multiGpu" in e for e in setup._nurec_launch_prereq_errors(spg=False)))

    async def test_spg_prerequisite_satisfied_when_enabled(self) -> None:
        """omni.rtx.spg is a dependency (enabled), so it is not reported as an unmet prerequisite."""
        self.assertFalse(any("omni.rtx.spg" in e for e in setup._nurec_launch_prereq_errors(spg=True)))

    async def test_setup_passes_through_plain_stage(self) -> None:
        """A non-NuRec stage is left alone (no abort, no overrides), even with multi-GPU on."""
        self.assertEqual(setup.setup_for_rendering(Usd.Stage.CreateInMemory()), (True, False, False, []))


class TestIdentityExposure(omni.kit.test.AsyncTestCase):
    """Exposure-identity helper: report-only flags mismatches; override forces identity."""

    async def test_report_only_then_override(self) -> None:
        """Report-only flags a non-identity camera without mutating it; override forces identity, then is a no-op."""
        from isaacsim.replicator.nurec_utils import render as r

        cam_stage = Usd.Stage.CreateInMemory()
        cam = cam_stage.DefinePrim("/cam", "Camera")
        cam.CreateAttribute("exposure", Sdf.ValueTypeNames.Float).Set(5.0)
        cam.CreateAttribute("omni:rtx:autoExposure:enabled", Sdf.ValueTypeNames.Bool).Set(True)

        reported = r.ensure_identity_exposure(cam_stage, "/cam", override=False)
        self.assertTrue(any("exposure=" in s for s in reported) and any("autoExposure" in s for s in reported))
        self.assertEqual(cam.GetAttribute("exposure").Get(), 5.0)  # report-only does not mutate

        r.ensure_identity_exposure(cam_stage, "/cam", override=True)
        self.assertEqual(cam.GetAttribute("exposure").Get(), 0.0)
        self.assertEqual(cam.GetAttribute("exposure:fStop").Get(), 1.0)
        self.assertIs(cam.GetAttribute("omni:rtx:autoExposure:enabled").Get(), False)
        self.assertEqual(r.ensure_identity_exposure(cam_stage, "/cam"), [])  # already identity -> no-op

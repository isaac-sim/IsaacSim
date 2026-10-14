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

"""Tests for atomic RTX Radar WPM-DMAT authored-asset rules."""

from __future__ import annotations

import math

import omni.kit.test
from isaacsim.asset.validation.sensor_capabilities import CAPABILITIES
from isaacsim.asset.validation.sensor_rule_utils import RULES_BY_REQUIREMENT_ID
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender

from .utils import add_render_var as _add_render_var
from .utils import failures as _failures
from .utils import warnings as _warnings

_ROOT = Sdf.Path("/Asset")
_RADAR = _ROOT.AppendChild("Radar")
_PRODUCT = _ROOT.AppendChild("Product")
_CORE_PREFIX = "omni:sensor:WpmDmat:"
_SCAN_PREFIX = f"{_CORE_PREFIX}scan:s001:"


def _core(radar: Usd.Prim, name: str) -> Usd.Attribute:
    return radar.GetAttribute(f"{_CORE_PREFIX}{name}")


def _scan(radar: Usd.Prim, name: str, instance: str = "s001") -> Usd.Attribute:
    return radar.GetAttribute(f"{_CORE_PREFIX}scan:{instance}:{name}")


def _radar_stage(*, with_output: bool = True) -> Usd.Stage:
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Scope.Define(stage, _ROOT)
    radar = stage.DefinePrim(_RADAR, "OmniRadar")
    radar.ApplyAPI("OmniSensorGenericRadarWpmDmatAPI")
    product = UsdRender.Product.Define(stage, _PRODUCT)
    product.CreateCameraRel().SetTargets([_RADAR])
    product.CreateResolutionAttr(Gf.Vec2i(1, 1))
    if with_output:
        _add_render_var(stage, product, "GenericModelOutput")
    return stage


class TestRadarSensorRules(omni.kit.test.AsyncTestCase):
    """Exercise Radar structure, outputs, FAST bins, waveform, and antennas."""

    async def test_radar_presence_requires_omniradar_in_asset_scope(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, _ROOT)
        stage.SetDefaultPrim(root.GetPrim())
        stage.DefinePrim("/OtherRadar", "OmniRadar")

        self.assertEqual(len(_failures(stage, "RADAR-STRUCT-001")), 1)

        stage.DefinePrim(_RADAR, "OmniRadar")
        self.assertEqual(_failures(stage, "RADAR-STRUCT-001"), [])

    async def test_default_prim_excludes_unrelated_radar(self) -> None:
        stage = _radar_stage()
        stage.SetDefaultPrim(stage.GetPrimAtPath(_ROOT))
        stage.DefinePrim("/OmniverseKit_Radar", "OmniRadar")

        self.assertEqual(_failures(stage, "RADAR-SCHEMA-002"), [])

    async def test_every_radar_requirement_has_an_atomic_checker(self) -> None:
        requirement_ids = {
            requirement.code.rsplit(".", 1)[-1] for requirement in CAPABILITIES["RTX Radar WPM-DMAT"].requirements
        }

        self.assertEqual(len(requirement_ids), 116)
        self.assertTrue(requirement_ids <= RULES_BY_REQUIREMENT_ID.keys())
        self.assertEqual(
            len({RULES_BY_REQUIREMENT_ID[requirement_id] for requirement_id in requirement_ids}),
            len(requirement_ids),
        )

    async def test_schema_defaults_pass_all_radar_failures(self) -> None:
        stage = _radar_stage()
        requirement_ids = tuple(
            requirement.code.rsplit(".", 1)[-1] for requirement in CAPABILITIES["RTX Radar WPM-DMAT"].requirements
        )

        self.assertEqual(_failures(stage, *requirement_ids), [])

    async def test_schema_dependencies_are_separate(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Scope.Define(stage, _ROOT)
        radar = stage.DefinePrim(_RADAR, "OmniRadar")

        self.assertEqual(len(_failures(stage, "RADAR-SCHEMA-002")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-SCHEMA-003")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-SCHEMA-004")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-SCHEMA-005")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-SCHEMA-006")), 1)

        radar.ApplyAPI("OmniSensorGenericRadarWpmDmatAPI")
        self.assertEqual(_failures(stage, "RADAR-SCHEMA-002"), [])

    async def test_case_fold_collision_is_a_warning(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        radar.ApplyAPI("OmniSensorGenericRadarWpmDmatScanCfgAPI", "S001")

        self.assertEqual(_failures(stage, "RADAR-SCHEMA-009"), [])
        self.assertEqual(len(_warnings(stage, "RADAR-SCHEMA-009")), 1)

    async def test_output_channel_rules_are_independent(self) -> None:
        stage = _radar_stage(with_output=False)
        product = UsdRender.Product(stage.GetPrimAtPath(_PRODUCT))
        _add_render_var(stage, product, "GenericModelOutput", channels=["BASIC", "FULL"])
        _add_render_var(stage, product, "PointCloud", channels=["Coordinates", "Coordinates", "Bad"])
        _add_render_var(stage, product, "RtxSensorMetadata", channels=[])

        self.assertEqual(len(_failures(stage, "RADAR-OUTPUT-002")), 1)
        self.assertEqual(_failures(stage, "RADAR-OUTPUT-003"), [])
        point_cloud_issues = _failures(stage, "RADAR-OUTPUT-005")
        self.assertEqual(len(point_cloud_issues), 1)
        self.assertIn("RadialVelocityMs", point_cloud_issues[0].message)
        self.assertEqual(len(_failures(stage, "RADAR-OUTPUT-006")), 1)
        applicability_issues = _failures(stage, "RADAR-OUTPUT-008")
        self.assertEqual(len(applicability_issues), 1)
        self.assertIn("GenericModelOutput", applicability_issues[0].message)

    async def test_token_domain_messages_list_accepted_values(self) -> None:
        """Core and scan token failures identify their allowed values."""
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _core(radar, "elementsCoordsType").Set("UNKNOWN")
        _scan(radar, "elevMode").Set("UNKNOWN")
        _scan(radar, "windowSlowTime").Set("UNKNOWN")
        _scan(radar, "txAntennaGainMode").Set("UNKNOWN")

        for requirement_id, accepted in (
            ("RADAR-PARAM-001", "CARTESIAN"),
            ("RADAR-PARAM-022", "FULL_EL"),
            ("RADAR-PARAM-024", "BLACKMAN"),
            ("RADAR-PARAM-025", "PATCH_ARRAY_MODEL"),
        ):
            issues = _failures(stage, requirement_id)
            self.assertEqual(len(issues), 1)
            self.assertIn(accepted, issues[0].message)

    async def test_spg_produced_outputs_are_not_native_radar_outputs(self) -> None:
        stage = _radar_stage(with_output=False)
        product = UsdRender.Product(stage.GetPrimAtPath(_PRODUCT))
        output = _add_render_var(stage, product, "Custom")
        source = stage.DefinePrim("/Asset/Kernel", "Shader").CreateAttribute(
            "outputs:Custom", Sdf.ValueTypeNames.Opaque
        )
        output.CreateAttribute("omni:rtx:aov", Sdf.ValueTypeNames.Opaque).SetConnections([source.GetPath()])

        self.assertEqual(_failures(stage, "RADAR-OUTPUT-001"), [])

    async def test_core_domains_transform_and_tick_rate_are_independent(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _core(radar, "waveLengthMm").Set(0.0)
        _core(radar, "traceTreeDepth").Set(256)
        radar.GetAttribute("omni:sensor:tickRate").Set(math.inf)
        _core(radar, "customFrameOfReferenceTrafo").Set([0.0] * 5)

        self.assertEqual(len(_failures(stage, "RADAR-PARAM-004")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-007")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-008")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-009")), 1)

    async def test_scan_finiteness_and_primary_bounds_are_independent(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "debugForceRange").Set(math.nan)
        _scan(radar, "raysPerDeg").Set(0.0)
        _scan(radar, "maxAzAngDeg").Set(91.0)

        self.assertEqual(len(_failures(stage, "RADAR-PARAM-010")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-011")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-016")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-018")), 1)

    async def test_velocity_and_rcs_arrays_have_atomic_contracts(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "maxVelMpsSequence").Set([1.0, -1.0])
        _scan(radar, "rcsTuningCoefficients").Set([0.0, 0.0, -1.0])

        self.assertEqual(_failures(stage, "RADAR-PARAM-027"), [])
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-029")), 1)
        self.assertEqual(_failures(stage, "RADAR-PARAM-030"), [])
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-032")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-039")), 1)

    async def test_decay_and_chirp_step_allow_negative_finite_values(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "exponentialDecayFactor").Set(-1.0)
        self.assertEqual(_failures(stage, "RADAR-PARAM-045"), [])
        _scan(radar, "exponentialDecayFactor").Set(math.nan)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-045")), 1)

        _scan(radar, "datacubeMode").Set("WAVEFORM")
        _scan(radar, "frequencyStepBetweenChirps").Set(-1.0)
        self.assertEqual(_failures(stage, "RADAR-PARAM-083"), [])
        _scan(radar, "frequencyStepBetweenChirps").Set(math.nan)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-083")), 1)

    async def test_cfar_effective_dimensions_follow_mode(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "cfarAznT").Set(0)
        _scan(radar, "cfarAznG").Set(1)

        self.assertEqual(_failures(stage, "RADAR-PARAM-050"), [])
        _core(radar, "cfarMode").Set("4D")
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-050")), 1)

    async def test_fast_derived_and_explicit_bins_are_mode_specific(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "rangeResM").Set(0.0)

        self.assertEqual(len(_failures(stage, "RADAR-PARAM-052")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-056")), 1)
        self.assertEqual(_failures(stage, "RADAR-PARAM-060"), [])

        _scan(radar, "binsFromSpec").Set(False)
        _scan(radar, "rBins").Set(0)
        self.assertEqual(_failures(stage, "RADAR-PARAM-052", "RADAR-PARAM-056"), [])
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-060")), 1)

    async def test_bin_products_use_renderer_effective_dimensions(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "binsFromSpec").Set(False)
        for name in ("rBins", "vBins", "azBins", "elBins"):
            _scan(radar, name).Set(65535)

        self.assertEqual(len(_failures(stage, "RADAR-PARAM-064")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-065")), 1)

    async def test_valid_waveform_configuration_passes_waveform_rules(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "datacubeMode").Set("WAVEFORM")
        _scan(radar, "apertureArrayAzimuthElements").Set(1)
        _scan(radar, "apertureArrayElevationElements").Set(1)
        _scan(radar, "apertureAzimuthElementSpacing").Set(0.01)
        _scan(radar, "apertureElevationElementSpacing").Set(0.01)
        requirement_ids = tuple(f"RADAR-PARAM-{index:03d}" for index in range(67, 89) if index != 89)

        self.assertEqual(_failures(stage, *requirement_ids), [])

    async def test_waveform_padding_and_invertibility_fail_independently(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "datacubeMode").Set("WAVEFORM")
        _scan(radar, "slowTimePaddingSamples").Set(1)
        _scan(radar, "chirpBandwidth").Set(0.0)

        self.assertEqual(len(_failures(stage, "RADAR-PARAM-076")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-088")), 1)

    async def test_custom_antenna_grid_contracts_and_coverage_warning(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "txAntennaGainMode").Set("CUSTOM")

        self.assertEqual(
            _failures(
                stage,
                "RADAR-PARAM-090",
                "RADAR-PARAM-091",
                "RADAR-PARAM-092",
                "RADAR-PARAM-093",
            ),
            [],
        )
        self.assertEqual(_warnings(stage, "RADAR-PARAM-100"), [])

        _scan(radar, "txAntennaAzDeg").Set([0.0, -1.0])
        _scan(radar, "txAntennaCoGain").Set([1.0])
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-092")), 1)
        self.assertEqual(len(_failures(stage, "RADAR-PARAM-093")), 1)
        self.assertEqual(len(_warnings(stage, "RADAR-PARAM-100")), 1)

    async def test_patch_array_requirements_activate_per_role(self) -> None:
        stage = _radar_stage()
        radar = stage.GetPrimAtPath(_RADAR)
        _scan(radar, "rxAntennaGainMode").Set("PATCH_ARRAY_MODEL")

        for requirement_id in (
            "RADAR-PARAM-096",
            "RADAR-PARAM-097",
            "RADAR-PARAM-098",
            "RADAR-PARAM-099",
        ):
            self.assertEqual(len(_failures(stage, requirement_id)), 1)

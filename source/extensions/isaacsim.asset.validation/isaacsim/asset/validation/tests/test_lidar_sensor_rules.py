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

"""Tests for atomic RTX Lidar authored-asset rules."""

from __future__ import annotations

import math

import omni.kit.test
from isaacsim.asset.validation.sensor_capabilities import CAPABILITIES
from isaacsim.asset.validation.sensor_rule_utils import RULES_BY_REQUIREMENT_ID
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender

from .utils import add_render_var as _add_render_var
from .utils import failures as _failures

_ROOT = Sdf.Path("/Asset")
_LIDAR = _ROOT.AppendChild("Lidar")
_PRODUCT = _ROOT.AppendChild("Product")
_CORE_PREFIX = "omni:sensor:Core:"
_EMITTER_PREFIX = f"{_CORE_PREFIX}emitterState:s001:"


def _core(lidar: Usd.Prim, name: str) -> Usd.Attribute:
    return lidar.GetAttribute(f"{_CORE_PREFIX}{name}")


def _emitter(lidar: Usd.Prim, name: str) -> Usd.Attribute:
    return lidar.GetAttribute(f"{_EMITTER_PREFIX}{name}")


def _lidar_stage(*, with_output: bool = True) -> Usd.Stage:
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Scope.Define(stage, _ROOT)
    lidar = stage.DefinePrim(_LIDAR, "OmniLidar")
    lidar.ApplyAPI("OmniSensorGenericLidarCoreAPI")
    product = UsdRender.Product.Define(stage, _PRODUCT)
    product.CreateCameraRel().SetTargets([_LIDAR])
    product.CreateResolutionAttr(Gf.Vec2i(1, 1))
    if with_output:
        _add_render_var(stage, product, "GenericModelOutput")
    return stage


class TestLidarSensorRules(omni.kit.test.AsyncTestCase):
    """Exercise Lidar outputs, schema contracts, arrays, and conditional modes."""

    async def test_lidar_presence_requires_omnilidar_in_asset_scope(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, _ROOT)
        stage.SetDefaultPrim(root.GetPrim())
        stage.DefinePrim("/OtherLidar", "OmniLidar")

        self.assertEqual(len(_failures(stage, "LIDAR-STRUCT-001")), 1)

        stage.DefinePrim(_LIDAR, "OmniLidar")
        self.assertEqual(_failures(stage, "LIDAR-STRUCT-001"), [])

    async def test_default_prim_excludes_unrelated_lidar(self) -> None:
        stage = _lidar_stage()
        stage.SetDefaultPrim(stage.GetPrimAtPath(_ROOT))
        stage.DefinePrim("/OmniverseKit_Lidar", "OmniLidar")

        self.assertEqual(_failures(stage, "LIDAR-SCHEMA-001"), [])

    async def test_every_catalogued_lidar_requirement_has_an_atomic_checker(self) -> None:
        requirement_ids = {
            requirement.code.rsplit(".", 1)[-1] for requirement in CAPABILITIES["RTX Lidar Core"].requirements
        }

        self.assertNotIn("LIDAR-PARAM-038", requirement_ids)
        self.assertTrue(requirement_ids <= RULES_BY_REQUIREMENT_ID.keys())
        self.assertEqual(
            len({RULES_BY_REQUIREMENT_ID[requirement_id] for requirement_id in requirement_ids}),
            len(requirement_ids),
        )

    async def test_schema_defaults_pass_every_catalogued_lidar_rule(self) -> None:
        stage = _lidar_stage()
        requirement_ids = tuple(
            requirement.code.rsplit(".", 1)[-1] for requirement in CAPABILITIES["RTX Lidar Core"].requirements
        )

        self.assertEqual(_failures(stage, *requirement_ids), [])

    async def test_render_var_rules_are_atomic_and_spg_outputs_are_not_native(self) -> None:
        stage = _lidar_stage(with_output=False)
        product = UsdRender.Product(stage.GetPrimAtPath(_PRODUCT))
        invalid = _add_render_var(stage, product, "NotNative")

        source_issues = _failures(stage, "LIDAR-VAR-001")
        self.assertEqual(len(source_issues), 1)
        self.assertIn("GenericModelOutput", source_issues[0].message)
        self.assertIn("RtxSensorMetadata", source_issues[0].message)
        self.assertEqual(_failures(stage, "LIDAR-VAR-007"), [])

        aov = invalid.CreateAttribute("omni:rtx:aov", Sdf.ValueTypeNames.Opaque)
        source = stage.DefinePrim("/Asset/Kernel", "Shader").CreateAttribute(
            "outputs:NotNative", Sdf.ValueTypeNames.Opaque
        )
        aov.SetConnections([source.GetPath()])
        self.assertEqual(_failures(stage, "LIDAR-VAR-001"), [])
        native_issues = _failures(stage, "LIDAR-VAR-007")
        self.assertEqual(len(native_issues), 1)
        self.assertIn("PointCloud", native_issues[0].message)

    async def test_channel_allowlists_and_api_applicability_are_independent(self) -> None:
        stage = _lidar_stage()
        product = UsdRender.Product(stage.GetPrimAtPath(_PRODUCT))
        gmo = _add_render_var(stage, product, "GenericModelOutput", channels=["BASIC", "FULL"])
        point_cloud = _add_render_var(stage, product, "PointCloud", channels=["Coordinates", "Coordinates"])
        metadata = _add_render_var(stage, product, "RtxSensorMetadata", channels=[])

        gmo_issues = _failures(stage, "LIDAR-VAR-004")
        self.assertEqual(len(gmo_issues), 1)
        self.assertIn("EXTRA", gmo_issues[0].message)
        point_cloud_issues = _failures(stage, "LIDAR-VAR-005")
        self.assertEqual(len(point_cloud_issues), 1)
        self.assertIn("HitNormal", point_cloud_issues[0].message)
        applicability_issues = _failures(stage, "LIDAR-VAR-002")
        self.assertEqual(len(applicability_issues), 1)
        self.assertIn("GenericModelOutput", applicability_issues[0].message)
        self.assertTrue(gmo and point_cloud and metadata)

    async def test_core_api_model_and_emitter_application_fail_separately(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Scope.Define(stage, _ROOT)
        lidar = stage.DefinePrim(_LIDAR, "OmniLidar")

        self.assertEqual(len(_failures(stage, "LIDAR-SCHEMA-001")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-MODEL-001")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-EMITTER-001")), 1)

        wrong_target = UsdGeom.Xform.Define(stage, "/Asset/WrongTarget").GetPrim()
        wrong_target.ApplyAPI("OmniSensorGenericLidarCoreEmitterStateAPI", "aux")
        self.assertEqual(len(_failures(stage, "LIDAR-EMITTER-002")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-EMITTER-003")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-EMITTER-004")), 1)
        self.assertTrue(lidar)

    async def test_emitter_array_contracts_and_channel_upper_bound_are_atomic(self) -> None:
        stage = _lidar_stage()
        lidar = stage.GetPrimAtPath(_LIDAR)
        _core(lidar, "numberOfEmitters").Set(2)
        _core(lidar, "numberOfChannels").Set(2)
        _emitter(lidar, "azimuthDeg").Set([0.0])
        _emitter(lidar, "elevationDeg").Set([0.0, 0.0])
        _emitter(lidar, "fireTimeNs").Set([0, 1])
        _emitter(lidar, "vertOffsetM").Set([0.0])
        _emitter(lidar, "channelId").Set([1, 3])

        self.assertEqual(len(_failures(stage, "LIDAR-ARRAY-001")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-ARRAY-002")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-ARRAY-003")), 1)

    async def test_emitter_value_bounds_are_independent(self) -> None:
        stage = _lidar_stage()
        lidar = stage.GetPrimAtPath(_LIDAR)
        _emitter(lidar, "focalDistM").Set([-1.0] * 128)
        _emitter(lidar, "emitterPeakPowerW").Set([-2.0] * 128)

        self.assertEqual(len(_failures(stage, "LIDAR-ARRAY-004")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-ARRAY-005")), 1)

    async def test_nonfinite_and_scalar_bounds_are_reported_by_their_own_rules(self) -> None:
        stage = _lidar_stage()
        lidar = stage.GetPrimAtPath(_LIDAR)
        lidar.GetAttribute("omni:sensor:tickRate").Set(math.nan)
        _core(lidar, "patternFiringRateHz").Set(0)
        _core(lidar, "reflectionPowerFraction").Set(-0.5)
        _core(lidar, "minReflectance").Set(0.0)
        _core(lidar, "minReflectionRangeM").Set(0.0)

        self.assertEqual(len(_failures(stage, "LIDAR-PARAM-001")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-PARAM-003")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-PARAM-012")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-PARAM-014")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-PARAM-019")), 1)
        self.assertEqual(_failures(stage, "LIDAR-FIRE-002"), [])
        _core(lidar, "reflectionPowerFraction").Set(1.5)
        self.assertEqual(_failures(stage, "LIDAR-PARAM-012"), [])

    async def test_beam_forms_and_origin_vector_sizes_are_independent(self) -> None:
        stage = _lidar_stage()
        lidar = stage.GetPrimAtPath(_LIDAR)
        _core(lidar, "beamWaistHorM").Set(0.0)
        _core(lidar, "beamWaistVerM").Set(0.0)
        _core(lidar, "divergenceHorDeg").Set(1.0)
        _core(lidar, "divergenceVerDeg").Set(1.0)
        self.assertEqual(_failures(stage, "LIDAR-PARAM-035"), [])

        _core(lidar, "beamWaistHorM").Set(1.0)
        _core(lidar, "beamWaistVerM").Set(1.0)
        self.assertEqual(len(_failures(stage, "LIDAR-PARAM-035")), 1)

        _core(lidar, "divergenceHorDeg").Set(0.0)
        _core(lidar, "divergenceVerDeg").Set(0.0)
        self.assertEqual(_failures(stage, "LIDAR-PARAM-035"), [])
        _core(lidar, "beamWaistVerM").Set(0.0)
        _core(lidar, "originErrorMean").Set([0.0, 0.0])
        _core(lidar, "originErrorStd").Set([0.0, 0.0])

        self.assertEqual(len(_failures(stage, "LIDAR-PARAM-035")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-PARAM-036")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-PARAM-037")), 1)

    async def test_range_cardinality_ordering_and_ids_are_independent(self) -> None:
        stage = _lidar_stage()
        lidar = stage.GetPrimAtPath(_LIDAR)
        _core(lidar, "rangeCount").Set(2)
        _core(lidar, "rangesMinM").Set([2.0])
        _core(lidar, "rangesMaxM").Set([1.0, 3.0])
        _emitter(lidar, "rangeId").Set([0] * 127 + [2])

        self.assertEqual(len(_failures(stage, "LIDAR-RANGE-001")), 1)
        self.assertEqual(_failures(stage, "LIDAR-RANGE-002"), [])
        self.assertEqual(len(_failures(stage, "LIDAR-RANGE-005")), 1)
        self.assertEqual(_failures(stage, "LIDAR-RANGE-006"), [])
        self.assertEqual(len(_failures(stage, "LIDAR-RANGE-007")), 1)
        self.assertEqual(_failures(stage, "LIDAR-RANGE-008"), [])

        _emitter(lidar, "rangeId").Set([])
        self.assertEqual(len(_failures(stage, "LIDAR-RANGE-008")), 1)

    async def test_fire_time_order_and_tick_bound_are_independent(self) -> None:
        stage = _lidar_stage()
        lidar = stage.GetPrimAtPath(_LIDAR)
        values = list(range(128))
        values[-2:] = [30_000, 29_000]
        _emitter(lidar, "fireTimeNs").Set(values)

        self.assertEqual(len(_failures(stage, "LIDAR-FIRE-001")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-FIRE-002")), 1)

    async def test_custom_frame_contract_is_conditional(self) -> None:
        stage = _lidar_stage()
        lidar = stage.GetPrimAtPath(_LIDAR)
        transform = _core(lidar, "customFrameOfReferenceTrafo")
        transform.Set([0.0] * 6)

        self.assertEqual(_failures(stage, "LIDAR-COND-001"), [])
        _core(lidar, "outputFrameOfReference").Set("CUSTOM")
        transform.Set([0.0] * 5)
        self.assertEqual(len(_failures(stage, "LIDAR-COND-001")), 1)

    async def test_solid_state_rules_do_not_depend_on_checker_scan_state(self) -> None:
        stage = _lidar_stage()
        lidar = stage.GetPrimAtPath(_LIDAR)
        _core(lidar, "scanType").Set("SOLID_STATE")
        _core(lidar, "numLines").Set(2)
        _core(lidar, "numRaysPerLine").Set([64, 64])

        self.assertEqual(
            _failures(
                stage,
                "LIDAR-COND-003",
                "LIDAR-COND-004",
                "LIDAR-COND-005",
                "LIDAR-COND-006",
                "LIDAR-COND-009",
            ),
            [],
        )

        _core(lidar, "numRaysPerLine").Set([65_536])
        self.assertEqual(len(_failures(stage, "LIDAR-COND-004")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-COND-005")), 1)
        self.assertEqual(len(_failures(stage, "LIDAR-COND-006")), 1)

    async def test_rotary_roi_rules_are_independent(self) -> None:
        stage = _lidar_stage()
        lidar = stage.GetPrimAtPath(_LIDAR)
        _emitter(lidar, "isROIState").Set(True)
        _emitter(lidar, "roi").Set([False] * 128)

        self.assertEqual(len(_failures(stage, "LIDAR-COND-010")), 1)
        self.assertEqual(_failures(stage, "LIDAR-COND-011"), [])
        self.assertEqual(len(_failures(stage, "LIDAR-COND-012")), 1)

        _core(lidar, "minAzimuthROI").Set(360.0)
        self.assertEqual(_failures(stage, "LIDAR-COND-010"), [])

        _emitter(lidar, "roi").Set([True])
        self.assertEqual(len(_failures(stage, "LIDAR-COND-011")), 1)
        self.assertEqual(_failures(stage, "LIDAR-COND-012"), [])

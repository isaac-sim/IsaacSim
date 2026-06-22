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

"""Tests RTSPCameraHelper writer registration and render-product setup.

The coverage focuses on writer lookup, graph-input routing, SRTX sensor-set
forwarding, setup skipping for invalid render products, and RenderVar
compression state for H.264 and raw streams.
"""

from __future__ import annotations

import json
import sys
import unittest
from typing import Any
from unittest.mock import patch

import carb
import omni.graph.core as og
import omni.kit.app
import omni.kit.test
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.nodes import BaseWriterNode
from isaacsim.streaming.rtsp.impl.rtsp_writer import WRITER_NAME, RTSPStreamWriter
from pxr import Sdf


class TestRTSPExtensionRegistration(omni.kit.test.AsyncTestCase):
    """Verify the RTSP streaming extension registers the writer correctly on startup."""

    async def setUp(self) -> None:
        """Advance one Kit frame so extension startup registration has settled."""
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Yield one Kit frame after registry assertions complete."""
        await omni.kit.app.get_app().next_update_async()

    async def test_writer_is_registered(self) -> None:
        """Confirm the RTSP writer name is present in Replicator's registry."""
        writers = rep.WriterRegistry.get_writers()
        self.assertIn(WRITER_NAME, writers)

    async def test_writer_get_returns_correct_type(self) -> None:
        """Confirm registry lookup constructs an ``RTSPStreamWriter`` instance."""
        writer = rep.writers.get(WRITER_NAME)
        self.assertIsInstance(writer, RTSPStreamWriter)

    async def test_writer_in_default_writers_list(self) -> None:
        """Confirm the extension advertises RTSP as a default writer."""
        self.assertIn(WRITER_NAME, rep.WriterRegistry._default_writers)

    async def test_registered_writer_exposes_writer_interface(self) -> None:
        """Confirm the registered object exposes the Replicator writer methods."""
        writer = rep.writers.get(WRITER_NAME)
        self.assertTrue(callable(writer.write))
        self.assertTrue(callable(writer.detach))


class TestRTSPCameraHelperEncoding(omni.kit.test.AsyncTestCase):
    """Verify the OGN node passes the correct encoding based on useRawEncoding."""

    SRTX_ENABLED_SETTING = "/exts/omni.replicator.srtx/enabled"
    SENSOR_SET_NAME_SETTING = "/exts/omni.replicator.srtx/sensorSetName"
    SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING = "/exts/omni.replicator.srtx/sensorSetNameByRenderProductPath"
    SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING = "/exts/omni.replicator.srtx/sensorSetRenderProductPathsByName"

    async def setUp(self) -> None:
        """Create a fresh stage and save SRTX settings that affect resolution."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._settings = carb.settings.get_settings()
        self._saved_settings = {
            self.SRTX_ENABLED_SETTING: self._settings.get(self.SRTX_ENABLED_SETTING),
            self.SENSOR_SET_NAME_SETTING: self._settings.get(self.SENSOR_SET_NAME_SETTING),
            self.SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING: self._settings.get(
                self.SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING
            ),
            self.SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING: self._settings.get(
                self.SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING
            ),
        }
        self._clear_srtx_settings()

    async def tearDown(self) -> None:
        """Restore SRTX settings modified by the sensor-set routing tests."""
        for setting_name, value in self._saved_settings.items():
            if setting_name == self.SRTX_ENABLED_SETTING:
                self._settings.set_bool(setting_name, bool(value) if value is not None else False)
            else:
                self._settings.set(setting_name, value if value is not None else "")
        await omni.kit.app.get_app().next_update_async()

    def _clear_srtx_settings(self) -> Any:
        """Clear SRTX settings that affect RTSPCameraHelper writer construction.

        Returns:
            None.
        """
        self._settings.set_bool(self.SRTX_ENABLED_SETTING, False)
        self._settings.set(self.SENSOR_SET_NAME_SETTING, "")
        self._settings.set(self.SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING, "")
        self._settings.set(self.SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING, "")

    def _create_rtsp_graph(self, use_raw_encoding: Any) -> Any:
        """Build an action graph with a RenderProduct-backed RTSP helper node.

        Args:
            use_raw_encoding: Value to set on ``RTSPHelper.inputs:useRawEncoding``.

        Returns:
            None.
        """
        stage = omni.usd.get_context().get_stage()
        stage.DefinePrim("/TestRenderProduct", "RenderProduct")

        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/TestGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RTSPHelper", "isaacsim.streaming.rtsp.RTSPCameraHelper"),
                ],
                keys.SET_VALUES: [
                    ("RTSPHelper.inputs:renderProductPath", "/TestRenderProduct"),
                    ("RTSPHelper.inputs:port", 8554),
                    ("RTSPHelper.inputs:mountPath", "/stream"),
                    ("RTSPHelper.inputs:useRawEncoding", use_raw_encoding),
                    ("RTSPHelper.inputs:enabled", True),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RTSPHelper.inputs:execIn"),
                ],
            },
        )

    async def _evaluate_graph(self) -> Any:
        """Play the timeline for one tick to trigger the action graph.

        Returns:
            None.
        """
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    def _capture_writer_init(self) -> Any:
        """Return (init_calls, context_manager) that intercepts RTSPStreamWriter construction.

        Does NOT call through to the real __init__ to avoid creating live
        annotators that would receive frames during timeline playback.

        Returns:
            Tuple containing the captured init-call list and patch context manager.
        """
        init_calls = []

        def stub_init(writer_self: Any, *args: Any, **kwargs: Any) -> Any:
            init_calls.append(kwargs)

        ctx = patch.object(RTSPStreamWriter, "__init__", stub_init)
        return init_calls, ctx

    async def test_use_raw_encoding_passes_raw(self) -> None:
        """Verify ``useRawEncoding=True`` creates the writer with raw encoding."""
        self._create_rtsp_graph(use_raw_encoding=True)
        init_calls, spy_ctx = self._capture_writer_init()

        with spy_ctx, patch.object(BaseWriterNode, "append_writer"), patch.object(BaseWriterNode, "attach_writers"):
            await self._evaluate_graph()

        self.assertGreaterEqual(len(init_calls), 1)
        self.assertEqual(init_calls[0]["port"], 8554)
        self.assertEqual(init_calls[0]["mountPath"], "/stream")
        self.assertEqual(init_calls[0]["encoding"], "raw")

    async def test_default_encoding_passes_h264(self) -> None:
        """Verify the default helper path creates an H.264 writer."""
        self._create_rtsp_graph(use_raw_encoding=False)
        init_calls, spy_ctx = self._capture_writer_init()

        with spy_ctx, patch.object(BaseWriterNode, "append_writer"), patch.object(BaseWriterNode, "attach_writers"):
            await self._evaluate_graph()

        self.assertGreaterEqual(len(init_calls), 1)
        self.assertEqual(init_calls[0]["port"], 8554)
        self.assertEqual(init_calls[0]["mountPath"], "/stream")
        self.assertEqual(init_calls[0]["encoding"], "h264")

    @unittest.skipIf(
        not sys.platform.startswith("linux"),
        "omni.replicator.srtx is only available on Linux for RTSP sensor-set resolution",
    )
    async def test_configured_sensor_set_passed_to_writer(self) -> None:
        """Configured per-render-product sensor set should be passed to RTSPStreamWriter."""
        self._settings.set_bool(self.SRTX_ENABLED_SETTING, True)
        self._settings.set(
            self.SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/TestRenderProduct": "ss-configured"}),
        )
        self._settings.set(
            self.SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
            json.dumps({"ss-configured": ["/TestRenderProduct", "/OtherRenderProduct"]}),
        )
        self._create_rtsp_graph(use_raw_encoding=False)
        init_calls, spy_ctx = self._capture_writer_init()

        with spy_ctx, patch.object(BaseWriterNode, "append_writer"), patch.object(BaseWriterNode, "attach_writers"):
            await self._evaluate_graph()

        self.assertGreaterEqual(len(init_calls), 1)
        self.assertEqual(init_calls[0]["sensorSetName"], "ss-configured")

    @unittest.skipIf(
        not sys.platform.startswith("linux"),
        "omni.replicator.srtx is only available on Linux for RTSP sensor-set resolution",
    )
    async def test_process_sensor_set_fallback_passed_to_writer(self) -> None:
        """Process-wide sensor set should be passed when no complete per-RP mapping exists."""
        self._settings.set_bool(self.SRTX_ENABLED_SETTING, True)
        self._settings.set(self.SENSOR_SET_NAME_SETTING, "bridge-sensor-set")
        self._create_rtsp_graph(use_raw_encoding=False)
        init_calls, spy_ctx = self._capture_writer_init()

        with spy_ctx, patch.object(BaseWriterNode, "append_writer"), patch.object(BaseWriterNode, "attach_writers"):
            await self._evaluate_graph()

        self.assertGreaterEqual(len(init_calls), 1)
        self.assertEqual(init_calls[0]["sensorSetName"], "bridge-sensor-set")

    async def test_empty_render_product_path_skips_setup(self) -> None:
        """Verify an empty render-product input does not construct a writer."""
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/TestGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RTSPHelper", "isaacsim.streaming.rtsp.RTSPCameraHelper"),
                ],
                keys.SET_VALUES: [
                    ("RTSPHelper.inputs:renderProductPath", ""),
                    ("RTSPHelper.inputs:enabled", True),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RTSPHelper.inputs:execIn"),
                ],
            },
        )
        init_calls, spy_ctx = self._capture_writer_init()

        with spy_ctx:
            await self._evaluate_graph()

        self.assertEqual(len(init_calls), 0)

    async def test_missing_render_product_prim_skips_setup(self) -> None:
        """Verify a non-existent render-product prim leaves writer setup skipped."""
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/TestGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RTSPHelper", "isaacsim.streaming.rtsp.RTSPCameraHelper"),
                ],
                keys.SET_VALUES: [
                    ("RTSPHelper.inputs:renderProductPath", "/NonExistentPrim"),
                    ("RTSPHelper.inputs:enabled", True),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RTSPHelper.inputs:execIn"),
                ],
            },
        )
        init_calls, spy_ctx = self._capture_writer_init()

        with spy_ctx:
            await self._evaluate_graph()

        self.assertEqual(len(init_calls), 0)

    async def test_disabled_node_does_not_create_writer(self) -> None:
        """Verify a disabled helper node does not create or attach a writer."""
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/TestGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RTSPHelper", "isaacsim.streaming.rtsp.RTSPCameraHelper"),
                ],
                keys.SET_VALUES: [
                    ("RTSPHelper.inputs:renderProductPath", "/TestRenderProduct"),
                    ("RTSPHelper.inputs:enabled", False),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RTSPHelper.inputs:execIn"),
                ],
            },
        )
        init_calls, spy_ctx = self._capture_writer_init()

        with spy_ctx:
            await self._evaluate_graph()

        self.assertEqual(len(init_calls), 0)


class TestRTSPCameraHelperRenderVarSetup(omni.kit.test.AsyncTestCase):
    """Verify the OGN node materialises the SRTX-required RenderVar child prim.

    These tests target the ``ensure_render_var_on_product`` integration: the
    helper must guarantee that a ``RenderVar`` child of the render product
    exists with the right ``sourceName``, is registered in the render
    product's ``orderedVars`` relationship, and carries the
    ``srtx:compression:type`` attribute set to a value that matches the
    helper's chosen encoding mode (overwriting any pre-authored value).
    """

    RENDER_PRODUCT_PATH = "/TestRenderProduct"
    AOV_NAME = "LdrColor"
    SRTX_COMPRESSION_ATTR = "srtx:compression:type"

    async def setUp(self) -> None:
        """Create a fresh stage for RenderVar side-effect assertions."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Yield one Kit frame after RenderVar graph evaluation."""
        await omni.kit.app.get_app().next_update_async()

    def _create_render_product(self) -> Any:
        """Define a bare ``RenderProduct`` prim, no rendervar children.

        Returns:
            None.
        """
        stage = omni.usd.get_context().get_stage()
        stage.DefinePrim(self.RENDER_PRODUCT_PATH, "RenderProduct")

    def _create_render_product_with_rendervar(self, preauthored_compression: str) -> Any:
        """Define a ``RenderProduct`` with a pre-authored ``RenderVar`` child.

        Mirrors the layout used by scenes that explicitly declare a render
        var (e.g. via Replicator's ``render_vars=[...]``): the rendervar is
        a child of the RP, ``sourceName`` is set to the AOV name, the RP's
        ``orderedVars`` relationship targets it, and ``srtx:compression:type``
        is pre-authored so we can verify the helper overwrites it.

        Args:
            preauthored_compression: Value to author on the rendervar's
                ``srtx:compression:type`` attribute before ``compute()`` runs.

        Returns:
            None.
        """
        stage = omni.usd.get_context().get_stage()
        rp_prim = stage.DefinePrim(self.RENDER_PRODUCT_PATH, "RenderProduct")
        rendervar_path = f"{self.RENDER_PRODUCT_PATH}/{self.AOV_NAME}"
        rv_prim = stage.DefinePrim(rendervar_path, "RenderVar")
        rv_prim.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set(self.AOV_NAME)
        rv_prim.CreateAttribute(self.SRTX_COMPRESSION_ATTR, Sdf.ValueTypeNames.String).Set(preauthored_compression)
        rp_prim.CreateRelationship("orderedVars").AddTarget(rendervar_path)

    def _build_graph(self, use_raw_encoding: bool) -> Any:
        """Build a graph that feeds encoding mode and render product into the helper.

        Args:
            use_raw_encoding: Value to set on ``RTSPHelper.inputs:useRawEncoding``.

        Returns:
            None.
        """
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/TestGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RTSPHelper", "isaacsim.streaming.rtsp.RTSPCameraHelper"),
                ],
                keys.SET_VALUES: [
                    ("RTSPHelper.inputs:renderProductPath", self.RENDER_PRODUCT_PATH),
                    ("RTSPHelper.inputs:port", 8554),
                    ("RTSPHelper.inputs:mountPath", "/stream"),
                    ("RTSPHelper.inputs:useRawEncoding", use_raw_encoding),
                    ("RTSPHelper.inputs:enabled", True),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RTSPHelper.inputs:execIn"),
                ],
            },
        )

    async def _tick_graph(self) -> Any:
        """Play the timeline for one tick to trigger ``compute()``.

        Returns:
            None.
        """
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def _run_helper(self, use_raw_encoding: bool) -> Any:
        """Run the helper once with ``BaseWriterNode`` and ``RTSPStreamWriter`` stubbed.

        We only care about the USD side-effects of ``compute()``, not the live
        writer/server, so the writer's ``__init__`` and the
        ``BaseWriterNode.append_writer`` / ``attach_writers`` calls are
        intercepted (matching the pattern in ``TestRTSPCameraHelperEncoding``).

        Args:
            use_raw_encoding: Value to set on ``RTSPHelper.inputs:useRawEncoding``.

        Returns:
            None.
        """
        self._build_graph(use_raw_encoding=use_raw_encoding)

        def stub_init(writer_self: Any, *args: Any, **kwargs: Any) -> Any:
            pass

        with (
            patch.object(RTSPStreamWriter, "__init__", stub_init),
            patch.object(BaseWriterNode, "append_writer"),
            patch.object(BaseWriterNode, "attach_writers"),
        ):
            await self._tick_graph()

    def _assert_rendervar_state(self, expected_compression: str) -> Any:
        """Assert the post-``compute()`` USD layout matches the SRTX contract.

        Args:
            expected_compression: Expected ``srtx:compression:type`` value.

        Returns:
            None.
        """
        stage = omni.usd.get_context().get_stage()
        rp_prim = stage.GetPrimAtPath(self.RENDER_PRODUCT_PATH)
        self.assertTrue(rp_prim.IsValid(), "Render product prim missing")

        rv_path = f"{self.RENDER_PRODUCT_PATH}/{self.AOV_NAME}"
        rv_prim = stage.GetPrimAtPath(rv_path)
        self.assertTrue(rv_prim.IsValid(), f"Expected RenderVar child at {rv_path}")
        self.assertEqual(rv_prim.GetTypeName(), "RenderVar")

        source_name_attr = rv_prim.GetAttribute("sourceName")
        self.assertTrue(source_name_attr, "RenderVar missing sourceName attribute")
        self.assertEqual(source_name_attr.Get(), self.AOV_NAME)

        ordered_vars_rel = rp_prim.GetRelationship("orderedVars")
        self.assertTrue(ordered_vars_rel, "Render product missing orderedVars relationship")
        self.assertIn(Sdf.Path(rv_path), ordered_vars_rel.GetTargets())

        compression_attr = rv_prim.GetAttribute(self.SRTX_COMPRESSION_ATTR)
        self.assertTrue(compression_attr, "RenderVar missing srtx:compression:type attribute")
        self.assertEqual(compression_attr.Get(), expected_compression)

    async def test_h264_mode_creates_rendervar_with_h264_compression(self) -> None:
        """Verify H.264 setup creates ``LdrColor`` with ``h264`` compression."""
        self._create_render_product()
        await self._run_helper(use_raw_encoding=False)
        self._assert_rendervar_state(expected_compression="h264")

    async def test_raw_mode_creates_rendervar_with_empty_compression(self) -> None:
        """Verify raw setup creates ``LdrColor`` with empty compression."""
        self._create_render_product()
        await self._run_helper(use_raw_encoding=True)
        self._assert_rendervar_state(expected_compression="")

    async def test_raw_mode_overwrites_preauthored_h264_compression(self) -> None:
        """Verify raw setup clears a pre-authored H.264 compression hint."""
        self._create_render_product_with_rendervar(preauthored_compression="h264")
        await self._run_helper(use_raw_encoding=True)
        self._assert_rendervar_state(expected_compression="")

    async def test_h264_mode_overwrites_preauthored_empty_compression(self) -> None:
        """Verify H.264 setup replaces an empty compression hint with ``h264``."""
        self._create_render_product_with_rendervar(preauthored_compression="")
        await self._run_helper(use_raw_encoding=False)
        self._assert_rendervar_state(expected_compression="h264")

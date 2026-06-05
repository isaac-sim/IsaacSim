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

from __future__ import annotations

import copy
import json
from datetime import datetime, timezone
from unittest.mock import MagicMock, patch

import numpy as np
import omni.kit.test
from isaacsim.streaming.rtsp.impl.rtsp_writer import _SEI_METADATA_UUID, WRITER_NAME, RTSPStreamWriter


class TestRTSPWriterInit(omni.kit.test.AsyncTestCase):
    """Tests for RTSPStreamWriter construction and parameter validation."""

    async def test_default_construction(self):
        writer = RTSPStreamWriter()
        self.assertEqual(writer._port, 8554)
        self.assertEqual(writer._mount_path, "/stream")
        self.assertEqual(writer._encoding, "h264")
        self.assertEqual(writer._configured_width, 1920)
        self.assertEqual(writer._configured_height, 1080)
        self.assertIsNone(writer._server)
        self.assertFalse(writer._stream_failed)
        self.assertEqual(writer._frame_num, 0)

    async def test_custom_construction(self):
        writer = RTSPStreamWriter(port=9000, mountPath="/cam1", encoding="raw", width=640, height=480)
        self.assertEqual(writer._port, 9000)
        self.assertEqual(writer._mount_path, "/cam1")
        self.assertEqual(writer._encoding, "raw")
        self.assertEqual(writer._configured_width, 640)
        self.assertEqual(writer._configured_height, 480)

    async def test_invalid_port_raises(self):
        for bad_port in (0, -1, 65536, 100000):
            with self.assertRaises(ValueError):
                RTSPStreamWriter(port=bad_port)

    async def test_boundary_ports_accepted(self):
        self.assertEqual(RTSPStreamWriter(port=1)._port, 1)
        self.assertEqual(RTSPStreamWriter(port=65535)._port, 65535)

    async def test_invalid_mount_path_raises(self):
        for bad_path in ("stream", "", "no-leading-slash"):
            with self.assertRaises(ValueError):
                RTSPStreamWriter(mountPath=bad_path)

    async def test_invalid_encoding_raises(self):
        for bad_enc in ("mjpeg", "", "H264", "RAW"):
            with self.assertRaises(ValueError):
                RTSPStreamWriter(encoding=bad_enc)

    async def test_h264_encoding_creates_annotators(self):
        writer = RTSPStreamWriter(encoding="h264")
        self.assertEqual(len(writer.annotators), 2)

    async def test_h264_requests_reset_on_stop_simulation_time_annotator(self):
        ldr_annotator = MagicMock()
        sim_time_annotator = MagicMock()
        with patch(
            "isaacsim.streaming.rtsp.impl.rtsp_writer.AnnotatorRegistry.get_annotator",
            side_effect=[ldr_annotator, sim_time_annotator],
        ) as mock_get_annotator:
            writer = RTSPStreamWriter(encoding="h264")

        self.assertEqual(writer.annotators, [ldr_annotator, sim_time_annotator])
        self.assertEqual(
            mock_get_annotator.call_args_list,
            [
                (("LdrColor",), {"init_params": {"compression": "h264"}}),
                (("IsaacReadSimulationTime",), {"init_params": {"resetOnStop": True}}),
            ],
        )

    async def test_raw_encoding_creates_annotator(self):
        writer = RTSPStreamWriter(encoding="raw")
        self.assertEqual(len(writer.annotators), 1)

    async def test_version_and_node_type_set(self):
        writer = RTSPStreamWriter()
        self.assertEqual(writer.version, "0.1.0")
        self.assertEqual(writer.node_type_id, WRITER_NAME)

    async def test_kwargs_captured(self):
        writer = RTSPStreamWriter(
            port=9000,
            mountPath="/test",
            encoding="raw",
            width=800,
            height=600,
            sensorSetName="ss-configured",
        )
        expected = {
            "port": 9000,
            "mountPath": "/test",
            "encoding": "raw",
            "width": 800,
            "height": 600,
            "sensorSetName": "ss-configured",
        }
        self.assertEqual(writer._kwargs, expected)

    async def test_annotators_list_synced(self):
        writer = RTSPStreamWriter()
        self.assertEqual(len(writer._annotators), len(writer.annotators))


class TestRTSPWriterDeepCopy(omni.kit.test.AsyncTestCase):
    """Tests for RTSPStreamWriter.__deepcopy__."""

    async def test_deepcopy_returns_new_instance(self):
        original = RTSPStreamWriter(port=9000, mountPath="/cam1", encoding="raw", width=640, height=480)
        cloned = copy.deepcopy(original)
        self.assertIsNot(original, cloned)
        self.assertIsInstance(cloned, RTSPStreamWriter)

    async def test_deepcopy_preserves_parameters(self):
        original = RTSPStreamWriter(port=9000, mountPath="/cam1", encoding="raw", width=640, height=480)
        cloned = copy.deepcopy(original)
        self.assertEqual(cloned._port, 9000)
        self.assertEqual(cloned._mount_path, "/cam1")
        self.assertEqual(cloned._encoding, "raw")
        self.assertEqual(cloned._configured_width, 640)
        self.assertEqual(cloned._configured_height, 480)

    async def test_deepcopy_resets_mutable_state(self):
        original = RTSPStreamWriter()
        original._frame_num = 42
        original._stream_failed = True
        cloned = copy.deepcopy(original)
        self.assertEqual(cloned._frame_num, 0)
        self.assertFalse(cloned._stream_failed)
        self.assertIsNone(cloned._server)


class TestRTSPWriterSrtxSensorSetInitParams(omni.kit.test.AsyncTestCase):
    """Tests for forwarding SRTX sensor-set names through annotator init params."""

    async def test_h264_encoding_adds_sensor_set_name_to_ldrcolor_init_params(self):
        """H.264 LdrColor annotator should receive compression and sensor-set init params."""
        ldr_annotator = MagicMock()
        sim_time_annotator = MagicMock()
        with patch(
            "isaacsim.streaming.rtsp.impl.rtsp_writer.AnnotatorRegistry.get_annotator",
            side_effect=[ldr_annotator, sim_time_annotator],
        ) as mock_get_annotator:
            writer = RTSPStreamWriter(encoding="h264", sensorSetName="ss-configured")

        self.assertEqual(writer.annotators, [ldr_annotator, sim_time_annotator])
        self.assertEqual(
            mock_get_annotator.call_args_list[0],
            (("LdrColor",), {"init_params": {"compression": "h264", "sensorSetName": "ss-configured"}}),
        )

    async def test_raw_encoding_adds_sensor_set_name_to_ldrcolor_init_params(self):
        """Raw LdrColor annotator should receive sensor-set init params with CUDA routing."""
        ldr_annotator = MagicMock()
        with patch(
            "isaacsim.streaming.rtsp.impl.rtsp_writer.AnnotatorRegistry.get_annotator",
            return_value=ldr_annotator,
        ) as mock_get_annotator:
            writer = RTSPStreamWriter(encoding="raw", sensorSetName="ss-configured")

        self.assertEqual(writer.annotators, [ldr_annotator])
        mock_get_annotator.assert_called_once_with(
            "LdrColor",
            init_params={"sensorSetName": "ss-configured"},
            device="cuda",
            do_array_copy=False,
        )


class TestRTSPWriterDetach(omni.kit.test.AsyncTestCase):
    """Tests for RTSPStreamWriter.detach."""

    async def test_detach_resets_stream_state(self):
        writer = RTSPStreamWriter()
        writer._stream_failed = True
        writer._frame_num = 10
        writer.detach()
        self.assertFalse(writer._stream_failed)
        self.assertEqual(writer._frame_num, 0)

    async def test_detach_stops_running_server(self):
        writer = RTSPStreamWriter()
        mock_server = MagicMock()
        writer._server = mock_server
        writer.detach()
        mock_server.stop.assert_called_once()
        mock_server.close.assert_called_once()
        self.assertIsNone(writer._server)

    async def test_detach_without_server_is_safe(self):
        writer = RTSPStreamWriter()
        self.assertIsNone(writer._server)
        writer.detach()
        self.assertIsNone(writer._server)


class TestRTSPWriterWrite(omni.kit.test.AsyncTestCase):
    """Tests for RTSPStreamWriter.write routing logic."""

    async def test_write_increments_frame_num(self):
        writer = RTSPStreamWriter(encoding="h264")
        with patch.object(writer, "_push_encoded_frame"):
            writer.write({"LdrColor": b"fake"})
            self.assertEqual(writer._frame_num, 1)
            writer.write({"LdrColor-rp123": b"fake"})
            self.assertEqual(writer._frame_num, 2)

    async def test_write_routes_h264_to_push_encoded_frame(self):
        writer = RTSPStreamWriter(encoding="h264")
        with patch.object(writer, "_push_encoded_frame") as mock_push:
            writer.write({"LdrColor": b"data"})
            mock_push.assert_called_once_with(b"data", 0.0)

    async def test_write_routes_raw_to_push_frame(self):
        writer = RTSPStreamWriter(encoding="raw")
        with patch.object(writer, "_push_frame") as mock_push:
            sentinel = object()
            writer.write({"LdrColor": sentinel})
            mock_push.assert_called_once_with(sentinel)

    async def test_write_ignores_non_ldrcolor_keys(self):
        writer = RTSPStreamWriter(encoding="h264")
        with patch.object(writer, "_push_encoded_frame") as mock_push:
            writer.write({"Normals": b"data", "Depth": b"data"})
            mock_push.assert_not_called()
            self.assertEqual(writer._frame_num, 0)

    async def test_write_matches_ldrcolor_with_render_product_suffix(self):
        writer = RTSPStreamWriter(encoding="h264")
        with patch.object(writer, "_push_encoded_frame") as mock_push:
            writer.write({"LdrColor-/Render/RenderProduct_001": b"data"})
            mock_push.assert_called_once()

    async def test_write_processes_only_first_ldrcolor_key(self):
        writer = RTSPStreamWriter(encoding="h264")
        with patch.object(writer, "_push_encoded_frame") as mock_push:
            writer.write({"LdrColor-rp1": b"a", "LdrColor-rp2": b"b"})
            mock_push.assert_called_once()
            self.assertEqual(writer._frame_num, 1)

    async def test_stream_failed_skips_h264_push(self):
        writer = RTSPStreamWriter(encoding="h264")
        writer._stream_failed = True
        with patch.object(writer, "_start_server") as mock_start:
            writer._push_encoded_frame(np.array([1, 2, 3], dtype=np.uint8))
            mock_start.assert_not_called()

    async def test_stream_failed_skips_raw_push(self):
        writer = RTSPStreamWriter(encoding="raw")
        writer._stream_failed = True
        with patch.object(writer, "_start_server") as mock_start:
            writer._push_frame(MagicMock())
            mock_start.assert_not_called()


class TestRTSPWriterServerLifecycle(omni.kit.test.AsyncTestCase):
    """Tests for RTSP server start/stop lifecycle."""

    async def test_stop_server_noop_when_none(self):
        writer = RTSPStreamWriter()
        writer._stop_server()
        self.assertIsNone(writer._server)

    async def test_stop_server_clears_state(self):
        writer = RTSPStreamWriter()
        mock_server = MagicMock()
        writer._server = mock_server
        writer._width = 1920
        writer._height = 1080
        writer._stop_server()
        self.assertEqual(writer._width, 0)
        self.assertEqual(writer._height, 0)
        self.assertIsNone(writer._server)
        mock_server.stop.assert_called_once()
        mock_server.close.assert_called_once()

    async def test_h264_push_starts_server_lazily(self):
        writer = RTSPStreamWriter(encoding="h264", width=1280, height=720)
        self.assertIsNone(writer._server)

        def _start_server(width, height):
            writer._width = width
            writer._height = height
            writer._base_timestamp = datetime(2025, 1, 1, tzinfo=timezone.utc)
            writer._server = MagicMock()
            writer._server.stream_video_pre_encoded_with_metadata.return_value = True

        with patch.object(writer, "_start_server", side_effect=_start_server) as mock_start:
            writer._push_encoded_frame(np.array([1, 2, 3], dtype=np.uint8), sim_time_s=0.0)
            mock_start.assert_called_once_with(1280, 720)

    async def test_h264_push_skips_empty_data(self):
        writer = RTSPStreamWriter(encoding="h264")
        with patch.object(writer, "_start_server") as mock_start:
            writer._push_encoded_frame(np.array([], dtype=np.uint8))
            mock_start.assert_not_called()

    async def test_h264_push_failure_sets_stream_failed(self):
        writer = RTSPStreamWriter(encoding="h264", width=1280, height=720)
        mock_server = MagicMock()
        mock_server.stream_video_pre_encoded_with_metadata.side_effect = RuntimeError("connection lost")
        writer._server = mock_server
        writer._width = 1280
        writer._height = 720
        writer._base_timestamp = datetime(2025, 1, 1, tzinfo=timezone.utc)
        with patch("isaacsim.streaming.rtsp.impl.rtsp_writer.carb.log_error") as mock_log_error:
            writer._push_encoded_frame(np.array([1, 2, 3], dtype=np.uint8))
        mock_log_error.assert_called_once()
        self.assertTrue(writer._stream_failed)
        self.assertIsNone(writer._server)

    async def test_raw_push_failure_sets_stream_failed(self):
        writer = RTSPStreamWriter(encoding="raw")
        mock_server = MagicMock()
        mock_server.stream_video_cuda_buffer.side_effect = RuntimeError("cuda error")
        writer._server = mock_server
        writer._width = 640
        writer._height = 480
        mock_cuda = MagicMock()
        mock_cuda.shape = (480, 640, 4)
        mock_cuda.ptr = 0x12345
        with patch("isaacsim.streaming.rtsp.impl.rtsp_writer.carb.log_error") as mock_log_error:
            writer._push_frame(mock_cuda)
        mock_log_error.assert_called_once()
        self.assertTrue(writer._stream_failed)
        self.assertIsNone(writer._server)

    async def test_raw_push_starts_server_from_cuda_shape(self):
        writer = RTSPStreamWriter(encoding="raw")
        self.assertIsNone(writer._server)
        mock_cuda = MagicMock()
        mock_cuda.shape = (480, 640, 4)
        mock_cuda.ptr = 0x12345

        def _start_server(width, height):
            writer._width = width
            writer._height = height
            writer._server = MagicMock()
            writer._server.stream_video_cuda_buffer.return_value = True

        with patch.object(writer, "_start_server", side_effect=_start_server) as mock_start:
            writer._push_frame(mock_cuda)
            mock_start.assert_called_once_with(640, 480)


class TestRTSPWriterMetadata(omni.kit.test.AsyncTestCase):
    """Tests for SEI metadata construction and injection."""

    async def test_build_sei_metadata_payload_structure(self):
        writer = RTSPStreamWriter(encoding="h264")
        writer._base_timestamp = datetime(2025, 6, 15, 10, 0, 0, tzinfo=timezone.utc)
        writer._frame_num = 5
        metadata = writer._build_sei_metadata(2_000_000_000)
        payload = json.loads(metadata)
        self.assertEqual(payload["publish_sim_time_ns"], 2_000_000_000)
        self.assertEqual(payload["frame_num"], 5)
        self.assertEqual(payload["timestamp_iso8601"], "2025-06-15T10:00:02.000Z")
        self.assertIsInstance(payload["timestamp"], int)
        self.assertGreater(payload["timestamp"], 0)

    async def test_build_sei_metadata_zero_sim_time(self):
        writer = RTSPStreamWriter(encoding="h264")
        writer._base_timestamp = datetime(2025, 1, 1, 0, 0, 0, tzinfo=timezone.utc)
        writer._frame_num = 1
        metadata = writer._build_sei_metadata(0)
        payload = json.loads(metadata)
        self.assertEqual(payload["publish_sim_time_ns"], 0)
        self.assertEqual(payload["timestamp_iso8601"], "2025-01-01T00:00:00.000Z")

    async def test_build_sei_metadata_raises_when_no_base_timestamp(self):
        writer = RTSPStreamWriter(encoding="h264")
        writer._frame_num = 1
        self.assertIsNone(writer._base_timestamp)
        with self.assertRaises(RuntimeError):
            writer._build_sei_metadata(0)

    async def test_write_extracts_simulation_time(self):
        writer = RTSPStreamWriter(encoding="h264")
        with patch.object(writer, "_push_encoded_frame") as mock_push:
            writer.write(
                {
                    "IsaacReadSimulationTime-rp1": {"simulationTime": 1.5},
                    "LdrColor-rp1": b"data",
                }
            )
            mock_push.assert_called_once_with(b"data", 1.5)

    async def test_write_handles_missing_sim_time(self):
        writer = RTSPStreamWriter(encoding="h264")
        with patch.object(writer, "_push_encoded_frame") as mock_push:
            writer.write({"LdrColor": b"data"})
            mock_push.assert_called_once_with(b"data", 0.0)

    async def test_h264_push_calls_metadata_api(self):
        writer = RTSPStreamWriter(encoding="h264", width=1280, height=720)
        mock_server = MagicMock()
        mock_server.stream_video_pre_encoded_with_metadata.return_value = True
        writer._server = mock_server
        writer._width = 1280
        writer._height = 720
        writer._base_timestamp = datetime(2025, 1, 1, tzinfo=timezone.utc)
        writer._frame_num = 1

        writer._push_encoded_frame(np.array([1, 2, 3], dtype=np.uint8), sim_time_s=2.0)

        mock_server.stream_video_pre_encoded_with_metadata.assert_called_once()
        call_kwargs = mock_server.stream_video_pre_encoded_with_metadata.call_args
        self.assertEqual(call_kwargs.kwargs["start_time_ns"], 2_000_000_000)
        self.assertEqual(call_kwargs.kwargs["ended_time_ns"], 2_000_000_000)
        self.assertEqual(call_kwargs.kwargs["metadata_uuid"], _SEI_METADATA_UUID)
        payload = json.loads(call_kwargs.kwargs["metadata"])
        self.assertEqual(payload["publish_sim_time_ns"], 2_000_000_000)
        self.assertEqual(payload["frame_num"], 1)

    async def test_stop_server_resets_base_timestamp(self):
        writer = RTSPStreamWriter(encoding="h264")
        mock_server = MagicMock()
        writer._server = mock_server
        writer._base_timestamp = datetime(2025, 1, 1, tzinfo=timezone.utc)
        writer._stop_server()
        self.assertIsNone(writer._base_timestamp)

    async def test_sei_metadata_uuid_is_16_bytes(self):
        self.assertEqual(len(_SEI_METADATA_UUID), 16)

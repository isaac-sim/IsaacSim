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

"""Replicator Writer that streams LdrColor frames over RTSP."""

from __future__ import annotations

import json
import uuid
from datetime import datetime, timedelta, timezone
from typing import Literal, Optional

import carb
import numpy as np
import warp as wp
from omni.kit.livestream.core import Server, ServerConfig, VideoCodec
from omni.replicator.core import AnnotatorRegistry, Writer

WRITER_NAME = "RTSPStreamWriter"
_BYTES_PER_PIXEL_RGBA = 4
_SEI_METADATA_UUID = uuid.uuid5(uuid.NAMESPACE_DNS, "isaacsim.streaming.rtsp.sei_metadata").bytes

_RTSPWriterEncoding = Literal["raw", "h264"]


class RTSPStreamWriter(Writer):
    """Replicator Writer that streams LdrColor frames over RTSP.

    Supports two modes controlled by the ``encoding`` parameter:

    - ``"h264"`` (default): The LdrColor annotator is requested with
      ``init_params={"compression": "h264"}`` so the render pipeline
      produces H.264-encoded bytes. These are passed to
      ``stream_video_pre_encoded_with_metadata``, and also have the ability
      to enable per-frame SEI metadata injection.

    - ``"raw"``: The LdrColor annotator delivers a CUDA RGBA buffer which is
      passed to ``stream_video_cuda_buffer``. The kit-livestream RTSP backend
      handles encoding internally. Metadata injection is not supported in this
      mode.

    The RTSP server is started lazily on the first rendered frame. Calling
    :meth:`detach` (which happens automatically when ``BaseWriterNode`` resets
    on timeline stop) shuts the server down cleanly.

    Args:
        port: RTSP server port (1 to 65535).  Each simultaneous stream needs a unique port.
        mountPath: RTSP mount path (e.g. ``/stream``); must start with ``/``.
        encoding: ``"h264"`` (default) for pre-encoded H.264 with
            metadata support, ``"raw"`` for uncompressed CUDA path.
        width: Frame width in pixels.  Used to configure the RTSP server
            when encoding is ``"h264"`` (the encoded byte-stream does not
            carry resolution).  Ignored when encoding is ``"raw"`` since
            the resolution is read from the CUDA buffer shape.
        height: Frame height in pixels.  See ``width``.
        sensorSetName: Optional SRTX sensor-set name passed to the LdrColor
            annotator through ``init_params``.

    Raises:
        ValueError: If ``port``, ``mountPath``, or ``encoding`` is invalid.
    """

    def __init__(
        self,
        port: int = 8554,
        mountPath: str = "/stream",
        encoding: _RTSPWriterEncoding = "h264",
        width: int = 1920,
        height: int = 1080,
        *,
        sensorSetName: Optional[str] = None,
    ) -> None:
        # node_type_id, _kwargs, and _annotators are required by
        # WriterRequest.__repr__ in BaseWriterNode, which logs writer
        # details when attaching to a render product.  Python Writer
        # subclasses registered via WriterRegistry.register do not get
        # these set automatically, so we provide them ourselves.
        if port < 1 or port > 65535:
            raise ValueError(f"port must be between 1 and 65535 inclusive, got {port}")
        if not mountPath.startswith("/"):
            raise ValueError(f"mountPath must start with '/', got {mountPath!r}")
        if encoding not in ("raw", "h264"):
            raise ValueError(f"encoding must be 'raw' or 'h264', got '{encoding}'")

        self.version = "0.1.0"
        self.node_type_id = WRITER_NAME
        self._kwargs = {"port": port, "mountPath": mountPath, "encoding": encoding, "width": width, "height": height}
        if sensorSetName:
            self._kwargs["sensorSetName"] = sensorSetName
        self._configured_width = width
        self._configured_height = height
        self._encoding: _RTSPWriterEncoding = encoding

        ldr_init_params = {}
        if encoding == "h264":
            ldr_init_params["compression"] = "h264"
        if sensorSetName:
            ldr_init_params["sensorSetName"] = sensorSetName

        if encoding == "raw":
            annotator_kwargs = {"device": "cuda", "do_array_copy": False}
            if ldr_init_params:
                annotator_kwargs["init_params"] = ldr_init_params
            self.annotators = [AnnotatorRegistry.get_annotator("LdrColor", **annotator_kwargs)]
        elif encoding == "h264":
            self.annotators = [
                AnnotatorRegistry.get_annotator("LdrColor", init_params=ldr_init_params),
                AnnotatorRegistry.get_annotator("IsaacReadSimulationTime", init_params={"resetOnStop": True}),
            ]
        else:
            raise ValueError(
                f"RTSP writer has no annotator configuration for encoding {encoding!r}; "
                "supported values are 'raw' and 'h264'"
            )

        self._annotators = list(self.annotators)
        self._port = port
        self._mount_path = mountPath
        self._server = None
        self._width: int = 0
        self._height: int = 0
        self._stream_failed = False
        self._frame_num: int = 0
        self._base_timestamp: datetime | None = None

    def __deepcopy__(self, memo: dict) -> "RTSPStreamWriter":
        """Return a fresh instance instead of cloning.

        ``BaseWriterNode.append_writer`` deep-copies writers before storing
        them.  The default deep-copy fails because the LdrColor annotator
        holds C++/pybind11 binding objects that do not support Python's
        pickle/copy protocol.  Constructing a new instance sidesteps this
        by obtaining a fresh annotator from the registry.
        """
        return RTSPStreamWriter(
            port=self._port,
            mountPath=self._mount_path,
            encoding=self._encoding,
            width=self._configured_width,
            height=self._configured_height,
            sensorSetName=self._kwargs.get("sensorSetName"),
        )

    def write(self, data: dict) -> None:
        """Called by Replicator each frame with annotator data.

        The data dict is keyed by annotator name.  Replicator appends a
        render-product suffix to the key (e.g. ``"LdrColor-<rp_name>"``),
        so we match with ``startswith`` rather than an exact lookup.
        """
        sim_time_s = 0.0
        for key in data:
            if key.startswith("IsaacReadSimulationTime"):
                sim_time_s = data[key]["simulationTime"]
                break

        for key in data:
            if key.startswith("LdrColor"):
                self._frame_num += 1
                if self._encoding == "raw":
                    self._push_frame(data[key])
                elif self._encoding == "h264":
                    self._push_encoded_frame(data[key], sim_time_s)
                else:
                    raise ValueError(
                        f"RTSP writer cannot route frame for encoding {self._encoding!r}; "
                        "supported values are 'raw' and 'h264'"
                    )
                return

    def detach(self) -> None:
        """Stop the RTSP server and detach from the render product."""
        self._stop_server()
        self._stream_failed = False
        self._frame_num = 0
        super().detach()

    def _push_encoded_frame(self, frame_data: np.ndarray, sim_time_s: float = 0.0) -> None:
        """Push a pre-encoded H.264 frame with SEI metadata to the RTSP server.

        The compressed LdrColor annotator delivers a 1-D ``numpy.ndarray``
        of ``uint8`` containing the H.264 byte-stream.  Empty arrays
        (encoder warm-up) are silently skipped.

        Each frame is accompanied by SEI metadata containing the simulation
        timestamp, a wall-clock-anchored ISO 8601 timestamp, and the frame
        number.

        On failure the server is torn down and ``_stream_failed`` is set,
        which causes all subsequent calls to short-circuit silently until
        :meth:`detach` clears the flag.

        Args:
            frame_data: H.264 encoded bytes from the LdrColor annotator.
            sim_time_s: Simulation time in seconds from ``IsaacReadSimulationTime``.
        """
        if self._stream_failed:
            return

        try:
            encoded_bytes = frame_data.tobytes()
            if not encoded_bytes:
                return

            if self._server is None:
                self._start_server(self._configured_width, self._configured_height)

            sim_time_ns = int(sim_time_s * 1e9)
            metadata = self._build_sei_metadata(sim_time_ns)
            self._server.stream_video_pre_encoded_with_metadata(
                encoded_bytes,
                len(encoded_bytes),
                self._width,
                self._height,
                start_time_ns=sim_time_ns,
                ended_time_ns=sim_time_ns,
                metadata=metadata,
                metadata_uuid=_SEI_METADATA_UUID,
            )
        except Exception as e:
            self._stop_server()
            self._stream_failed = True
            carb.log_error(
                f"RTSP stream (h264) failed on port {self._port}: {e}. Suppressing further attempts until timeline restart"
            )

    def _push_frame(self, cuda_data: wp.array) -> None:
        """Push a single raw CUDA frame to the RTSP server.

        On failure the server is torn down and ``_stream_failed`` is set,
        which causes all subsequent calls to short-circuit silently until
        :meth:`detach` clears the flag.
        """
        if self._stream_failed:
            return

        try:
            if self._server is None:
                height, width = cuda_data.shape[0], cuda_data.shape[1]
                self._start_server(width, height)

            ptr = cuda_data.ptr
            pitch_bytes = self._width * _BYTES_PER_PIXEL_RGBA
            self._server.stream_video_cuda_buffer(ptr, pitch_bytes, self._width, self._height)
        except Exception as e:
            self._stop_server()
            self._stream_failed = True
            carb.log_error(
                f"RTSP stream failed on port {self._port}: {e}. Suppressing further attempts until timeline restart"
            )

    def _build_sei_metadata(self, sim_time_ns: int) -> bytes:
        """Build the JSON SEI metadata payload for a single frame.

        The payload uses a simple schema with simulation time, wall-clock
        timestamp, and frame number, allowing downstream analytics
        consumers to correlate streamed frames.

        Args:
            sim_time_ns: Simulation time in nanoseconds for this frame.

        Returns:
            UTF-8 encoded JSON bytes.
        """
        if self._base_timestamp is None:
            raise RuntimeError("_build_sei_metadata called before _start_server set _base_timestamp")
        base = self._base_timestamp
        frame_dt = base + timedelta(seconds=sim_time_ns / 1e9)
        iso8601 = frame_dt.isoformat(timespec="milliseconds").replace("+00:00", "Z")
        ns_since_epoch = int(frame_dt.timestamp() * 1e9)

        payload = {
            "publish_sim_time_ns": sim_time_ns,
            "timestamp_iso8601": iso8601,
            "timestamp": ns_since_epoch,
            "frame_num": self._frame_num,
        }
        return json.dumps(payload).encode("utf-8")

    def _start_server(self, width: int, height: int) -> None:
        """Create and start the kit-livestream RTSP server.

        Args:
            width: Frame width in pixels.
            height: Frame height in pixels.
        """
        self._width = width
        self._height = height
        self._base_timestamp = datetime.now(timezone.utc)
        self._server = Server("rtsp")
        if self._encoding == "h264":
            video_pre_encode = VideoCodec.H264
        elif self._encoding == "raw":
            video_pre_encode = VideoCodec.NONE
        else:
            raise ValueError(
                f"RTSP writer has no VideoCodec mapping for encoding {self._encoding!r}; "
                "supported values are 'raw' and 'h264'"
            )
        config = ServerConfig(
            width=self._width,
            height=self._height,
            stream_port=self._port,
            rtsp_mount_point=self._mount_path,
            video_pre_encode=video_pre_encode,
        )
        self._server.start(config)
        carb.log_info(
            f"RTSP stream started on rtsp://localhost:{self._port}{self._mount_path} "
            f"({self._width}x{self._height}, encoding={self._encoding})"
        )

    def _stop_server(self) -> None:
        """Stop and close the RTSP server, releasing the port.

        Safe to call when no server is running (no-op).
        """
        if self._server is not None:
            self._server.stop()
            self._server.close()
            self._server = None
            self._width = 0
            self._height = 0
            self._base_timestamp = None
            carb.log_info(f"RTSP server stopped on port {self._port}")

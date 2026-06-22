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

"""OmniGraph helper that wires a camera render product to an RTSP writer."""

from __future__ import annotations

import traceback
from typing import Optional

import carb
import omni.graph.core as og
import omni.usd
from isaacsim.core.nodes import BaseWriterNode
from isaacsim.streaming.rtsp.impl.render_var_utils import ensure_render_var_on_product
from isaacsim.streaming.rtsp.impl.rtsp_writer import RTSPStreamWriter
from pxr import Usd

# AOVs that `RTSPStreamWriter` requests via `AnnotatorRegistry.get_annotator`.
#
# For each entry, the helper materialises a matching `RenderVar` child prim
# under the render product before attaching the writer, so that SRTX's
# `AnnotatorSRTX.attach()` validation (which requires the rendervar to exist
# as a child of the render product) succeeds.
_WRITER_AOVS: tuple[str, ...] = ("LdrColor",)


def _resolve_srtx_sensor_set_name(render_product_path: str) -> Optional[str]:
    """Resolve the SRTX sensor-set name for a render product, or ``None``.

    Args:
        render_product_path: USD path of the render product prim.

    Returns:
        SRTX sensor-set name for the render product, or ``None`` if SRTX
        routing is unavailable.
    """
    try:
        from omni.replicator.srtx import resolve_sensor_set_name_for_render_product
    except ImportError:
        return None
    return resolve_sensor_set_name_for_render_product(render_product_path)


class OgnRTSPCameraHelperInternalState(BaseWriterNode):
    """Per-instance state for the RTSPCameraHelper OmniGraph node.

    Inherits from ``BaseWriterNode`` which manages a list of Replicator
    writers and handles attach/detach/reset lifecycle.  Setup is deferred
    to ``compute``.
    """

    def __init__(self) -> None:
        super().__init__(initialize=False)


class OgnRTSPCameraHelper:
    """Create and attach an RTSP writer from RTSPCameraHelper graph inputs."""

    @staticmethod
    def internal_state() -> OgnRTSPCameraHelperInternalState:
        """OGN framework callback: create per-instance state for this node.

        Returns:
            New RTSP camera helper internal state.
        """
        return OgnRTSPCameraHelperInternalState()

    @staticmethod
    def compute(db: og.Database) -> bool:
        """Set up RTSP streaming from the node's graph inputs.

        Reads ``renderProductPath``, ``port``, ``mountPath``, ``useRawEncoding``,
        and ``enabled`` from the OmniGraph database.  Setup is skipped when the
        node is disabled, already initialized, or the render product path cannot
        be resolved.  For a valid render product, the helper ensures the writer's
        required ``LdrColor`` ``RenderVar`` exists, authors the SRTX compression
        hint used by H.264 versus raw routing, constructs ``RTSPStreamWriter``,
        and attaches it to the render product.

        Returns ``True`` on success or when already initialized, ``False``
        when prerequisites aren't met.  The return value is purely
        diagnostic, since this is a leaf node and there is no subsequent evaluation.

        Args:
            db: OmniGraph database containing node inputs and per-instance state.

        Returns:
            ``True`` when setup succeeds or no setup is needed, otherwise ``False``.
        """
        state = db.per_instance_state
        if not db.inputs.enabled:
            if state.initialized:
                state.custom_reset()
            return True

        if state.initialized:
            return True

        render_product_path = db.inputs.renderProductPath
        if not render_product_path:
            carb.log_warn("RTSPCameraHelper: renderProductPath is empty, skipping setup")
            return False

        stage = omni.usd.get_context().get_stage()
        rp_prim = stage.GetPrimAtPath(render_product_path) if stage is not None else None
        if stage is None or not rp_prim or not rp_prim.IsValid():
            carb.log_warn(
                f"RTSPCameraHelper: render product '{render_product_path}' does not exist yet, skipping setup"
            )
            return False

        port = db.inputs.port
        mount_path = db.inputs.mountPath

        if port < 1 or port > 65535:
            carb.log_error(f"Invalid RTSP port {port}, must be 1-65535")
            return False
        if not mount_path.startswith("/"):
            carb.log_error(f"RTSP mount path must start with '/', got '{mount_path}'")
            return False

        encoding = "raw" if db.inputs.useRawEncoding else "h264"
        # SRTX server compression hint authored on the rendervar prim.
        # Empty string is the canonical "raw / no compression" signal;
        # "h264" routes through NVENC.
        compression_type = "" if db.inputs.useRawEncoding else "h264"

        resolution = rp_prim.GetAttribute("resolution").Get()
        if not resolution:
            carb.log_error(f"RTSPCameraHelper: render product '{render_product_path}' has no resolution attribute")
            return False
        width, height = int(resolution[0]), int(resolution[1])

        try:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                for aov in _WRITER_AOVS:
                    success, _ = ensure_render_var_on_product(stage, render_product_path, aov, compression_type)
                    if not success:
                        carb.log_error(
                            f"RTSPCameraHelper: failed to materialise RenderVar '{aov}' "
                            f"under '{render_product_path}'"
                        )
                        return False

                writer = RTSPStreamWriter(
                    port=port,
                    mountPath=mount_path,
                    encoding=encoding,
                    width=width,
                    height=height,
                    sensorSetName=_resolve_srtx_sensor_set_name(render_product_path),
                )
                state.append_writer(writer)
                state.attach_writers(render_product_path)
        except Exception:
            carb.log_error(f"Failed to set up RTSP writer: {traceback.format_exc()}")
            return False

        state.initialized = True
        return True

    @staticmethod
    def release_instance(node: og.Node, graph_instance_id: int) -> None:
        """OGN framework callback: clean up when a graph instance is destroyed.

        Retrieves the per-instance state and calls ``reset()``, which
        detaches all writers (stopping any running RTSP servers).  The
        lookup is guarded because the state may not exist if the node
        was never computed.

        Args:
            node: OmniGraph node whose instance state is being released.
            graph_instance_id: Identifier of the graph instance being released.
        """
        try:
            state = OgnRTSPCameraHelperInternalState.per_instance_internal_state(node)
        except Exception:
            state = None

        if state is not None:
            state.reset()

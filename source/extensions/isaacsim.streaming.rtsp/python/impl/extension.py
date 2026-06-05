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

"""Extension startup/shutdown: registers RTSPStreamWriter with Replicator."""

from __future__ import annotations

import carb
import omni.ext
import omni.replicator.core as rep

from .rtsp_writer import WRITER_NAME, RTSPStreamWriter


class RTSPStreamingExtension(omni.ext.IExt):
    """Registers the RTSPStreamWriter writer with Replicator on startup."""

    def on_startup(self, ext_id: str) -> None:
        """Register the RTSPStreamWriter writer with Replicator's WriterRegistry.

        The writer is also appended to ``_default_writers`` so that
        Replicator includes it in telemetry tracking.
        """
        rep.WriterRegistry.register(RTSPStreamWriter)
        if WRITER_NAME not in rep.WriterRegistry._default_writers:
            rep.WriterRegistry._default_writers.append(WRITER_NAME)
        carb.log_info(f"RTSP Nodes: registered '{WRITER_NAME}' writer")

    def on_shutdown(self) -> None:
        """Unregister the writer and remove it from the telemetry list."""
        rep.writers.unregister_writer(WRITER_NAME)
        if WRITER_NAME in rep.WriterRegistry._default_writers:
            rep.WriterRegistry._default_writers.remove(WRITER_NAME)

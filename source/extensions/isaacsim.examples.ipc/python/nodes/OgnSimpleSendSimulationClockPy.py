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
"""Python implementation of the SimpleSendSimulationClock OmniGraph node."""

import socket
import struct

import omni.graph.core as og
from isaacsim.core.nodes import BaseResetNode


class OgnSimpleSendSimulationClockPyInternalState(BaseResetNode):
    """Per-instance TCP client state for sending simulation clock values."""

    def __init__(self) -> None:
        self.sock = None
        self.uri = ""
        super().__init__(initialize=False)

    def custom_reset(self) -> None:
        """Close the socket and reset the cached URI."""
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        self.uri = ""


class OgnSimpleSendSimulationClockPy:
    """Send simulation time over TCP as signed nanoseconds."""

    @staticmethod
    def internal_state() -> OgnSimpleSendSimulationClockPyInternalState:
        """Create per-instance state for the node."""
        return OgnSimpleSendSimulationClockPyInternalState()

    @staticmethod
    def compute(db: object) -> bool:
        """Send one simulation clock value without changing graph behavior."""
        state = db.per_instance_state
        uri = db.inputs.uri
        if state.sock is not None and state.uri != uri:
            state.custom_reset()

        if state.sock is None:
            try:
                host, port_str = uri.rsplit(":", 1)
                port = int(port_str)
            except ValueError:
                db.outputs.execOut = (
                    og.ExecutionAttributeState.ENABLED
                )  # pulse execOut even on failure so downstream nodes keep running
                return False
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((host, port))
            except OSError:
                s.close()
                db.outputs.execOut = (
                    og.ExecutionAttributeState.ENABLED
                )  # pulse execOut even on failure so downstream nodes keep running
                return False
            state.sock = s
            state.uri = uri

        sec = float(db.inputs.simulationTime)
        payload = int(round(sec * 1e9))
        try:
            state.sock.sendall(struct.pack("<q", payload))
        except OSError:
            state.custom_reset()
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return False

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True

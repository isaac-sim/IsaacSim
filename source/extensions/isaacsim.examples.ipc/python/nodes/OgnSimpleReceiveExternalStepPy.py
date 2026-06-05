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
"""Python implementation of the SimpleReceiveExternalStep OmniGraph node."""

import socket
import struct

import omni.graph.core as og
from isaacsim.core.nodes import BaseResetNode


class OgnSimpleReceiveExternalStepPyInternalState(BaseResetNode):
    """Per-instance TCP server state for receiving external step values."""

    def __init__(self) -> None:
        self.listen_sock = None
        self.client_sock = None
        self.buf = bytearray()
        self.uri = ""
        super().__init__(initialize=False)

    def custom_reset(self) -> None:
        """Close sockets and clear buffered input."""
        if self.client_sock is not None:
            try:
                self.client_sock.close()
            except OSError:
                pass
            self.client_sock = None
        if self.listen_sock is not None:
            try:
                self.listen_sock.close()
            except OSError:
                pass
            self.listen_sock = None
        self.buf.clear()
        self.uri = ""


class OgnSimpleReceiveExternalStepPy:
    """Receive a uint32 simulation step over TCP."""

    @staticmethod
    def internal_state() -> OgnSimpleReceiveExternalStepPyInternalState:
        """Create per-instance state for the node."""
        return OgnSimpleReceiveExternalStepPyInternalState()

    @staticmethod
    def compute(db: object) -> bool:
        """Accept and read one external step value without blocking."""
        state = db.per_instance_state
        uri = db.inputs.uri
        if state.listen_sock is not None and state.uri != uri:
            state.custom_reset()

        if state.listen_sock is None:
            try:
                host, port_str = uri.rsplit(":", 1)
                port = int(port_str)
            except ValueError:
                return False
            try:
                ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                ls.bind((host, port))
                ls.listen(1)
                ls.setblocking(False)  # non-blocking so accept() returns immediately when no client is waiting
            except OSError:
                ls.close()
                return False
            state.listen_sock = ls
            state.uri = uri

        if state.client_sock is None:
            try:
                client, _ = state.listen_sock.accept()
            except BlockingIOError:
                return False  # no client waiting yet; retry next evaluation
            except OSError:
                return False  # listen socket error
            client.setblocking(False)
            state.client_sock = client
            state.buf.clear()

        try:
            chunk = state.client_sock.recv(4 - len(state.buf))
        except BlockingIOError:
            return False  # no data yet; partial buffer preserved, retry next evaluation
        except OSError:
            state.client_sock.close()
            state.client_sock = None
            state.buf.clear()
            return False

        if not chunk:
            state.client_sock.close()
            state.client_sock = None
            state.buf.clear()
            return False

        state.buf.extend(chunk)
        if len(state.buf) < 4:
            return False

        (step,) = struct.unpack("<I", state.buf)
        state.buf.clear()
        db.outputs.step = step
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True

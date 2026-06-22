# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for isaacsim.ucx.core Python bindings."""

import os

import ucxx._lib.libucxx as ucx_api
from isaacsim.test.utils import TimedAsyncTestCase
from isaacsim.ucx.core import (
    UCXListener,
    add_listener,
    is_listener_registered,
    remove_listener,
)

CONNECT_TIMEOUT_MS = 5000


class TestUcxCoreBindings(TimedAsyncTestCase):
    """Tests for the UCX Core Python bindings."""

    async def setUp(self) -> None:
        """Set up the test fixture."""
        await super().setUp()
        os.environ["UCX_TLS"] = "tcp,self"
        os.environ["UCX_NET_DEVICES"] = "all"

        self._listener = None
        self._client_context = None
        self._client_worker = None
        self._client_endpoint = None

    async def tearDown(self) -> None:
        """Tear down the test fixture."""
        if self._client_worker:
            try:
                self._client_worker.stop_progress_thread()
            except Exception:
                pass
        self._client_endpoint = None
        self._client_worker = None
        self._client_context = None

        if self._listener:
            port = self._listener.get_port()
            try:
                self._listener.shutdown()
            except Exception:
                pass
            self._listener = None
            try:
                remove_listener(port)
            except Exception:
                pass

        await super().tearDown()

    def _connect_client(self, port: int) -> None:
        """Connect a UCXX client to the given port.

        Args:
            port: TCP port for the UCX listener.
        """
        self._client_context = ucx_api.UCXContext()
        self._client_worker = ucx_api.UCXWorker(self._client_context)
        self._client_endpoint = ucx_api.UCXEndpoint.create(
            self._client_worker, "127.0.0.1", port, endpoint_error_handling=True
        )
        self._client_worker.start_progress_thread()

    async def test_imports(self) -> None:
        """Verify all public symbols are importable from isaacsim.ucx.core."""
        self.assertIsNotNone(UCXListener)
        self.assertIsNotNone(add_listener)
        self.assertIsNotNone(remove_listener)
        self.assertIsNotNone(is_listener_registered)

    async def test_add_listener_auto_port(self) -> None:
        """add_listener(0) assigns an ephemeral port."""
        self._listener = add_listener(0)
        self.assertIsInstance(self._listener, UCXListener)
        self.assertGreater(self._listener.get_port(), 0)

    async def test_add_listener_fixed_port(self) -> None:
        """add_listener returns the same listener object for the same port."""
        self._listener = add_listener(0)
        port = self._listener.get_port()
        same = add_listener(port)
        self.assertEqual(same.get_port(), port)

    async def test_is_not_connected_initially(self) -> None:
        """A fresh listener reports not connected before any client joins."""
        self._listener = add_listener(0)
        self.assertFalse(self._listener.is_connected())

    async def test_wait_for_connection_timeout(self) -> None:
        """wait_for_connection returns False when no client connects within the timeout."""
        self._listener = add_listener(0)
        self.assertFalse(self._listener.wait_for_connection(timeout_ms=200))
        self.assertFalse(self._listener.is_connected())

    async def test_is_listener_registered(self) -> None:
        """is_listener_registered reflects the registry state correctly."""
        self._listener = add_listener(0)
        port = self._listener.get_port()
        self.assertTrue(is_listener_registered(port))

        self._listener.shutdown()
        remove_listener(port)
        self._listener = None
        self.assertFalse(is_listener_registered(port))

    async def test_shutdown_and_remove(self) -> None:
        """shutdown() + remove_listener() clean up without error."""
        listener = add_listener(0)
        port = listener.get_port()
        listener.shutdown()
        remove_listener(port)
        self.assertFalse(is_listener_registered(port))

    async def test_connection_established(self) -> None:
        """A client connecting to the listener sets is_connected() to True."""
        self._listener = add_listener(0)

        self._connect_client(self._listener.get_port())

        connected = self._listener.wait_for_connection(timeout_ms=CONNECT_TIMEOUT_MS)
        self.assertTrue(connected, "Listener should report connected after client joins")
        self.assertTrue(self._listener.is_connected())

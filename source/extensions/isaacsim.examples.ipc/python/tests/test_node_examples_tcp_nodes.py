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

"""Integration tests for TCP clock/step OmniGraph nodes (C++ and Python).

Send nodes take ``simulationTime`` in **seconds** (``double``)—the same attribute as
``isaacsim.core.nodes.IsaacReadSimulationTime``—not raw nanoseconds.
"""

import math
import socket
import struct
import threading
import time

import omni
import omni.graph.core as og
import omni.kit.test


def _llround_seconds_to_ns(sec: float) -> int:
    """Match C++ ``std::llround(sec * 1e9)`` (half away from zero)."""
    x = sec * 1e9
    if x >= 0.0:
        return int(math.floor(x + 0.5))
    return int(math.ceil(x - 0.5))


def _pick_free_port() -> int:
    """Pick an available localhost TCP port."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _tcp_listen_int64(
    port_holder: list[int | None], received_holder: list[bytes | None], started: threading.Event
) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", 0))
    port_holder[0] = s.getsockname()[1]
    s.listen(1)
    started.set()
    conn, _ = s.accept()
    try:
        received_holder[0] = conn.recv(8)
    finally:
        conn.close()
        s.close()


def _tcp_client_send_uint32(port: int, value: int, delay_s: float) -> None:
    time.sleep(delay_s)
    c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        c.connect(("127.0.0.1", port))
        c.sendall(struct.pack("<I", value))
    finally:
        c.close()


class TestNodeExamplesTcpNodes(omni.kit.test.AsyncTestCase):
    """Integration tests for TCP clock and external-step OmniGraph nodes."""

    graph_path = "/ActionGraph"

    async def setUp(self) -> None:
        """Create a new stage before each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Stop playback and clear the stage after each test."""
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing():
            timeline.stop()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def _trigger_impulse(self) -> None:
        og.Controller.attribute(f"{self.graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

    async def test_cpp_send_simulation_clock_tcp(self) -> None:
        """Send one simulation clock value through the C++ TCP node."""
        port_holder = [None]
        received_holder = [None]
        started = threading.Event()
        listener = threading.Thread(
            target=_tcp_listen_int64,
            args=(port_holder, received_holder, started),
            daemon=True,
        )
        listener.start()
        self.assertTrue(started.wait(15.0), "listener failed to bind")
        port = port_holder[0]
        self.assertIsNotNone(port)

        uri = f"127.0.0.1:{port}"
        # Wire payload is llround(simulationTime_s * 1e9) (C++ send node).
        time_ns = 42
        simulation_time_s = time_ns * 1e-9
        expected = struct.pack("<q", time_ns)

        og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("Sender", "isaacsim.examples.ipc.SimpleSendSimulationClockCpp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Sender.inputs:uri", uri),
                    ("Sender.inputs:simulationTime", simulation_time_s),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "Sender.inputs:execIn"),
                ],
            },
        )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await self._trigger_impulse()

        listener.join(timeout=15.0)
        self.assertFalse(listener.is_alive(), "TCP listener thread did not finish")
        self.assertIsNotNone(received_holder[0])
        self.assertEqual(len(received_holder[0]), 8)
        self.assertEqual(received_holder[0], expected)

    async def test_cpp_send_clock_wired_from_isaac_read_simulation_time(self) -> None:
        """``IsaacReadSimulationTime.simulationTime`` → ``SimpleSendSimulationClockCpp.simulationTime``."""
        port_holder = [None]
        received_holder = [None]
        started = threading.Event()
        listener = threading.Thread(
            target=_tcp_listen_int64,
            args=(port_holder, received_holder, started),
            daemon=True,
        )
        listener.start()
        self.assertTrue(started.wait(15.0), "listener failed to bind")
        port = port_holder[0]
        self.assertIsNotNone(port)

        uri = f"127.0.0.1:{port}"

        og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Sender", "isaacsim.examples.ipc.SimpleSendSimulationClockCpp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Sender.inputs:uri", uri),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "Sender.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "Sender.inputs:simulationTime"),
                ],
            },
        )
        await omni.kit.app.get_app().next_update_async()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await self._trigger_impulse()

        listener.join(timeout=15.0)
        self.assertFalse(listener.is_alive(), "TCP listener thread did not finish")
        self.assertIsNotNone(received_holder[0])
        self.assertEqual(len(received_holder[0]), 8)

        (got_ns,) = struct.unpack("<q", received_holder[0])
        sim_s = float(og.Controller.attribute(f"{self.graph_path}/ReadSimTime.outputs:simulationTime").get())
        expected_ns = _llround_seconds_to_ns(sim_s)
        self.assertEqual(got_ns, expected_ns)

    async def test_py_send_simulation_clock_tcp(self) -> None:
        """Send one simulation clock value through the Python TCP node."""
        port_holder = [None]
        received_holder = [None]
        started = threading.Event()
        listener = threading.Thread(
            target=_tcp_listen_int64,
            args=(port_holder, received_holder, started),
            daemon=True,
        )
        listener.start()
        self.assertTrue(started.wait(15.0), "listener failed to bind")
        port = port_holder[0]

        uri = f"127.0.0.1:{port}"
        # Python send node uses int(round(simulationTime * 1e9)).
        time_ns = 100
        simulation_time_s = time_ns * 1e-9
        expected = struct.pack("<q", time_ns)

        og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("Sender", "isaacsim.examples.ipc.SimpleSendSimulationClockPy"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Sender.inputs:uri", uri),
                    ("Sender.inputs:simulationTime", simulation_time_s),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "Sender.inputs:execIn"),
                ],
            },
        )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await self._trigger_impulse()

        listener.join(timeout=15.0)
        self.assertFalse(listener.is_alive(), "TCP listener thread did not finish")
        self.assertIsNotNone(received_holder[0])
        self.assertEqual(received_holder[0], expected)

    async def test_py_send_clock_wired_from_isaac_read_simulation_time(self) -> None:
        """``IsaacReadSimulationTime.simulationTime`` → ``SimpleSendSimulationClockPy.simulationTime``."""
        port_holder = [None]
        received_holder = [None]
        started = threading.Event()
        listener = threading.Thread(
            target=_tcp_listen_int64,
            args=(port_holder, received_holder, started),
            daemon=True,
        )
        listener.start()
        self.assertTrue(started.wait(15.0), "listener failed to bind")
        port = port_holder[0]

        uri = f"127.0.0.1:{port}"

        og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Sender", "isaacsim.examples.ipc.SimpleSendSimulationClockPy"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Sender.inputs:uri", uri),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "Sender.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "Sender.inputs:simulationTime"),
                ],
            },
        )
        await omni.kit.app.get_app().next_update_async()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await self._trigger_impulse()

        listener.join(timeout=15.0)
        self.assertFalse(listener.is_alive(), "TCP listener thread did not finish")
        self.assertIsNotNone(received_holder[0])
        self.assertEqual(len(received_holder[0]), 8)

        (got_ns,) = struct.unpack("<q", received_holder[0])
        sim_s = float(og.Controller.attribute(f"{self.graph_path}/ReadSimTime.outputs:simulationTime").get())
        expected_ns = int(round(sim_s * 1e9))
        self.assertEqual(got_ns, expected_ns)

    async def _run_receive_step_test(self, node_type_name: str, step_value: int) -> None:
        port = _pick_free_port()
        uri = f"127.0.0.1:{port}"

        og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("Receiver", f"isaacsim.examples.ipc.{node_type_name}"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Receiver.inputs:uri", uri),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "Receiver.inputs:execIn"),
                ],
            },
        )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        # First compute runs TcpStepServer::connect (listen). Start the client only after that
        # so we never race connect() against a listener that does not exist yet.
        await self._trigger_impulse()

        client = threading.Thread(
            target=_tcp_client_send_uint32,
            args=(port, step_value, 0.2),
            daemon=True,
        )
        client.start()

        step_attr = og.Controller.attribute(f"{self.graph_path}/Receiver.outputs:step")
        got = None
        for _ in range(240):
            await self._trigger_impulse()
            got = step_attr.get()
            if int(got) == step_value:
                break
        else:
            self.fail(f"Timed out waiting for step {step_value}, last outputs:step={got!r}")

        client.join(timeout=5.0)
        self.assertFalse(client.is_alive())

    async def test_cpp_receive_external_step_tcp(self) -> None:
        """Receive one external step value through the C++ TCP node."""
        await self._run_receive_step_test("SimpleReceiveExternalStepCpp", 12345)

    async def test_py_receive_external_step_tcp(self) -> None:
        """Receive one external step value through the Python TCP node."""
        await self._run_receive_step_test("SimpleReceiveExternalStepPy", 4294967290)

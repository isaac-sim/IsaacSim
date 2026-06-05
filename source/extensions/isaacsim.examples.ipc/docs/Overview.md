# Node Examples

This extension ships tutorial-oriented OmniGraph nodes that use **TCP/IP** (BSD sockets only; no extra native dependencies) so you can stream a clock value to an external process or receive an external **step** command from Python or C++ clients.

See the Isaac Sim documentation: **Omnigraph: Custom IPC nodes** (`omnigraph_custom_ipc_nodes`) for prerequisites, wire format, and graph wiring.

## Nodes

| Node | Language | Role |
|------|----------|------|
| `SimpleSendSimulationClockCpp` / `SimpleSendSimulationClockPy` | C++ / Python | TCP **client**; takes `simulationTime` in seconds (e.g. from **Isaac Read Simulation Time**), converts to nanoseconds, sends int64 little-endian on the wire. |
| `SimpleReceiveExternalStepCpp` / `SimpleReceiveExternalStepPy` | C++ / Python | TCP **server**; non-blocking recv of uint32 little-endian **step**; `execOut` when a full message arrives. |

## Wire format

- **Clock message:** 8 bytes, signed int64, **little-endian** (`round(simulationTime * 1e9)` on the Python send node; C++ uses `llround`).
- **Step message:** 4 bytes, unsigned int32, **little-endian**.

External tools can use the standard library only (e.g. Python `socket` + `struct.pack` / `struct.unpack`).

## External scripts

Runnable helper for this tutorial lives under `source/extensions/isaacsim.examples.ipc/python/scripts/`:

- `tcp_tutorial_playback_bridge.py` — for **On Playback Tick** → **Receive External Step** → **Send Simulation Clock** (with **Isaac Read Simulation Time** on the send node): listens for each 8-byte clock, sends each 4-byte step so playback can advance. See `--help` and the user guide (**Omnigraph: Custom IPC nodes**).

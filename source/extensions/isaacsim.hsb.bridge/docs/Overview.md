```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Overview

The isaacsim.hsb.bridge extension provides OmniGraph nodes that use the Holoscan Sensor Bridge (HSB) emulator to stream camera frames from Isaac Sim to external receivers with low latency.

## Nodes

### HSB Camera Helper (`HSBCameraHelper`)

The easiest way to stream a camera. Given a render product, it automatically wires the full pipeline (RGB conversion → CSI packing → HSB send) using Replicator writers registered by the extension.

| Input | Type | Description |
|-------|------|-------------|
| `renderProductPath` | token | Render product to stream |
| `type` | token | `vb1940_csi_coe` (3p4b, AGX Thor) or `vb1940_csi_linux` (4p5b, RoCE player). Default: `vb1940_csi_coe` |
| `ipAddress` | string | HSB target IP. Default: `127.0.0.1` |
| `dataPlaneType` | token | `coe` (Camera-over-Ethernet) or `linux` (RoCEv2 UDP). Default: `coe` |
| `dataPlaneId` | uint | Data plane ID |
| `sensorId` | uint | Sensor ID |
| `useSystemTime` | bool | Use system clock instead of simulation time |
| `resetSimulationTimeOnStop` | bool | Reset simulation timestamp on stop |

**Typical setup:** Add an `OnPlaybackTick` node and an `Isaac Create Render Product` node. Connect their execution ports in order, then connect `Isaac Create Render Product.outputs:renderProductPath` to `HSBCameraHelper.inputs:renderProductPath`. Configure `type`, `ipAddress`, and `dataPlaneType` on the helper to match your receiver.

---

### RGB to VB1940 (`RGBToVB1940`)

Converts an RGB(A) image to a VB1940 CSI frame entirely on the GPU (CUDA). Outputs a 1D byte buffer ready to pass to **HSB Send**. Used internally by the Replicator writers wired by **HSB Camera Helper**, but can also be placed manually in a graph.

| Input | Type | Description |
|-------|------|-------------|
| `data` | uchar[] (cpu) | RGB(A) pixel data (optional if `dataPtr` is set) |
| `dataPtr` | uint64 | Pointer to raw image data on GPU or CPU |
| `cudaDeviceIndex` | int | CUDA device for `dataPtr` (-1 = CPU pointer) |
| `bufferSize` | uint | Byte size of `dataPtr` buffer |
| `width` / `height` | uint | Image dimensions in pixels |
| `encoding` | token | `rgb8` or `rgba8` |
| `outputMode` | token | `vb1940_csi_linux` (4p5b, 8-byte line align) or `vb1940_csi_coe` (3p4b, 64-byte line align). Default: `vb1940_csi_linux` |

| Output | Type | Description |
|--------|------|-------------|
| `data` | uchar[] (cpu) | VB1940 CSI frame: 1 leading embedded line + image lines + 2 trailing embedded lines |

---

### HSB Send (`HSBSend`)

Sends a 1D byte buffer to an HSB receiver. The send is dispatched asynchronously via `carb::tasking` so the OmniGraph thread returns immediately; the previous frame's send is drained at the start of the next frame.

| Input | Type | Description |
|-------|------|-------------|
| `data` | uchar[] (cpu) | Buffer to send (connect from `RGBToVB1940.outputs:data`) |
| `ipAddress` | string | HSB target IP. Default: `127.0.0.1` |
| `dataPlaneType` | string | `coe` or `linux`. Default: `coe` |
| `dataPlaneId` | uint | Data plane ID |
| `sensorId` | uint | Sensor ID |
| `timeStamp` | double | Timestamp in seconds |

---

## Payload modes

| Mode | `outputMode` | `dataPlaneType` | Frame format |
|------|-------------|-----------------|--------------|
| VB1940 CSI Linux | `vb1940_csi_linux` | `linux` | RAW10 4p5b, 8-byte line align |
| VB1940 CSI over COE | `vb1940_csi_linux` | `coe` | RAW10 4p5b, 8-byte line align |
| VB1940 CSI COE | `vb1940_csi_coe` | `coe` | RAW10 3p4b, 64-byte line align |

> **Note:** The `linux_coe` players expect the same 4p5b frame as the RoCE player. Using `vb1940_csi_coe` (3p4b) with a Linux COE player causes "Ignoring contents for a packet" because the frame size exceeds the player's buffer.

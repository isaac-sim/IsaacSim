# Overview

The `isaacsim.streaming.rtsp` extension provides RTSP streaming for camera render products in Isaac Sim. It captures rendered frames through a Replicator writer and publishes them over RTSP via `omni.kit.livestream.rtsp`. OmniGraph nodes are included for graph-based streaming setup. External clients can connect with any standard RTSP player using `rtsp://<host>:<port><mountPath>`.

## Usage

The extension is not enabled by default in the Isaac Sim full application. Enable it explicitly from the Extension Manager or start Isaac Sim with `--enable isaacsim.streaming.rtsp` when RTSP streaming is needed.

## Functionality

The extension registers {class}`RTSPStreamWriter <isaacsim.streaming.rtsp.RTSPStreamWriter>` with the Replicator WriterRegistry on startup. The writer supports two encoding modes: pre-encoded H.264 with per-frame SEI metadata injection, and raw CUDA buffer passthrough where encoding is handled by the livestream backend. The RTSP server is started lazily on the first rendered frame and stopped when the writer detaches. Multiple simultaneous streams are supported when each stream uses a unique port and mount path.

## Integration

The extension depends on `omni.replicator.core` for frame capture and annotator management, `omni.kit.livestream.core` and `omni.kit.livestream.rtsp` for the RTSP server backend, and `isaacsim.core.nodes` for the OmniGraph writer node base class. Camera render products are created elsewhere in the application or graph and passed to the writer at attach time.

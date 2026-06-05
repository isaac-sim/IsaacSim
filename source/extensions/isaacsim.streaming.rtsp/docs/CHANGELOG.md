# Changelog

## [0.1.3] - 2026-04-27
### Added
- Pass configured SRTX sensor-set names from `RTSPCameraHelper` to `RTSPStreamWriter` and into the `LdrColor` annotator `init_params`. Resolution is delegated to `omni.replicator.srtx.resolve_sensor_set_name_for_render_product()`.
- Optional dependency on `omni.replicator.srtx` for sensor-set resolution.

## [0.1.2] - 2026-04-24
### Fixed
- Materialize the `RenderVar` child required by SRTX under the render product before attaching `RTSPStreamWriter`, and author the matching `srtx:compression:type` for raw and H.264 streaming.

## [0.1.1] - 2026-04-21
### Fixed
- Pass simulation time as `start_time_ns`/`ended_time_ns` to the RTSP server so streamed frame timestamps reflect sim time.

## [0.1.0] - 2026-03-31
### Added
- `RTSPStreamWriter` Replicator writer with H.264 pre-encoded and raw CUDA streaming modes.
- Per-frame SEI metadata injection (simulation time, wall-clock timestamp, frame number).
- `RTSPCameraHelper` OmniGraph node for graph-based streaming setup.

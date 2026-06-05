# Changelog

## [1.4.3] - 2026-05-12
### Added
- Added TestMultiSensorWarmup as smoke test for WAR low-frequency fatal crash when multiple Lidars and Radars are in the same scene.

## [1.4.2] - 2026-05-08
### Fixed
- Fixed TestRadarSensor.test_gmo_writer timing check.

## [1.4.1] - 2026-05-07
### Added
- `SUPPORTED_CAMERA_CONFIGS` / `SUPPORTED_CAMERA_VARIANT_SET_NAME` and `config=` parameter on `RtxCamera.create()`. The camera registry value is a metadata dict (rather than the variant spec directly) carrying `display_name` and an optional `is_depth_sensor` flag; `vendor` and `prim_prefix` are derived from the asset path. `get_camera_metadata(config_path)` returns the normalized record for UI consumption.

### Changed
- `Radar.create()` and `Acoustic.create()` config matching now accepts the same five alias forms as `Lidar.create()` (full asset path, USD stem, stem with underscores → spaces, vendor-stripped stem, vendor-stripped stem with underscores → spaces); previously only the full path and stem were accepted.
- `Lidar.create()` / `Radar.create()` / `Acoustic.create()` / `RtxCamera.create()` `'config not found'` error now lists the short (vendor-stripped) config names instead of the full asset paths, includes a "Did you mean..." suggestion when there is a near-match, and points the reader to `SUPPORTED_<TYPE>_CONFIGS` for the full mapping.

### Fixed
- `SingleViewDepthCameraSensor` no longer spams `SdPostRenderVarTextureToBuffer : corrupted input renderVar DepthSensorDistance` (and the same for the other `DepthSensor*` render vars) once `set_enabled_post_processing(True)` is called. The four `depth_sensor_*` annotators are now attached on the host Replicator pipeline (matching the deprecated `isaacsim.sensors.camera.SingleViewDepthSensor` default), which routes through `SdPostRenderVarToHost` instead of the device-buffer node that does not support these render vars.
- `CameraSensor.get_data` now promotes non-Warp array results (e.g. `numpy.ndarray` returned by host-pipeline annotators when a CUDA device is requested) to a `wp.array` on the requested device, so `wp.copy` no longer fails with `"Copy source and destination must be arrays"` when a pre-allocated `out=` buffer is provided.

## [1.4.0] - 2026-05-05
### Added
- `SUPPORTED_RADAR_CONFIGS` / `SUPPORTED_RADAR_VARIANT_SET_NAME` and `config` parameter on `Radar.create()`, with Texas Instruments IWRL6432AOP as the first entry
- `SUPPORTED_ACOUSTIC_CONFIGS` / `SUPPORTED_ACOUSTIC_VARIANT_SET_NAME` and `config` parameter on `Acoustic.create()` (empty for now; OEM acoustic assets slot in here)
- `test_rtx_radar_configs.py` and `test_rtx_acoustic_configs.py` validating each config via `SensorCheckerUtil`
- `SICK_LMS4000` (3 variants) and `SICK_LMS5xx` (61 variants) lidar configs
- `variant=` parameter on `Lidar.create()` / `Radar.create()` / `Acoustic.create()` now accepts `dict[str, str]` for USDs with multiple variant sets (e.g. SICK `Product` × `Profile`)

### Changed
- `SUPPORTED_LIDAR_CONFIGS` value type widened to `dict[str, set[str] | list[dict[str, str]]]`; flat `set[str]` entries still work via the `"sensor"` default
- SICK lidar entries restructured to match the new SICK family-USD bundle and converted to dict form (Product/Profile pairs); `SICK/{LRS4581R,MRS1104C,multiScan136,multiScan165,picoScan150}` replaced by `SICK/{LRS4000,MRS1000,multiScan100,multiScan100,picoScan100}`; `SICK_nanoScan3` corrected from `{"Lidar"}` to `set()`

## [1.3.1] - 2026-05-05
### Fixed
- Authoring APIs no longer clobber tickRate and other attributes if already set on wrapped prim or loaded USD

## [1.3.0] - 2026-05-05
### Added
- StructuredLightCamera authoring API, allowing users to specify time-sequenced projection patterns with the camera.

## [1.2.0] - 2026-05-04
### Added
- Add _asset_root_path attribute to _SensorAuthoring to handle assets which have multiple sensor prims.
- RtxCamera.create method allows loading USD assets like the other authoring classes
- New APIs in SingleViewDepthCameraSensor for functionality like deprecated isaacsim.sensors.rtx.SingleViewDepthSensorAsset

### Changed
- Radar.__init__ mBVH warning made clearer

### Fixed
- Lidar.create uses same config resolution logic as deprecated isaacsim.sensors.rtx.commands

## [1.1.2] - 2026-04-29
### Fixed
- `Radar` and `Lidar` authoring classes now auto-materialize a missing parent prim on the pxr USD stage before invoking `rep.functional.create.omni_radar` / `omni_lidar`. Replicator's parent-valid check runs strictly against pxr, so newly opened large scenes (where the parent exists on the Fabric/USDRT side but not yet on pxr) would previously raise `ValueError: Parent /World is not a valid prim`. Callers no longer need the `stage_utils.define_prim("/World", "Xform")` workaround.

## [1.1.1] - 2026-04-28
### Fixed
- `resolve_lidar_object_ids.py` standalone example was using only the lower 32 bits of each 128-bit object ID as the `StableIdMap` lookup key, which only matched in trivial scenes where the upper 96 bits happened to be zero. The example now uses `parse_object_ids()` to extract full 128-bit ints, so the lookup also resolves correctly for multi-subset meshes and procedural geometry.

### Changed
- `parse_stable_id_map_data()` docstring now warns that some LiDAR object IDs may not have a map entry. The renderer combines the per-instance base stable ID with an upper index (submesh index for meshes, primitive index for procedural geometry); the map only registers per-instance and per-`GeomSubset` entries, so hits on procedural geometry or unmapped submesh indices produce IDs with no entry. Callers should use `map.get(id, ...)` rather than `map[id]`.

## [1.1.0] - 2026-04-23
### Added
- `RtxCamera` authoring class for creating/wrapping USD Camera prims with OmniSensorAPI schema
- `CameraSensor` runtime class for single-camera annotator data retrieval with resolution-aware render products
- `TiledCameraSensor` runtime class for batched multi-camera rendering with shared annotators
- `SingleViewDepthCameraSensor` runtime class extending `CameraSensor` with stereoscopic depth post-processing
- `draw_annotator_data_to_image` utility for converting annotator output to images
- Camera-specific annotator spec registry (`_camera_common.py`)
- `register_annotator_spec`, `unregister_annotator_spec`, `register_writer_spec`, `unregister_writer_spec` for companion extension integration
- `aux_output_level` parameter to `Radar.create()` and `Acoustic.create()` (matching `Lidar.create()`)
- `aux_output_level` to `Radar` and `Acoustic` class docstrings

## [1.0.1] - 2026-04-22
### Fixed
- Mark extension as platform-specific (`writeTarget.platform = true`) so the registry publishes a separate artifact per platform. Without this, consumers on Linux would pull a Windows-built package containing `.pyd` instead of `.so` from `generic-model-output`/`sensor-checker` (or vice versa) and fail to load.

## [1.0.0] - 2026-04-06
### Added
- `Radar` authoring class for creating/wrapping OmniRadar prims via `omni.replicator.core.functional.create.omni_radar`
- `Acoustic` authoring class for creating/wrapping OmniAcoustic prims with auto-applied multi-instance schemas (sensorMount, rxGroup)
- `RadarSensor` runtime class for radar annotator data retrieval
- `AcousticSensor` runtime class for acoustic annotator data retrieval
- `tick_rate` parameter on all authoring constructors (`Lidar`, `Radar`, `Acoustic`) for setting `omni:sensor:tickRate`
- `omni.sensors.nv.acoustic` extension dependency

### Changed
- Split `LidarSensor` into separate `Lidar` (authoring) and `LidarSensor` (runtime) classes
- `Lidar` authoring class now uses `omni.replicator.core.functional.create.omni_lidar` for prim creation
- Renamed `RtxLidarSensor` to `LidarSensor`
- Extracted `_SensorAuthoring` and `_SensorRuntime` base classes to reduce code duplication
- `omni:sensor:tickRate` in attributes dict overrides the `tick_rate` parameter (with warning)

### Fixed
- `parse_generic_model_output_data` returning the `GenericModelOutput` class instead of an instance in fallback paths

## [0.1.0] - 2026-03-17
### Added
- Initial release

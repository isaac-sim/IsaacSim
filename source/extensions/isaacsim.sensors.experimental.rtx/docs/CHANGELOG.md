# Changelog

## [2.0.1] - 2026-09-22
### Changed
- `StructuredLightCamera`: document the anti-aliasing modes that preserve projected pattern detail.

## [2.0.0] - 2026-08-31
### Added
- `RtxCamera` can wrap or create multiple Camera prims from a path list or regex.
- `TiledCameraSensor` accepts `tick_rate`, `schemas`, and `attributes` when authoring cameras from paths.

### Changed
- `TiledCameraSensor` authors cameras through `RtxCamera`; `authoring_object` now returns that wrapper.
- Sensor authoring warns before replacing a different authored tick rate on a shared prim.
- `TiledCameraSensor` warns when camera tick rates differ because the first camera controls the whole batch.

### Fixed
- Single-prim runtimes reject multi-prim inputs before authoring changes the stage.

## [1.9.1] - 2026-09-15
### Added
- New golden images for Luxonis OAK-D ToF sensor.

### Changed
- Orbbec Gemini 335 now marked as not-a-depth-sensor, until asset is updated to use SingleViewDepthSensor APIs.
- Depth-camera tests use fixed-range PNG goldens and writer callbacks to compare consecutive sensor emissions.

## [1.9.0] - 2026-08-24
### Added
- `has_data()` on `SensorRuntime`, reporting render completion with annotator output while the render product still exists and annotators are attached. This lets callers bound an annotator warm-up wait without repeatedly fetching data when no render engine could be bound. It is a render-product-level gate, so it does not guarantee that a specific annotator has a frame ready.

### Changed
- `TiledCameraSensor` now derives from `SensorRuntime` and shares the annotator and render product lifecycle with the other sensor runtimes instead of duplicating it. The tiled-specific properties are unchanged, and the class additionally exposes the inherited `authoring_object` property, `attach_writer`/`detach_writer`, `annotator_init_params` and `writers` constructor arguments, and an `annotator_init_params` argument on `attach_annotators`. Setting `render_vars` now logs a warning, since tiled render products ignore it.
- `CameraSensor.get_data` and `TiledCameraSensor.get_data`: information reported alongside an empty warm-up payload (e.g. `frameId`) is now preserved instead of being dropped. The payload itself is still `None` during warm-up.
- `SensorRuntime` subclasses may set `_ALLOW_MULTIPLE_AUTHORING_OBJECTS` to wrap more than one prim, as batched runtimes require.
- Depend on `omni.kit.hydra_texture` to observe Hydra render-completion events, which is what lets `has_data()` report readiness without fetching annotator data.

## [1.8.2] - 2026-08-21
### Changed
- Updated Luxonis OAK depth-sensor golden coverage for revised sensor assets.

## [1.8.1] - 2026-08-21
### Fixed
- Retain aliased input buffers for the lifetime of parsed generic model output structures.

## [1.8.0] - 2026-08-18
### Changed
- `RtxCamera`, `Lidar`, `Radar`, `Acoustic` (and their sensor runtime classes): wrapping an existing prim of the expected type that lacks the sensor's API schema now applies the schema instead of raising `ValueError`.

## [1.7.1] - 2026-08-14
### Changed
- Document that `annotator_init_params` cannot filter semantics per annotator: `semanticTypes`/`semanticFilter` applies to the whole render product, so bounding box and segmentation annotators sharing one render product share one filter.

## [1.7.0] - 2026-08-11
### Changed
- `CameraSensor`: disable RTX post anti-aliasing on sensor-owned render products smaller than 300 pixels in either dimension, where DLSS returns buffers that do not match the requested resolution.

### Fixed
- `CameraSensor.get_data`: raise a descriptive error when an annotator returns an unexpected element count, instead of failing in the reshape.

## [1.6.2] - 2026-08-08
### Fixed
- Prevent SPG Lua launch scripts from being rejected by the sandbox validator.

## [1.6.1] - 2026-07-28
### Added
- `SensorAuthoring.asset_root_path` exposes the reference root of the USD asset a sensor was loaded from (`None` when not loaded from an asset).

### Changed
- `SensorAuthoring._create_from_usd` now returns the constructed sensor instance instead of the sensor prim path, and accepts the transform and constructor arguments it forwards. Existing keyword arguments keep their names and meaning; `usd_path` is now optional.

### Fixed
- `Lidar.create`, `Radar.create`, `Acoustic.create`, `RtxCamera.create`: author `positions` / `translations` / `orientations` / `scales` on the loaded asset's reference root instead of the sensor prim nested inside it. Previously the transform detached the sensor origin from the housing geometry and overwrote the vendor's mounting offset (e.g. the Ouster OS1 z-offset, the TI IWRL6432AOP three-axis offset). Assets whose sensor prim is the reference root, and sensors created without an asset, are unaffected.
- `Lidar`, `Radar`, `Acoustic` now record the asset reference root like `RtxCamera` did, so pre-authored render-product discovery is scoped to the asset subtree rather than the whole stage.

## [1.6.0] - 2026-07-21
### Added
- `SPGNode` and `SensorAuthoring.author_spg` (e.g. `RtxCamera.author_spg`) to author RTX Sensor Processing Graph (SPG) prim structure — one `UsdShade.Shader` per node plus the required `RenderProduct`/`RenderVar` AOVs — from user-authored CUDA kernels and their co-located Lua launch scripts. Wiring uses an `omni.graph`-style declarative `(src, dst)` connection list. Structural validation only; SPG semantics remain owned by `omni.rtx.spg`. An optional `copy_to` vendors the kernel sources alongside a sensor asset.
- Standalone examples `spg_grayscale.py` and `spg_grayscale_invert.py` demonstrating single-shader and chained SPG authoring, and reading custom output AOVs back via `omni.replicator.core` `register_annotator_from_aov`.

### Changed
- Promote `_SensorAuthoring` / `_SensorRuntime` to public abstract bases `SensorAuthoring` / `SensorRuntime` (do not instantiate directly). Shared APIs such as `author_spg` are documented on the public bases.

## [1.5.0] - 2026-07-13
### Added
- Runtime sensors now accept per-annotator initialization parameters via `annotator_init_params` at construction time and `attach_annotators(..., annotator_init_params=...)` after construction.

### Fixed
- `CameraSensor` and `SingleViewDepthCameraSensor` now attach the `pointcloud` annotator with `includeUnlabelled=True` by default, matching the deprecated camera API and allowing point clouds from unlabeled geometry.

## [1.4.10] - 2026-07-07
### Changed
- `SUPPORTED_CAMERA_CONFIGS`: prefix the SICK (Inspector83x, InspectorP61x, safeVisionary2, Visionary-T Mini) and Stereolabs (ZED_X) camera `display_name` values with their vendor, so menu labels and derived action IDs are consistent with the other vendors.

## [1.4.9] - 2026-07-06
### Changed
- `SingleViewDepthCameraSensor` now attaches to a pre-authored `RenderProduct` in a loaded USD asset when one exists, deriving `resolution` and `annotators` from it (both now optional), instead of always creating a new one. Requires `omni.replicator.core >= 1.13.28`.

## [1.4.8] - 2026-06-29
### Removed
- Removed the retired SICK TiM781 asset from `SUPPORTED_LIDAR_CONFIGS`; use the SICK picoScan100 family instead.

## [1.4.7] - 2026-06-12
### Fixed
- `SingleViewDepthCameraSensor`: correctly populates render vars for depth sensor SPG.

## [1.4.6] - 2026-06-12
### Fixed
- `camera_utils.draw_annotator_data_to_image`: defer the `cv2` import to first use.

## [1.4.5] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.4.4] - 2026-05-22
### Changed
- `attach_writer()` now returns the attached Writer instance.
- `attach_annotators()` now returns the attached annotator instances.

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

# Changelog

## [3.0.2] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [3.0.1] - 2026-05-10
### Fixed
- Refresh IMU prim-data reader initialization when reading or stepping so IMU data is available after physics-only simulation steps
- Add physics-only stepping coverage for contact, effort, IMU, joint state, and raycast sensors

## [3.0.0] - 2026-04-30
### Changed
- Aligned `IMUSensor`, `ContactSensor`, and `RaycastSensor` with `isaacsim.sensors.experimental.rtx`:
  - Split authoring (`IMU`/`Contact`/`Raycast`) from runtime. Runtime constructors take a path or authoring instance; `create()` lives only on the authoring classes — replace `IMUSensor.create(path, ...)` with `IMUSensor(IMU.create(path, ...))` (same for `Contact`/`Raycast`).
  - Constructors take keyword-only `positions`/`translations`/`orientations`/`scales` arrays (`(N, 3)`/`(N, 4)` wxyz) plus `reset_xform_op_properties` (default `True`), instead of singular `Gf` types. Constructor parameter `prim_path` renamed to `path`.
  - Multi-prim `path`, conflicting `positions`/`translations`, and wrapping an existing prim whose USD type does not match the sensor's `_PRIM_TYPE` now raise `ValueError`. Dropped unused `name` parameter.
  - Removed `__getattr__` attribute forwarding from the runtime; go through the typed `sensor.imu` / `sensor.contact` / `sensor.raycast` accessor for `paths`, `prims`, `set_visibilities`, `get_world_poses`, etc.
- Removed `*Backend` classes (`ImuSensorBackend`, `ContactSensorBackend`, `RaycastSensorBackend`, `EffortSensorBackend`, `JointStateSensorBackend`). The runtime sensors now own the C++ Carbonite interface directly and expose both `get_data()` (dict) and `get_sensor_reading()` (raw struct) — replace `XxxSensorBackend(path)` with `XxxSensor(path)`. Renamed `get_current_frame()` → `get_data()`; removed the `get_current_frame` alias, the `prim_path` property, and the no-op `initialize()` method. Removed `ContactSensor.{get,set}_min_threshold` / `_max_threshold` / `_radius` shims (use `sensor.contact.<method>` instead). Added `get_data()` to `EffortSensor` and `JointStateSensor` so all five sensors expose a consistent `get_data()` + `get_sensor_reading()` pair.
- Hardened the C++ runtime against prim deletion and recreation. `getSensorReading` (and Contact's `getRawContacts`) now invalidate the cached entry when the underlying USD prim has been removed across all five sensors. The `createSensor` early-out tears down the cached entry when the parent rigid body / articulation root no longer matches and refreshes config so attribute changes on a re-authored prim at the same path are picked up.

## [2.7.1] - 2026-04-22
### Fixed
- Fix `ContactSensor.__init__` accessing `self._prim` before assignment when applying threshold/radius overrides

## [2.7.0] - 2026-04-22
### Changed
- Replaced command classes with `create()` static methods on sensor wrapper classes (`ContactSensor.create()`, `IMUSensor.create()`, `RaycastSensor.create()`) matching the `isaacsim.sensors.experimental.rtx` pattern
- Removed command classes, standalone factory functions, and `sensor_helpers.py`; sensor prim creation logic is now inlined in each sensor class
- Removed `commands_api.md`; updated `api.rst` and `Overview.md` to document class-based creation API
- Refactored backend classes to use shared `_PhysicsSensorBase` lifecycle (lazy interface, ensure/retry, reset, timeline stop), eliminating ~300 lines of duplicated code
- Extracted `_create_sensor_prim` helper for shared prim creation boilerplate across Contact, IMU, and Raycast sensors
- Removed deprecated `_SensorStepManager.get_imu_backend()` method
- Fixed `ContactSensor.__init__` parent validation to walk up the hierarchy and accept both `CollisionAPI` and `RigidBodyAPI` ancestors

## [2.6.0] - 2026-04-22
### Changed
- Add update() flush calls in EffortSensorImpl and ImuSensorImpl before reading physics data

## [2.5.0] - 2026-04-21
### Changed
- Port ContactSensor to use IPrimDataReader contact API (`enableContactReporting`/`getContactReport`) instead of direct PhysX contact event subscription

## [2.4.0] - 2026-04-17
### Added
- Add Raycast Sensor C++ implementation, Python bindings, backend, commands, and examples

## [2.3.4] - 2026-04-15
### Changed
- Fix test failure after moving to codeless USD schemas

## [2.3.3] - 2026-04-09
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [2.3.2] - 2026-03-26
### Changed
- Moved Python binding module to `bindings/` subdirectory

## [2.3.1] - 2026-03-22
### Changed
- Add null guards for deferred prim data reader provider loading in sensor _initializeStage

## [2.3.0] - 2026-03-20
### Changed
- Replace int64 sensorID with prim path string for deterministic sensor identity across runs
- Bump C++ Carbonite interface versions to 2.0 (IImuSensor, IContactSensor, IEffortSensor, IJointStateSensor)
- createSensor() now returns bool instead of int64; removeSensor/getSensorReading/getRawContacts take prim path instead of sensor ID
- Internal sensor maps keyed by prim path (std::string) instead of monotonic counter

## [2.2.1] - 2026-03-12
### Changed
- Migrate ContactSensor and ImuSensor world-transform reads from computeWorldXformNoCache to IXformDataView (IPrimDataReader)

## [2.2.0] - 2026-03-05
### Added
- Add joint state sensor that reads all DOF positions, velocities, and efforts per articulation

## [2.1.1] - 2026-03-05
### Changed
- Fix api and docs syntax issues

## [2.1.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [2.0.0] - 2026-02-27
### Changed
- Rewrite sensor implementations to use C++ core experimental prims APIs

## [1.1.0] - 2026-02-10
### Changed
- IMU and Contact sensor creation commands renamed to include Experimental in their name to avoid name collision with deprecated sensor commands

## [1.0.0] - 2026-02-01
### Changed
- Rebuilt physics sensors on core experimental APIs with Python-first implementations
- Added Python backends for contact and IMU sensors plus legacy interface shims for compatibility
- Added new command-based prim creation and expanded Python tests for sensors and OmniGraph nodes
- Removed sensor period and frequency parameters, all sensors use the physics frequency by default

## [0.7.0] - 2026-01-28
### Added
- Refactor IMU and contact sensor tests to use experimental core APIs

## [0.6.3] - 2025-12-07
### Changed
- Fix clang tidy issues in cpp code

## [0.6.2] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [0.6.1] - 2025-12-03
### Changed
- Remove TODOs.

## [0.6.0] - 2025-11-25
### Added
- Add dedicated GPU codepath for IMU to use separate stream and pinned memory buffer

## [0.5.4] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [0.5.3] - 2025-10-31
### Changed
- Update deprecated python unittest methods

## [0.5.2] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [0.5.1] - 2025-10-22
### Changed
- Remove deprecated time related APIs from CoreNodes interface

## [0.5.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [0.4.3] - 2025-10-02
### Fixed
- Orientation bug for contact and IMU sensor

## [0.4.2] - 2025-09-23
### Changed
- Fixed contact sensor implementation

## [0.4.1] - 2025-08-29
### Changed
- Renamed CARB profiling zones to include [IsaacSim] prefix

## [0.4.0] - 2025-08-07
### Changed
- Use device-generic memory buffer implementation to enable tensor API processing on the GPU for IMU sensor

## [0.3.27] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.3.26)

## [0.3.26] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.3.25] - 2025-06-27
### Fixed
- Use rolling average for the Imu Ogn node test

## [0.3.24] - 2025-06-25
### Changed
- Add --reset-user to test args

## [0.3.23] - 2025-06-18
### Changed
- Track change from isaacsim.core.include Pose.h

## [0.3.22] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [0.3.21] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.3.20] - 2025-05-15
### Changed
- UsdUtilities.h was updated

## [0.3.19] - 2025-05-11
### Changed
- Enable FSD in test settings

## [0.3.18] - 2025-05-10
### Changed
- Remove internal build time dependency

## [0.3.17] - 2025-05-07
### Changed
- Switch to omni.physics interface

## [0.3.16] - 2025-05-02
### Changed
- Remove all Dynamic control compile time dependencies

## [0.3.15] - 2025-04-09
### Changed
- Update all test args to be consistent
- Update Isaac Sim NVIDIA robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [0.3.14] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [0.3.13] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.3.12] - 2025-03-17
### Changed
- Cleanup and rename BridgeApplication to PrimManager for clarity

## [0.3.11] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [0.3.10] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [0.3.9] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [0.3.8] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [0.3.7] - 2025-02-18
### Fixed
- Make sure prims are valid before using

## [0.3.6] - 2025-01-28
### Fixed
- Windows signing issue

## [0.3.5] - 2025-01-26
### Changed
- Update test settings

## [0.3.4] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.3.3] - 2024-12-16
### Changed
- Moved samples to isaacsim.sensors.physics.examples extension.

## [0.3.2] - 2024-12-05
### Changed
- Updated Nova carter path

## [0.3.1] - 2024-11-25
### Fixed
- Error when user attempts to create contact sensor without having selected valid parent prim

## [0.3.0] - 2024-10-31
### Changed
- Moved examples from menu to browser

## [0.2.0] - 2024-10-30
### Changed
- Use USDRT for component initialization using prim types directly rather than traversing stage for better performance

## [0.1.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [0.1.0] - 2024-09-24
### Added
- Initial version of isaacsim.sensors.physics
- Includes contact sensor, IMU sensor, effort sensor

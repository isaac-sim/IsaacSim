# Changelog

## [2.3.0] - 2026-10-01
### Added
- Add SimReady feature and capability definitions for authored RTX camera, depth, lens calibration, ISP, SPG, Lidar, and Radar sensor assets.
- Register independently selectable checkers for 372 sensor requirements, including warnings where authored USD cannot establish the result.

### Changed
- Validate sensor definitions within the default-prim or explicitly selected prim subtree, including composed variant selections, without checking unrelated stage prims.
- Require at least one RenderProduct for a sensor asset and warn about individual sensor prims without one.
- Consolidate the six Lidar rule groups into the 75-requirement RTX Lidar Core capability and organize rule implementations by capability.
- Remove duplicate Radar requirements, read only OpenEXR headers for LUT metadata, and share SPG output-port traversal.
- Exclude permissive SPG statements from validation, warn for legacy parameters and duplicate SPG output names, and refine Shader eligibility and Lua launch-argument checks.

## [2.2.0] - 2026-08-10
### Added
- Expose SimReady foundation tier rules in the Asset Validator interface.

### Fixed
- Register `RGBSensorUsdRule` explicitly on startup so it survives the Asset Validator registry reset.

## [2.1.0] - 2026-07-29
### Added
- Added `RGBSensorUsdRule` for validating authored RGB sensor USD assets.

## [2.0.0] - 2026-07-15
### Removed
- Removed all in-extension validation rules (physics, joints, drives, robot schema, materials) and their unit tests. These now live in the SimReady foundation validation tiers and are the single source of truth.

### Changed
- The extension now provisions `simready-validate` and the `simready-foundation-tier-core` / `simready-foundation-tier-isaac` wheels via `[python.pipapi]` (internal Artifactory index) instead of registering rules with the Omniverse Asset Validator.
- Dropped the `omni.asset_validator.core` dependency; physics and robot-schema Kit dependencies are retained for tier validators that need the runtime.

## [1.3.6] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.3.5] - 2026-05-07
### Fixed
- Repaired failing validator dedup mixin test.

## [1.3.4] - 2026-05-02
### Changed
- Added DedupBaseRuleChecker to filter duplicate errors and warnings caused by multiple variants.

## [1.3.3] - 2026-05-01
### Changed
- Removed redundant `fetch_results()` after `simulate()` when collecting initial collider pairs in physics validation rules.

## [1.3.2] - 2026-04-24
### Fixed
- RigidBodyHasMassAPI no longer flags unauthored principalAxes on assets where the attribute falls back to the schema default.
- RigidBodyHasMassAPI no longer crashes on rigid bodies missing MassAPI or required mass attributes; missing-attr errors are collected up front.
- NonAdjacentCollisionMeshesDoNotClash skips pairs where either collider is outside the defaultPrim subtree, filtering out environment ground planes.
- NonAdjacentCollisionMeshesDoNotClash walks to the nearest RigidBodyAPI ancestor for adjacency lookup, fixing false positives on nested collision layouts.
- HasArticulationRoot, JointsExist, and LinksExist only fire on stages that actually contain joints.

## [1.3.1] - 2026-04-14
### Fixed
- Fixed bug in JointHasCorrectTransformAndState where antipodal quaternions were not matching correctly
- Fixed physics layer naming rules to accept transformer output naming convention
- Fixed false positives in drive joint and collision mesh purpose rules

## [1.3.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [1.2.1] - 2026-02-28
### Changed
- Updated physics_rules to use new omni.physics.core API (`IPhysicsSimulation.initialize`/`close` replaces removed `attach_stage`/`detach_stage`)

## [1.2.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [1.1.0] - 2025-09-19
### Added
- Add diagonal inertia triangle inequality check

## [1.0.4] - 2025-09-16
### Changed
- Update tolerances for mis-configured joint poses

## [1.0.3] - 2025-07-16
### Changed
- Fix asset validator module import

## [1.0.2] - 2025-06-03
### Changed
- Fix incorrect licenses and add missing licenses

## [1.0.1] - 2025-05-30
### Changed
- Update to typed name for schema instead of hard-coded strings

## [1.0.0] - 2025-05-02
### Added
- First Version

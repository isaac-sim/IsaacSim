# Changelog

## [Unreleased]

### Fixed

- Migrate only values owned by the edit target, without capturing referenced or stronger-layer names.

- Migrate joint-only legacy name overrides in the edit layer while preserving canonical names and referenced assets.

## [7.0.0a1] - 2026-09-16

### Added

- Robot USD schemas are generated, tested, and packaged through the standalone library CMake build.
- Pure Python wheel variants support the standalone USD 25.05 runtime and the extension USD 25.11 runtime.
- Schema tooling discovers rigid links and joints in disconnected components behind non-rigid frames.

### Changed

- Pure Python wheel variants require Python 3.12.

# Changelog

## [Unreleased]

### Changed

- Load OVStage from the declared `ovstage` Python dependency instead of embedding a second native runtime in the
  Foundation wheel.

## [7.0.0a1] - 2026-09-16

### Added

- Windows packages install the OVStage plugin and USD schema trees beside the module binary.

## [7.0.0a1] - 2026-09-16

### Added

- A thin C++ wrapper over OVStage exposes stage lifecycle, prim authoring, schema queries, and attribute I/O through a
  stable ABI based on opaque `int64_t` stage handles and standard C++ types.
- Installed C++ headers contain the public wrapper API; OVStage helper code is a private implementation detail.

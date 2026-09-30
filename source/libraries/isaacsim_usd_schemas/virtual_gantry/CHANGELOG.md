# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- `IsaacVirtualGantry` is an `Xform`-derived typed prim whose world transform anchors a one-sided spring-damper rope.
  It provides `isaac:gantry:*` attributes and `attachBody` and `articulation` relationships and is generated, tested,
  and packaged through the standalone library CMake build.

### Changed

- The pure-Python wheel requires Python 3.12 and `usd-exchange==2.3.0`.

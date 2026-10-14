# Changelog

## [Unreleased]

### Changed

- `Simulation.initialize()` releases the Python GIL while it runs. Other client calls continue to hold it.

## [7.0.0a1] - 2026-09-16

### Added

- OV SIM client interfaces with `timestamp` parameters, `get...` exception-property accessors, and `...Function`
  callable aliases.
- Python dictionary-based client configuration, including gRPC endpoints.
- Python API reference docstrings for public client classes, methods, properties, and factory functions.

### Changed

- The `isaacsim-ovsim` distribution has runtime dependencies on `isaacsim-common`, `isaacsim-foundation`, and
  `isaacsim-physics`.

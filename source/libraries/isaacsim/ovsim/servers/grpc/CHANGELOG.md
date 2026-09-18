# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- A synchronous OV SIM lifecycle and world-state gRPC server with `getEndpoint` endpoint inspection.
- A standalone insecure server executable with an explicit listen address.
- Generic single-attribute writes for protocol-supported values.
- `GetSimulation` responses that include asynchronous failure codes, messages, and details.
- Production-only CMake configurations support `BUILD_TESTING=OFF`.

### Changed

- The `isaacsim-ovsim` distribution has runtime dependencies on `isaacsim-common`, `isaacsim-foundation`, and
  `isaacsim-physics`.

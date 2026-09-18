# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- Remote lifecycle management, readiness polling, drift-free stepping, latest-value reads, and position writes.
- Insecure endpoint configuration and client-owned UUIDv4 simulation identities.
- Asynchronous server failure status propagation during initialization.
- Verb-first `get...` and `set...` accessors for client configuration and provider parameters, with `timestamp`
  parameters for time-sampled data access.

### Changed

- The `isaacsim-ovsim` distribution has runtime dependencies on `isaacsim-common`, `isaacsim-foundation`, and
  `isaacsim-physics`.

# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- C, C++, and Python logging facades use a pinned static Carbonite backend and provide a stable C symbol baseline,
  immutable channel loggers, and source-location preservation.
- `{fmt}`-based C++ logging macros skip argument evaluation for filtered records and provide thread-safe warning-once
  and deprecation-once macros, Python severity methods with opt-in per-call source capture and a `warn` alias, and
  backend-aware admission queries.
- C, C++, and Python `report` APIs write unconditional results to standard output with a channel prefix. An explicit
  process-wide `flush()` operation is also available.
- Patch-style process-wide and per-channel Carbonite configuration controls filtering, destinations, file output,
  rendering, asynchronous delivery, and multi-process settings.
- The standalone default logger displays elapsed milliseconds without an absolute timestamp.
- An opt-in opaque host integration API lets adapters borrow an existing Carbonite backend. Backend selection occurs
  on the first logging, admission-query, or configuration operation, allowing hosts to attach Carbonite after feature
  loggers are constructed.
- The C++ wrapper exposes `ChannelLoggingConfiguration` and `GlobalLoggingConfiguration`; C and Python expose the
  corresponding `Config` names.
- Installed C++ headers expose the public logging API; Carbonite backend details remain private.

### Changed

- The distribution metadata uses the canonical `isaacsim-common` project name.

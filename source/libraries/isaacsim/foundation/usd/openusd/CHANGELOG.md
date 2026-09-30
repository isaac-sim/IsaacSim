# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- A thin C++ wrapper over OpenUSD exposes stage lifecycle, prim authoring, schema queries, variant selection, and
  attribute I/O through a stable ABI based on opaque `int64_t` stage handles and standard C++ types.
- The packaged bindings support Windows.
- Installed C++ headers contain the public wrapper API; OpenUSD helper code is a private implementation detail.

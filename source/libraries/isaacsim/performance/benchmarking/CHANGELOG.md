# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- `benchmark.measure()` provides named timing regions, and `benchmark.record()` records numeric sample values.
- Python benchmark authoring supports operation, subsystem, and workflow scopes.
- In-process execution provides warmup, calibrated operation batching, lifecycle management, validation, process,
  Kit-independent NVML GPU and VRAM, and simulation metrics, descriptive statistics, and optional atomic result
  reporting.
- `benchmark.run()` and `benchmark.run_async()` execute decorated targets from ordinary and async Python
  applications, with unique default result files, an environment-configurable output root, explicit output paths, and
  in-memory-only execution.

### Changed

- The distribution metadata uses the canonical `isaacsim-performance` name and includes Python 3.12 classifiers.

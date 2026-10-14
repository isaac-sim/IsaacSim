# Changelog

## [Unreleased]

### Added

- Use the released ovperf-collector 0.3.1 for fresh process and GPU observations on Linux x86_64, with explicit
  `resource_backend` selection between ovperf and builtin providers.
- Add optional `capture_collector` artifacts and native MEASURE and sample ranges, including for timing-only benchmarks.
  Periodic collection is disabled unless capture is requested.
- Check native GPU discovery before calibration or warmup.

### Changed

- Make the native collector optional for public installs. Auto resource collection quietly uses builtin providers
  when the package is absent; explicit ovperf selection and capture still require the released collector.
- Use result.v3 to distinguish corrected CPU measurement boundaries and GPU attribution from historical results.
  The result field layout and metric names are unchanged.

### Fixed

- Skip Python namespace staging when Python support is disabled.
- Measure process CPU around the workload stopwatch for both providers, excluding synchronous resource queries.
  Captures retain these CPU boundaries; historical CPU KPIs that included query costs are not directly comparable.
- Distinguish process inactivity from unavailable PID observations in both providers. Omit unavailable process VRAM
  and never substitute unrelated devices for an inactive process.

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

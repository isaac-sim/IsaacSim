# Changelog

## [Unreleased]

### Added
- `ExecutionReport.composition_errors` reports unresolved composition arcs on the output stage; `composition_errors_truncated` flags an incomplete search.

### Fixed
- Release cyclic USD stage owners before removing staged exports, including failed conversions, so Windows file locks do not mask the execution report.
- Every export isolates rule execution from stale output files and cached layer opinions, including empty or deleted destinations. Preserve unrelated files and roll back failed publication; permission errors identify the locked destination and explain how to retry.
- Content composed through sublayers, references, and payloads is no longer dropped when `flatten_source` is `false`; the output is not relocatable.
- A failed source-stage export now fails the transform instead of transforming a stale base layer.
- `_collect_assets` preserves existing same-named assets, reuses identical files, and adds a numbered suffix for different content.

## [7.0.0a1] - 2026-09-16

### Added

- The independently built `isaacsim-asset` wheel includes `isaacsim.asset.transformer`.

### Changed

- The `isaacsim-asset` distribution is a platform-independent pure-Python wheel.

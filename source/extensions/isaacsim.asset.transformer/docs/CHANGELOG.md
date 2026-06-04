# Changelog

## [1.2.3] - 2026-05-27
### Changed
- Documented the input-stage mutation contract on `RuleInterface`: `args["input_stage"]` is informational only and must be treated as fully read-only, including its session layer. Rules that need to author overrides while reading from the original input must open a private `Usd.Stage` from `args["input_stage_path"]` and author into that stage's session layer. The contract exists because (a) `args["input_stage"]` may be a caller-owned Stage whose session layer carries user-driven overrides (visibility toggles, purpose settings, camera opinions), and (b) its root `Sdf.Layer` is shared via USD's process-wide layer cache with any other Stage observing the same file (notably the editor's active Stage); mutations there fire change notifications that have been observed to crash `librtx.hydra`.

## [1.2.2] - 2026-05-22
### Fixed
- `AssetTransformerManager._collect_assets` now anchors dependency discovery and asset-path remapping to the source layer's directory instead of the freshly-exported output layer. Previously, a source stage containing relative asset paths (e.g. `../textures/foo.png`) written to an output `payloads/` directory at a different filesystem depth would leave the relative strings verbatim in `payloads/base.usd`, producing unresolvable references at render time. The new behavior reuses the source resolver context to discover, copy, and rewrite each asset path so the output is portable regardless of the relative depth of `package_root` vs the source.
- `_collect_assets` now resolves relative asset paths against a priority-ordered list of candidate anchor directories (source root layer first, then every used sublayer's directory). This fixes the multi-sublayer case where an asset path authored in a sublayer was anchored at the sublayer's directory, not the root layer's: previously the path survived flatten verbatim and the remap missed it.
- `_collect_assets` now skips URI-style asset paths (`omni://`, `http://`, `file://`, etc.) in its remap step rather than attempting filesystem-relative resolution. Windows drive-letter paths (`C:\foo`) do not contain `://` and fall through to the `os.path.isabs` branch, which handles them correctly.

## [1.2.1] - 2026-05-05
### Fixed
- `make_explicit_relative` now normalizes backslash separators to forward slashes so USD asset paths, sublayer identifiers, and references emitted by the asset transformer are portable across platforms (e.g. avoids `./payloads\base.usda` on Windows).
- `AssetTransformerManager.run` writes the flattened `payloads/<base>.usd` path with forward slashes so the layer identifier and any downstream relative-path computation are platform-independent.

## [1.2.0] - 2026-04-09
### Changed
- Remove direct Omni / carb dependencies

## [1.1.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [1.0.3] - 2026-03-10
### Fixed
- Canonicalize quaternion orientations (real >= 0) on the flattened base layer so double-transform produces identical output

## [1.0.2] - 2026-03-02
### Fixed
- Ensure collected asset paths use explicit `./` or `../` relative prefixes

## [1.0.1] - 2026-02-13
### Fixed
- Tests failing due to misconfiguration of tests

## [1.0.0] - 2026-02-12
### Added
- First version of Asset Transformer manager.

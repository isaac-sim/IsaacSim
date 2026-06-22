# Changelog

## [1.1.2] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.1.1] - 2026-05-22
### Fixed
- `AssetTransformerWindow._run_actions()` no longer returns silently when the active stage is unsaved, when no actions are configured, when the output directory is unset, or when `AssetTransformerManager.run()` raises. Each failure now opens a modal dialog inside the Asset Transformer window in addition to the existing `carb.log_error` developer-console entry.
- The "unsaved active stage" precheck now shows a terse "Save Stage?" Yes / No / Cancel dialog before doing anything else. Yes routes through Kit's standard save (`omni.kit.window.file.save` — Save As file picker for new stages, in-place save for dirty saved stages) and re-runs the transformer on success. No skips the save and runs the transformer against the current on-disk state. Cancel aborts.
- A cancelled or failed Save (user dismissed the Save As dialog, write failed, etc.) now surfaces an error dialog instead of silently aborting -- the user no longer wonders why Execute Actions appeared to do nothing.
- `AssetTransformerWindow._run_actions()` now inspects `ExecutionReport.results` after a successful manager call and surfaces any per-rule failures (e.g. misconfigured rules that raise from `process_rule`) via the same error dialog. The dialog caps the listed failures at 10 entries with an "...and N more" suffix so a profile with many misconfigured rules cannot overflow the modal.

### Changed
- `AssetTransformerWindow._resolve_input_stage_path()` now returns `(path, error_message)` so the caller can render the specific failure reason in the UI.
- `AssetTransformerWindow._is_active_stage_unsaved()` now returns True for both (a) never-saved stages (`omni.usd.UsdContext.is_new_stage()`) and (b) previously-saved stages with pending in-memory edits (`omni.usd.get_dirty_layers(stage, recursive=True)` non-empty). Case (b) is required because the transformer reads from disk; without it a dirty saved stage would be transformed against the stale on-disk version.
- `AssetTransformerWindow._run_actions()` now invokes the unsaved-stage check before path resolution so it cannot be skipped by any branch in `_resolve_input_stage_path`.
- `AssetTransformerWindow._show_confirmation_dialog()` now accepts optional `confirm_label` and `cancel_label` arguments. Button widths are fixed (`ui.Pixel(120)`) rather than derived from a character-count heuristic that broke under custom fonts, high DPI, and non-Latin glyphs.
- Modal helpers (`_show_save_prompt`, `_show_error_dialog`, `_show_confirmation_dialog`) now share a single `_build_modal_window` factory backed by `_dismiss_active_modal`, which properly destroys any displaced modal (previously the displaced `ui.Window` leaked because no deferred-destroy ever ran for it). Modal displacement also logs a warning so latent collisions are visible.

### Added
- `AssetTransformerWindow._show_error_dialog()` modal helper with a single OK button.
- `AssetTransformerWindow._save_active_stage_and_run(on_saved)` thin wrapper around `omni.kit.window.file.save` that re-invokes `_run_actions()` from its completion callback (and surfaces an error dialog on save failure).
- `AssetTransformerWindow._show_save_prompt(on_yes, on_no)` 3-button modal helper (Yes / No / Cancel) used by the unsaved-stage precheck.
- `AssetTransformerWindow._run_actions_after_precheck()` extracted from `_run_actions()` so both the "stage already saved" path and the Save prompt callbacks share the same pipeline.
- `AssetTransformerWindow._dismiss_active_modal()` / `_build_modal_window()` shared helpers for modal-slot management; modals are now reliably destroyed (not just hidden) when replaced.
- New extension dependency: `omni.kit.window.file` (the Save / Save As implementation already shipped in the standard Isaac Sim app; the dependency is now declared explicitly).

## [1.1.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [1.0.1] - 2026-02-17
### Changed
- Minor UI workflow improvements

## [1.0.0] - 2026-02-12
### Added
- Created isaacsim.asset.transformer.ui

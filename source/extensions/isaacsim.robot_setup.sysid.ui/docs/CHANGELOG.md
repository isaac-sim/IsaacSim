# Changelog

## [1.5.0] - 2026-10-02

### Added
- **MuJoCo substeps** setting under Newton (MuJoCo) advanced simulation settings.

### Fixed
- Read parameter baselines from the selected physics backend, refresh on engine or backend changes, and retry failed reads.

## [1.4.0] - 2026-10-01
### Fixed

- Show the requested number of decimals in exact-entry float fields. The format string was silently ignored because `ui.FloatField` exposes `precision` rather than `format`.
- Deliver exact-entry value-changed and end-edit callbacks consistently, and re-clamp the displayed value when a bound changes.
- Clamp a recipe-supplied automatic-holdout training share to the same 5-95 range the field accepts.

### Changed

- Setting an exact-entry numeric field programmatically no longer fires its value-changed callback. Telemetry load and recipe apply already notify run state once, so the field previously notified twice.

## [1.3.1] - 2026-10-01
### Changed

- Telemetry preview messages are logged through `carb.log_*` instead of standard Python logging.

### Fixed

- Exporting a run report assesses telemetry quality once instead of twice.

## [1.3.0] - 2026-09-10
### Added

- Untuned-versus-tuned validation RMSE details and non-holdout fallback labeling on Results.

### Changed

- Exposed physics backend, all actuator runtimes, and explicit-actuator promotion policy without coercing settings when the engine changes.
- Exported actuator policy and backend selections, and included them in Check invalidation.
- Made interactive parameter writeback use verified instance-local stage overrides and report the live PhysX refresh.
- Documented the parameter and residual boundaries enforced for Isaac-hosted and standalone Newton backends.
- Replaced draggable numeric controls with bounded exact-entry fields, and reorganized Data into explicit analysis-window, automatic-holdout, manual-chunk, and preview sections.

## [1.2.2] - 2026-08-31
### Fixed

- Preserve the selected optimization parameters (optimize flag, bounds, initial) across a telemetry reload, so applying a Check command-alignment suggestion no longer clears the selection.
- Clear the stale Check report and suggestions after a suggestion is applied so the already-applied suggestion no longer stays displayed.
- Pin telemetry-preview authoring and teardown to the session layer so preview geometry never lands in the saved asset and tears down deterministically.
- Corrected the live-charts hover cursor on DPI-scaled displays and released hover polling on clear, so hovering no longer shows the wrong chart's series.
- Capped the telemetry chunk table at a scrollable height so many recorded chunks no longer push the rest of the panel off-screen.

### Changed

- Restyled the Results page onto the shared card design with a header, parameter-confidence verdict chips, an exported-artifacts summary, and a pre-run empty state.

### Added

- Added bulk parameter-bounds editing to the parameter table.

## [1.2.1] - 2026-08-11
### Fixed

- Exposed Newton controller, effort-clamp, and CUDA graph-capture settings plus Isaac Sim offline stepping in the UI and exported run specifications.
- Constrained differentiable Featherstone rollouts to their supported controller and effort-clamp modes.
- Added reproducible seed controls for CMA-ES, Bayesian optimization, and gradient-descent solvers.

## [1.2.0] - 2026-07-30
### Fixed

- Block Run when the pre-solve Check report is not ok instead of only requiring a fresh check.

## [1.1.0] - 2026-07-29
### Added

- Added the interactive System Identification window, panels, actions, telemetry preview, and UI tests.
- Added interactive controls for the explicit Newton PD actuator compatibility model and command-delay parameter.

### Changed

- Persist optimizer settings through the backend's explicit Carbonite settings adapter.
- Replaced the obsolete skeleton-only test filter with the UI extension's own test discovery.
- Routed telemetry preview messages through standard Python logging.
- Reworked the window into a six-stage workflow with persistent readiness, run status, navigation, and actions.
- Exposed optimization side effects before Run and grouped advanced simulation and parameter controls.

### Removed

- Removed contact-offset tuning controls from the workflow and parameter table.

### Fixed

- Constrained the recipe selector to its content height so it no longer consumes unused vertical space.
- Replaced the fragile live-cost `XYPlot` layout with a compact fixed-height graph.
- Made Check read the active controller-feedforward ComboBox value so a newly selected mode is never reported as the previous configured mode.
- Invalidated stale Check results after robot, residual, or solver changes and re-evaluated the gate when Run is clicked.
- Refreshed run readiness on timeline Play, Pause, and Stop events.
- Reported requested USD writeback failures as errors instead of successful completion.

## [1.0.0] - 2026-07-13
### Added

- Registered the `isaacsim.robot_setup.sysid.ui` extension ID as a config-only skeleton with a dependency on the headless backend skeleton.
- Added a Kit smoke test for the UI-to-backend dependency boundary.

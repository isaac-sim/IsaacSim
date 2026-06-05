# Changelog

## [1.1.2] - 2026-05-22
### Fixed
- `RobotPoser.solve_ik` cold-start (no `seed=` and no cached `_last_solution`) now seeds the Levenberg-Marquardt solver from a prioritized ladder — joint-limit midpoint, deterministic random restarts within joint limits, and finally zeros — instead of always starting from the all-zero configuration. This fixes silent `success=False` failures on redundant arms (e.g. Franka Panda 7-DOF) where the zero configuration is in the wrong convergence basin.
- When all cold-start candidate seeds fail, `solve_ik` now logs a single warning recommending an explicit `seed=` and returns the lowest-error attempt (instead of an arbitrary zero-seeded result), so callers can still inspect or visualize the near-miss.
- `RobotPoser.solve_ik` no longer trusts a stale `_last_solution` blindly: the cached seed is still tried first (single solve, lowest latency for tracking targets), but if it does not converge the full cold-start ladder is run in parallel as a fallback. Previously a wrong-basin cached solution from a far-away previous target would return `success=False` even when the ladder could have converged.

### Changed
- Cold-start (and `_last_solution` fallback) ladder attempts now run concurrently on a `concurrent.futures.ThreadPoolExecutor`. The latency of a cold-start ladder is bounded by the slowest single solve instead of the sum of all attempts, removing the ~5× cold-start tax. The LM solver and `chain.compute_fk` are stateless / pure with respect to the chain, so concurrent execution is safe.
- `RobotPoser.apply_pose` now accepts either a `dict[str, float]` (legacy) or a `PoseResult`. When a `PoseResult` with `success=False` is passed the call is a no-op and logs a warning, preventing callers from accidentally teleporting the robot to the lowest-error random-restart configuration returned on cold-start failure. Pass `result.joints` explicitly to override.

## [1.1.1] - 2026-05-21
### Fixed
- Sanitize pose names via `pxr.Tf.MakeValidIdentifier`; empty, colon-containing, and non-ASCII names no longer crash `store_named_pose` / `get_named_pose` / `delete_named_pose`.
- `store_named_pose` logs a warning when a sanitized name collides with an existing pose stored under a different raw name (for example `"home:v2"` and `"home/v2"`).

## [1.1.0] - 2026-03-05
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.0.0] - 2026-02-09
### Added
- Tools for robot inverse kinematics, named pose management, and joint state application in USD/Isaac Sim.

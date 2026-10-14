# Changelog

## [3.11.0] - 2026-09-29
### Added
- `GainTuner.apply_self_collision_override` / `restore_self_collision_override` / `self_collision_override_active`: disable articulation self-collisions under PhysX and Newton for a test run, without modifying the asset. Applying reports when a stronger opinion on the stage keeps them enabled.

### Changed
- Joint setup and the inertia link tree read the robot's `robotJoints` and `robotLinks` relationships, including nested robots those relationships name.

### Fixed
- Joints listed after a DOF the robot schema does not cover no longer show another joint's name and type.
- Joint setup names the articulation DOFs missing from the robot schema.

## [3.10.0] - 2026-08-30
### Added
- `joint_schema_attrs`: the advanced joint params (armature, joint friction, max joint velocity) as per-backend values. `resolve_joint_param` / `resolve_joint_params` return a `JointParamResolution` naming every schema's authored value, the schema the given backend and solver actually resolve, the effective value, and which authored values are shadowed or never read. Reads follow Newton's real schema-resolver order rather than assuming one schema per backend: under Newton an unauthored `newton:armature` / `newton:velocityLimit` falls back to `physxJoint:*`, `newton:friction` has no PhysX fallback at all, and under the MuJoCo solver `mjc:armature` and `mjc:frictionloss` resolve ahead of PhysX. `resolver_chain` / `param_resolver_chain` / `candidate_param_chains` expose the chain, and report it as undetermined when the solver is unknown rather than assuming the non-MuJoCo order.
- `author_joint_param` writes only the active backend's schema and leaves the other backend's authored value alone. The two solvers need different tuning, so a Newton/PhysX difference is an authoring choice rather than a conflict; `diverging_joint_params` reports such differences as a statement of fact, and `copy_joint_param_to_backend` copies one backend's value onto the other as an explicit opt-in action.
- `JointParamSpec.engine_default` and `NEWTON_DEFAULT_ARMATURE`: the value each engine simulates for an unauthored param. Newton copies its `NewtonConfig.armature` onto `ModelBuilder.default_joint_cfg`, so an unauthored armature is 0.1 under Newton and 0.0 under PhysX, and neither backend clamps an unauthored velocity limit. Callers previously had to guess, and zero is right only for friction.
- `SUPPORTED_BACKENDS` / `backend_supported` / `backend_display_label`, alongside `backend_write_schema` / `other_backend` / `newton_backend_selected` and the `BACKEND_*`, `SCHEMA_*`, `SOLVER_*` tokens.
- `backend_reads_usd_while_playing`: whether authoring one of these params mid-run changes what is being simulated. Measured rather than documented, and the two backends differ -- PhysX applies a mid-run `physxJoint:*` write to the running articulation, while Newton builds its model once in `ModelBuilder.add_usd` and registers no USD notice handler, so a `newton:*` write is unread until the next play. Callers should say so rather than let the edit look applied.
- `newton_solver_type`: the live Newton solver token, resolved from the stage's Newton physics scene when the solver config has not initialized yet. The resolver order depends on the solver, so this is required for a correct read, not just for display.
- `JointParamResolution.resolved_from_fallback_schema`: whether the value in effect comes from a schema an edit would not write -- Newton simulating `physxJoint:armature` because `newton:armature` is unauthored. Nothing about the number says so, and the first edit moves it onto the backend's own schema, which then wins the chain, so callers can now state both.
- `GainTuner.get_dof_effective_max_velocity` and `GainTuner.get_dof_engine_armature`: the velocity limit and armature the running engine actually uses, in the units both joint schemas store, or `None` before the timeline plays. The velocity sweeps scale their commands by the enforced limit, so a panel showing only the authored value could report one limit while the sweep excited the joint at another. `max_velocity_agrees` compares the two, tolerating the radian-to-degree conversion noise.
- `GainTuner.invalidate_physics_views`: release the articulation handle and the recorded response while keeping the robot selection. Switching the active physics engine invalidates the tensor views behind the bound articulation without nulling the handle, so callers need a way to re-acquire; a recorded response also belongs to the solver that produced it.
- `usd_layer_utils.is_physx_layer`: identify the `physx.usda` overlay alongside the existing physics/MuJoCo layer predicates.

### Changed
- `backend_write_schema` accepts only the two backends whose schemas are known, returning `None` for anything else. `SimulationManager.get_active_physics_engine` is typed to include `remotesim`, and a failed query reports nothing at all; both previously resolved to `physxJoint:*`, so an edit went to a schema with no evidence the running engine reads it. Callers are expected to disable the edit instead.
- `max_velocity_agrees` reports an unauthored USD limit against an enforced one as a disagreement. An unauthored limit means unlimited, so treating it as "nothing to compare" disabled the check in exactly the case that produces a wrong reading. Genuinely unknowable comparisons -- either side `None`, or NaN -- stay quiet.
- `get_dof_effective_max_velocity` distinguishes an unclamped joint from one it could not query: an infinite or practically infinite engine limit now reads as `inf` rather than `None`. Newton substitutes a finite 1e6 rad/s for an unclamped DOF, which is why the threshold is not a plain infinity test.
- `gain_save_targets.build_gain_save_plan` persists the advanced joint params, routing the `newton:*` value to the selected (neutral `physics.usda`) target and the `physxJoint:*` value to the `physx.usda` overlay. Only explicitly authored values are saved: applying either joint schema makes all of its attributes resolvable at their fallbacks, so saving whatever resolved would have written an untouched joint's `maxJointVelocity` into the asset as an explicit `inf` overriding every weaker opinion.
- Depends on `omni.usd.schema.newton`, which registers the codeless `NewtonJointAPI` the advanced joint params are read from and authored on.

### Fixed
- `GainTuner.stop_test`: return without touching the robot when no articulation is bound. Cancelling a test after the tuner was reset (stage close, robot swap, or an emptied robot dropdown) raised `AttributeError` instead of stopping cleanly.
- `gain_save_targets.apply_gain_save_plan`: apply the joint API schema on the prim spec it writes the value to. `ApplyAPI` records `apiSchemas` in the stage's edit target rather than the payload layer, so a saved `physx.usda` held `physxJoint:*` values on a prim with no `PhysxJointAPI`, which consumers that iterate applied schemas ignore.
- `newton_mujoco_solver_active` reported False before the first play. Newton's `solver_cfg.solver_type` is the placeholder `"None"` until the solver initializes, so the solver now falls back to the stage's Newton physics scene; it takes an optional `stage` for that.

## [3.9.0] - 2026-08-28
### Added
- `divergence` module deciding where a recorded joint response stops being physically meaningful: `divergence_limit` derives a per-joint magnitude bound from the joint's own travel limits (falling back to its commanded peak for a continuous joint), and `clip_series_to_valid` truncates each recorded series at its first unusable sample. Used by the Gain Tuner charts, which previously carried this criterion in the UI extension.

## [3.8.0] - 2026-08-27
### Changed
- Natural-frequency drive math now takes a required `is_angular` argument naming the drive's stored-gain convention, and the four conversions are renamed from `*_revolute_position` to `*_position_drive`.
- Gains already saved into an asset are not corrected on load; re-author any drive whose gains were written through these conversions. Importer-authored gains are unaffected.

### Fixed
- Revolute drives tuned by natural frequency or damping ratio no longer author damping 180/pi times too large; angular stiffness and damping are both stored per degree, so both take the same scale.
- The reported damping ratio no longer understates a revolute joint's true value by 180/pi, so a critically damped joint reads 1.0.
- Prismatic drives tuned by natural frequency no longer have the angular per-degree scale applied to their linear gains.

## [3.7.1] - 2026-08-25
### Fixed
- `gain_save_targets.list_gain_save_target_layers`: resolve save targets from the `payloads/Physics/` folder of the joints being tuned even when no joint authors a drive stiffness/damping. URDF/MJCF importer assets author only `maxForce`, leaving the gains at their schema fallbacks, and so offered an unresolved Save Target row instead of `physics.usda` / `physx.usda` / `mujoco.usda`. Discovery is anchored on those joints, so a scene holding several robots never offers one robot's physics layers as targets for another.
- `gain_save_targets.build_gain_save_plan`: author gains at the target layer's own prim path. A robot referenced into a scene composes under the scene namespace (`/World/Robot/joint`) while its asset layers author `/Robot/joint`, so saving into the asset wrote a spec the asset never reads and silently discarded the tuning.

## [3.7.0] - 2026-08-03
### Added
- Multi-backend gain sources: resolve each joint's active gains across PhysX DriveAPI, MuJoCo-native, and Newton actuators, reading Kp/Kd(/Ki) with source-appropriate labels. Only the active source is editable; MuJoCo-native gains are editable only while the Newton MuJoCo solver runs.
- Source-aware gain saving: route DriveAPI gains to a chosen physics layer, optionally mirror them into MuJoCo params (off by default), and write Newton actuator gains to their own layer.
- Command actuator- and MuJoCo-driven joints during the step, sinusoidal, snap-to-limits, and stress tests, classified from the gains that actually drive them.
- Natural-frequency drive-math conversions and joint effective-inertia queries.
- Discretization Sweep test mode: probe single-step position accuracy across a logarithmic physics-timestep ladder and classify each joint at a target dt; invalid dt bounds are rejected up front.
- Public results-ingest API on `GainTuner`: `ingest_sweep_results`, `clear_test_results`, `get_robot_prim_path`, `snapshot_recorded_trajectory`.

### Changed
- `gain_sources`: narrow defensive `except` guards to the expected USD/value error types so unexpected failures surface.

### Fixed
- Resolve Kp/Kd to a well-defined drive-mode fallback so joints without an inferable drive mode no longer misreport their mode.

## [3.6.2] - 2026-07-29
### Removed
- Remove unused bundled font assets.

## [3.6.1] - 2026-07-14
### Fixed
- Carried the Newton mimic-joint detection and flat observed-joint plot fixes (from 3.5.7) into the split core library: `joint_drive_attrs.is_joint_mimic` now recognizes Newton `NewtonMimicAPI` joints (with mimic gain-attribute getters returning `None` for them), and `gains_tuner` copies observed joint samples so the built-in Step/Sinusoidal plots show the real trajectory.

## [3.6.0] - 2026-07-14
### Changed
- Split UI and Kit dependencies into `isaacsim.robot_setup.gain_tuner.ui`; this extension now exposes the core tuning library (`GainTuner`, test runners, USD drive helpers).

## [3.5.7] - 2026-07-14
### Fixed
- `is_joint_mimic`: recognize Newton `NewtonMimicAPI` mimic joints alongside legacy PhysX mimic joints, so they show as Mimic instead of regular driven joints; mimic gain-attribute getters return `None` for Newton mimic joints.
- Built-in Step and Sinusoidal tests: fix observed-joint plot showing a flat line instead of the actual joint trajectory.

## [3.5.6] - 2026-07-09
### Fixed
- Expose the `IExt` entry point from the Python module declared in `extension.toml`.

## [3.5.5] - 2026-07-07
### Changed
- Use supported package-root imports for cross-extension APIs.

## [3.5.4] - 2026-06-29
### Changed
- Classify the gain-tuner UI builder as an internal lifecycle implementation detail; it remains import-compatible but is no longer published Python API.

## [3.5.3] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [3.5.2] - 2026-05-12
### Changed
- Menu entry is under Tools > Robotics > Asset Editors > Gain Tuner (matches `isaacsim.gui.menu` layout and user docs).

### Fixed
- Menu and `CreateUIExtension:Gain Tuner` show and focus the panel instead of toggling visibility, so a second open does not hide it.
- Visibility callback branches on the `visible` argument so the hide path runs when the window is closed (not only when `self._window.visible` was already false).

## [3.5.1] - 2026-05-07
### Added
- Extension tests for PD oscillation identification, drive-parameter math (`gain_tuner_drive_math`), registered `RobotTest` path, and USD drive spec queries
- Closed-form theory unit tests (no physics solver); golden literal stiffness/damping and damped-period references; RK4 discrete damped-oscillator test for `_analyze_oscillation` (isolates peak analysis from PhysX)
- Unit tests for `project_inertia_onto_axis` degenerate-axis warnings, `JointDriveMode` enum distinctness, and force-drive natural-frequency `m_eq` fallback behavior
- `GainTuner.add_inertia_updated_callback()` and `JointListModel.refresh_inertia_derived_columns()` so the gains table can refresh Natural Frequency / Damping Ratio after deferred inertia computation
- `usd_layer_utils` helpers to resolve the robot physics layer for save (prim stack, nested subLayers, on-disk ``payloads/Physics/physics.usda``) and unit tests for nested physx→physics subLayer composition
- Expanded unit tests: ``collect_gain_save_edits``, drive math edge cases, joint-table cell applicability, snap-to-limits hold classification, stress-test setup, built-in sinusoidal/step commands, and D6/articulation-root helpers

### Fixed
- `project_inertia_onto_axis()` logs a `carb.log_warn` when the joint rotation axis norm is below `1e-9` instead of silently returning `0.0` (which could zero out gain recommendations for affected joints)
- `JointDriveMode.MIMIC` is `3` (was `0`, aliased to `NONE`), so mimic joints are distinct in `list(JointDriveMode)` and NONE-mode joints no longer enter mimic-only NF/DR update paths
- Natural Frequency and Damping Ratio table columns refresh when accumulated joint inertia becomes available (after deferred `compute_joints_accumulated_inertia`), instead of staying at values computed with the `m_eq = 1` fallback at `JointItem` init
- `JointDriveMode` member docstrings corrected (were copy-pasted)
- Velocity-drive joints: Natural Frequency and Damping Ratio cells are blank in Natural Frequency table mode (tune damping via Stiffness mode instead; NF-based edits did not update drives when ``k=0``); non-applicable cells show a lighter gray background with no displayed values
- Save Gains to Physics Layer targets ``physics.usda`` / ``_physics.usd`` via joint prim-stack lookup, nested subLayers, and the conventional ``payloads/Physics/physics.usda`` path beside the root asset (instead of the strongest composition opinion on root or session from live UI edits); writable on-disk layers are savable even when Kit reports the nested layer as non-editable
- Fixed first column collapsing on small window sizes
- Revolute damping-ratio readback and natural-frequency damping updates use the same radian-equivalent stiffness as natural-frequency helpers (`gain_tuner_drive_math` / `JointItem`)
- Gain tuner oscillation tests: clear tuner state between robot builds and avoid duplicate unittest collection of the oscillation base
- Oscillation harness deletes ``/World/robot`` before each rebuild (avoids PhysX tensor views on deleted ``root_joint``), uses ``base_mass=1.0`` for oscillation scenarios so effective mass matches table math, and retries ``GainTuner.setup`` until articulation is bindable after scene updates
- Oscillation assertions compare inferred motion to the **USD-linear** natural frequency computed from drive ``GetStiffnessAttr`` / ``GetDampingAttr`` readback and GainTuner ``I_eq``; when PhysX motion is far below that model while USD still matches the design target, tests **skip** with an explicit message for PhysX bug triage
- Newton property query treats a missing articulation tensor backend as an empty articulation view instead of raising when reading ``count``
- ``GainTuner`` deferred link-mass update skips ``compute_joints_accumulated_inertia`` when physics tensors are not yet valid (avoids asyncio task errors during rapid scene rebuilds)
- Extension test ``stdoutFailPatterns.exclude`` widened so benign Kit log lines (SyntheticData frame history, USD stage open/close races) do not fail the harness on successful unittest runs

### Changed
- Removed the dedicated extension ``[[test]] name = "newton"`` oscillation job and ``TestGainTunerOscillationDynamicsNewton`` until Newton-backed dynamics testing is fully integrated

## [3.5.0] - 2026-04-23
### Added
- `Snap to Limits` test mode with per-joint pass/blocked/fail results
- `Stress Test` mode with Random Walk and Adversarial sub-modes
- Pluggable `RobotTest` registry on `GainTuner`
- Disable self-collisions and disable velocity limits toggles
- `get_test_result_metrics()` API

### Changed
- Default test mode is now `Snap to Limits`
- Per-mode duration controls replace shared `Test Duration` field

### Fixed
- Bulk edit preserves multi-selection across widget clicks
- Self-collision restore auto-restarts timeline for re-cook
- Stale test metrics cleared on new run
- Creep vs blocked classification uses error variance check

## [3.4.2] - 2026-03-06
### Fixed
- Clear physics, render, and assets-loaded subscriptions when window is hidden to avoid per-frame callbacks running while the panel is not visible

## [3.4.1] - 2026-03-04
### Fixed
- Fix api errors
- Fixed incorrect type annotations

## [3.4.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [3.3.1] - 2026-02-27
### Fixed
- Hang on test exit

## [3.3.0] - 2026-02-26
### Changed
- Migrate mass property queries from PhysX property query interface to Articulation tensor API (`isaacsim.core.experimental.prims.Articulation`).
- Remove `omni.physics.physx` dependency; mass, COM, and inertia are now queried via `get_link_masses()`, `get_link_coms()`, and `get_link_inertias()`.

## [3.2.1] - 2026-02-26
### Fixed
- Fix Vec3f/Vec3d type mismatch in inertia accumulation caused by robot schema double-precision change
- Fixed hanging shutdown on tests

## [3.2.0] - 2026-02-25
### Added
- Added unit tests for equivalent inertia computation

## [3.1.5] - 2025-12-16
### Changed
- Fixed Natural Frequency calculation:
    - Convert computed unit from Radians to Degree when storing the Stiffness
    - Properly handle Fixed robot chains
    - Properly use only the proper axis inertia when Revolute Joint, or directly use mass when prismatic.

## [3.1.4] - 2025-12-16
### Changed
- Consume Asset Changed events

## [3.1.3] - 2025-12-15
### Fixed
- Fixed consumption of events downstream on UI builder

## [3.1.2] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [3.1.1] - 2025-10-27
### Removed
- Remove unused import statement and commented code

## [3.1.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [3.0.6] - 2025-08-04
### Fixed
- Natural Frequency and Damping Ratio computations
- Unresponsive UI when bulk editing min step value

## [3.0.5] - 2025-07-02
### Changed
- Fixed refresh when robot changes on stage
- Fixed batch editing when tuning gains
- Removed "strength" and use stiffness/damping
- Pop up a warning asking for confirmation when save Gains to Physics Layer
- Make frames and table resizable and add scroll bar

## [3.0.4] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [3.0.3] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [3.0.2] - 2025-05-14
### Changed
- Fix deregiter of action when extension is shutdown without version number

## [3.0.1] - 2025-05-12
### Changed
- Register action without version number

## [3.0.0] - 2025-05-09
### Changed
- Update extension to use new Core Prim API and Robot Schema
- Redesigned the UI to be more user friendly and intuitive

### Added
- Added a new "Save Gains to Physics Layer" button to the UI
- Added new Sequential mode to the gains test
- Added Natural Frequency and Damping Ratio fields to the gains tuning mode.
- Added automatic selection between Position and Velocity command based on the joint gains
- Added switching between Acceleration and Force modes for the joint setting.

## [2.0.7] - 2025-05-07
### Changed
- Switch to omni.physics interface

## [2.0.6] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.0.5] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.0.4] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [2.0.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.0.2] - 2024-12-03
### Changed
- Isaac Util menu to Tools->Robotics menu

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-01
### Changed
- Extension renamed to isaacsim.robot_setup.gain_tuner.

## [1.1.2] - 2024-06-13
### Fixed
- Fixed bug where robot with zero-gains causes a math error in trying to take log(0).

## [1.1.1] - 2024-04-23
### Fixed
- Fixed post-test behavior where an Articulation is left with a non-zero velocity command.

## [1.1.0] - 2024-04-16
### Fixed
- Fixed bug where Gains Test Settings Panel had multiple ways of accumulating or forgetting state between tests when switching robots or toggling STOP/PLAY

### Added
- Added fields to add position and velocity impulses to the start of the robot trajectory.

## [1.0.1] - 2024-03-14
### Fixed
- Fixed logic around selecting Articulation on STOP/PLAY given new behavior in Core get_prim_object_type() function.

### Added
- Added helper text telling the user to click the play button to get started.
- Added more text clarifying Gains Test settings.

## [1.0.0] - 2024-03-08
### Added
- Initial version of Gain Tuner Extension

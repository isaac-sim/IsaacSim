# Changelog
## [0.3.4] - 2026-05-22
### Fixed
- `TestMaskingState` now snapshots the live `MaskingState` singleton in `setUp` and restores it in `tearDown`, so the `mock.Mock` backends installed by these tests no longer leak into later test classes (`TestRobotInspectorUI`, `TestSchemaUI`). Previously the leaked Mock caused `MaskingState.get_masking_layer_id()` to return a Mock that was passed to `Stage.MuteLayer()` from `generate_robot_hierarchy_stage`, raising `Boost.Python.ArgumentError` and cascading into a viewport-push count regression in `test_selection_pins_active_robot_scope`.
- `MaskingState.toggle_bypassed`, `toggle_anchored`, `toggle_deactivated`, and their `set_*` siblings no longer silently report success and pollute the in-memory `_bypassed_paths` / `_anchored_paths` / `_deactivated_paths` sets when the underlying `MaskingOperations` rejected the prim (e.g. RigidBody-only link, unsupported type). The in-memory state and the USD masking sublayer are now guaranteed to stay in sync: `toggle_*` returns `True` only when a USD opinion was actually written, and the in-memory set is mutated only on success.
- `MaskingState.toggle_bypassed` (and `set_bypassed`) now restore the prior mask if a pre-emptive unmask was performed and the subsequent bypass was rejected by the backend, so a failed bypass no longer silently un-masks an already-masked prim.
- The unbypass path of `toggle_bypassed` / `_set_bypassed_impl` now honors the `(success, _)` return of `_do_unbypass` and leaves the in-memory sets unchanged when the backend rejects the unbypass. Previously a rejected unbypass would still clear `_bypassed_paths` / `_deactivated_paths`, re-introducing the same in-memory/USD divergence on the opposite direction.

### Changed
- `MaskingOperations.bypass_prim` and `MaskingOperations.unbypass_prim` (internal to this extension) now return `(success: bool, joint_info: tuple[str, str] | None)` instead of `tuple[str, str] | None`. The previous overloaded `None` (which conflated joint-bypass success with rejection failure) is the root of the silent-success defect above. The asymmetry between `bypass_prim` (returns `(False, None)` when the masking sublayer cannot be acquired) and `unbypass_prim` (returns `(True, None)` when no sublayer exists -- nothing to undo) is now documented inline; the two operations have different storage contracts (write vs. read).

## [0.3.3] - 2026-05-07
### Changed
- Gate USD `SELECTION_CHANGED` subscription and `ObjectsChanged` listener on effective visibility. Background tabs and hidden windows no longer pay per-click or per-stage-tick handler cost.
- Resolve the active robot via `usdrt.Usd.Stage.GetPrimsWithAppliedAPIName("IsaacRobotAPI")` against Fabric, replacing the per-selection USD `prim.HasAPI` ancestor walk.
- Scope hierarchy generation and change tracking to the selected robot.
- Pin the active robot scope across selection changes, eliminating the per-click flash.
- Selecting a child robot in nested-robot setups now shows only the child, matching the new "single active robot" policy.
- `SelectionWatch._on_selection_changed` early-returns when the resolved tree-view item set is unchanged.

### Fixed
- Inspector no longer crashes with `AttributeError: '_StageModel__usdrt_stage'` when filtering the tree after a view-mode switch.

## [0.3.2] - 2026-05-01
### Changed
- Robot Inspector UI tests now open the window through the menu and use shared menu UI retry helpers for more robust CI behavior.

## [0.3.1] - 2026-03-06
### Changed
- Robot Inspector performance and stability improvements.
- Schema UI tests: tearDown now waits for stage assets to finish loading before cleanup to reduce flakiness.

## [0.3.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [0.2.0] - 2026-02-23
### Added
- Robot masking: deactivate, bypass, and anchor controls for joints and links via stage columns
- Masking state management with session-scoped USD masking layer
- Robot Inspector window replacing Robot Hierarchy with component inspection and masking UI
- Change robot hierarchy based on different view modes (Flat, Tree, Mujoco-style)
- Custom stage column delegates for deactivate, bypass, and anchor toggles
- Automatic masking layer cleanup on stage open/close

### Changed
- Renamed "Robot Hierarchy" window to "Robot Inspector"

## [0.1.0] - 2026-02-12
### Added
- Initial version

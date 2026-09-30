# Changelog

## [0.3.1] - 2026-09-03
### Fixed
- Avoid touching the global debug-draw interface when an empty manager resets during timeline or stage teardown.

## [0.3.0] - 2026-07-28
### Changed
- Moved the Create menu, property panel and `G` / `[` / `]` hotkeys into a new `isaacsim.robot_setup.virtual_gantry.ui` extension. This extension no longer depends on `isaacsim.gui.menu`, `omni.kit.menu.utils`, `omni.kit.property.usd` or `omni.kit.window.property`, so the rope runtime can be enabled without any UI stack. No behaviour change.

### Added
- `get_manager()`, returning the live `VirtualGantryManager` (or `None` when the extension is not running), so the UI extension can drive the running ropes from its hotkeys.

## [0.2.0] - 2026-07-28
### Added
- `newton` test lane, so the extension is exercised with `default_engine=newton` alongside the default PhysX lane.

### Changed
- Documented that the extension runs on both the PhysX and Newton backends. No code change was needed: the articulation tensor view used for the pose read and the wrench application is engine-agnostic. Verified in a local build on both `isaac-sim.sh` and `isaac-sim.newton.sh`.

## [0.1.0] - 2026-06-26
### Added
- Initial Virtual Gantry extension. Drives the `IsaacVirtualGantry` schema: a one-sided spring-damper "rope" that suspends one articulation body from an anchor prim, applied as a physics pre-step external wrench (no USD joint). Includes a `Create > Robotics > Virtual Gantry` menu entry, a property panel with Enable/Disable + live rope length, `G` / `[` / `]` keyboard hotkeys (toggle / shorten / lengthen the rope), and a runtime manager that drives every gantry prim on the stage (subscribes to the physics step on Play). On each enable the anchor prim re-anchors above the attach body's current position (matching the MuJoCo / Isaac ROS deploy gantry), so disable -> fall -> enable catches the robot in place. The rope line + anchor marker are drawn as an ephemeral `debug_draw` overlay (nothing is authored into the stage). PhysX backend; anchor position only (orientation reserved).

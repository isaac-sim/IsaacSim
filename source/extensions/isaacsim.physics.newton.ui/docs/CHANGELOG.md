# Changelog

## [1.7.0] - 2026-09-24
### Added
- Added UI widgets for Newton MPM and deformable schemas.

### Changed
- Updated Newton USD schema compatibility to 0.5.0.

## [1.6.1] - 2026-08-25
### Fixed
- Match the registered Newton simulator name when testing simulation capabilities.

## [1.6.0] - 2026-08-21
### Added
- Add a type-filtered **Physics > Mujoco** menu for applying MuJoCo API schemas and their required Newton API dependencies.
- Add a **Create > Physics > Mujoco** menu to instantiate the concrete MuJoCo typed prims (`MjcActuator`, `MjcKeyframe`, `MjcTendon`) that are standalone prim types rather than applied API schemas.
- Add a pop-up array editor for MuJoCo numeric array attributes (keyframe/actuator/tendon vectors) that lists every entry by index, shows the attribute name and a scrollable description, supports per-row remove and a single append control, and commits edits to USD as one undoable change only on **OK** (**Cancel** discards them).
- Enforce MuJoCo's fixed vector lengths (e.g. `mjc:gear` = 6, `mjc:gainPrm`/`mjc:biasPrm`/`mjc:dynPrm` = 10, solver `solref`/`solimp` families) in the array editor: the append control disables at the limit and **OK** pads any removed trailing entries with the attribute's MuJoCo per-index defaults so the written array stays a valid length.
- Route every MuJoCo numeric array attribute through the pop-up editor, including joint/tendon solver arrays, scene contact overrides, collision/equality `solimp`/`solref`, and `mjc:springlength`.

### Changed
- Direct Newton setup documentation to the physics schema menus instead of the generic **Edit API Schema** dialog.

### Fixed
- Render the pop-up array editor for MuJoCo arrays on applied API schemas (`MjcJointAPI`, `MjcSceneAPI`, `MjcCollisionAPI`, `MjcEquality*API`), which the custom Apply widgets own and therefore never resolved through the physics property-builder database; they previously fell back to a read-only text field.
- Preserve resolver-aware property builders, property ordering, and private ownership for schemas rendered by custom Apply widgets.
- Reject invalid unsigned-array values safely and restore unauthored array state on undo.
- Declare direct dependencies on `omni.usd.schema.mujoco` and `omni.usd.schema.newton` so schema discovery no longer relies on a transitive load order.
- Add remove-schema controls to Newton Scene, Joint, XPBD, and Kamino property frames.

## [1.5.0] - 2026-08-20
### Added
- Add MuJoCo and XPBD solver selection to the Newton scene UI.

### Changed
- Hide the MJCF angle conversion setting from USD scene properties.

## [1.4.1] - 2026-07-29
### Changed
- Stop PhysX UI from displaying newton's ui schema

## [1.4.0] - 2026-07-21
### Changed
- Compatible with isaacsim.physics.newton 0.11.0 (Newton 1.4.0)

## [1.3.2] - 2026-06-30
- Adding UI menu to apply Newton API schemas

## [1.3.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.3.0] - 2026-03-10
### Changed
- Compatible with isaacsim.physics.newton 0.6.0 (Newton 1.0.0).

## [1.2.2] - 2026-03-06
### Changed
- Upgraded Newton pip package from 1.0.0rc2 to 1.0.0rc3.

## [1.2.1] - 2026-03-05
### Changed
- Mujoco schema properties are now sorted alphabetically with the exception of properties that are common to Newton and Mujoco: those are put first in the list.

## [1.2.0] - 2026-03-04
### Added
- Resolver-aware property visibility: Newton and Mujoco schema properties are hidden or disabled when the other resolver (Newton vs MuJoCo) provides the value. Preference is determined by first authored value, otherwise Newton.

### Changed
- Property builders in `mujoco_schemas` and `newton_schemas` use callbacks to hide scene, joint, shape, and material properties when the resolver mapping is provided by the other backend.

## [1.1.0] - 2026-03-04
### Changed
- Add Overview.md, python_api.md and update docstrings

## [1.0.0] - 2026-02-06
### Changed
- Initial submit

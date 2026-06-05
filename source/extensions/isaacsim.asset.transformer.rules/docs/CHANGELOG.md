# Changelog

## [1.7.8] - 2026-05-27
### Fixed
- `FlattenRule.process_rule` no longer mutates any caller-owned USD state. Previously the rule deleted entries from `prim_spec.variantSelections` on the input stage's root layer and called `Reload()` on it twice; this fired change notifications on every other Stage observing the same layer through USD's process-wide layer cache. When the Asset Transformer was run against the editor's active stage, those notifications invalidated Hydra render product prims mid-frame and crashed `librtx.hydra` with `Unable to find RP Prim from previous update pass!`. The rule now ignores any `args["input_stage"]` passed by the caller and opens a fresh, private `Usd.Stage` from `args["input_stage_path"]`. Variant blocks and selections are authored to that private stage's session layer via `Usd.EditContext`; `Stage.Flatten()` composes the overrides into the output identically to the prior behavior. The private stage is garbage-collected on return, so no explicit cleanup is needed and no caller state (notably the editor's session layer, which carries user-driven overrides such as visibility toggles, purpose settings, and camera opinions) is ever touched.
- `FlattenRule._clear_all_variant_selections` now writes block opinions through `UsdVariantSet.BlockVariantSelection()` (which respects the active edit target) instead of directly deleting entries from the root layer's prim specs.

### Added
- Regression test `test_flatten_fires_no_change_notifications_on_input_root_layer` subscribes to `Sdf.Notice.LayersDidChange` for the duration of `process_rule()` and asserts the input root layer is never reported as changed; this catches mid-execution mutations even when the rule resets the layer to disk-clean state before returning (as the previous buggy implementation did).
- Regression test `test_flatten_preserves_caller_session_layer` seeds a non-rule opinion on a caller-owned stage's session layer, passes that stage via `args["input_stage"]`, runs the rule, and asserts the seeded opinion survives. Catches any future change that authors into (or clears) the caller's session layer.
- Regression test `test_flatten_does_not_mutate_input_root_layer_disk_content` copies the input asset to a tempfile, runs the rule, and asserts the file's bytes and mtime are unchanged.

## [1.7.7] - 2026-05-22
### Fixed
- `GeometriesRoutingRule._create_deduplicated_instance` no longer crashes with `pxr.Tf.ErrorException: Cannot create spec </Instances/<name>/<name>.purpose> because it already exists` when the prototype mesh's `purpose` attribute spec was already authored. The attribute spec is now reused if one already exists (e.g. when the template mesh authored `purpose` directly, or when the rule is re-run against an `instances.usda` reopened from disk via `Sdf.Layer.FindOrOpen`). The fallback catches the narrow `pxr.Tf.ErrorException` only, so typos and bad type-name arguments still surface as errors.
- `GeometriesRoutingRule` now propagates the *effective* purpose computed via `UsdGeom.Imageable.ComputePurpose()` rather than only checking the immediate parent's authored value. Collision meshes whose `purpose=guide` is authored on a grandparent (or any ancestor) -- e.g. a `collisions` Xform several levels above the mesh -- are now correctly kept hidden through the routed instance.
- `GeometriesRoutingRule._compute_instance_delta_hash` now incorporates the effective inherited purpose so that a prototype mesh referenced by both a visual Xform (purpose=default) and a collision Xform (purpose=guide) is split into separate `/Instances` entries. Previously the two usages were deduplicated into a single instance, which either left collisions visible or incorrectly hid visuals depending on which source was selected as the template.
- `GeometriesRoutingRule._make_references_non_instanceable` no longer writes the flattened stage back to the source layer's `realPath` on disk. The pre-process step now exports onto a working file at `<package_root>/<destination_path>/<basename>` and reopens the source stage there. When the rule runs via `AssetTransformerManager`, the source stage is already opened at that path so the mutation happens in place and `output_stage_path` continues to point at the manager's working file (preserving `f(f(x)) == f(x)` idempotency for the full Isaac Sim profile). When the rule runs directly with a caller-owned input outside `<destination_path>`, the working file lands under `<destination_path>` and the caller's input asset is left untouched.

## [1.7.6] - 2026-05-22
### Changed
- `PrimRoutingRule.process_rule()` now raises `ValueError` when `prim_types` is unconfigured (missing, `None`, or empty) instead of silently logging and returning `None`. The error message directs users to the `enabled = false` rule-spec flag as the canonical way to disable a rule entry. The "No matching prims found" path remains a successful no-op.

## [1.7.5] - 2026-05-18
### Added
- Standalone smoke tests now import every shipped rule module and statically reject `pxr` schema imports that are unavailable in isolated pip wheel environments.

### Fixed
- `JointStateAPIRule` no longer imports `pxr.PhysxSchema`; it uses shared PhysX schema tokens from `isaacsim.asset.importer.utils` so standalone wheel imports do not require Kit-only schema bindings.
- Added the standalone `isaacsim.asset.importer.utils` wheel dependency required by the rule token helper.

## [1.7.4] - 2026-05-11
### Changed
- `MjcToPhysxConversionRule` and `UrdfToMjcPhysxConversionRule` no longer author `PhysxMimicJointAPI` from joints with `NewtonMimicAPI`. The runtime consumes `NewtonMimicAPI` directly, so the equivalent PhysX mimic schema is redundant and is now omitted.
- No longer deletes physx mimic and articulation roots by default in isaacsim structure json file

## [1.7.3] - 2026-05-08
### Added
- `JointStateAPIRule` applies `PhysxSchema.JointStateAPI` (`linear` on prismatic, `angular` on revolute) to non-fixed joints missing it. Wired into the Isaac Sim profile between `Fix Physics Joint Poses` and `Route Materials`.
- `MakeListsNonExplicitRule` now normalizes the `isaac:physics:robotLinks` and `isaac:physics:robotJoints` relationships to `prepend` list ops, walking each prim's `GetPrimStack()` so sublayered PrimSpecs are also rewritten. A second profile entry is added to `isaacsim_structure.json` targeting these relationships.

## [1.7.2] - 2026-05-05
### Added
- `canonical_builtin_mdl_path` helper that returns the Kit-resolvable bare/suffix form of a built-in MDL path.
- `refresh_builtin_mdl_cache_async` helper that does a best-effort upgrade of the built-in MDL cache from `omni.kit.material.library.get_mdl_list_async` (filtered to `${kit}` / `${app}` token paths). The Kit extension awaits this on startup; standalone callers can ignore it.
- `omni.kit.material.library` declared as an `optional = true` dependency so the rules extension is usable in environments where the material library is not present.

### Changed
- `is_builtin_mdl` and `canonical_builtin_mdl_path` now consult an in-memory cache that is initialized at import time from a hardcoded fallback (the same Kit MDL names previously published as `BUILTIN_MDL_FILES` / `BUILTIN_MDL_PATH_SUFFIXES`) and upgraded by `refresh_builtin_mdl_cache_async` when `omni.kit.material.library` is available. Callers no longer have to await any setup before classification works, and the previously public `BUILTIN_MDL_FILES` / `BUILTIN_MDL_PATH_SUFFIXES` constants have been replaced by private fallback constants (`_FALLBACK_BUILTIN_MDL_BASENAMES`, `_FALLBACK_BUILTIN_MDL_SUFFIXES`).

### Fixed
- `MaterialsRoutingRule` writing material asset paths with backslash separators on Windows; all transferred-asset paths are now normalized to forward slashes via `make_explicit_relative`.
- `MaterialsRoutingRule` leaving absolute asset paths in the materials layer when an asset was not transferred; `_update_asset_paths_in_material` now relativizes any remaining absolute filesystem path against the materials layer directory.
- `MaterialsRoutingRule` now rewrites absolute or explicit-relative paths to project-local copies of Kit built-in MDLs (e.g. `C:/Dev/.../OmniPBR.mdl`) into their canonical bare form (`OmniPBR.mdl`) so Kit's MDL search paths resolve them. Previously these paths were left absolute, leaking host-specific paths into the package; relocating the file is unsafe because Kit's MDL system ties module identity to filesystem location.
- MDL texture path rewriting in `_remap_mdl_texture_paths` consolidated through the same forward-slash, relative helper for consistency.

## [1.7.1] - 2026-04-30
### Added
- `discover_rule_classes()` helper that walks the extension package and returns every concrete `RuleInterface` subclass
- Test that asserts every discovered rule class ends up in the global `RuleRegistry` after `register_all_rules()`

### Changed
- `register_all_rules()` now discovers rule implementations dynamically via `discover_rule_classes()` instead of a hard-coded list, so new rule modules added to the package are registered automatically

### Fixed
- `MergeMeshRule`, `MjcToPhysxConversionRule`, and `UrdfToMjcPhysxConversionRule` are now registered with the global `RuleRegistry`

## [1.7.0] - 2026-04-22
### Added
- MergeMeshRule to merge visual mesh prims grouped under rigid bodies via Scene Optimizer
- MjcToPhysxConversionRule to convert MJCF actuator/joint attributes to PhysX drive and joint schemas
- UrdfToMjcPhysxConversionRule to convert URDF joint attributes to MJCF actuators and PhysX schemas

## [1.6.0] - 2026-04-20
### Added
- `GeometriesRoutingRule` now quantizes hashed geometry values to a physical 1 mm grid derived from the stage's `metersPerUnit` (and the mesh's extent magnitude), so meshes authored in m, cm, or mm deduplicate against the same resolution instead of relying on raw float string comparison. Large arrays (points, normals, UVs) are quantized via NumPy in C.
- `GeometriesRoutingRule` now tracks a global `used_geometry_names` set so newly emitted geometry names cannot collide with pre-existing entries when deduplication is enabled.
- `visibility` added to `_INSTANCE_SPECIFIC_PROPERTIES` so it is no longer baked into the geometry hash.

### Changed
- `GeometriesRoutingRule._compute_geometry_hash` now ignores xform properties and replaces the previous `str(value)` hashing with a unit-aware quantization via the new `_quantize_value` helper.

### Fixed
- `MaterialsRoutingRule` no longer duplicates local texture files when the target path already exists with identical content; a `files_are_identical` check short-circuits the rename loop before appending a counter.

## [1.5.0] - 2026-04-14
### Added
- `DEFAULT_PROFILE_PATH` module-level constant exposing the path to the bundled `isaacsim_structure.json` profile
- `register_all_rules()` standalone function for registering all built-in rules without Kit's extension lifecycle

## [1.4.0] - 2026-04-09
### Added
- Test verifying multiple sibling meshes under a parent Xform are preserved as separate instanceable references

### Fixed
- GeometriesRoutingRule `_is_subtree_empty` not checking composition arcs, causing premature merging of sibling meshes that carry references but no authored properties
- GeometriesRoutingRule setting empty `typeName` on prim specs, which triggers a USD error
- GeometriesRoutingRule not propagating `purpose=guide` from parent Xform to child mesh prim in the instances layer
- `is_builtin_mdl` not recognizing `omnisurfacepresets.mdl`, `core_definitions.mdl`, and path-suffix forms like `nvidia/core_definitions.mdl`

## [1.3.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [1.2.0] - 2026-03-09
### Added
- PhysicsJointPoseFixRule to correct joint local poses after body world transforms change
- Idempotency test that runs the Isaac Sim profile twice and asserts stable output

### Fixed
- VariantRoutingRule leaving self-referencing payload arcs on extracted variant layers
- GeometriesRoutingRule non-idempotent merge decisions when VisualMaterials scopes or empty siblings remain after geometry extraction
- GeometriesRoutingRule floating-point precision drift in xformOp values across consecutive serialization round-trips
- GeometriesRoutingRule retaining root-level prims outside the defaultPrim hierarchy after flattening
- GeometriesRoutingRule quaternion sign ambiguity (q vs -q) causing idempotency test failure on double-transform
- SchemaRoutingRule, PropertyRoutingRule, and PrimRoutingRule using root layer defaultPrim instead of composed stage defaultPrim
- RobotSchemaRule not setting defaultPrim when no matching Isaac Robot schemas are found

## [1.1.2] - 2026-03-02
### Fixed
- Ensure all generated relative paths (references, payloads, sublayers) use explicit `./` or `../` prefixes

## [1.1.1] - 2026-02-26
### Changed
- Updated isaacsim_structure.json to delete Newton Mimic api in physx to avoid conflict

## [1.1.0] - 2026-02-17
### Added
- Overwrite schema rule

## [1.0.2] - 2026-02-16
### Fixed
- Fix geometry rule for negative scale and proper deduplicate

## [1.0.1] - 2026-02-13
### Fixed
- Tests failing due to misconfiguration of tests

## [1.0.0] - 2026-02-12
### Added
- First version of Asset Transformer core rules

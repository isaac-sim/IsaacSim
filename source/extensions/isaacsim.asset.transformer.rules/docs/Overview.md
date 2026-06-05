# Overview

Isaac Sim Asset Transformer Rules
==================================

Purpose
-------
Rule implementations for the Isaac Sim Asset Transformer. Each rule operates on a USD stage, extracting, routing, or correcting specific data to produce a structured, layered asset package suitable for simulation.

Built-in Rules
--------------

### Structure Rules

- **VariantRoutingRule** – Extracts variant set contents into separate layer files.
- **FlattenRule** – Flattens the source stage with selected variant choices into a base layer.
- **InterfaceConnectionRule** – Generates the interface layer that ties all payload layers together via references, sublayers, and variant sets. Also recovers extraneous root-level prims from the base layer into the interface layer so they remain reachable in the composed stage.

### Core Routing Rules

- **SchemaRoutingRule** – Moves applied API schemas matching a set of patterns to a dedicated layer.
- **PropertyRoutingRule** – Moves properties matching name patterns to a dedicated layer.
- **PrimRoutingRule** – Moves prims matching type patterns to a dedicated layer.
- **RemoveSchemaRule** – Removes specific API schemas from a target layer.

### Performance Rules

- **GeometriesRoutingRule** – Extracts mesh geometry into a shared geometries layer, deduplicates identical meshes, and builds an instances layer with instanceable references. Physics-purpose material bindings (`material:binding:physics`) are preserved as-is in the instance delta, pointing to their original target paths rather than being rerouted through `VisualMaterials`.
- **MaterialsRoutingRule** – Extracts visual materials into a shared materials layer, deduplicates identical materials, and updates material bindings. Materials with `PhysicsMaterialAPI` applied are skipped so they remain in the base layer and `material:binding:physics` relationships continue to resolve.

### Isaac Sim Rules

- **RobotSchemaRule** – Creates and populates the Isaac Robot schema (links, joints, sites) on a robot layer.
- **MakeListsNonExplicitRule** – Converts explicit list-op metadata (e.g. `apiSchemas`) to prepend list-ops so that downstream layers can extend rather than replace them.
- **PhysicsJointPoseFixRule** – Corrects physics joint local poses (`localPos0/1`, `localRot0/1`) after upstream rules (e.g. GeometriesRoutingRule) change body world transforms. Compares joint world poses between the original input asset and the current working stage, and updates the affected body's local pose so the joint world pose matches the original.

Writing New Rules
-----------------

All rules must subclass `isaacsim.asset.transformer.RuleInterface` and implement `process_rule()`. Register the class in `extension.py` via `RuleRegistry.register()` and add an entry to `data/isaacsim_structure.json` if it should be part of the default Isaac Sim profile.

### Idempotency

Rules **must** be idempotent: running the full profile twice on the same asset (once on the original, then on the first run's output) must produce identical USD layers, except for the `doc` metadata field which embeds absolute paths. The extension includes an idempotency test (`test_profile_idempotency.py`) that enforces this by transforming a test asset twice and asserting that all generated layers match byte-for-byte (after stripping `doc`).

Common sources of non-idempotency to avoid:

- **Floating-point drift** – Matrix composition/decomposition round-trips introduce noise. Quantize values to a fixed number of significant digits, or read canonical xformOp values directly when possible.
- **Stale composition arcs** – Extracting content from variant specs can leave behind self-referencing payloads or references on the destination prim. Strip any arcs that were not present in the source.
- **Inconsistent defaultPrim** – Use `self.source_stage.GetDefaultPrim()` (composed stage) rather than `self.source_stage.GetRootLayer().defaultPrim` (root layer only) to reliably resolve the default prim across re-transforms.
- **Non-deterministic merge decisions** – When deciding whether to merge a child into its parent, account for scaffolding prims (e.g. `VisualMaterials` scopes, empty overs) left behind by previous runs.
- **Extraneous root-level prims** – After flattening, root-level prims outside the `defaultPrim` hierarchy (e.g. viewport/render artifacts) do not survive a reference-based round-trip. The `InterfaceConnectionRule` handles this automatically by moving such prims from the base layer into the interface layer, but custom rules that introduce root-level prims should be aware of this composition constraint.

Usage
-----
Enable `isaacsim.asset.transformer.rules` in the Extension Manager. Rules are automatically registered on startup and become available to the Asset Transformer manager and UI.

Development
-----------
- Entry point: `isaacsim.asset.transformer.rules.extension.Extension`
- Default profile: `data/isaacsim_structure.json`
- Test suite: `isaacsim/asset/transformer/rules/tests/`

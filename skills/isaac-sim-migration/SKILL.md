---
name: isaac-sim-migration
description: "Audit and validate Isaac Sim release migrations; currently covers 5.1 to 6.0. Do NOT use for new-project setup or unrelated runtime bugs."
license: Apache-2.0
metadata:
  author: Hammad Mazhar <info@nvidia.com>
---

# Isaac Sim Migration

## Purpose

Move a project from one Isaac Sim release to another: inventory the affected
surfaces with the release audit tool, triage findings into owned work, and prove
each workflow still behaves correctly in the target release. This skill currently
covers Isaac Sim 5.1 -> 6.0; for projects that skip releases, see
"Migrating Across Releases" below.

## Prerequisites

- An Isaac Sim source checkout that provides
  `source/tools/isaac_sim_migration/audit_isaac_sim_6_0.py`, or a copy of that
  script placed in the project.
- Python 3 on `PATH`. The audit tool uses only the standard library.
- `$WORKSPACE_DIR` pointing at the project being migrated.
- A built or installed target-release Isaac Sim for the runtime validation step.

## Limitations

- Covers Isaac Sim 5.1 -> 6.0 only; other hops need that release's own audit tool.
- The audit is a static inventory helper, not a complete analyzer. It can miss
  project wrappers, generated files, binary USD layers, dynamic imports, custom
  OmniGraph nodes, extension settings, and behavior that appears only at runtime.
- A clean report is not proof that the project is migrated.
- The installed Isaac Sim 6.0 package does not bundle `tools/isaac_sim_migration/`.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| `audit_isaac_sim_6_0.py` not found | Running against an installed package instead of a source checkout | Copy the script from the Isaac Sim repository into the project and run it with any Python 3 |
| Audit reports nothing but the app still breaks | Static scan cannot see dynamic imports, generated files, or binary USD layers | Fall back to runtime validation per workflow; do not treat a clean report as done |
| CI fails on `review` findings | `--fail-on` set too broadly for the project | Gate on `--fail-on required` and triage `review` findings manually |
| Imports resolve but behavior changed | Migration changed timing, authored USD schemas, graph data flow, or output metadata | Run the empirical smoke test for that workflow instead of relying on import success |
| Deprecated extension imports cleanly | Deprecation is logged as a warning, not an error | Treat the warning as a migration finding and move to the replacement extension |

## Instructions

### 1. Scope first

Before editing code, identify:

- Source and target Isaac Sim versions.
- Install type: source checkout, binary package, container, or pip packages.
- Customer-owned surfaces: app `.kit` files, extension dependencies, Python,
  USD/USDA stages, Action Graphs, ROS 2 workspaces, robot assets, sensor
  configs, benchmark scripts, CI jobs, and generated data pipelines.
- Runtime smoke tests that prove the workflow still behaves correctly.

For Isaac Sim 5.1 -> 6.0, start with the 6.0 migration guide and the project
audit tool in `source/tools/isaac_sim_migration/audit_isaac_sim_6_0.py`.

### 2. Run the static audit

Run the audit tool from an Isaac Sim source checkout:

```bash
python source/tools/isaac_sim_migration/audit_isaac_sim_6_0.py "$WORKSPACE_DIR" --max-results 0
```

For automation, use JSON output. The default exit code is success so teams can
collect migration inventory without breaking CI:

```bash
python source/tools/isaac_sim_migration/audit_isaac_sim_6_0.py "$WORKSPACE_DIR" --json --max-results 0
```

Gate only when the project explicitly wants unresolved findings to fail CI:

```bash
python source/tools/isaac_sim_migration/audit_isaac_sim_6_0.py "$WORKSPACE_DIR" --fail-on required
```

Do not gate on `review` findings without project-specific triage rules.

### 3. Triage findings

Treat each finding as an owner assignment:

- `required`: likely breaking or removed behavior; assign a workflow owner and
  validate before closing.
- `review`: migration inventory; inspect manually and validate if the code path
  is active in the project.

Do not clear a finding because imports resolve. Isaac Sim migrations often
change timing, authored USD schemas, graph data flow, package layout, and output
metadata without producing immediate import errors. A deprecated extension may
also import successfully while logging a deprecation warning; treat those
warnings as migration findings and move the workflow to the replacement
extension.

### 4. Validate at runtime

For each affected workflow, run an empirical smoke test in the target Isaac Sim
release:

- Extension/app: start with the migrated dependency set and open one stage.
- Simulation: reset, step, and verify expected prim/articulation state.
- Sensors: create or load the sensor, step frames, and inspect cadence and
  output fields.
- Action Graphs: open the USD, let deprecation-manager graph migration run,
  save a migrated layer, then reopen it.
- ROS 2: launch, publish/subscribe one message path, and verify frame IDs,
  message fields, QoS-sensitive behavior, and package names.
- Robot assets: verify catalog path, validation status, articulation load,
  mimic joints, and importer/exporter round trip where relevant.
- Benchmarks: run one warmup and one measured phase; compare KPI names and
  recorder output shape against the old baseline.

Record evidence: target Isaac Sim version, command, app experience, input asset
or workspace, expected 5.x behavior, observed 6.0 behavior, pass/fail result,
and owner for remaining gaps.

## Examples

Audit a customer workspace and read the full inventory:

```bash
export WORKSPACE_DIR=/path/to/customer_project
python source/tools/isaac_sim_migration/audit_isaac_sim_6_0.py "$WORKSPACE_DIR" --max-results 0
```

Collect the same inventory as JSON for a migration tracking ticket, without
failing the job:

```bash
python source/tools/isaac_sim_migration/audit_isaac_sim_6_0.py "$WORKSPACE_DIR" \
  --json --max-results 0 > migration-audit.json
```

Gate a CI job on unresolved breaking findings only:

```bash
python source/tools/isaac_sim_migration/audit_isaac_sim_6_0.py "$WORKSPACE_DIR" --fail-on required
```

Validate one triaged ROS 2 finding in the target release before closing it:

```bash
"$ISAAC_SIM_DIR/python.sh" -c "import isaacsim; print(isaacsim.__file__)"
# then launch the project's ROS 2 graph and verify one publish/subscribe path,
# frame IDs, and package names against the 5.x baseline
```

## Deprecation Warnings

Use deprecation warnings as a signal, not as the only migration mechanism.
`isaacsim.core.deprecation_manager` currently migrates deprecated settings and
saved OmniGraph node namespaces, and logs what it changes. It is not a general
system that guarantees every deprecated Python symbol warns on import or use.

## Migrating Across Releases

Each release migration has its own audit tool and guides; this skill currently
covers Isaac Sim 5.1 -> 6.0 via `audit_isaac_sim_6_0.py`. When a project skips
releases, run each release-to-release migration in order (for example
5.1 -> 6.0, then 6.0 -> 6.1), using that release's audit tool and guides, and
validate the project at each hop before starting the next one.

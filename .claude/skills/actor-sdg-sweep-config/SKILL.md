---
name: actor-sdg-sweep-config
description: "Generate validated Isaac Sim Replicator Agent YAML variations and optionally batch-run Actor SDG. Use when sweeping an IRA config or preparing batch Actor SDG runs."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# Actor SDG Sweep Config

## Purpose

Generate reproducible variations of an Isaac Sim Replicator Agent (IRA) YAML configuration. A sweep samples selected
configuration fields across multiple variants. Each variant receives a distinct random seed and writer output
directory. Optionally run `actor_sdg.py` on each generated variant via `scripts/batch_actor_sdg.py`.

## Security

Schema validation runs by default. `--omit-schema-check` writes variants without JSON Schema verification; use only
when the caller explicitly accepts unchecked YAML (for example schema unavailable offline). Prefer `--dry-run` to
preview without writing files. Batch execution launches the local `actor_sdg.py` entry point via subprocess with a
fixed argument list (`shell=False`).

## Prerequisites

- Python 3 with PyYAML.
- `jsonschema` and the IRA `config_schema` directory for schema validation.
- An existing IRA YAML configuration with the `isaacsim.replicator.agent` root key.
- A writable output directory, preferably under `$WORKSPACE_DIR`.
- An Isaac Sim install or source build for optional Actor SDG execution.

## Limitations

- The generator changes fields that already exist in the base configuration. It may add the standard root `seed`
  field when the base omits it.
- Schema validation cannot run without `jsonschema` and the matching schema from the installed IRA extension.
- Config generation is deterministic. Actor SDG execution is not idempotent and can replace dataset outputs.
- The batch runner executes variants sequentially. It does not schedule work across machines or GPUs.

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/config_variation_generator.py` | Generate validated config variants and `manifest.csv`. | CLI flags through `argparse`; run with `--help` for details. |
| `scripts/batch_actor_sdg.py` | Preview or run `actor_sdg.py` for generated variants. | CLI flags through `argparse`; run with `--help` for details. |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke a script with `run_script()`:

```text
run_script(
    "scripts/config_variation_generator.py",
    args=[
        "--base", "base.yaml",
        "--sweep", "sweep.yaml",
        "--output", "variants",
        "--dry-run",
    ],
)
```

From the repository root, run the generator with Python:

```bash
python3 skills/actor-sdg-sweep-config/scripts/config_variation_generator.py \
  --base base.yaml \
  --sweep sweep.yaml \
  --output "$WORKSPACE_DIR/actor_sdg_variants" \
  --dry-run
```

## Sweep specification

The sweep file contains a positive `count` and a `parameters` list:

```yaml
count: 12
parameters:
  - path: isaacsim.replicator.agent.character.groups.workers.num
    type: choice
    values: [5, 10, 20]
  - path: isaacsim.replicator.agent.simulation_duration
    type: fixed
    value: 30
```

Supported sampling types are:

| Type | Required value |
|---|---|
| `choice` | A non-empty `values` list. |
| `uniform` | An ordered numeric `range`. |
| `random_int` | An ordered integer `range`. |
| `linspace` | An ordered numeric `range`. |
| `fixed` | A `value`. |

## Workflow

### 1. Validate the inputs

1. Resolve the base configuration and sweep file.
2. Parse both files as YAML mappings.
3. Require the `isaacsim.replicator.agent` root key.
4. Confirm that every dotted sweep path exists in the base configuration, except the standard root `seed`.
5. Confirm the count and sampling values are valid.

### 2. Preview the sweep

Run the generator with `--dry-run`:

```bash
python3 skills/actor-sdg-sweep-config/scripts/config_variation_generator.py \
  --base base.yaml \
  --sweep sweep.yaml \
  --output "$WORKSPACE_DIR/actor_sdg_variants" \
  --seed 42 \
  --dry-run
```

Report the count, parameter paths, sampling types, seed, schema location, and output directory. The generator refuses
to replace existing variant directories by default. Pass `--overwrite` only after the user approves replacement.

### 3. Generate the configurations

Remove `--dry-run` after the preview is correct:

```bash
python3 skills/actor-sdg-sweep-config/scripts/config_variation_generator.py \
  --base base.yaml \
  --sweep sweep.yaml \
  --output "$WORKSPACE_DIR/actor_sdg_variants" \
  --seed 42
```

Use `--schema-dir` when automatic schema discovery cannot find the installed IRA schema. Do not use
`--omit-schema-check` unless the user accepts unvalidated output.

### 4. Verify the configurations

1. Confirm the command exits with code `0`.
2. Confirm the number of `variant_*/config.yaml` files matches the requested count.
3. Read `manifest.csv` and one generated configuration.
4. Report whether schema validation passed, was skipped, or was unavailable.
5. Do not call the configurations valid when schema validation did not run.

### 5. Preview optional data generation

Use the batch runner in dry-run mode:

```bash
python3 skills/actor-sdg-sweep-config/scripts/batch_actor_sdg.py \
  --variants "$WORKSPACE_DIR/actor_sdg_variants" \
  --python-sh "$ISAAC_SIM_DIR/python.sh" \
  --actor-sdg "$ISAAC_SIM_DIR/tools/actor_sdg/actor_sdg.py" \
  --dry-run
```

Before removing `--dry-run`, report the selected configurations, output locations, and expected runtime. Actor SDG
execution can replace generated data, so run it only after the user approves the batch.
Execution also requires `--allow-trusted-scripts`. Explain that configurations and referenced assets can execute
Python with the user's permissions, and obtain explicit acknowledgement that all selected configurations and
referenced assets are trusted. Do not add this flag automatically. Dry-run mode does not require it.

### 6. Report the result

Report the base configuration, sweep file, output directory, count, seed, manifest path, validation status, and any
failed variant IDs. After data generation, read `run_results.csv` and report each failed variant.

## Troubleshooting

| Error or symptom | Cause | Solution |
|---|---|---|
| Sweep path does not exist | The dotted path is absent from the base configuration. | Correct the path or add the field to the base configuration first. |
| Schema validation unavailable | `jsonschema` or the matching IRA schema is missing. | Install `jsonschema` and pass `--schema-dir`. |
| Output variants already exist | The generator protects existing files. | Inspect the variants, then rerun with `--overwrite` after approval. |
| `actor_sdg.py` not found | The install or source path was not detected. | Pass the exact path through `--actor-sdg`. |
| A batch variant fails | Actor SDG returned a nonzero exit code or timed out. | Inspect its log and the matching row in `run_results.csv`. |

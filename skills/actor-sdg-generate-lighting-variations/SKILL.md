---
name: actor-sdg-generate-lighting-variations
description: "Generate deterministic USD lighting override sublayers for Actor SDG. Use when randomizing lights, creating lighting variations, or preparing IRA prop_asset_paths."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# Actor SDG Generate Lighting Variations

## Purpose

Generate deterministic lighting variations without changing the base USD stage. Each output is a USD sublayer, a
file that overrides selected properties from another stage. Use the sublayers with Isaac Sim Replicator Agent (IRA)
through `environment.prop_asset_paths`.

## Prerequisites

- An Isaac Sim install or source build exposed through `$ISAAC_SIM_DIR`.
- A local USD stage or an `omniverse://` stage URL.
- A writable output directory, preferably under `$WORKSPACE_DIR`.
- The Isaac Sim Python launcher, because the generator uses `pxr`.

## Limitations

- The generator changes existing `DomeLight`, `DistantLight`, `SphereLight`, `RectLight`, `CylinderLight`, and
  `DiskLight` prims. It does not create lights.
- The generated sublayers override intensity, color temperature, exposure, and optional color tint. They do not
  randomize light transforms or textures.
- A successful file-generation run does not prove that every variation renders well. Inspect representative frames
  before starting a large dataset run.
- `omniverse://` inputs require an authenticated Omniverse connection in the Isaac Sim runtime.

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/lighting_variation_generator.py` | Scan a stage and generate lighting override sublayers plus `manifest.csv`. | CLI flags through `argparse`; run with `--help` for details. |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke the generator with `run_script()`:

```text
run_script(
    "scripts/lighting_variation_generator.py",
    args=["warehouse.usd", "--dry-run"],
)
```

From the repository root, use the Isaac Sim Python launcher:

```bash
"$ISAAC_SIM_DIR/python.sh" \
  skills/actor-sdg-generate-lighting-variations/scripts/lighting_variation_generator.py \
  warehouse.usd --dry-run
```

On Windows, replace `python.sh` with `python.bat`.

## Workflow

### 1. Validate the stage

Run a dry scan before writing files:

```bash
"$ISAAC_SIM_DIR/python.sh" \
  skills/actor-sdg-generate-lighting-variations/scripts/lighting_variation_generator.py \
  warehouse.usd --dry-run
```

Stop if the stage cannot open or contains no supported lights. Report the discovered light paths and types.

### 2. Prepare the output

Use these defaults unless the user specifies other values:

| Setting | Default |
|---|---|
| Variation count | `10` |
| Seed | `42` |
| Intensity multiplier | `0.3 3.0` |
| Color temperature in Kelvin | `2700 7500` |
| Exposure | `-2.0 2.0` |
| Color tint | `0.0` |

The generator refuses to replace matching output files by default. If they exist, report their count and path. Pass
`--overwrite` only after the user approves replacing those files.

### 3. Generate the sublayers

Run with explicit parameters:

```bash
"$ISAAC_SIM_DIR/python.sh" \
  skills/actor-sdg-generate-lighting-variations/scripts/lighting_variation_generator.py \
  warehouse.usd \
  --count 10 \
  --output-dir "$WORKSPACE_DIR/lighting_variations" \
  --seed 42 \
  --intensity-range 0.3 3.0 \
  --temperature-range 2700 7500 \
  --exposure-range -2.0 2.0 \
  --color-tint 0.0
```

The same stage, seed, parameters, and output names produce the same results. This is idempotent: repeating the command
produces the same final files.

### 4. Verify the outputs

1. Confirm the command exits with code `0`.
2. Confirm the number of generated `.usda` files matches `--count`.
3. Read `manifest.csv` and one generated sublayer.
4. Confirm the sample contains only lighting override opinions.
5. Add one generated path to `environment.prop_asset_paths` and render a representative frame.

### 5. Report the result

Report the base stage, output directory, count, seed, parameter ranges, manifest path, and checks performed. Do not
claim that the outputs rendered correctly unless a render check completed.

## Troubleshooting

| Error or symptom | Cause | Solution |
|---|---|---|
| `No supported light prims found` | The stage has no supported USD light types. | Inspect the stage and provide a stage with existing lights. |
| `No module named pxr` | The script ran with system Python. | Run it with the Isaac Sim Python launcher. |
| Stage does not open | The path is invalid or the Omniverse session is unavailable. | Check the local path or authenticate the Omniverse connection. |
| Output files already exist | The generator protects existing variations. | Inspect the files, then rerun with `--overwrite` after approval. |

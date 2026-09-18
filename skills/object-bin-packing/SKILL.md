---
name: object-bin-packing
description: "Pack boxes/parcels into a bin, pallet, or container and render SDG with IRO's bin_pack harmonizer. Use for warehouse/logistics packing synthetic data."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# Object Bin Packing (IRO `bin_pack` harmonizer)

## Purpose

Procedurally **pack many 3D objects into a cuboid bin** (bin / pallet / container / carton) and
render synthetic data (RGB, 2D/3D boxes, segmentation) — a focused specialization of
`object-simulation-sdg` built on IRO's `bin_pack` **harmonizer** (`isaacsim.replicator.object`).
Placement is **gravity-stable**: boxes are placed bottom-up and only where they rest on the bin
floor or are supported from below, so no box floats in mid-air.

**Read `object-simulation-sdg` first** (the description-file schema, run modes, output layout) and
`action-and-event-data-generation` for the launcher and env contract. This skill covers only the
bin-packing specialization on top of them.

## Limitations

- Requires `isaacsim.replicator.object` (IRO); packing runs inside a live Isaac Sim, not offline.
- `bin_pack` places objects gravity-stable inside a container; it does not simulate the packing
  motion, so it produces end-state layouts rather than a manipulation trajectory.
- Layouts vary run to run unless the config pins a seed.
- Only the `scripts/bin_pack_config.py` generator is offline-validated here; a live headless IRO
  render depends on the box set produced by `demo_bin_pack`.

## When to use (vs siblings)

- **Use this** to densely pack/stack static objects into a bin by their bounding boxes — warehouse
  parcels, palletized cartons, shipping totes.
- For general randomized *scatter* placement (not packed), a table drop, or non-box geometry, use
  `object-simulation-sdg` directly.
- For *moving* people/robots use `actor-simulation-sdg`; for rare events (topple/spill) use
  `event-generation`.

## Prerequisites

- Installed Isaac Sim with the Action and Event Data Generation app (`$ISAAC_SIM_DIR`), including
  `isaacsim.replicator.object.core`. Gravity-stable packing and `fill_ratio` are assumed; both are
  present in the build this app pins.
- NVIDIA GPU + current driver for an actual render (the offline generator needs neither).
- Env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$WORKSPACE_DIR`, `$AEDG_OUTPUT`.

## Available scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/bin_pack_config.py` | Generate a self-contained IRO bin-pack description (offline); optionally `--validate` it | CLI flags via argparse (`--help`) |

The generator packs **SimReady cardboard boxes streamed from the content server** — no local
assets, no `PATH_TO_BOXES`. It derives the config `version` from your build with `--from-ext` so it
stays in sync, defines the content-server macro chain inline (self-contained for a headless run),
and writes a camera + dome light + output switches so the config renders as-is.

```bash
# generate a runnable config (version matched to your build)
python3 scripts/bin_pack_config.py --from-ext "$ISAAC_SIM_DIR" \
    --output-path "$AEDG_OUTPUT/iro_bin_pack" > scene_bin_pack.yaml

# partially loaded bin (~60% of bin volume), bigger bin, scale the box counts
python3 scripts/bin_pack_config.py --from-ext "$ISAAC_SIM_DIR" --output-path "$AEDG_OUTPUT/iro_bin_pack" \
    --bin-size 500 250 250 --fill-ratio 0.6 --scale-counts 1.5 > scene_bin_pack.yaml

# offline gate: confirm the config is runnable on IRO (no launch) — prints OK / problems
python3 scripts/bin_pack_config.py --from-ext "$ISAAC_SIM_DIR" \
    --output-path "$AEDG_OUTPUT/iro_bin_pack" --validate
```

From agent runtimes with skill-execution helpers: `run_script("scripts/bin_pack_config.py", args=["--help"])`.

## The `bin_pack` harmonizer

A **harmonizer** correlates how multiple mutables randomize together. `bin_pack` packs every
geometry group that references it into one cuboid, by each object's axis-aligned bounding box:

```yaml
isaacsim.replicator.object:
  version: EXT_VERSION            # from the installed extension; derive with --from-ext
  # ... settings, camera, light ...
  bin_pack_H:
    harmonizer_type: bin_pack
    bin_size: [400, 200, 200]     # cuboid dimensions
    fill_ratio: 0.6               # optional (0, 1]: cap packed volume at this fraction of the bin

  boxes:
    count: 16
    type: geometry
    subtype: mesh
    tracked: true                 # only tracked mutables appear in labels/segmentation/3D boxes
    transform_operators:
    - transform:                  # <-- placement comes from the harmonizer
        distribution_type: harmonized
        harmonizer_name: bin_pack_H
        pitch: local_aabb
    - scale:
        distribution_type: range
        start: [1.2, 1.2, 1.2]
        end:   [1.25, 1.25, 1.25]
    usd_path: $[/cardboard_boxes_root]/White_A/WhiteCorrugatedBox_A08_30x30x30cm_PR_NVD_01.usd
```

Key behaviors:

- **Gravity-stable placement** — deepest-bottom-left-fill with a base-support + center-of-mass
  constraint; boxes rest on the floor or on boxes below them and never float.
- **`fill_ratio`** (optional, `(0, 1]`) — cap the total packed box volume at this fraction of the
  bin, for partially loaded bins. Invalid values are rejected at load.
- **Layout diversity** — placement is seeded, so it is reproducible per `seed` but varies frame to
  frame; different `seed`s give different packings.
- **Overflow** — objects that do not fit are moved out of view (they still exist in the stage).
- **Size variety comes from choosing differently sized assets**, not per-axis scale hacks; the
  generator curates a brown/white box set spanning ~15 cm flats to ~77 cm cubes.

## Run (headless)

```bash
cd "$ISAAC_SIM_DIR"
./isaac-sim.sh --no-window --allow-root \
  --enable isaacsim.replicator.object.core \
  --/log/level=warn --/windowless=True \
  --/config/file="$PWD/scene_bin_pack.yaml"
```

Or in the **Object SDG** panel (enable `isaacsim.replicator.object.core` + `.ui`): pick
`demo_bin_pack` from the dropdown below **Simulate** (it needs no local assets) and click
**Simulate**. Extension log lines are tagged `[METROPERF][isaacsim.replicator.object.core:<version>]`.

## Verify it worked

IRO writes one folder per enabled output switch under `output_path` (NOT per-camera):

```bash
for d in images labels segmentation; do
  n=$(ls "$AEDG_OUTPUT/iro_bin_pack/$d" 2>/dev/null | wc -l); echo "$d: $n files"
done
```

A non-empty `images/` with boxes packed into the bin (and, on inspection, no magenta/black boxes
and no floating boxes) means it worked. An empty `images/` means the run captured nothing — check
`tracked: true` on the groups and that the `version` matches your build.

## Integration points

- **Consumes:** SimReady cardboard boxes from the content server (`$[/cardboard_boxes_root]/...`);
  or your own `usd_path` box USDs.
- **Produces:** RGB + 2D/3D boxes + segmentation under `output_path`; can also emit captions in-run
  via `caption_writer: CombinedIROSceneGraphWriter` (see `vlm-scene-captioning`).
- **Parent:** `object-simulation-sdg` (schema, run modes) → `action-and-event-data-generation`.

## Troubleshooting

- **Boxes float in mid-air** — the packer fell back to intersection-only checks, which means the
  resolved `isaacsim.replicator.object.core` predates gravity-stable packing. Run the app whose
  `.kit` pins the extension rather than an ad-hoc `--enable`.
- **Magenta or near-black boxes** — a White_A/Flat_A box asset attaches materials via a payload and
  was instanced through the scene_instance cache. The pinned build auto-disables instancing for
  such assets; if you still hit it, set `is_instance: false` on that box group.
- **Bin looks sparse** — the object footprints are large relative to `bin_size`; either enlarge
  `--bin-size`, raise `--scale-counts`, or add smaller box assets (unfittable boxes are teleported
  out of view, so raising counts alone does not densify).
- **Layout not preserved after physics** — `physics: rigidbody` makes boxes fall/reorder, discarding
  the bin-pack layout; omit physics to keep the packed arrangement.
- **`PATH_TO_BOXES` errors** — the shipped `demo_bin_pack.yaml` and this generator need no local
  assets; other demos (`demo_bins_of_bins*`, `demo_table`) still require `PATH_TO_BOXES`.
- **`version` mismatch** — set `version` to the installed `isaacsim.replicator.object.core`
  (use `--from-ext "$ISAAC_SIM_DIR"`); shipped samples can lag.

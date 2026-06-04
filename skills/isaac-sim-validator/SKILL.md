---
name: isaac-sim-validator
description: >
  Validate Isaac Sim simulation scripts and outputs before delivery. Rejects
  black frames, deprecated `omni.isaac.*` imports, hardcoded user paths/IPs,
  missing lights, wrong render mode, undersized SimulationApp configs, and bad
  RL output formats. Use when a sub-agent produces a script, a render, a
  training artifact, or any USD scene that's about to be handed to the user.
---

# Isaac Sim Validator

Final gate before delivery. Reject outputs that fail the checks below; do not soften results.

## Universal checklist (apply to every output)

| Check | Pass | On fail |
|---|---|---|
| Script path | exists, readable | reject: `Script path invalid: <path>` |
| File size | > 1 KB | reject: `Script empty or corrupted` |
| `isaacsim` import | uses `isaacsim.*`, not `omni.isaac.core` | reject: `Use isaacsim.* namespace; omni.isaac.core is deprecated` |
| `SimulationApp` config | explicit `width` and `height` | reject: `SimulationApp without window args causes swapchain-capture size mismatch` |
| Lights present | `DomeLight` (intensity >= 100) **and** `DistantLight` (>= 500) | reject: `No lighting; render will be black` |
| Render mode | `RayTracedLighting` for iteration; `PathTracing` only for hero shots | reject: `PathTracing too slow for iteration; use RayTracedLighting` |
| ACES tonemap | `/rtx/post/tonemap/op=4`, `filmIso` 200 default / 600 deep-aisle / 400 aerial | suggest: `Enable ACES (op=4) for contrast` |
| Final render | >= 150 KB and mean_RGB > 30 | reject: `Render is black or low-energy; check lighting` |
| User paths | no `/home/<user>/`, `C:\Users\<name>\`, or per-agent home paths | reject: `Hardcoded user path; use $ISAAC_SIM_DIR, $ISAAC_LAB_DIR, $WORKSPACE_DIR, or argparse` |
| Network endpoints | no bare IPs (`192.168.x.x`, `10.x.x.x`) or non-localhost hosts | reject: `Hardcoded endpoint; parameterize via env var or config` |
| Output dir | configurable (env var, CLI, or function param) | reject: `Output dir hardcoded; accept as parameter or read $OUTPUT_DIR` |
| VRAM headroom | `nvidia-smi` headroom > 20% after settle | warn at >90% utilization; likely leak or scale issue |
| `pkill -f "kit/kit"` used | flag for kill+relaunch bug | warn: `Verify zero kit processes before next launch` |
| Sim duration | >= 3 s of physics | reject: `Sim ran < 3 s; physics has not settled` |

## File structure validation (USD scenes)

| Item | Valid | Invalid | Action |
|---|---|---|---|
| `$ISAAC_LAB_DIR/scripts/` | read-only reference | writing outputs there | redirect to `$OUTPUT_DIR` or `$WORKSPACE_DIR/outputs/` |
| `get_assets_root_path() -> s3://...` | runtime OK | baked into delivered output | cache locally; reference via configurable path |
| `Melting` / `Splat` MDL-only shaders | OK in raw form | MDL-only at export | filter out; MDL-only causes black render |
| `MakeInstanceable` for RL | required (`make_instanceable: true`) | absent | reject |
| `usdPreviewSurface` on surfaces | required | absent | reject; suggest adding `UsdPreviewSurface` |
| `convex_decompose_mesh` | `false` (with `clean_sdf_dc`) | `true` | recommend `false` for stability |
| `mpu=0.01` | OK for warehouse shell (13 m tiles) | on a robot | reject |

## Render validation (frame analysis)

| Metric | Good | Bad | Action |
|---|---|---|---|
| File size | 1–4 MB | ~82 KB | 82 KB = black, reject |
| Mean RGB | 80–160 | < 20 | reject: `Too dark; add lights` |
| Max RGB | 200–250 | 0 | reject: `No light in scene` |
| Pixel variance | > 15 | < 5 | flat = missing shadows; add accents |
| Color histogram | multi-modal | single peak | overexposed or flat lighting; adjust intensity |
| Object visibility | >= 3 robots or 50+ assets | < 10 visible | suggest: `Scene too sparse; add set dressing` |
| Shadow consistency | consistent shadows | random black blobs | reject: `Shadow artifacts; check light cone bounds` |

## RL training validation

| Issue | Action |
|---|---|
| `AppLauncher` not first | reject: `AppLauncher must precede any isaaclab imports` |
| Task `__init__.py` non-empty | reject: `Task __init__.py must be empty; do not import configs` |
| `omni.isaac.lab` import | reject: `Use isaaclab.*; omni.isaac.lab is deprecated` |
| PhysX backend (when Newton applies) | reject: `Use Newton; it is the default on Kit 110` |
| Missing `PYTHONUNBUFFERED=1` | warn: `Set PYTHONUNBUFFERED=1 to see logs in real time` |
| No GPU memory logging | suggest: `Log torch.cuda.memory_allocated()/1e6 to W&B` |
| Batch size exceeds VRAM | suggest: `Reduce num_envs (try 2048)` |
| Wrong checkpoint name | reject: `Must be model_XXXX.zip, not policy.pt` |
| Single seed | suggest: `Run seeds 42, 123, 456 for valid comparison` |

## Asset & shell validation

| Input | Check | Action |
|---|---|---|
| `docker run` command | `-v` mount for `/workspace/output` | reject: `Output folder not mounted` |
| Sub-agent spawn | `workdir=$WORKSPACE_DIR` | reject: `Sub-agent workdir hardcoded; use $WORKSPACE_DIR` |
| Blender USD export | correct mpu, dual-shader (UsdPreviewSurface + MDL), correct export type | reject if any missing |
| Scene complexity | renders within 10 settle steps | reject: `Scene too complex; simplify assets` |
| Ghost artifacts | cleanup of `/dev/shm/carb-*` | reject: `Use workspace_cleanup.py to clear /dev/shm/carb-*` |

## Decision tree

```mermaid
graph TD
    A[Received Output] --> B{Has script?}
    B -->|No| C[Reject: "No script provided"]
    B -->|Yes| D{File size >1KB?}
    D -->|No| E[Reject: "Script empty or corrupted"]
    D -->|Yes| F{Python?}
    F -->|Yes| G{isaacsim.* imports?}
    G -->|No| H[Reject: "Use isaacsim.* not omni.isaac.core"]
    G -->|Yes| I{SimulationApp or --exec?}
    I -->|Neither| J[Reject: "Must use SimulationApp with window args OR isaac-sim.sh --exec"]
    I -->|SimulationApp| K{width and height set?}
    K -->|No| L[Reject: "SimulationApp requires explicit width and height"]
    K -->|Yes| M{Dome + Distant lights?}
    M -->|No| N[Reject: "Add DomeLight and DistantLight (min intensity 100 and 500)"]
    M -->|Yes| O{RayTracedLighting?}
    O -->|No| P[Reject: "Use RayTracedLighting for iteration"]
    O -->|Yes| Q{Render output exists?}
    Q -->|No| R[Reject: "No render output"]
    Q -->|Yes| S{>= 150 KB and mean_RGB > 30?}
    S -->|No| T[Reject: "Render is black, too dark, or corrupted"]
    S -->|Yes| U{Hardcoded user paths or bare IPs?}
    U -->|Yes| V[Reject: "Replace hardcoded user paths/IPs with env vars or args"]
    U -->|No| Y[VALID - deliver]
```

## Training output

| Item | Action |
|---|---|
| `policy_only_XXXX.pt` | accept |
| `model_XXXX.zip` | accept |
| `log.csv` or `wandb` URL | accept |
| `results.json` with episode data | accept |
| No reward curve / metrics | reject: `Log episode/reward_mean and train/value_loss` |

## Reply format

- Pass: `Valid: <one-line summary>`
- Fail: `Invalid: <list of failed checks>`
- Suggest (only when otherwise valid): `Could be improved: <tip>`

Do not hedge ("I think", "maybe"). Do not deliver a black frame.

## Input sources

- `isaac-sim-orchestrator`, `isaac-sim-rendering`, other producer skills, and sub-agent outputs.
- Invoked on: "validate this", "check this render", "review this code".

## Feedback loop

- Rejections feed back into the producing skill's failure-mode catalog.
- Recurring failures become new validator rules.

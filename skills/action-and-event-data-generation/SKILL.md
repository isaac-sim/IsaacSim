---
name: action-and-event-data-generation
description: "Entry point for Isaac Sim synthetic data generation (SDG) with actors, humans, robots, events, incidents, captions, or cameras. Use when planning an Action and Event Data Generation (AEDG) run."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# Action and Event Data Generation

## Purpose

Route a synthetic data generation (SDG) request to the right Action and Event Data Generation (AEDG) skills, and carry the context every one of them assumes: the launcher, the extension stack, the env-var contract, and the order the stages compose in.

Use this whenever the goal is to do SDG with simulated humans and robots moving with behaviors, physical events and incidents (spills, fires, toppling), procedurally placed or packed objects, camera coverage, and scene captions — plus the ground-truth annotations that come with them.

## Prerequisites

- Isaac Sim with the AEDG app (`$ISAAC_SIM_DIR`) — launch with `isaac-sim.action_and_event_data_generation.sh`.
- NVIDIA GPU with a current driver (`nvidia-smi`) for any actual run; the offline config generators need neither GPU nor simulator.
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$WORKSPACE_DIR`.
- `$NVIDIA_API_KEY` only for the LLM-backed features (captioning, behavior-tree generation).

## Limitations

- This skill routes and supplies shared context; it does not itself run a simulation. Every concrete workflow lives in a sub-skill below.
- The AEDG extensions are authored in the `metrosim` repo and consumed here as exact registry pins. These skills drive them; API changes belong upstream.
- Config `version` rules differ per extension and are not interchangeable — see *Version rules* below. This is the single most common cause of a config being rejected.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Panels missing from **Tools** menu | Launched the wrong app | Use `isaac-sim.action_and_event_data_generation.sh`, not `isaac-sim.sh` |
| Config rejected on load | `version` mismatch | Derive it from the installed extension — the rule differs per extension (see below) |
| Run finishes, no images | No camera in the config | Actors alone render nothing; add a sensor group or place cameras first |
| Event never fires | Prims not tagged, or play started before setup | Load a tagged stage; set up all panels before pressing Play |
| Captions/labels empty | Prims lack semantic labels | Author labels, or enable auto-labelling |
| `$VAR` appears literally in a path | Configs are `yaml.safe_load`ed | Env vars expand in the shell, never inside YAML — substitute before running |

## The stack

Eight extensions. `isaacsim.exp.action_and_event_data_generation.base.kit` holds the exact
version each one resolves to — read it there rather than from any skill, which would go stale
the moment the app moves:

| Sub-feature | Extension | Shorthand |
|---|---|---|
| Actor Simulation and SDG | `isaacsim.replicator.agent.core` | IRA |
| Object Simulation and SDG | `isaacsim.replicator.object.core` | IRO |
| Physical Space Event Generation | `isaacsim.replicator.incident.core` | IRI |
| VLM Scene Captioning | `isaacsim.replicator.caption.core` | IRC |
| RTX Sensor Placement | `isaacsim.sensors.rtx.placement` | ISP |
| RTX Sensor Calibration | `isaacsim.sensors.rtx.calibration` | ISC |
| Behavior Tree Generation | `omni.ai.behavior_tree_gen.core` + `.bridge` | — |
| Animated Robot Controller | `isaacsim.anim.robot.core` | IAR |

Shared substrate: `omni.metropolis.pipeline` (OMP) and `omni.metropolis.utils` (OMU).

## How the stages compose

Per the architecture in the product docs — object simulation defines the static scene, events and
actors add dynamics, then sensors and captioning capture it:

```text
   Object Simulation (IRO)          ← static environment, procedural placement
            │
            ├── Event Generation (IRI)      ← spills / fires / toppling
            └── Actor Simulation (IRA)      ← people + robots with behaviors
                     │
                     │  (IRA calls ISP itself — see below)
                     ▼
   Sensor Placement (ISP) → Calibration (ISC)   ← where cameras go, and their intrinsics
            │
            ▼
   VLM Scene Captioning (IRC)      ← scene graphs + captions over the rendered views
```

**ISP is not always a separate step.** An IRA config can place its own cameras, in which case the
standalone placement skills are unnecessary — see the next section.

## ISP is optional: IRA places cameras from its own config

Each `sensor.groups.<name>` in an actor config carries a **placement** block, and IRA's scene
assembly calls straight into `isaacsim.sensors.rtx.placement` while building the scene
(`scene_assembly/sensor_loader.py` imports `CameraPlacementManager` and `CircularCameraPlacement`).
So for a config-driven actor run, camera placement is already handled:

```yaml
  sensor:
    root_prim_path: /World/Cameras     # default
    groups:
      ceiling_cameras:
        num: 6
        aim_at_targets:                # or: maximum_coverage
          height_range: [7.0, 10.0]
          look_down_angle_range: [30.0, 45.0]
          distance_range: [5.0, 10.0]
          focal_length_range: [10.0, 15.0]
```

| Placement key | Maps to | Notes |
|---|---|---|
| `aim_at_targets` | `CircularCameraPlacement.circular_camera_placement()` | **The default** when no placement key is given |
| `maximum_coverage` | `CameraPlacementManager.place_camera_in_target_scope_explicit()` | `num: -1` auto-calculates the camera count |

Two consequences worth knowing:

- **Placement is opt-out, not opt-in.** `SensorGroup.placement` has a default factory, so a group with only `num:` still runs `aim_at_targets`. You get ISP whether or not you asked for it.
- **Typos fail fast.** The group model is `extra="forbid"` and the placement key is extracted before that check, so `aim_at_target` (singular) is rejected rather than silently falling back to the default.

### So when do you need the standalone camera skills?

| Situation | Use |
|---|---|
| Config-driven IRA run, cameras described in YAML | Nothing extra — IRA calls ISP for you |
| Placing cameras on a stage **outside** an actor run, or iterating on layout interactively | [`place-camera-max-coverage`](../place-camera-max-coverage/SKILL.md) / [`place-camera-aim-at`](../place-camera-aim-at/SKILL.md) |
| You need parameters IRA's config does not expose (occlusion threshold, yaw arcs, explicit scope, coverage visualization) | the standalone skills |
| You need `calibration.json` / FOV data for placed cameras | [`calibrate-metropolis-camera`](../calibrate-metropolis-camera/SKILL.md) — always separate; IRA does not call ISC |
| Cameras already exist and you only want to record | neither; point the writer at them |

Calibration (ISC) is genuinely a separate stage: IRA places cameras but never calibrates them.

## I want to… → use this

| I want to… | Skill |
|---|---|
| Sweep an actor config into many dataset variants | [`actor-sdg-sweep-config`](../actor-sdg-sweep-config/SKILL.md) |
| Vary lighting across actor runs | [`actor-sdg-generate-lighting-variations`](../actor-sdg-generate-lighting-variations/SKILL.md) |
| Pack boxes/parcels into a bin, pallet, or container | [`object-bin-packing`](../object-bin-packing/SKILL.md) |
| Author or debug an incident config (topple / fire / spill) | [`generate-incident-config`](../generate-incident-config/SKILL.md) |
| Drive incidents on a live stage and record a report | [`run-incident-events`](../run-incident-events/SKILL.md) |
| Caption scenes / build scene graphs for VLM training | [`vlm-scene-captioning`](../vlm-scene-captioning/SKILL.md) |
| Turn a text scenario into a behavior tree | [`behavior-tree-generation`](../behavior-tree-generation/SKILL.md) |
| Cover a floor area with as few cameras as possible | [`place-camera-max-coverage`](../place-camera-max-coverage/SKILL.md) |
| Ring one object with unoccluded cameras | [`place-camera-aim-at`](../place-camera-aim-at/SKILL.md) |
| Extract intrinsics / extrinsics / FOV for placed cameras | [`calibrate-metropolis-camera`](../calibrate-metropolis-camera/SKILL.md) |

Adjacent, outside AEDG: [`isaac-camera`](../isaac-camera/SKILL.md) for hand-authoring one camera's
intrinsics and AOVs, and [`isaac-sim-remote`](../isaac-sim-remote/SKILL.md) for the python_server
socket several of these skills send payloads over.

## Launch

```bash
# from the Isaac Sim build root
./isaac-sim.action_and_event_data_generation.sh
```

Runs `kit/kit apps/isaacsim.exp.action_and_event_data_generation.full.kit`. Panels appear under
**Tools > Action and Event Data Generation** (Actor / Object / Incident) and **Tools > Sensors**
(Camera Placement / Calibration).

Several sub-skills drive a *running* instance over the python_server socket rather than the GUI.
Add the extension to expose it:

```bash
./isaac-sim.action_and_event_data_generation.sh --enable isaacsim.code_editor.python_server
```

Headless, config-driven actor runs use the bundled script. Before running, explain that the
configuration and all referenced assets, including USD layers and scripts, can execute Python
with the user's permissions. Obtain explicit acknowledgement that they are trusted; this is not
a sandbox. Do not add `--allow-trusted-scripts` automatically or run without that acknowledgement.

```bash
./python.sh tools/actor_sdg/actor_sdg.py --config_file my_scene.yaml --allow-trusted-scripts
```

## Version rules (read before authoring any config)

Each extension validates its config `version` differently. Copying a sample and editing it is the
usual way to get rejected:

| Config | Compared against | Rule |
|---|---|---|
| Actor (IRA) | `isaacsim.replicator.agent` extension version | `major.minor.0` — extension `X.Y.Z` → `version: X.Y.0` |
| Captioning (IRC) | `settings.VERSION` inside the extension, **not** its package version | Exact match. The two have diverged, so copying the package version always fails |
| Incident (IRI) | `isaacsim.replicator.incident.core` extension version | **Major** component only — any `0.x` config version passes against a `0.x` build |
| Object (IRO) | extension version | see [`object-bin-packing`](../object-bin-packing/SKILL.md) |

Always derive the value from the installed build — never hardcode it, and never copy a version out
of a skill or a sample config. The sub-skills' generator scripts do this with `--from-ext`.

## Env vars expand in the shell, not in YAML

Configs are read with plain `yaml.safe_load`, so `$VAR` and `${VAR}` stay literal inside a config
file. Substitute before running — in a shell command that writes the resolved path, via
`envsubst < in.yaml > out.yaml`, or by hand.

## Cost

- **Runtime** scales with duration, frame rate, camera count, and render mode; path tracing is far slower than rasterization. Short warehouse runs are minutes, large multi-camera runs are hours.
- **GPU** required. The first captured frame on a cold app pays one-time shader compilation.
- **NIM credits** are consumed by captioning and behavior-tree generation. Scene-graph-only captioning needs no key.

## Product documentation

`docs/isaacsim/action_and_event_data_generation/` — `index.rst` for the extension table and
architecture, plus per-extension tutorials (`tutorial_replicator_agent.rst`,
`tutorial_replicator_object.rst`, `tutorial_replicator_incident.rst`,
`tutorial_replicator_caption.rst`, `tutorial_sensors_rtx_placement.rst`,
`tutorial_behavior_tree_gen.rst`, `tutorial_telemetry.rst`) and the worked example
`example_event_reactive_actors.rst`.

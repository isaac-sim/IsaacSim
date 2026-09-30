---
name: run-incident-events
description: "Tag prims and drive IRI topple, fire, and spill incidents on a live stage, then record a report. Use when running incidents; to author the config YAML use generate-incident-config."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
  owner: Isaac Sim
  service: isaacsim.replicator.incident
  version: "1.0.0"
  reviewed: "2026-07-27"
---

# Run IRI Incident Events

## Purpose

Make incidents happen on a loaded stage using the `isaacsim.replicator.incident.core`
Python API: tag prims as loose / flammable / leakable / spillable-area, build
topple, fire, and spill events against them, and record an
`incidents_report.json`. Works from a config file or directly through the event
managers.

## Prerequisites

- Isaac Sim 6 / Kit 110 running with the Python server (`isaacsim.code_editor.python_server`,
  port 8226) — see `isaac-sim-remote`.
- `isaacsim.replicator.incident.core` resolvable. It is pinned by the Action and
  Event Data Generation app; launch that app, or
  `--enable isaacsim.replicator.incident.core`.
- A loaded stage. Topple items tagged `NavMesh` also need a bakeable navmesh.
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$WORKSPACE_DIR`.

## Limitations

- **This repo does not build the extension.** `isaacsim.replicator.incident.core`
  is authored in the `metrosim` repo and consumed here from `extscache`. This
  skill drives it; API changes belong upstream.
- Needs a live Kit session. Physics, the pyro solver, and the navmesh have no
  offline equivalent.
- **Every `generate_*_event` failure path returns `None` and logs through
  `carb`.** Nothing raises. The script re-reads each manager's registry to turn a
  silent skip into an error.
- **Never read `GlobalValues.incident_manager`.** On builds where `on_startup`
  assigns the manager and then overwrites it with the `ConfigFileLoader`, that slot
  holds the wrong object. Reach the manager through
  `get_instance().get_incident_manager()`, which is correct on every build.
- A config file's `flammable_nearby_radius` has no effect; see
  `generate-incident-config`. The API's `pyro_nearby_radius` does work.
- Incident tags live in USD attributes. They persist only if you save the stage.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| `IRI_PREFLIGHT: ext=unavailable` | App does not resolve the extension | Launch the Action and Event Data Generation app, or add `--enable isaacsim.replicator.incident.core` |
| `IRI_PREFLIGHT: ... loose=0 flammable=0 leakable=0` | Nothing on the stage is tagged | Run `action=tag`, or open `Isaac/Samples/Replicator/Incidents/full_warehouse_with_incident_tags.usd` |
| `'<path>' is not a tagged prim` in the console | Event names a prim that carries no matching tag | Tag it first; confirm the tag type matches the event type |
| Event was not registered | Random pool exhausted, or the item was consumed by an earlier event | Tag more items, pin `item` to distinct paths, or reduce `radius` |
| Two fire events land on the same prim | On builds where the pyro pool never shrinks, random selection can repeat | Pin `item` to distinct paths |
| `tagged_on_stage` is lower than requested | The apply command retargets onto the nearest prim holding a `RigidBodyAPI`, and skips a prim whose ancestor/descendant is already tagged | Expected. Check the reported paths |
| Toppled items fall the wrong way | `NavMesh` direction resolved without a baked navmesh | Keep `bake_navmesh=true`, or retag as `RandomDir` |
| Liquid spawns at z=0 instead of on the floor, `[SpillFloorSearch] No floor found` in the log | No tagged `spillable_area` sits *beneath the leaking item*. Warehouse floors are many prims, so tagging one tile covers only what is above that tile | Check `spill_covered` in preflight; tag the specific tile under the item, or every tile. Harmless when the floor is already at z=0, wrong on a raised or multi-level floor |
| Fire missing from segmentation output | Fire is a volume prim, skipped by rasterization annotators | Set `/syntheticdata/sensors/captureVolumes` to `true` |
| Report file never appears | Recording never started, or the timeline never played | `start_record` → play → `stop_record`; recording only accumulates while playing |
| Report times do not match trigger times | `simulation_data` is in frames, `trigger_data` in seconds | Convert with the timeline FPS |

## When to use this skill

| You want | Skill |
|---|---|
| Tag prims, fire incidents on a live stage, record a report | **this skill** |
| Write or validate the incident config YAML | `generate-incident-config` |
| Capture RGB / segmentation of the incident | `data-collection-sim`, `isaac-sim-sensor` |
| Place the cameras that watch the incident | `place-camera-aim-at`, `place-camera-max-coverage` |

## How the pieces fit

```
tag prims (USD attributes)  ─┐
                             ├─► event managers ──► triggers ──► timeline plays
config YAML  ────────────────┘         │
                                       ▼
                          IncidentReport ──► incidents_report.json
```

`IncidentManager` owns one lazily-created manager per event type and a single
shared `IncidentReport`. Events resolve their target against the tagged items
present when the event is generated, so **tag before you generate**. Triggers come
from `TriggersManager` and fire against the timeline, so nothing happens until
you play.

Full API surface is in [`references/api-reference.md`](references/api-reference.md);
report semantics and the silent-failure paths are in
[`references/runtime-notes.md`](references/runtime-notes.md). An index of both is in [`references/README.md`](references/README.md).

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/run_incident_events.py` | Preflight, tag prims, add events, apply a config, record a report | injected `key=value` args (see script docstring) |

## Running scripts

The script is a python_server payload, not a standalone program. From agent
runtimes that expose skill execution helpers, invoke it with `run_script()`:

```python
run_script("scripts/run_incident_events.py", args=["action=preflight"])
```

From a built tree, send it through the `isaac-sim-remote` client:

```bash
python skills/isaac-sim-remote/scripts/isaacsim_send.py \
  --file skills/run-incident-events/scripts/run_incident_events.py \
  --arg action=tag --arg prims=/World/Box01,/World/Box02 --arg tag=loose
```

## Workflow

1. **Preflight.** `action=preflight` (the default) reports the extension, stage,
   per-tag counts, navmesh, and recording state without touching anything. It
   never raises.

   ```python
   run_script("scripts/run_incident_events.py", args=["action=preflight"])
   ```

   Expect `IRI_PREFLIGHT: ext=enabled stage=ok loose=12 flammable=4 leakable=2
   spillable_area=1 spill_covered=2/2 navmesh=ok recording=no`. Resolve anything
   that is not `ok` first — the remedy lines say how.

   `spill_covered` is how many leakable items actually have a tagged spillable
   area beneath them; below `N/N` means those spills will fall back to `z=0.0`.

2. **Tag.** Skip if the stage is already tagged.

   ```python
   run_script("scripts/run_incident_events.py", args=[
       "action=tag", "prims=/World/Box01,/World/Box02", "tag=loose", "tag_type=NavMesh"])
   ```

   | `tag` | `tag_type` values | Meaning |
   |---|---|---|
   | `loose` | `NavMesh`, `RandomDir`, `ClosestWaypoint` | Direction the item topples toward |
   | `flammable` | `Box` | Can catch fire; needs a visible mesh as fuel |
   | `leakable` | `Item` | Can spill liquid |
   | `spillable_area` | `Floor` | Surface the liquid lands on — must be **beneath the leaking item** |

   The report shows what actually carries the tag, which can differ from what you
   asked for — the command retargets onto the nearest prim holding a
   `RigidBodyAPI`. **Save the stage** to persist tags across sessions.

   For spills, a floor is usually split into many prims — the sample warehouse has
   ~80 `SM_floor*` tiles — and only a tile **under** the leaking item counts. Tag
   the right tile, or all of them when the item is chosen at random. Re-run
   preflight and check `spill_covered=<covered>/<leakable>`; anything short of
   `N/N` is listed by prim path.

3. **Create events.** Either path works; pick one.

   *From a config file* — the whole file at once:

   ```python
   run_script("scripts/run_incident_events.py", args=[
       "action=setup", "config=$WORKSPACE_DIR/iri_config.yaml"])
   ```

   Expect `IRI_SETUP_RESULT: ... requested=3 registered=3 ...`. **Any gap between
   `requested` and `registered` is a real failure** the extension only logged.

   *Through the API* — one event at a time, no file:

   ```python
   run_script("scripts/run_incident_events.py", args=[
       "action=add_event", "event=topple", "name=shelf collapse", "time=3", "radius=1.5"])
   ```

   Omit `item` to draw from the tagged pool; pass `item=/World/Box01` to pin one.

4. **Record.** Start recording, play the timeline, then stop. Recording only
   accumulates while the timeline runs.

   ```python
   run_script("scripts/run_incident_events.py", args=[
       "action=start_record", f"report_dir={workspace}/EventsResult"])
   # ... play the timeline past the last trigger time ...
   run_script("scripts/run_incident_events.py", args=["action=stop_record"])
   ```

5. **Verify.** Confirm the JSON exists and holds one top-level key per event name.
   `end_recording()` logs write failures through `carb` rather than raising.

   ```bash
   python3 -c "import json,sys; print(list(json.load(open(sys.argv[1]))))" \
       "$WORKSPACE_DIR/EventsResult/incidents_report.json"
   ```

## Calling the API directly

```python
from isaacsim.replicator.incident.core import get_instance
from isaacsim.replicator.incident.core.extension import IncidentExt
from isaacsim.replicator.incident.core.settings import IncidentSettings
from omni.metropolis.pipeline.triggers import TriggersManager
import omni.kit.commands

omni.kit.commands.execute("ApplyLooseItemTagCommand", prims="/World/Box01", loose_item_type="NavMesh")

manager = get_instance().get_incident_manager()      # never GlobalValues.incident_manager
report = manager.get_incident_report()
trigger = TriggersManager.get_instance().create_trigger_by_dict({"trigger": {"type": "time", "time": 3.0}})

topple = manager.create_topple_event_manager(seed=12345, report=report)
topple.generate_topple_event(
    name="shelf collapse",
    selected_loose_item=IncidentSettings.RANDOM_LOOSE_ITEM,
    topple_nearby_radius=1.5,
    trigger=trigger,
)
if "shelf collapse" not in topple.topple_events:
    raise RuntimeError("event was skipped; see the carb log")

report.start_recording("/workspace/EventsResult")    # then play the timeline
```

`generate_*_event` returns `None` on both success and failure — checking the
manager's registry is the only way to tell them apart. Fire events additionally
need `data_path=IncidentExt.data_path` when creating their manager.

## Related skills

- `generate-incident-config` — authors and validates the config `action=setup` consumes.
- `isaac-sim-remote` — the transport this skill's script rides on.
- `data-collection-sim` — Replicator writers that capture the incident semantic labels.
- `isaac-sim-sensor` — annotators for segmentation and bounding boxes over the incident.
- `physics-simulation` — the physics scene toppling and spilling depend on.
